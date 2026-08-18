#include "core/algorithms/cam/travel_path_planner.h"

#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <Bnd_Box.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_Reader.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QTextStream>

#include <cmath>

namespace {

int fail(const QString& message)
{
    QTextStream(stderr) << message << '\n';
    return 1;
}

lcnc::cam_algo::TravelEndpoint endpoint(std::uint64_t id,
                                        double x, double y, double z)
{
    lcnc::cam_algo::TravelEndpoint value;
    value.contourId = id;
    value.pose.axes = {x, y, z, 0.0, 0.0};
    value.pose.kinematicAxes = value.pose.axes;
    value.pose.activeMask = 0x07;
    value.pose.kinematicAxisMask = 0x07;
    value.pose.tcpX = x;
    value.pose.tcpY = y;
    value.pose.tcpZ = z;
    value.pose.surfaceNormalZ = 1.0;
    return value;
}

TopoDS_Shape placedAt(const TopoDS_Shape& local, double x, double y, double z)
{
    gp_Trsf transform;
    transform.SetTranslation(gp_Vec(x, y, z));
    BRepBuilderAPI_Transform placed(local, transform, Standard_True);
    return placed.IsDone() ? placed.Shape() : TopoDS_Shape{};
}

double distanceToWorkpiece(const TopoDS_Shape& cutter, const TopoDS_Shape& workpiece)
{
    BRepExtrema_DistShapeShape distance(cutter, workpiece);
    distance.SetDeflection(0.025);
    distance.SetMultiThread(Standard_False);
    distance.Perform();
    return distance.IsDone() ? distance.Value() : -1.0;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    const QString fixture = QString::fromUtf8(LCNC_HALF_SPHERE_MODEL_PATH);
    if (!QFileInfo::exists(fixture))
        return fail(QStringLiteral("Half-sphere STEP fixture is missing: %1").arg(fixture));

    STEPControl_Reader reader;
    if (reader.ReadFile(fixture.toUtf8().constData()) != IFSelect_RetDone
        || !reader.TransferRoots())
        return fail(QStringLiteral("Cannot import half-sphere STEP fixture"));
    const TopoDS_Shape workpiece = reader.OneShape();
    if (workpiece.IsNull())
        return fail(QStringLiteral("Half-sphere STEP fixture has no shape"));

    Bnd_Box bounds;
    BRepBndLib::Add(workpiece, bounds);
    if (bounds.IsVoid())
        return fail(QStringLiteral("Half-sphere STEP fixture has invalid bounds"));
    Standard_Real xMin = 0.0, yMin = 0.0, zMin = 0.0;
    Standard_Real xMax = 0.0, yMax = 0.0, zMax = 0.0;
    bounds.Get(xMin, yMin, zMin, xMax, yMax, zMax);
    const double centerX = (xMin + xMax) * 0.5;
    const double centerY = (yMin + yMax) * 0.5;

    constexpr double kCuttingHeightMm = 1.0;
    constexpr double kRapidHeightMm = 5.0;
    constexpr double kSafetyClearanceMm = 0.5;
    const TopoDS_Shape cone = BRepPrimAPI_MakeCone(0.01, 1.0, 15.0).Shape();
    if (cone.IsNull())
        return fail(QStringLiteral("Cannot create simulated cone"));

    // The proxy's tip is at local Z=0.  At the configured 1 mm cutting
    // height and 5 mm rapid height it must remain clear of the actual STEP
    // workpiece.  This catches a regression where continuous rapid IK erased
    // the cutting-height contribution after the first contour.
    // 中文翻译：代理尖端位于局部 Z=0；在切割高度 1 mm 与空程高度 5 mm 时必须始终避开真实 STEP 工件，用于防止连续空程 IK 清除后续轮廓切割高度的回归。
    const double cutDistance = distanceToWorkpiece(
        placedAt(cone, centerX, centerY, zMax + kCuttingHeightMm), workpiece);
    const double rapidDistance = distanceToWorkpiece(
        placedAt(cone, centerX, centerY, zMax + kRapidHeightMm), workpiece);
    if (cutDistance <= kSafetyClearanceMm || rapidDistance <= kSafetyClearanceMm) {
        return fail(QStringLiteral("Configured cutting/rapid heights intersect the half-sphere fixture "
                                   "(cut=%1, rapid=%2)")
                        .arg(cutDistance, 0, 'g', 12)
                        .arg(rapidDistance, 0, 'g', 12));
    }

    lcnc::cam_algo::TravelPlanningRequest request;
    request.workpiece = workpiece;
    request.cutterCollisionProxy = cone;
    request.minimumClearanceMm = kSafetyClearanceMm;
    request.maximumSafetyOffsetMm = 50.0;
    request.surfacePathStepMm = 1.0;
    request.collisionSampleStepMm = 2.0;
    request.motionProfile.supportedCoordinatedMask = 0x07;
    request.motionProfile.velocity.fill(100.0);
    request.motionProfile.acceleration.fill(1000.0);
    request.motionProfile.jerk.fill(10000.0);
    const auto source = endpoint(1, centerX - 5.0, centerY, zMax + kCuttingHeightMm);
    const auto target = endpoint(2, centerX + 5.0, centerY, zMax + kCuttingHeightMm);
    request.transitions.append({source, target, kRapidHeightMm,
                                kCuttingHeightMm, kCuttingHeightMm});

    QElapsedTimer timer;
    timer.start();
    const auto plan = lcnc::cam_algo::TravelPathPlanner::plan(request);
    if (!plan.isExecutable() || plan.transitions.size() != 1
        || plan.transitions.front().segments.isEmpty()) {
        return fail(QStringLiteral("Half-sphere 1 mm / 5 mm rapid plan was not executable: %1")
                        .arg(plan.failureReason));
    }
    if (timer.elapsed() > 5000)
        return fail(QStringLiteral("Half-sphere rapid planning exceeded 5 seconds"));

    for (const auto& segment : plan.transitions.front().segments) {
        const auto& pose = segment.target;
        const double distance = distanceToWorkpiece(
            placedAt(cone, pose.tcpX, pose.tcpY, pose.tcpZ), workpiece);
        if (distance <= kSafetyClearanceMm) {
            return fail(QStringLiteral("Half-sphere rapid plan contains a collision at (%1, %2, %3), distance=%4")
                            .arg(pose.tcpX, 0, 'g', 12)
                            .arg(pose.tcpY, 0, 'g', 12)
                            .arg(pose.tcpZ, 0, 'g', 12)
                            .arg(distance, 0, 'g', 12));
        }
    }
    return 0;
}
