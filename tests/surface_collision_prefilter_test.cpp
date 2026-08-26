#include "core/algorithms/cam/surface_collision_prefilter.h"

#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRep_Builder.hxx>
#include <Bnd_Box.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_Reader.hxx>
#include <gp_Trsf.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pln.hxx>
#include <gp_Vec.hxx>
#include <TopoDS_Compound.hxx>

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QTextStream>

#include <atomic>
#include <cmath>
#include <thread>
#include <vector>

namespace {

int fail(const QString& message)
{
    QTextStream(stderr) << message << '\n';
    return 1;
}

gp_Trsf translation(double x, double y, double z)
{
    gp_Trsf result;
    result.SetTranslation(gp_Vec(x, y, z));
    return result;
}

gp_Trsf cutterPose(const gp_Pnt& tcp, const gp_Dir& outward)
{
    gp_Trsf result;
    result.SetDisplacement(gp_Ax3(), gp_Ax3(tcp, outward));
    return result;
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
    const TopoDS_Shape cone = BRepPrimAPI_MakeCone(0.2, 5.0, 20.0).Shape();

    std::string error;
    const auto workpieceSurface = lcnc::cam_algo::SurfaceCollisionModel::build(
        workpiece, 0.05, &error);
    if (!workpieceSurface.isValid())
        return fail(QStringLiteral("Cannot build workpiece surface BVH: %1")
                    .arg(QString::fromStdString(error)));
    const auto cutterSurface = lcnc::cam_algo::SurfaceCollisionModel::build(cone, 0.05, &error);
    if (!cutterSurface.isValid())
        return fail(QStringLiteral("Cannot build cutter surface BVH: %1")
                    .arg(QString::fromStdString(error)));
    const auto restoredWorkpieceSurface =
        lcnc::cam_algo::SurfaceCollisionModel::buildFromTriangleSoup(
            workpieceSurface.triangleSoup(),
            workpieceSurface.linearDeflectionMm(),
            workpieceSurface.isClosedSolid(), &error);
    if (!restoredWorkpieceSurface.isValid()
        || restoredWorkpieceSurface.triangleCount()
            != workpieceSurface.triangleCount()
        || restoredWorkpieceSurface.isClosedSolid()
            != workpieceSurface.isClosedSolid()) {
        return fail(QStringLiteral("Persisted triangle soup did not rebuild the surface BVH: %1")
                    .arg(QString::fromStdString(error)));
    }

    Bnd_Box bounds;
    BRepBndLib::Add(workpiece, bounds);
    double xMin = 0.0, yMin = 0.0, zMin = 0.0;
    double xMax = 0.0, yMax = 0.0, zMax = 0.0;
    bounds.Get(xMin, yMin, zMin, xMax, yMax, zMax);
    const double centerX = (xMin + xMax) * 0.5;
    const double centerY = (yMin + yMax) * 0.5;
    constexpr double clearanceMm = 0.5;
    const double guardedDistance = clearanceMm
        + workpieceSurface.linearDeflectionMm() + cutterSurface.linearDeflectionMm();

    const auto safe = lcnc::cam_algo::prefilterSurfaceCollision(
        cutterSurface, translation(centerX, centerY, zMax + 1.0),
        workpieceSurface, gp_Trsf(), guardedDistance);
    if (!safe.valid || !safe.definitelySeparated)
        return fail(QStringLiteral("1 mm cutting offset was not rejected by the surface BVH"));
    const auto restoredSafe = lcnc::cam_algo::prefilterSurfaceCollision(
        cutterSurface, translation(centerX, centerY, zMax + 1.0),
        restoredWorkpieceSurface, gp_Trsf(), guardedDistance);
    if (!restoredSafe.valid
        || restoredSafe.definitelySeparated != safe.definitelySeparated)
        return fail(QStringLiteral("Restored surface BVH changed a separation decision"));

    const auto penetrating = lcnc::cam_algo::prefilterSurfaceCollision(
        cutterSurface, translation(centerX, centerY, zMax - 2.0),
        workpieceSurface, gp_Trsf(), guardedDistance);
    if (!penetrating.valid || penetrating.definitelySeparated)
        return fail(QStringLiteral("Surface BVH missed a penetrating cutter pose"));

    // Workpiece-proxy cutting nodes intentionally touch at the TCP.  Verify
    // that the lightweight outward probe can distinguish that contact before
    // the CAM scan enters serialized exact BRep distance.
    // 中文翻译：非机台模式切割节点在 TCP 正常接触；验证外移 BVH 探针可在精确 BRep 前排除该接触。
    const auto fineWorkpieceSurface = lcnc::cam_algo::SurfaceCollisionModel::build(
        workpiece, 0.01, &error);
    const auto fineCutterSurface = lcnc::cam_algo::SurfaceCollisionModel::build(
        cone, 0.01, &error);
    if (!fineWorkpieceSurface.isValid() || !fineCutterSurface.isValid())
        return fail(QStringLiteral("Cannot build fine workpiece-proxy collision meshes"));
    const gp_Dir topNormal(0.0, 0.0, 1.0);
    const gp_Trsf touchingPose = cutterPose(
        gp_Pnt(centerX, centerY, zMax), topNormal);
    const double fineGuard = fineWorkpieceSurface.linearDeflectionMm()
        + fineCutterSurface.linearDeflectionMm() + 1.0e-7;
    const auto touching = lcnc::cam_algo::prefilterSurfaceCollision(
        fineCutterSurface, touchingPose,
        fineWorkpieceSurface, gp_Trsf(), fineGuard);
    if (!touching.valid || touching.definitelySeparated)
        return fail(QStringLiteral("Tangent cutter contact was unexpectedly rejected"));
    gp_Trsf outwardProbe;
    outwardProbe.SetTranslation(gp_Vec(topNormal) * 0.05);
    const auto separatedProbe = lcnc::cam_algo::prefilterSurfaceCollision(
        fineCutterSurface, outwardProbe.Multiplied(touchingPose),
        fineWorkpieceSurface, gp_Trsf(), fineGuard);
    if (!separatedProbe.valid || !separatedProbe.definitelySeparated)
        return fail(QStringLiteral("Outward cutter-tip probe did not reject tangent contact"));

    const auto outerBox = lcnc::cam_algo::SurfaceCollisionModel::build(
        BRepPrimAPI_MakeBox(100.0, 100.0, 100.0).Shape(), 0.05, &error);
    const auto innerBox = lcnc::cam_algo::SurfaceCollisionModel::build(
        BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape(), 0.05, &error);
    const auto contained = lcnc::cam_algo::prefilterSurfaceCollision(
        innerBox, translation(49.5, 49.5, 49.5), outerBox, gp_Trsf(), guardedDistance);
    if (!contained.valid || contained.definitelySeparated)
        return fail(QStringLiteral("Surface BVH incorrectly rejected a contained solid"));

    const TopoDS_Shape mixedSolid =
        BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
    const TopoDS_Shape mixedOpenFace = BRepBuilderAPI_MakeFace(
        gp_Pln(gp_Pnt(20.0, 0.0, 0.0), gp_Dir(1.0, 0.0, 0.0)),
        -50.0, 50.0, -50.0, 50.0).Shape();
    TopoDS_Compound mixedBody;
    BRep_Builder mixedBuilder;
    mixedBuilder.MakeCompound(mixedBody);
    mixedBuilder.Add(mixedBody, mixedSolid);
    mixedBuilder.Add(mixedBody, mixedOpenFace);
    const auto solidOnlySurface = lcnc::cam_algo::SurfaceCollisionModel::build(
        mixedSolid, 0.05, &error);
    const auto mixedSurface = lcnc::cam_algo::SurfaceCollisionModel::build(
        mixedBody, 0.05, &error);
    if (!solidOnlySurface.isValid() || !mixedSurface.isValid()
        || mixedSurface.triangleCount() <= solidOnlySurface.triangleCount()) {
        return fail(QStringLiteral(
            "Mixed solid/open-face collision mesh dropped the open face"));
    }
    const auto containedInMixed =
        lcnc::cam_algo::SurfaceCollisionModel::build(
            BRepPrimAPI_MakeBox(gp_Pnt(2.0, 2.0, 2.0), 2.0, 2.0, 2.0).Shape(),
            0.05, &error);
    if (!containedInMixed.isValid()
        || !lcnc::cam_algo::surfaceCollisionContainmentDetected(
            containedInMixed, gp_Trsf(), mixedSurface, gp_Trsf())) {
        return fail(QStringLiteral(
            "Mixed solid/open-face collision mesh lost solid containment"));
    }
    const auto restoredMixedSurface =
        lcnc::cam_algo::SurfaceCollisionModel::buildFromTriangleSoup(
            mixedSurface.triangleSoup(), mixedSurface.containmentTriangleSoup(),
            mixedSurface.linearDeflectionMm(), mixedSurface.isClosedSolid(), &error);
    if (!restoredMixedSurface.isValid()
        || !lcnc::cam_algo::surfaceCollisionContainmentDetected(
            containedInMixed, gp_Trsf(), restoredMixedSurface, gp_Trsf())) {
        return fail(QStringLiteral(
            "Persisted mixed collision mesh lost solid containment"));
    }

    constexpr int threadCount = 4;
    constexpr int queriesPerThread = 2500;
    std::atomic_bool failed{false};
    QElapsedTimer timer;
    timer.start();
    std::vector<std::thread> workers;
    for (int thread = 0; thread < threadCount; ++thread) {
        workers.emplace_back([&, thread]() {
            for (int queryIndex = 0; queryIndex < queriesPerThread; ++queryIndex) {
                // Rotate around the dome so the cutter is inside the
                // workpiece's maximum AABB while remaining 1 mm outside the
                // actual surface. This exercises BVH traversal rather than
                // the trivial root-box rejection.
                // 中文翻译：沿半球旋转锥头，使其进入工件最大包围盒但仍距真实表面 1 mm，
                // 专门覆盖误检和慢速的非根节点路径。
                const double phase = static_cast<double>(queryIndex + thread * queriesPerThread);
                const double theta = 0.15 + 0.85 * std::fmod(phase * 0.017, 1.0);
                const double phi = std::fmod(phase * 0.071, 6.283185307179586);
                const double radius = zMax - zMin;
                const gp_Dir outward(std::sin(theta) * std::cos(phi),
                                     std::sin(theta) * std::sin(phi),
                                     std::cos(theta));
                const gp_Pnt surface(centerX + radius * outward.X(),
                                     centerY + radius * outward.Y(),
                                     zMin + radius * outward.Z());
                const gp_Pnt tcp = surface.Translated(gp_Vec(outward) * 1.0);
                const auto result = lcnc::cam_algo::prefilterSurfaceCollision(
                    cutterSurface, cutterPose(tcp, outward),
                    workpieceSurface, gp_Trsf(), guardedDistance);
                if (!result.valid || !result.definitelySeparated) {
                    failed.store(true);
                    return;
                }
            }
        });
    }
    for (auto& worker : workers)
        worker.join();
    if (failed.load())
        return fail(QStringLiteral("Parallel surface BVH query produced an inconsistent result"));
#if defined(LCNC_TEST_WITH_ASAN)
    constexpr qint64 performanceLimitMs = 15000;
#else
    constexpr qint64 performanceLimitMs = 5000;
#endif
    if (timer.elapsed() > performanceLimitMs)
        return fail(QStringLiteral("10,000 parallel surface BVH queries exceeded %1 ms: %2 ms")
                    .arg(performanceLimitMs).arg(timer.elapsed()));

    QTextStream(stdout) << "workpiece_triangles=" << workpieceSurface.triangleCount()
                        << " cutter_triangles=" << cutterSurface.triangleCount()
                        << " queries=10000 elapsed_ms=" << timer.elapsed() << '\n';
    return 0;
}
