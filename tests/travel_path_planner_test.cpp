#include "core/algorithms/cam/travel_path_planner.h"

#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>

#include <QCoreApplication>
#include <QElapsedTimer>
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
    lcnc::cam_algo::TravelEndpoint result;
    result.contourId = id;
    result.pose.axes = {x, y, z, 0.0, 0.0};
    result.pose.kinematicAxes = result.pose.axes;
    result.pose.activeMask = 0x1f;
    result.pose.kinematicAxisMask = 0x1f;
    result.pose.tcpX = x;
    result.pose.tcpY = y;
    result.pose.tcpZ = z;
    result.pose.surfaceNormalZ = 1.0;
    return result;
}

lcnc::cam_algo::TravelPlanningRequest baseRequest()
{
    lcnc::cam_algo::TravelPlanningRequest request;
    request.workpiece = BRepPrimAPI_MakeBox(
        gp_Pnt(-5.0, -5.0, -1.0), 30.0, 10.0, 1.0).Shape();
    request.cutterCollisionProxy = BRepPrimAPI_MakeCone(0.1, 1.0, 10.0).Shape();
    request.minimumClearanceMm = 0.5;
    request.surfacePathStepMm = 1.0;
    request.motionProfile.supportedCoordinatedMask = 0x1f;
    request.motionProfile.velocity.fill(100.0);
    request.motionProfile.acceleration.fill(1000.0);
    request.motionProfile.jerk.fill(10000.0);
    return request;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    {
        auto request = baseRequest();
        request.workpiece = {};
        request.endpoints = {endpoint(1, 0.0, 0.0, 0.0),
                             endpoint(2, 10.0, 0.0, 0.0)};
        const auto plan = lcnc::cam_algo::TravelPathPlanner::plan(request);
        if (plan.isExecutable() || plan.failureReason.isEmpty())
            return fail(QStringLiteral("Missing workpiece collision geometry was accepted"));
    }

    {
        // A non-machining boss crossing the nominal surface route forces the
        // complete cone proxy to a higher safe trajectory.  This is computed
        // offline; controller idle height may only add more clearance later.
        auto request = baseRequest();
        const TopoDS_Shape base = BRepPrimAPI_MakeBox(
            gp_Pnt(-5.0, -5.0, -1.0), 30.0, 10.0, 1.0).Shape();
        const TopoDS_Shape boss = BRepPrimAPI_MakeBox(
            gp_Pnt(8.0, 2.0, 0.0), 4.0, 2.0, 6.0).Shape();
        request.workpiece = BRepAlgoAPI_Fuse(base, boss).Shape();
        request.cutterCollisionProxy = BRepPrimAPI_MakeCone(0.2, 4.0, 15.0).Shape();
        request.maximumSafetyOffsetMm = 30.0;
        lcnc::cam_algo::TravelPlanningDiagnostics diagnostics;
        request.diagnostics = &diagnostics;
        request.endpoints = {endpoint(1, 0.0, 0.0, 0.0),
                             endpoint(2, 20.0, 0.0, 0.0)};
        const auto plan = lcnc::cam_algo::TravelPathPlanner::plan(request);
        if (!plan.isExecutable()
            || plan.transitions.front().maximumSurfaceOffsetMm <= 1.0) {
            return fail(QStringLiteral("Cutter proxy did not raise the rapid above a non-machining obstacle"));
        }
        if (diagnostics.exactDistanceCheckCount != 0
            || plan.transitions.front().segments.size() > 16) {
            return fail(QStringLiteral("Collision envelope regressed to exact checks or excessive motion segments"));
        }
        auto narrowRequest = request;
        narrowRequest.diagnostics = nullptr;
        narrowRequest.cutterCollisionProxy = BRepPrimAPI_MakeCone(0.1, 0.2, 15.0).Shape();
        const auto narrowPlan = lcnc::cam_algo::TravelPathPlanner::plan(narrowRequest);
        if (!narrowPlan.isExecutable()
            || narrowPlan.transitions.front().maximumSurfaceOffsetMm >=
               plan.transitions.front().maximumSurfaceOffsetMm) {
            return fail(QStringLiteral("Changing simulated cone dimensions did not change the collision envelope"));
        }
        const auto& preview = plan.transitions.front().surfacePreviewPoints;
        if (preview.isEmpty() || std::abs(preview.front().z) > 1e-9
            || std::abs(preview.back().z) > 1e-9) {
            return fail(QStringLiteral("Collision solving modified the raw display curve"));
        }
    }

    {
        // A generic machining face uses one bounded Bezier reference curve.
        // Its interior stays on the normal side of the endpoint plane, while
        // the exact tool idle/cutting offsets remain absent from CAM geometry.
        auto request = baseRequest();
        request.workpiece = BRepPrimAPI_MakeBox(
            gp_Pnt(-5.0, -5.0, -1.0), 20.0, 20.0, 1.0).Shape();
        request.endpoints = {endpoint(1, 0.0, 0.0, 0.0),
                             endpoint(2, 10.0, 0.0, 0.0)};
        const auto plan = lcnc::cam_algo::TravelPathPlanner::plan(request);
        if (!plan.isExecutable() || plan.transitions.size() != 1)
            return fail(QStringLiteral("Generic surface reference curve was not planned"));
        const auto& transition = plan.transitions.front();
        if (transition.surfacePreviewPoints.size() < 5
            || transition.segments.size() < 4
            || transition.pathKind != lcnc::cam::RapidPathKind::SurfaceOffset) {
            return fail(QStringLiteral("Generic rapid curve has an invalid geometry contract"));
        }
        double maximumZ = 0.0;
        for (const auto& point : transition.surfacePreviewPoints)
            maximumZ = std::max(maximumZ, point.z);
        if (maximumZ <= request.minimumClearanceMm)
            return fail(QStringLiteral("Generic rapid curve was not raised above the machining face"));
        for (const auto& segment : transition.segments) {
            if (segment.synchronization != lcnc::cam::RapidSynchronization::Coordinated)
                return fail(QStringLiteral("Surface rapid contains a sequential axis segment"));
        }
    }

    {
        // Spherical machining uses the short great-circle reference rather
        // than following tessellation edges.
        auto request = baseRequest();
        request.workpiece = BRepPrimAPI_MakeSphere(
            gp_Pnt(0.0, 0.0, 0.0), 20.0).Shape();
        auto source = endpoint(1, 20.0, 0.0, 0.0);
        auto target = endpoint(2, 0.0, 20.0, 0.0);
        source.pose.surfaceNormalX = 1.0;
        source.pose.surfaceNormalZ = 0.0;
        target.pose.surfaceNormalY = 1.0;
        target.pose.surfaceNormalZ = 0.0;
        request.endpoints = {source, target};
        const auto plan = lcnc::cam_algo::TravelPathPlanner::plan(request);
        if (!plan.isExecutable())
            return fail(QStringLiteral("Spherical reference curve was not planned"));
        for (const auto& point : plan.transitions.front().surfacePreviewPoints) {
            const double radius = std::sqrt(
                point.x * point.x + point.y * point.y + point.z * point.z);
            if (std::abs(radius - 20.0) > 1e-3 || std::abs(point.z) > 1e-3)
                return fail(QStringLiteral("Spherical reference left the short great-circle arc"));
        }
    }

    {
        // Pre-solve rotary estimates are distributed over the curve.  CAM's
        // later authoritative IK solve replaces them with complete poses.
        auto request = baseRequest();
        auto source = endpoint(1, 0.0, 0.0, 0.0);
        auto target = endpoint(2, 20.0, 0.0, 0.0);
        source.pose.axes[3] = 70.0;
        target.pose.axes[3] = -20.0;
        request.endpoints = {source, target};
        const auto plan = lcnc::cam_algo::TravelPathPlanner::plan(request);
        if (!plan.isExecutable())
            return fail(QStringLiteral("Rotary-continuous rapid reference was not planned"));
        double previousA = source.pose.axes[3];
        for (const auto& segment : plan.transitions.front().segments) {
            if (std::abs(segment.target.axes[3] - previousA) >= 45.0)
                return fail(QStringLiteral("Rapid reference retained a single rotary jump"));
            previousA = segment.target.axes[3];
        }
    }

    {
        // Machine geometry changes the mode only.  Real head/workpiece/fixture
        // validation is performed after CAM has solved the complete path.
        auto request = baseRequest();
        request.fullEnvironment = true;
        request.endpoints = {endpoint(1, 0.0, 0.0, 0.0),
                             endpoint(2, 10.0, 0.0, 0.0)};
        const auto plan = lcnc::cam_algo::TravelPathPlanner::plan(request);
        if (!plan.isExecutable()
            || plan.mode != lcnc::cam::TravelPlanningMode::FullEnvironment) {
            return fail(QStringLiteral("Machine-model mode rejected nominal curve generation"));
        }
    }

    {
        auto request = baseRequest();
        request.motionProfile.supportedCoordinatedMask = 0x07;
        auto source = endpoint(1, 0.0, 0.0, 0.0);
        auto target = endpoint(2, 10.0, 0.0, 0.0);
        target.pose.axes[3] = 30.0;
        request.endpoints = {source, target};
        const auto plan = lcnc::cam_algo::TravelPathPlanner::plan(request);
        if (plan.isExecutable() || plan.failureReason.isEmpty())
            return fail(QStringLiteral("Unsupported coordinated rotary group was accepted"));
    }

    {
        // The workload is linear in contour count and strictly bounded per
        // curve; no mesh graph, multi-height candidates or recursive collision
        // subdivision are allowed back into this generator.
        auto request = baseRequest();
        request.workpiece = BRepPrimAPI_MakeSphere(
            gp_Pnt(0.0, 0.0, 0.0), 20.0).Shape();
        constexpr int kContourCount = 72;
        constexpr double kPi = 3.14159265358979323846;
        request.endpoints.reserve(kContourCount);
        for (int index = 0; index < kContourCount; ++index) {
            const double angle = 2.0 * kPi * index / kContourCount;
            auto value = endpoint(static_cast<std::uint64_t>(index + 1),
                                  20.0 * std::cos(angle),
                                  20.0 * std::sin(angle), 0.0);
            value.pose.surfaceNormalX = std::cos(angle);
            value.pose.surfaceNormalY = std::sin(angle);
            value.pose.surfaceNormalZ = 0.0;
            request.endpoints.append(value);
        }
        lcnc::cam_algo::TravelPlanningDiagnostics diagnostics;
        request.diagnostics = &diagnostics;
        QElapsedTimer timer;
        timer.start();
        const auto plan = lcnc::cam_algo::TravelPathPlanner::plan(request);
        if (!plan.isExecutable()
            || plan.transitions.size() != kContourCount - 1
            || diagnostics.evaluatedCandidateCount != kContourCount - 1
            || timer.elapsed() > 2000) {
            return fail(QStringLiteral("Many-contour surface planning is not bounded"));
        }
    }

    return 0;
}
