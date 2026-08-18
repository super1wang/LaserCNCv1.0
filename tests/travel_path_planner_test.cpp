#include "core/algorithms/cam/travel_path_planner.h"

#include <BRepAlgoAPI_Fuse.hxx>
#include <BRep_Tool.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QTextStream>

#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopLoc_Location.hxx>

#include <cmath>
#include <limits>

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

bool hasFaceTriangulation(const TopoDS_Shape& shape)
{
    for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More(); explorer.Next()) {
        TopLoc_Location location;
        if (!BRep_Tool::Triangulation(TopoDS::Face(explorer.Current()), location).IsNull())
            return true;
    }
    return false;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    {
        // Full-machine verification now runs in a cancellable CAM task after
        // rapid planning. A pending plan may be previewed, but must never be
        // considered executable by Process.
        lcnc::cam::TravelPlanSnapshot pending;
        pending.stale = false;
        pending.fullEnvironmentVerificationPending = true;
        if (pending.isExecutable())
            return fail(QStringLiteral("Pending full-machine verification was executable"));
        pending.fullEnvironmentVerificationPending = false;
        if (!pending.isExecutable())
            return fail(QStringLiteral("Completed empty rapid plan was not executable"));
    }

    {
        auto request = baseRequest();
        request.workpiece = {};
        request.endpoints = {endpoint(1, 0.0, 0.0, 0.0),
                             endpoint(2, 10.0, 0.0, 0.0)};
        const auto plan = lcnc::cam_algo::TravelPathPlanner::plan(request);
        if (!plan.isExecutable())
            return fail(QStringLiteral("Geometry-only rapid planning incorrectly required collision geometry"));
    }

    {
        // Invalid planner inputs must fail before OCC creates a collision mesh.
        // This keeps a malformed CAM export from surfacing as a later Qt heap
        // failure while the global-generation completion callback is running.
        // 中文翻译：无效规划输入必须在 OCC 建网格前失败，不能在全局生成完成回调中延迟表现为 Qt 堆错误。
        auto request = baseRequest();
        request.motionProfile.supportedCoordinatedMask = 0;
        request.endpoints = {endpoint(1, 0.0, 0.0, 0.0),
                             endpoint(2, 10.0, 0.0, 0.0)};
        const auto plan = lcnc::cam_algo::TravelPathPlanner::plan(request);
        if (plan.isExecutable() || plan.failureReason.isEmpty()
            || hasFaceTriangulation(request.workpiece)) {
            return fail(QStringLiteral("Invalid rapid-planning mask entered collision meshing"));
        }
    }

    {
        auto request = baseRequest();
        request.minimumClearanceMm = std::numeric_limits<double>::quiet_NaN();
        request.endpoints = {endpoint(1, 0.0, 0.0, 0.0),
                             endpoint(2, 10.0, 0.0, 0.0)};
        const auto plan = lcnc::cam_algo::TravelPathPlanner::plan(request);
        if (plan.isExecutable() || plan.failureReason.isEmpty()
            || hasFaceTriangulation(request.workpiece)) {
            return fail(QStringLiteral("Non-finite rapid-planning input entered collision meshing"));
        }
    }

    {
        // Collision-envelope sampling may triangulate a private planning copy,
        // but it must never attach that coarse mesh to the source workpiece
        // used by the live shaded view.
        auto request = baseRequest();
        if (hasFaceTriangulation(request.workpiece))
            return fail(QStringLiteral("Travel planner source fixture unexpectedly starts meshed"));
        request.endpoints = {endpoint(1, 0.0, 0.0, 0.0),
                             endpoint(2, 10.0, 0.0, 0.0)};
        const auto plan = lcnc::cam_algo::TravelPathPlanner::plan(request);
        if (!plan.isExecutable() || hasFaceTriangulation(request.workpiece))
            return fail(QStringLiteral("Rapid planning polluted the source workpiece display mesh"));
    }

    {
        // Rapid geometry uses the configured strict normal offset. Obstacles
        // are reported by the separate CAM collision validation stage and may
        // not silently raise or otherwise modify the path.
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
            || std::abs(plan.transitions.front().maximumSurfaceOffsetMm - 5.0) > 1e-9) {
            return fail(QStringLiteral("Rapid planner did not preserve the strict configured offset"));
        }
        if (diagnostics.exactDistanceCheckCount != 0
            || plan.transitions.front().segments.size() > 64) {
            return fail(QStringLiteral("Collision envelope regressed to exact checks or excessive motion segments"));
        }
        auto narrowRequest = request;
        narrowRequest.diagnostics = nullptr;
        narrowRequest.cutterCollisionProxy = BRepPrimAPI_MakeCone(0.1, 0.2, 15.0).Shape();
        const auto narrowPlan = lcnc::cam_algo::TravelPathPlanner::plan(narrowRequest);
        if (!narrowPlan.isExecutable()
            || std::abs(narrowPlan.transitions.front().maximumSurfaceOffsetMm
                        - plan.transitions.front().maximumSurfaceOffsetMm) > 1e-9) {
            return fail(QStringLiteral("Collision proxy dimensions modified strict rapid geometry"));
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
        if (transition.surfacePreviewPoints.size() != transition.segments.size() + 1)
            return fail(QStringLiteral("Executable rapid preview and solved segment count diverged"));
        for (int index = 0; index < transition.segments.size(); ++index) {
            const auto& preview = transition.surfacePreviewPoints.at(index + 1);
            const auto& target = transition.segments.at(index).target;
            if (std::abs(preview.x - target.tcpX) > 1e-9
                || std::abs(preview.y - target.tcpY) > 1e-9
                || std::abs(preview.z - target.tcpZ) > 1e-9) {
                return fail(QStringLiteral("Displayed rapid path differs from executable CAM samples"));
            }
        }
    }

    {
        auto base = baseRequest();
        const auto source = endpoint(1, 0.0, 0.0, 0.0);
        const auto target = endpoint(2, 10.0, 0.0, 0.0);
        base.transitions = {{source, target, 2.0, 1.0, 1.0}};
        auto offset = base;
        offset.transitions = {{source, target, 5.0, 1.0, 1.0}};
        const auto basePlan = lcnc::cam_algo::TravelPathPlanner::plan(base);
        const auto offsetPlan = lcnc::cam_algo::TravelPathPlanner::plan(offset);
        if (!basePlan.isExecutable() || !offsetPlan.isExecutable())
            return fail(QStringLiteral("Tool-offset rapid path could not be planned"));
        const auto& baseTransition = basePlan.transitions.front();
        const auto& offsetTransition = offsetPlan.transitions.front();
        if (baseTransition.segments.size() != offsetTransition.segments.size())
            return fail(QStringLiteral("Tool offset changed rapid topology unexpectedly"));
        if (baseTransition.segments.front().phase != lcnc::cam::RapidSegmentPhase::Retract
            || baseTransition.segments.back().phase != lcnc::cam::RapidSegmentPhase::Approach)
            return fail(QStringLiteral("Rapid phases were not split into retract/traverse/approach"));
        if (std::abs(offsetTransition.segments.front().target.tcpZ
                     - baseTransition.segments.front().target.tcpZ - 3.0) > 1e-9
            || std::abs(offsetTransition.segments.back().target.tcpZ
                        - baseTransition.segments.back().target.tcpZ) > 1e-9) {
            return fail(QStringLiteral("Strict rapid offset or cutting-offset approach is incorrect"));
        }
    }

    {
        // Offsets are geometric normal offsets, never controller-Z additions.
        auto request = baseRequest();
        auto source = endpoint(1, 1.0, 0.0, 0.0);
        auto target = endpoint(2, 10.0, 1.0, 0.0);
        source.pose.surfaceNormalX = 1.0;
        source.pose.surfaceNormalZ = 0.0;
        target.pose.surfaceNormalY = 1.0;
        target.pose.surfaceNormalZ = 0.0;
        request.transitions = {{source, target, 5.0, 1.0, 1.0}};
        const auto plan = lcnc::cam_algo::TravelPathPlanner::plan(request);
        if (!plan.isExecutable() || plan.transitions.front().segments.size() < 3)
            return fail(QStringLiteral("Normal-offset rapid path was not planned"));
        const auto& retract = plan.transitions.front().segments.front();
        const auto& approach = plan.transitions.front().segments.back();
        if (retract.phase != lcnc::cam::RapidSegmentPhase::Retract
            || std::abs(retract.target.tcpX - 5.0) > 1e-9
            || std::abs(retract.target.tcpY) > 1e-9
            || std::abs(retract.target.tcpZ) > 1e-9
            || approach.phase != lcnc::cam::RapidSegmentPhase::Approach
            || std::abs(approach.target.tcpX - 10.0) > 1e-9
            || std::abs(approach.target.tcpY - 1.0) > 1e-9
            || std::abs(approach.target.tcpZ) > 1e-9) {
            return fail(QStringLiteral("Rapid offsets were applied to physical Z instead of local normals"));
        }
    }

    {
        // Spherical machining uses the short great-circle reference rather
        // than following tessellation edges.
        auto request = baseRequest();
        request.workpiece = BRepPrimAPI_MakeSphere(
            gp_Pnt(0.0, 0.0, 0.0), 20.0).Shape();
        auto source = endpoint(1, 21.0, 0.0, 0.0);
        auto target = endpoint(2, 0.0, 21.0, 0.0);
        source.pose.surfaceNormalX = 1.0;
        source.pose.surfaceNormalZ = 0.0;
        target.pose.surfaceNormalY = 1.0;
        target.pose.surfaceNormalZ = 0.0;
        request.endpoints = {source, target};
        const auto plan = lcnc::cam_algo::TravelPathPlanner::plan(request);
        if (!plan.isExecutable())
            return fail(QStringLiteral("Spherical reference curve was not planned"));
        const auto& preview = plan.transitions.front().surfacePreviewPoints;
        double minimumRadius = std::numeric_limits<double>::infinity();
        double maximumAbsZ = 0.0;
        for (const auto& point : preview) {
            const double radius = std::sqrt(
                point.x * point.x + point.y * point.y + point.z * point.z);
            minimumRadius = std::min(minimumRadius, radius);
            maximumAbsZ = std::max(maximumAbsZ, std::abs(point.z));
        }
        if (minimumRadius < 20.0 - 1e-3 || maximumAbsZ > 1e-3)
            return fail(QStringLiteral("Spherical reference left the short great-circle arc (r=%1, z=%2)")
                .arg(minimumRadius, 0, 'g', 12).arg(maximumAbsZ, 0, 'g', 12));
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
