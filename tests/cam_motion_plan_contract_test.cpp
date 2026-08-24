#include "core/algorithms/cam/collision_scan_policy.h"
#include "modules/cam/contracts/i_cam_initial_approach_planner.h"
#include "modules/cam/contracts/toolpath_export_dto.h"

#include <QCoreApplication>
#include <QTextStream>

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <cmath>
#include <limits>
#include <mutex>

namespace {

int fail(const QString& message)
{
    QTextStream(stderr) << message << '\n';
    return 1;
}

int verifyCollisionScanPolicy()
{
    const QSet<QString> sources = {
        QStringLiteral("axis:B"), QStringLiteral("axis:Z"),
        QStringLiteral("cutter"), QStringLiteral("axis:A")};
    const QStringList expected = {
        QStringLiteral("cutter"), QStringLiteral("axis:Z"),
        QStringLiteral("axis:A"), QStringLiteral("axis:B")};
    if (lcnc::cam_algo::orderedActiveCollisionSources(sources) != expected)
        return fail(QStringLiteral("Active collision sources are not staged deterministically"));

    using State = lcnc::cam::CollisionValidationState;
    if (lcnc::cam_algo::collisionStateSeverity(1)
            <= lcnc::cam_algo::collisionStateSeverity(3)
        || lcnc::cam_algo::collisionStateSeverity(3)
            <= lcnc::cam_algo::collisionStateSeverity(2)
        || lcnc::cam_algo::collisionStateSeverity(2)
            <= lcnc::cam_algo::collisionStateSeverity(0)
        || lcnc::cam_algo::classifyCollisionDistance(0.0, 0.5, 1e-7)
            != State::Collision
        || lcnc::cam_algo::classifyCollisionDistance(0.25, 0.5, 1e-7)
            != State::Warning
        || lcnc::cam_algo::classifyCollisionDistance(0.75, 0.5, 1e-7)
            != State::Safe
        || lcnc::cam_algo::classifyCollisionDistance(
               std::numeric_limits<double>::quiet_NaN(), 0.5, 1e-7)
            != State::Indeterminate) {
        return fail(QStringLiteral("Collision severity or distance classification is invalid"));
    }

    std::unique_lock<std::timed_mutex> owner(
        lcnc::cam_algo::collisionScanExecutionMutex());
    std::unique_lock<std::timed_mutex> cancelled(
        lcnc::cam_algo::collisionScanExecutionMutex(), std::defer_lock);
    if (lcnc::cam_algo::acquireCollisionScanExecution(cancelled, [] { return true; }))
        return fail(QStringLiteral("Cancelled scan acquired the global execution guard"));
    owner.unlock();

    std::unique_lock<std::timed_mutex> next(
        lcnc::cam_algo::collisionScanExecutionMutex(), std::defer_lock);
    if (!lcnc::cam_algo::acquireCollisionScanExecution(next, [] { return false; }))
        return fail(QStringLiteral("Released collision execution guard could not be acquired"));
    return 0;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    if (const int rc = verifyCollisionScanPolicy(); rc != 0)
        return rc;

    lcnc::cam::CollisionValidationSnapshot validation;
    validation.state = lcnc::cam::CollisionValidationState::Disabled;
    validation.complete = true;
    if (validation.blocksExecution())
        return fail(QStringLiteral("Disabled collision detection must not block execution"));

    validation.state = lcnc::cam::CollisionValidationState::Warning;
    if (!validation.blocksExecution(true) || validation.blocksExecution(false))
        return fail(QStringLiteral("Clearance warning policy is not configurable"));

    validation.state = lcnc::cam::CollisionValidationState::Collision;
    if (!validation.blocksExecution(false))
        return fail(QStringLiteral("Confirmed collision must always block execution"));
    if (lcnc::cam::collisionValidationStateForCertificate(
            lcnc::cam::CamMotionCertificateState::CertifiedSafe)
            != lcnc::cam::CollisionValidationState::Safe
        || lcnc::cam::collisionValidationStateForCertificate(
               lcnc::cam::CamMotionCertificateState::Blocked)
            != lcnc::cam::CollisionValidationState::Collision
        || lcnc::cam::collisionValidationStateForCertificate(
               lcnc::cam::CamMotionCertificateState::BoundaryUnknown)
            != lcnc::cam::CollisionValidationState::Indeterminate) {
        return fail(QStringLiteral(
            "Per-edge certificate state cannot be rendered independently"));
    }

    lcnc::cam::CamMotionPlanSnapshot plan;
    plan.revision = 42;
    lcnc::cam::CamMotionNode rapid;
    rapid.phase = lcnc::cam::CamMotionPhase::Rapid;
    rapid.rapidPhase = lcnc::cam::RapidSegmentPhase::Retract;
    rapid.contourId = 7;
    rapid.axisMask = 0x1f;
    plan.nodes.append(rapid);
    plan.collision = validation;
    if (plan.nodes.size() != 1 || plan.nodes.constFirst().contourId != 7
        || plan.nodes.constFirst().phase != lcnc::cam::CamMotionPhase::Rapid)
        return fail(QStringLiteral("Motion node contract did not preserve final CAM data"));
    if (plan.nodes.constFirst().rapidPhase != lcnc::cam::RapidSegmentPhase::Retract)
        return fail(QStringLiteral("Motion node contract lost the CAM rapid segment phase"));

    // Process may execute an initial approach only when CAM supplies exactly
    // one execution-eligible continuous certificate for every rapid segment.
    lcnc::cam::InitialApproachSnapshot initialApproach;
    initialApproach.transition.pathKind =
        lcnc::cam::RapidPathKind::InitialSafeZone;
    initialApproach.transition.segments.append(lcnc::cam::RapidMoveSegment{});
    initialApproach.collision.state = lcnc::cam::CollisionValidationState::Safe;
    initialApproach.collision.complete = true;
    if (initialApproach.isExecutable()) {
        return fail(QStringLiteral(
            "Initial approach without a continuous certificate was executable"));
    }
    lcnc::cam::CamMotionEdgeCertificate initialCertificate;
    initialCertificate.firstNode = 0;
    initialCertificate.lastNode = 1;
    initialCertificate.phase = lcnc::cam::CamMotionPhase::Rapid;
    initialCertificate.state =
        lcnc::cam::CamMotionCertificateState::CertifiedSafe;
    initialApproach.edgeCertificates.append(initialCertificate);
    if (!initialApproach.isExecutable()) {
        return fail(QStringLiteral(
            "Fully certified initial approach was not executable"));
    }
    initialApproach.edgeCertificates.front().state =
        lcnc::cam::CamMotionCertificateState::BoundaryUnknown;
    if (initialApproach.isExecutable()) {
        return fail(QStringLiteral(
            "Boundary-unknown initial approach was executable"));
    }

    lcnc::cam::CamMotionNode cutting;
    cutting.phase = lcnc::cam::CamMotionPhase::Cutting;
    cutting.tcpX = 10.0;
    cutting.tcpY = 0.0;
    cutting.tcpZ = 0.0;
    cutting.normalX = 1.0;
    cutting.normalY = 0.0;
    cutting.normalZ = 0.0;
    gp_Trsf rotation;
    rotation.SetRotation(gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
                         1.5707963267948966);
    gp_Trsf installation;
    installation.SetTranslation(gp_Vec(0.0, 0.0, 150.0));
    const gp_Trsf workpieceTransform = installation.Multiplied(rotation);
    lcnc::cam_algo::transformMotionNodeGeometry(&cutting, workpieceTransform);
    if (std::abs(cutting.tcpX) > 1e-9
        || std::abs(cutting.tcpY - 10.0) > 1e-9
        || std::abs(cutting.tcpZ - 150.0) > 1e-9
        || std::abs(cutting.normalX) > 1e-9
        || std::abs(cutting.normalY - 1.0) > 1e-9
        || std::abs(cutting.normalZ) > 1e-9) {
        return fail(QStringLiteral(
            "Workpiece installation was not applied to collision motion geometry"));
    }

    MachineAxisDef xAxis;
    xAxis.name = QStringLiteral("X");
    xAxis.currentPos = 417.0; // Simulated live controller feedback.
    MachineAxisDef aAxis;
    aAxis.name = QStringLiteral("A");
    aAxis.motionType = MachineAxisDef::Rotary;
    aAxis.currentPos = -37.0; // Must not leak into an offline CAM snapshot.
    lcnc::MachineModeDefinition definition;
    definition.interpolatedAxes.append(
        QStringLiteral("X"), lcnc::MachineAxisRole::LinearX);
    definition.lockedAxisTargets.insert(QStringLiteral("A"), 90.0);
    const QList<MachineAxisDef> baseline =
        lcnc::cam_algo::offlinePlanningAxisBaseline({xAxis, aAxis}, definition);
    if (baseline.size() != 2
        || std::abs(baseline.at(0).currentPos) > 1e-9
        || std::abs(baseline.at(1).currentPos - 90.0) > 1e-9) {
        return fail(QStringLiteral(
            "Offline CAM planning retained live controller axis positions"));
    }
    MachineKinematics offlineKinematics;
    offlineKinematics.setAxes(baseline);
    std::array<double, lcnc::MachineAxisLayout::kMaxAxes> plannedValues{};
    plannedValues[0] = 12.5;
    lcnc::cam_algo::applyOfflineMotionPose(
        &offlineKinematics, baseline, definition.interpolatedAxes,
        plannedValues, 0x01u);
    const MachineAxisDef* offlineX = offlineKinematics.findAxis(QStringLiteral("X"));
    const MachineAxisDef* offlineA = offlineKinematics.findAxis(QStringLiteral("A"));
    if (!offlineX || !offlineA
        || std::abs(offlineX->currentPos - 12.5) > 1e-9
        || std::abs(offlineA->currentPos - 90.0) > 1e-9) {
        return fail(QStringLiteral(
            "Offline motion pose did not use solved and locked axis values"));
    }

    // Asynchronous collision completion must advance the immutable execution
    // proof without replacing coordinates from the committed planned path.
    lcnc::cam::ToolpathExportSnapshot cached;
    cached.collisionSafety.enabled = true;
    cached.collisionSafety.jobOverlayRequired = true;
    cached.collisionSafety.jobOverlayBuildInProgress = true;
    cached.motionPlan.nodes.append(cutting);
    cached.motionPlan.nodes.front().tcpX = 123.0;
    lcnc::cam::ToolpathExportSnapshot verified;
    verified.collisionSafety.enabled = true;
    verified.collisionSafety.jobOverlayRequired = true;
    verified.collisionSafety.jobOverlayReady = true;
    verified.travelPlan.collision.state =
        lcnc::cam::CollisionValidationState::Safe;
    verified.travelPlan.collision.complete = true;
    lcnc::cam::CamMotionEdgeCertificate edgeCertificate;
    edgeCertificate.state =
        lcnc::cam::CamMotionCertificateState::CertifiedSafe;
    verified.motionPlan.collision = verified.travelPlan.collision;
    verified.motionPlan.edgeCertificates.append(edgeCertificate);
    cached.mergeCollisionProofFrom(verified);
    if (!cached.collisionSafety.jobOverlayReady
        || cached.collisionSafety.jobOverlayBuildInProgress
        || !cached.motionPlan.collision.complete
        || cached.motionPlan.edgeCertificates.size() != 1
        || cached.motionPlan.edgeCertificates.constFirst().state
            != lcnc::cam::CamMotionCertificateState::CertifiedSafe
        || cached.motionPlan.nodes.size() != 1
        || std::abs(cached.motionPlan.nodes.constFirst().tcpX - 123.0) > 1e-9) {
        return fail(QStringLiteral(
            "Collision proof refresh retained stale Job Overlay state or replaced committed coordinates"));
    }
    return 0;
}
