#include "modules/process/runtime/process_cutting_safety.h"
#include "modules/process/runtime/motion_feedback_validation.h"
#include "modules/process/runtime/process_run_coordinator.h"
#include "core/algorithms/cam/collision_policy.h"
#include "modules/cam/safety/machine_safety_package_manager.h"

#include <cassert>
#include <limits>

using namespace lcnc::process;

int main()
{
    // Physical log regression: commanded +360 degrees, encoder -360.003;
    // this must never become the next Group's rebased start position.
    if (stationaryFeedbackMatches(360000.0, -360003.0, 1000.0)
        || stationaryFeedbackMatches(316334.841, -1036335.0, 1000.0)
        || stationaryFeedbackMatches(360000.0, 0.0, 1000.0)
        || stationaryFeedbackMatches(0.0, 0.0, 0.0)
        || stationaryFeedbackMatches(std::numeric_limits<double>::quiet_NaN(), 0.0, 1000.0)
        || !stationaryFeedbackMatches(360000.0, 360003.0, 1000.0)
        || !stationaryFeedbackMatches(-403534.6, -403540.0, 10000.0))
        return 1;
    // The failure snapshot must not be replaced by the matching positions
    // observed after a later stop/reset or by a second axis fault.
    StationaryFeedbackFault feedbackFault;
    feedbackFault.capture(QStringLiteral("X"), 3, -41734.43087768555,
                          -37656.0, 10000.0, false);
    const QString firstMessage = feedbackFault.message();
    feedbackFault.capture(QStringLiteral("X"), 3, -37656.0, -37656.0, 10000.0, false);
    feedbackFault.capture(QStringLiteral("C"), 7, 360000.0, -360003.0, 1000.0, true);
    if (!feedbackFault.captured || feedbackFault.physicalAxis != 3
        || feedbackFault.profilePulse != -41734.43087768555
        || feedbackFault.message() != firstMessage
        || !firstMessage.contains(QStringLiteral("-4.173443"))
        || !firstMessage.contains(QStringLiteral("-3.765600"))
        || !firstMessage.contains(QStringLiteral("0.407843"))
        || !firstMessage.contains(QStringLiteral("mm")))
        return 1;
    feedbackFault = {};
    if (feedbackFault.captured || !feedbackFault.message().isEmpty()) return 1;
    feedbackFault.capture(QStringLiteral("C"), 7, 360000.0, -360003.0, 1000.0, true);
    if (!feedbackFault.message().contains(QStringLiteral("deg"))
        || !feedbackFault.message().contains(QStringLiteral("-720.003000"))) return 1;
    ProcessRunCoordinator coordinator;
    assert(coordinator.state() == lcnc::ProcessRunState::Idle);
    assert(!coordinator.transitionTo(lcnc::ProcessRunState::Paused));
    assert(coordinator.transitionTo(lcnc::ProcessRunState::Running));
    assert(coordinator.transitionTo(lcnc::ProcessRunState::Paused));
    assert(coordinator.transitionTo(lcnc::ProcessRunState::Running));
    assert(coordinator.transitionTo(lcnc::ProcessRunState::Stopped));
    assert(coordinator.transitionTo(lcnc::ProcessRunState::Idle));
    assert(coordinator.transitionTo(lcnc::ProcessRunState::Error));
    assert(!coordinator.transitionTo(lcnc::ProcessRunState::Running));
    assert(coordinator.transitionTo(lcnc::ProcessRunState::Stopped));

    ContourBoundaryHealth health;
    auto result = evaluateContourBoundaryHealth(health);
    assert(!result.success);
    assert(result.error.contains(QStringLiteral("not connected")));

    health.connected = true;
    result = evaluateContourBoundaryHealth(health);
    assert(!result.success);
    assert(result.error.contains(QStringLiteral("Unable to read")));

    health.statusReadable = true;
    health.faultCode = 17;
    result = evaluateContourBoundaryHealth(health);
    assert(!result.success);
    assert(result.error.contains(QStringLiteral("17")));

    health.faultCode = 0;
    health.missingAxes = {QStringLiteral("A")};
    result = evaluateContourBoundaryHealth(health);
    assert(!result.success);
    assert(result.error.contains(QStringLiteral("A")));

    health.missingAxes.clear();
    health.disabledAxes = {QStringLiteral("C")};
    result = evaluateContourBoundaryHealth(health);
    assert(!result.success);
    assert(result.error.contains(QStringLiteral("C")));

    health.disabledAxes.clear();
    result = evaluateContourBoundaryHealth(health);
    assert(result.success);
    assert(result.completion == DeviceCommandCompletion::Succeeded);

    lcnc::cam::ToolpathExportSnapshot snapshot;
    snapshot.contours.resize(1);
    snapshot.collisionSafety.verificationMode =
        lcnc::cam::CollisionVerificationMode::Required;
    snapshot.motionPlan.collision.state = lcnc::cam::CollisionValidationState::Pending;
    snapshot.motionPlan.collision.complete = false;
    assert(camExecutionBlockReason(snapshot).contains(QStringLiteral("incomplete")));

    snapshot.motionPlan.collision.state = lcnc::cam::CollisionValidationState::Collision;
    snapshot.motionPlan.collision.complete = true;
    assert(!camExecutionBlockReason(snapshot).isEmpty());

    snapshot.motionPlan.collision.state = lcnc::cam::CollisionValidationState::Warning;
    snapshot.motionPlan.collision.blockWarning = false;
    assert(camExecutionBlockReason(snapshot).isEmpty());

    snapshot.motionPlan.collision.state = lcnc::cam::CollisionValidationState::Disabled;
    snapshot.motionPlan.collision.blockWarning = true;
    snapshot.collisionSafety.verificationMode =
        lcnc::cam::CollisionVerificationMode::Disabled;
    assert(camExecutionBlockReason(snapshot).isEmpty());
    snapshot.collisionSafety.verificationMode =
        lcnc::cam::CollisionVerificationMode::Optional;
    snapshot.motionPlan.collision.state =
        lcnc::cam::CollisionValidationState::Collision;
    snapshot.motionPlan.collision.complete = true;
    assert(camExecutionBlockReason(snapshot, true).isEmpty());
    snapshot.motionPlan.collision.state =
        lcnc::cam::CollisionValidationState::Safe;

    // A missing machine package blocks activation of collision detection, not
    // the explicitly selected non-collision machining workflow.
    assert(!lcnc::cam_algo::canActivateCollisionDetection(false, false, false));
    assert(!lcnc::cam_algo::canActivateCollisionDetection(true, false, false));
    assert(!lcnc::cam_algo::canActivateCollisionDetection(true, true, true));
    assert(lcnc::cam_algo::canActivateCollisionDetection(true, true, false));
    lcnc::cam::MachineSafetyPackageStatus packageStatus;
    packageStatus.state = lcnc::cam::MachineSafetyPackageState::ReadyCurrent;
    assert(packageStatus.executionEligible());
    packageStatus.buildInProgress = true;
    assert(!packageStatus.executionEligible());
    snapshot.collisionSafety.enabled = false;
    snapshot.collisionSafety.verificationMode =
        lcnc::cam::CollisionVerificationMode::Disabled;
    snapshot.collisionSafety.machinePackageRequired = true;
    snapshot.collisionSafety.machinePackageReady = false;
    assert(camExecutionBlockReason(snapshot, true).isEmpty());

    snapshot.collisionSafety.enabled = true;
    snapshot.collisionSafety.verificationMode =
        lcnc::cam::CollisionVerificationMode::Required;
    snapshot.collisionSafety.machinePackageRequired = true;
    snapshot.collisionSafety.machinePackageReady = false;
    assert(camExecutionBlockReason(snapshot).isEmpty());
    assert(!camExecutionBlockReason(snapshot, true).isEmpty());
    snapshot.collisionSafety.machinePackageReady = true;
    assert(camExecutionBlockReason(snapshot, true).isEmpty());
    snapshot.collisionSafety.packageBuildInProgress = true;
    assert(!camExecutionBlockReason(snapshot, true).isEmpty());
    snapshot.collisionSafety.packageBuildInProgress = false;
    snapshot.collisionSafety.jobOverlayRequired = true;
    snapshot.collisionSafety.jobOverlayReady = false;
    assert(!camExecutionBlockReason(snapshot, true).isEmpty());
    snapshot.collisionSafety.jobOverlayReady = true;
    assert(camExecutionBlockReason(snapshot, true).isEmpty());

    snapshot.motionPlan.nodes.resize(2);
    assert(camExecutionBlockReason(snapshot, true).contains(
        QStringLiteral("certificates")));
    lcnc::cam::CamMotionEdgeCertificate certificate;
    certificate.firstNode = 0;
    certificate.lastNode = 1;
    certificate.state = lcnc::cam::CamMotionCertificateState::BoundaryUnknown;
    certificate.reason = QStringLiteral("boundary unknown");
    snapshot.motionPlan.edgeCertificates.append(certificate);
    assert(camExecutionBlockReason(snapshot, true)
        == QStringLiteral("boundary unknown"));
    snapshot.motionPlan.edgeCertificates[0].state =
        lcnc::cam::CamMotionCertificateState::CertifiedSafe;
    assert(camExecutionBlockReason(snapshot, true).isEmpty());
    snapshot.motionPlan.edgeCertificates[0].state =
        lcnc::cam::CamMotionCertificateState::Disabled;
    assert(!camExecutionBlockReason(snapshot, true).isEmpty());
    snapshot.collisionSafety.verificationMode =
        lcnc::cam::CollisionVerificationMode::Optional;
    assert(camExecutionBlockReason(snapshot, true).isEmpty());

    // Collision diagnostics and path construction have separate ownership.
    // Optional diagnostics may be incomplete without poisoning a valid rapid
    // path, while Required remains fail-closed for the same diagnostic state.
    lcnc::cam::ToolpathExportSnapshot policySnapshot;
    policySnapshot.contours.resize(2);
    policySnapshot.travelPlan.stale = false;
    policySnapshot.travelPlan.collision.state =
        lcnc::cam::CollisionValidationState::Indeterminate;
    policySnapshot.travelPlan.collision.complete = true;
    policySnapshot.travelPlan.collision.failureReason =
        QStringLiteral("collision environment missing");
    policySnapshot.motionPlan.collision = policySnapshot.travelPlan.collision;
    policySnapshot.collisionSafety.verificationMode =
        lcnc::cam::CollisionVerificationMode::Optional;
    assert(policySnapshot.travelPlan.isPathReady());
    assert(camExecutionBlockReason(policySnapshot, true).isEmpty());

    policySnapshot.collisionSafety.verificationMode =
        lcnc::cam::CollisionVerificationMode::Required;
    assert(!camExecutionBlockReason(policySnapshot, true).isEmpty());

    policySnapshot.travelPlan.failureReason = QStringLiteral("real IK failure");
    policySnapshot.motionPlan.collision.state =
        lcnc::cam::CollisionValidationState::Disabled;
    policySnapshot.motionPlan.collision.complete = true;
    policySnapshot.collisionSafety.verificationMode =
        lcnc::cam::CollisionVerificationMode::Disabled;
    assert(!policySnapshot.travelPlan.isPathReady());
    assert(camExecutionBlockReason(policySnapshot, true)
        == QStringLiteral("real IK failure"));
    policySnapshot.collisionSafety.verificationMode =
        lcnc::cam::CollisionVerificationMode::Optional;
    assert(camExecutionBlockReason(policySnapshot, true)
        == QStringLiteral("real IK failure"));
    return 0;
}
