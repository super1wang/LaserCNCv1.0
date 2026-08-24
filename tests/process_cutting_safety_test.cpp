#include "modules/process/runtime/process_cutting_safety.h"
#include "modules/process/runtime/process_run_coordinator.h"
#include "core/algorithms/cam/collision_policy.h"
#include "modules/cam/safety/machine_safety_package_manager.h"

#include <cassert>

using namespace lcnc::process;

int main()
{
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
    assert(camExecutionBlockReason(snapshot).isEmpty());

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
    snapshot.collisionSafety.machinePackageRequired = true;
    snapshot.collisionSafety.machinePackageReady = false;
    assert(camExecutionBlockReason(snapshot, true).isEmpty());

    snapshot.collisionSafety.enabled = true;
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
    return 0;
}
