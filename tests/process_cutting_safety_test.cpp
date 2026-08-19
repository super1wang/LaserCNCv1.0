#include "modules/process/runtime/process_cutting_safety.h"
#include "modules/process/runtime/process_run_coordinator.h"

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
    return 0;
}
