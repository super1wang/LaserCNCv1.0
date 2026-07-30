#include "modules/process/runtime/process_run_coordinator.h"

#include <cassert>

using lcnc::ProcessRunState;
using lcnc::process::ProcessRunCoordinator;

int main()
{
    ProcessRunCoordinator coordinator;
    assert(coordinator.state() == ProcessRunState::Idle);

    assert(!coordinator.transitionTo(ProcessRunState::Paused));
    assert(coordinator.state() == ProcessRunState::Idle);
    assert(coordinator.transitionTo(ProcessRunState::Running));
    assert(coordinator.transitionTo(ProcessRunState::Paused));
    assert(coordinator.transitionTo(ProcessRunState::Running));
    assert(coordinator.transitionTo(ProcessRunState::Stopped));
    assert(coordinator.transitionTo(ProcessRunState::Idle));

    assert(coordinator.transitionTo(ProcessRunState::Error));
    assert(!coordinator.transitionTo(ProcessRunState::Running));
    assert(coordinator.transitionTo(ProcessRunState::Stopped));
    assert(coordinator.transitionTo(ProcessRunState::Running));

    assert(coordinator.transitionTo(ProcessRunState::EmergencyStop));
    assert(!coordinator.transitionTo(ProcessRunState::Error));
    assert(!coordinator.transitionTo(ProcessRunState::Stopped));
    assert(coordinator.transitionTo(ProcessRunState::Idle));

    for (const auto state : {ProcessRunState::Idle, ProcessRunState::Running,
                            ProcessRunState::Paused, ProcessRunState::Stopped,
                            ProcessRunState::Error, ProcessRunState::EmergencyStop}) {
        ProcessRunCoordinator sameState;
        if (state != ProcessRunState::Idle)
            assert(sameState.transitionTo(ProcessRunState::EmergencyStop));
        const auto current = sameState.state();
        assert(sameState.transitionTo(current));
    }

    return 0;
}
