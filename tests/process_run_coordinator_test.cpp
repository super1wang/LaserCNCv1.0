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

    for (const auto state : {ProcessRunState::Idle, ProcessRunState::Running,
                            ProcessRunState::Paused, ProcessRunState::Stopped,
                            ProcessRunState::Error}) {
        ProcessRunCoordinator sameState;
        if (state == ProcessRunState::Running)
            assert(sameState.transitionTo(ProcessRunState::Running));
        else if (state == ProcessRunState::Paused) {
            assert(sameState.transitionTo(ProcessRunState::Running));
            assert(sameState.transitionTo(ProcessRunState::Paused));
        } else if (state == ProcessRunState::Stopped) {
            assert(sameState.transitionTo(ProcessRunState::Stopped));
        } else if (state == ProcessRunState::Error) {
            assert(sameState.transitionTo(ProcessRunState::Error));
        }
        const auto current = sameState.state();
        assert(sameState.transitionTo(current));
    }

    return 0;
}
