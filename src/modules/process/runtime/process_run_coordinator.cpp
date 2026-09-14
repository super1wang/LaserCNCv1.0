#include "modules/process/runtime/process_run_coordinator.h"

namespace lcnc::process {

bool ProcessRunCoordinator::transitionTo(ProcessRunState next) noexcept
{
    if (!canTransition(m_state, next))
        return false;

    m_state = next;
    return true;
}

bool ProcessRunCoordinator::canTransition(ProcessRunState current,
                                          ProcessRunState next) noexcept
{
    if (current == next)
        return true;
    if (next == ProcessRunState::Error)
        return true;

    switch (current) {
    case ProcessRunState::Idle:
        return next == ProcessRunState::Running || next == ProcessRunState::Stopped;
    case ProcessRunState::Running:
        return next == ProcessRunState::Idle || next == ProcessRunState::Paused
            || next == ProcessRunState::Stopped;
    case ProcessRunState::Paused:
        return next == ProcessRunState::Idle || next == ProcessRunState::Running
            || next == ProcessRunState::Stopped;
    case ProcessRunState::Stopped:
        return next == ProcessRunState::Idle || next == ProcessRunState::Running;
    case ProcessRunState::Error:
        return next == ProcessRunState::Idle || next == ProcessRunState::Stopped;
    }
    return false;
}

} // namespace lcnc::process
