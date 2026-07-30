#pragma once

#include "modules/process/i_process_facade.h"

namespace lcnc::process {

class ProcessRunCoordinator
{
public:
    [[nodiscard]] ProcessRunState state() const noexcept { return m_state; }
    [[nodiscard]] bool transitionTo(ProcessRunState next) noexcept;
    [[nodiscard]] static bool canTransition(ProcessRunState current,
                                            ProcessRunState next) noexcept;

private:
    ProcessRunState m_state{ProcessRunState::Idle};
};

} // namespace lcnc::process
