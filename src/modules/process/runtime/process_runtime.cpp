#include "modules/process/runtime/process_runtime.h"
#include "core/logging/logger.h"
#include "modules/process/i_process_facade.h"

namespace lcnc::process {

ProcessRuntime::ProcessRuntime(QObject* parent)
    : QObject(parent)
{
    connect(&m_stateMachine, &ProcessStateMachine::stateChanged,
            this, [this](ProcessStateMachine::State, const QString& message) {
                if (!message.isEmpty())
                    emit runtimeMessage(message);
            });
}

void ProcessRuntime::initialize()
{
    m_stateMachine.transitionTo(ProcessStateMachine::State::Idle, tr("Process runtime ready"));
}

void ProcessRuntime::applyFacadeState(lcnc::ProcessRunState state, const QString& message)
{
    ProcessStateMachine::State next = ProcessStateMachine::State::Idle;
    switch (state) {
    case lcnc::ProcessRunState::Idle:        next = ProcessStateMachine::State::Idle; break;
    case lcnc::ProcessRunState::Running:     next = ProcessStateMachine::State::Processing; break;
    case lcnc::ProcessRunState::Paused:      next = ProcessStateMachine::State::Paused; break;
    case lcnc::ProcessRunState::Error:       next = ProcessStateMachine::State::Error; break;
    case lcnc::ProcessRunState::EmergencyStop: next = ProcessStateMachine::State::EmergencyStop; break;
    }
    m_stateMachine.transitionTo(next, message);
}

bool ProcessRuntime::canChangeConfiguration(QString* reason) const
{
    if (m_stateMachine.isSafeForConfigurationChange())
        return true;
    if (reason)
        *reason = tr("当前运行状态禁止修改控制器或关键加工参数");
    return false;
}

} // namespace lcnc::process
