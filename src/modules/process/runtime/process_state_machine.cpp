#include "modules/process/runtime/process_state_machine.h"

#include "core/logging/logger.h"

namespace lcnc::process {

ProcessStateMachine::ProcessStateMachine(QObject* parent)
    : QObject(parent)
{
}

QString ProcessStateMachine::stateName() const
{
    return stateName(m_state);
}

QString ProcessStateMachine::stateName(State state)
{
    switch (state) {
    case State::Uninitialized: return QStringLiteral("Uninitialized");
    case State::Idle: return QStringLiteral("Idle");
    case State::Connecting: return QStringLiteral("Connecting");
    case State::Preparing: return QStringLiteral("Preparing");
    case State::Ready: return QStringLiteral("Ready");
    case State::Processing: return QStringLiteral("Processing");
    case State::Paused: return QStringLiteral("Paused");
    case State::Stopping: return QStringLiteral("Stopping");
    case State::Stopped: return QStringLiteral("Stopped");
    case State::Error: return QStringLiteral("Error");
    case State::EmergencyStop: return QStringLiteral("EmergencyStop");
    }
    return QStringLiteral("Unknown");
}

bool ProcessStateMachine::canTransitionTo(State next, QString* reason) const
{
    if (m_state == next)
        return true;
    if (next == State::EmergencyStop)
        return m_state != State::Uninitialized;

    bool allowed = false;
    switch (m_state) {
    case State::Uninitialized:
        allowed = next == State::Idle || next == State::Error;
        break;
    case State::Idle:
        allowed = next == State::Connecting || next == State::Preparing || next == State::Processing || next == State::Error;
        break;
    case State::Connecting:
        allowed = next == State::Idle || next == State::Error;
        break;
    case State::Preparing:
        allowed = next == State::Ready || next == State::Processing || next == State::Error || next == State::Stopping;
        break;
    case State::Ready:
        allowed = next == State::Processing || next == State::Idle || next == State::Error || next == State::Stopping;
        break;
    case State::Processing:
        allowed = next == State::Paused || next == State::Stopping || next == State::Error || next == State::Idle;
        break;
    case State::Paused:
        allowed = next == State::Processing || next == State::Stopping || next == State::Error;
        break;
    case State::Stopping:
        allowed = next == State::Stopped || next == State::Idle || next == State::Error;
        break;
    case State::Stopped:
        allowed = next == State::Idle || next == State::Preparing || next == State::Error;
        break;
    case State::Error:
        allowed = next == State::Idle || next == State::Preparing;
        break;
    case State::EmergencyStop:
        allowed = next == State::Idle || next == State::Error;
        break;
    }

    if (!allowed && reason) {
        *reason = QObject::tr("非法状态转换: %1 -> %2")
            .arg(stateName(m_state), stateName(next));
    }
    return allowed;
}

bool ProcessStateMachine::transitionTo(State next, const QString& message, QString* reason)
{
    QString localReason;
    if (!canTransitionTo(next, &localReason)) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "process.state: rejected {} -> {} reason='{}'",
                  stateName(m_state).toStdString(),
                  stateName(next).toStdString(),
                  localReason.toStdString());
        if (reason)
            *reason = localReason;
        emit transitionRejected(m_state, next, localReason);
        return false;
    }

    if (m_state == next)
        return true;

    LCNC_INFO(lcnc::LogCode::Generic,
              "process.state: {} -> {} message='{}'",
              stateName(m_state).toStdString(),
              stateName(next).toStdString(),
              message.toStdString());
    m_state = next;
    emit stateChanged(m_state, message);
    return true;
}

bool ProcessStateMachine::isSafeForConfigurationChange() const
{
    return m_state == State::Idle || m_state == State::Stopped || m_state == State::Error;
}

} // namespace lcnc::process