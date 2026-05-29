#pragma once

#include <QObject>
#include <QString>

namespace lcnc::process {

/**
 * @brief Central Process runtime state machine with legal transition checks.
 */
class ProcessStateMachine : public QObject
{
    Q_OBJECT
public:
    enum class State
    {
        Uninitialized,
        Idle,
        Connecting,
        Preparing,
        Ready,
        Processing,
        Paused,
        Stopping,
        Stopped,
        Error,
        EmergencyStop
    };

    explicit ProcessStateMachine(QObject* parent = nullptr);

    State state() const { return m_state; }
    QString stateName() const;
    static QString stateName(State state);

    bool canTransitionTo(State next, QString* reason = nullptr) const;
    bool transitionTo(State next, const QString& message = QString(), QString* reason = nullptr);
    bool isSafeForConfigurationChange() const;

signals:
    void stateChanged(lcnc::process::ProcessStateMachine::State state, const QString& message);
    void transitionRejected(lcnc::process::ProcessStateMachine::State current,
                            lcnc::process::ProcessStateMachine::State requested,
                            const QString& reason);

private:
    State m_state{State::Uninitialized};
};

} // namespace lcnc::process