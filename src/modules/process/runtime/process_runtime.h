#pragma once

#include "modules/process/runtime/process_execution_context.h"
#include "modules/process/runtime/process_state_machine.h"

#include <QObject>

namespace lcnc { enum class ProcessRunState; }

namespace lcnc::process {

/**
 * @brief Internal coordinator for Process state and runtime context.
 */
class ProcessRuntime : public QObject
{
    Q_OBJECT
public:
    explicit ProcessRuntime(QObject* parent = nullptr);

    ProcessStateMachine& stateMachine() { return m_stateMachine; }
    const ProcessStateMachine& stateMachine() const { return m_stateMachine; }
    const ProcessExecutionContext& context() const { return m_context; }

    void initialize();
    void applyFacadeState(lcnc::ProcessRunState state, const QString& message);
    bool canChangeConfiguration(QString* reason = nullptr) const;

signals:
    void runtimeMessage(const QString& message);

private:
    ProcessStateMachine m_stateMachine;
    ProcessExecutionContext m_context;
};

} // namespace lcnc::process
