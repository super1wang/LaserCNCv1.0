#pragma once

#include "modules/process/runtime/process_execution_context.h"
#include "modules/process/runtime/process_state_machine.h"
#include "modules/process/settings/process_settings_schema.h"

#include <QObject>

namespace lcnc { enum class ProcessRunState; class ProcessSettings; }
namespace lcnc::process { class ProcessDeviceManager; }

namespace lcnc::process {

/**
 * @brief Internal coordinator for Process state, settings and runtime context.
 */
class ProcessRuntime : public QObject
{
    Q_OBJECT
public:
    explicit ProcessRuntime(lcnc::ProcessSettings& settings,
                            ProcessDeviceManager& deviceManager,
                            QObject* parent = nullptr);

    ProcessStateMachine& stateMachine() { return m_stateMachine; }
    const ProcessStateMachine& stateMachine() const { return m_stateMachine; }
    const ProcessExecutionContext& context() const { return m_context; }
    const ProcessTypedSettingsSnapshot& settingsSnapshot() const { return m_settingsSnapshot; }

    void initialize();
    void applyFacadeState(lcnc::ProcessRunState state, const QString& message);
    bool refreshSettings(QString* errorMessage = nullptr);
    bool canChangeConfiguration(QString* reason = nullptr) const;

signals:
    void runtimeMessage(const QString& message);
    void executionContextUpdated(const lcnc::process::ProcessExecutionContext& context);

private:
    lcnc::ProcessSettings& m_settings;
    ProcessDeviceManager& m_deviceManager;
    ProcessStateMachine m_stateMachine;
    ProcessExecutionContext m_context;
    ProcessTypedSettingsSnapshot m_settingsSnapshot;
};

} // namespace lcnc::process