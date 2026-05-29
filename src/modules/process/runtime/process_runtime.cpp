#include "modules/process/runtime/process_runtime.h"

#include "core/logging/logger.h"
#include "modules/process/device/process_device_manager.h"
#include "modules/process/i_process_facade.h"
#include "modules/process/settings/process_settings.h"

namespace lcnc::process {

ProcessRuntime::ProcessRuntime(lcnc::ProcessSettings& settings,
                               ProcessDeviceManager& deviceManager,
                               QObject* parent)
    : QObject(parent)
    , m_settings(settings)
    , m_deviceManager(deviceManager)
{
    connect(&m_stateMachine, &ProcessStateMachine::stateChanged,
            this, [this](ProcessStateMachine::State, const QString& message) {
                if (!message.isEmpty())
                    emit runtimeMessage(message);
            });
}

void ProcessRuntime::initialize()
{
    QString errorMessage;
    refreshSettings(&errorMessage);
    m_stateMachine.transitionTo(ProcessStateMachine::State::Idle,
                                errorMessage.isEmpty() ? tr("Process runtime ready") : errorMessage);
}

void ProcessRuntime::applyFacadeState(lcnc::ProcessRunState state, const QString& message)
{
    ProcessStateMachine::State next = ProcessStateMachine::State::Idle;
    switch (state) {
    case lcnc::ProcessRunState::Idle:
        next = ProcessStateMachine::State::Idle;
        break;
    case lcnc::ProcessRunState::Running:
        next = ProcessStateMachine::State::Processing;
        break;
    case lcnc::ProcessRunState::Paused:
        next = ProcessStateMachine::State::Paused;
        break;
    case lcnc::ProcessRunState::Error:
        next = ProcessStateMachine::State::Error;
        break;
    case lcnc::ProcessRunState::EmergencyStop:
        next = ProcessStateMachine::State::EmergencyStop;
        break;
    }
    m_stateMachine.transitionTo(next, message);
}

bool ProcessRuntime::refreshSettings(QString* errorMessage)
{
    if (!m_stateMachine.isSafeForConfigurationChange()
        && m_stateMachine.state() != ProcessStateMachine::State::Uninitialized) {
        if (errorMessage)
            *errorMessage = tr("当前状态禁止刷新关键参数");
        return false;
    }

    m_settingsSnapshot = ProcessSettingsSchema::snapshotFrom(m_settings);
    m_context.activeMotionController = m_deviceManager.activeMotionController();
    m_context.activeLaserDevice = m_deviceManager.activeLaserDevice();
    m_context.laserEnergy = m_settingsSnapshot.tool.laserEnergy;
    m_context.laserFrequency = m_settingsSnapshot.tool.laserFrequency;
    m_context.laserPulseWidth = m_settingsSnapshot.tool.laserPulseWidth;
    emit executionContextUpdated(m_context);
    LCNC_INFO(lcnc::LogCode::Generic,
              "process.runtime: settings refreshed schema={} motion='{}' laser='{}'",
              m_settingsSnapshot.schemaVersion,
              m_settingsSnapshot.device.motionControllerName.toStdString(),
              m_settingsSnapshot.tool.laserDeviceName.toStdString());
    return true;
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