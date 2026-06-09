#include "modules/process/device/process_device_coordinator.h"

#include "core/kinematics/i_motion_controller.h"
#include "core/logging/logger.h"
#include "modules/process/device/i_laser_device.h"
#include "modules/process/device/i_process_io.h"
#include "modules/process/device/process_device_manager.h"

namespace lcnc::process {

ProcessDeviceCoordinator::ProcessDeviceCoordinator(ProcessDeviceManager& deviceManager, QObject* parent)
    : QObject(parent)
    , m_deviceManager(deviceManager)
{
}

void ProcessDeviceCoordinator::setMotionController(lcnc::IMotionController* controller)
{
    m_motionController = controller;
    m_context.activeMotionController = controller ? controller->id() : QString();
}

bool ProcessDeviceCoordinator::moveAxisTo(const QString& axisName, double position, QString* errorMessage)
{
    if (axisName.trimmed().isEmpty()) {
        if (errorMessage)
            *errorMessage = tr("轴名不能为空");
        return false;
    }

    if (m_motionController && !m_motionController->moveTo(axisName, position)) {
        const QString message = tr("运动控制器拒绝移动轴 %1").arg(axisName);
        if (errorMessage)
            *errorMessage = message;
        emit deviceError(message);
        return false;
    }

    m_context.axisPositions.insert(axisName, position);
    emit axisPositionUpdated(axisName, position);
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "process.device: axis '{}' -> {}",
               axisName.toStdString(),
               position);
    return true;
}

bool ProcessDeviceCoordinator::setLaserEnergy(double value, QString* errorMessage)
{
    auto* laser = m_deviceManager.laserDevice();
    if (!laser) {
        if (errorMessage)
            *errorMessage = tr("未配置激光器");
        return false;
    }
    if (!laser->setEnergy(value, errorMessage)) {
        emit deviceError(errorMessage ? *errorMessage : tr("设置激光能量失败"));
        return false;
    }
    m_context.laserEnergy = value;
    emit laserEnergyUpdated(value);
    return true;
}

bool ProcessDeviceCoordinator::setLaserOn(bool on, QString* errorMessage)
{
    auto* laser = m_deviceManager.laserDevice();
    if (!laser) {
        if (errorMessage)
            *errorMessage = tr("未配置激光器");
        return false;
    }
    const bool ok = on ? laser->startLaser(errorMessage) : laser->stopLaser(errorMessage);
    if (!ok) {
        emit deviceError(errorMessage ? *errorMessage : tr("切换激光状态失败"));
        return false;
    }
    return true;
}

bool ProcessDeviceCoordinator::setDigitalOutput(const QString& channel, bool value, QString* errorMessage)
{
    if (m_motionController && m_motionController->setDigitalOutput(channel, value, errorMessage)) {
        m_context.digitalOutputs.insert(channel, value);
        emit digitalOutputUpdated(channel, value);
        return true;
    }

    auto* io = m_deviceManager.processIo();
    if (!io) {
        if (errorMessage)
            *errorMessage = tr("未配置 IO 设备");
        return false;
    }
    if (!io->setDigitalOutput(channel, value, errorMessage)) {
        emit deviceError(errorMessage ? *errorMessage : tr("设置数字输出失败"));
        return false;
    }
    m_context.digitalOutputs.insert(channel, value);
    emit digitalOutputUpdated(channel, value);
    return true;
}

void ProcessDeviceCoordinator::emergencyStop()
{
    if (m_motionController)
        m_motionController->emergencyStop();
    LCNC_WARN(lcnc::LogCode::Generic, "process.device: emergency stop requested");
}

} // namespace lcnc::process