#include "modules/process/device/process_device_manager.h"

#include "modules/process/settings/process_settings.h"

namespace lcnc::process {

ProcessDeviceManager::ProcessDeviceManager()
    : m_activeMotionController(QStringLiteral("SimulatorCMHP"))
    , m_activeLaserDevice(QStringLiteral("Simulator"))
{
}

QStringList ProcessDeviceManager::availableMotionControllers() const
{
    return { QStringLiteral("SimulatorCMHP") };
}

QStringList ProcessDeviceManager::availableLaserDevices() const
{
    return { QStringLiteral("Simulator") };
}

void ProcessDeviceManager::syncFromSettings(const lcnc::ProcessSettings& settings)
{
    const QString motion = settings.motionControllerName().trimmed();
    m_activeMotionController = isMotionControllerAvailable(motion)
        ? motion
        : QStringLiteral("SimulatorCMHP");

    const QString laser = settings.laserDeviceName().trimmed();
    m_activeLaserDevice = isLaserDeviceAvailable(laser)
        ? laser
        : QStringLiteral("Simulator");
}

bool ProcessDeviceManager::isMotionControllerAvailable(const QString& name) const
{
    return availableMotionControllers().contains(name.trimmed(), Qt::CaseInsensitive);
}

bool ProcessDeviceManager::isLaserDeviceAvailable(const QString& name) const
{
    return availableLaserDevices().contains(name.trimmed(), Qt::CaseInsensitive);
}

} // namespace lcnc::process
