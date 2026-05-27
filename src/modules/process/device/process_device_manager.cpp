#include "modules/process/device/process_device_manager.h"

#include "core/logging/logger.h"
#include "modules/process/device/simulator_laser_device.h"
#include "modules/process/device/simulator_process_io.h"
#include "modules/process/settings/process_settings.h"

#include <algorithm>

namespace lcnc::process {

ProcessDeviceManager::ProcessDeviceManager()
    : m_activeMotionController(QStringLiteral("SimulatorCMHP"))
    , m_activeLaserDevice(QStringLiteral("Simulator"))
{
    registerBuiltInDevices();
    m_laserDevice = std::make_unique<SimulatorLaserDevice>();
    m_processIo = std::make_unique<SimulatorProcessIo>();
}

ProcessDeviceManager::~ProcessDeviceManager() = default;

QVector<ProcessDeviceDescriptor> ProcessDeviceManager::descriptors(ProcessDeviceKind kind) const
{
    QVector<ProcessDeviceDescriptor> result;
    for (const auto& descriptor : m_descriptors) {
        if (descriptor.kind == kind)
            result.append(descriptor);
    }
    return result;
}

QStringList ProcessDeviceManager::availableMotionControllers() const
{
    return names(ProcessDeviceKind::MotionController);
}

QStringList ProcessDeviceManager::availableLaserDevices() const
{
    return names(ProcessDeviceKind::Laser);
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

    if (!isDeviceUsable(ProcessDeviceKind::MotionController, m_activeMotionController)) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "process.device: motion controller '{}' is registered but unavailable",
                  m_activeMotionController.toStdString());
    }
    if (!isDeviceUsable(ProcessDeviceKind::Laser, m_activeLaserDevice)) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "process.device: laser device '{}' is registered but unavailable",
                  m_activeLaserDevice.toStdString());
    }

    if (auto* laser = laserDevice()) {
        laser->setEnergy(settings.laserEnergy());
        laser->setFrequency(settings.laserFrequency());
        laser->setPulseWidth(settings.laserPulseWidth());
    }
}

bool ProcessDeviceManager::isMotionControllerAvailable(const QString& name) const
{
    return descriptor(ProcessDeviceKind::MotionController, name) != nullptr;
}

bool ProcessDeviceManager::isLaserDeviceAvailable(const QString& name) const
{
    return descriptor(ProcessDeviceKind::Laser, name) != nullptr;
}

bool ProcessDeviceManager::isDeviceUsable(ProcessDeviceKind kind, const QString& name) const
{
    const auto* item = descriptor(kind, name);
    return item && item->availability != ProcessDeviceAvailability::Unavailable;
}

const ProcessDeviceDescriptor* ProcessDeviceManager::descriptor(ProcessDeviceKind kind, const QString& name) const
{
    const QString expected = normalizedName(name);
    for (const auto& item : m_descriptors) {
        if (item.kind == kind && normalizedName(item.name) == expected)
            return &item;
    }
    return nullptr;
}

void ProcessDeviceManager::registerBuiltInDevices()
{
    m_descriptors = {
        { ProcessDeviceKind::MotionController,
          QStringLiteral("SimulatorCMHP"),
          QStringLiteral("CMHP Simulator"),
          ProcessDeviceAvailability::Simulation,
          { QStringLiteral("axis"), QStringLiteral("jog"), QStringLiteral("home") } },
        { ProcessDeviceKind::MotionController,
          QStringLiteral("ACS"),
          QStringLiteral("ACS Motion Controller"),
          ProcessDeviceAvailability::Unavailable,
          { QStringLiteral("axis"), QStringLiteral("io"), QStringLiteral("laser-table") } },
        { ProcessDeviceKind::MotionController,
          QStringLiteral("GTN"),
          QStringLiteral("GTN Motion Controller"),
          ProcessDeviceAvailability::Unavailable,
          { QStringLiteral("axis"), QStringLiteral("io") } },
        { ProcessDeviceKind::Laser,
          QStringLiteral("Simulator"),
          QStringLiteral("Laser Simulator"),
          ProcessDeviceAvailability::Simulation,
          { QStringLiteral("energy"), QStringLiteral("frequency"), QStringLiteral("pulse-width") } },
        { ProcessDeviceKind::Laser,
          QStringLiteral("IPG"),
          QStringLiteral("IPG Laser"),
          ProcessDeviceAvailability::Unavailable,
          { QStringLiteral("energy"), QStringLiteral("monitor") } },
        { ProcessDeviceKind::Laser,
          QStringLiteral("Raycus"),
          QStringLiteral("Raycus Laser"),
          ProcessDeviceAvailability::Unavailable,
          { QStringLiteral("energy"), QStringLiteral("frequency"), QStringLiteral("pulse-width") } },
        { ProcessDeviceKind::Laser,
          QStringLiteral("RaycusQCW"),
          QStringLiteral("Raycus QCW Laser"),
          ProcessDeviceAvailability::Unavailable,
          { QStringLiteral("energy"), QStringLiteral("frequency"), QStringLiteral("pulse-width") } },
        { ProcessDeviceKind::Laser,
          QStringLiteral("Pharos"),
          QStringLiteral("Pharos Laser"),
          ProcessDeviceAvailability::Unavailable,
          { QStringLiteral("divider"), QStringLiteral("energy") } },
        { ProcessDeviceKind::Laser,
          QStringLiteral("Analog"),
          QStringLiteral("Analog Laser"),
          ProcessDeviceAvailability::Unavailable,
          { QStringLiteral("analog-output"), QStringLiteral("energy") } },
        { ProcessDeviceKind::Laser,
          QStringLiteral("ULTRON"),
          QStringLiteral("ULTRON Laser"),
          ProcessDeviceAvailability::Unavailable,
                    { QStringLiteral("energy"), QStringLiteral("frequency") } },
                { ProcessDeviceKind::Io,
                    QStringLiteral("SimulatorIO"),
                    QStringLiteral("Simulator IO"),
                    ProcessDeviceAvailability::Simulation,
                    { QStringLiteral("digital-in"), QStringLiteral("digital-out"), QStringLiteral("analog-in"), QStringLiteral("analog-out") } },
                { ProcessDeviceKind::Io,
                    QStringLiteral("BDAQ"),
                    QStringLiteral("BDAQ IO"),
                    ProcessDeviceAvailability::Unavailable,
                    { QStringLiteral("digital-in"), QStringLiteral("digital-out"), QStringLiteral("analog-in"), QStringLiteral("analog-out") } }
    };
}

QStringList ProcessDeviceManager::names(ProcessDeviceKind kind) const
{
    QStringList result;
    for (const auto& item : m_descriptors) {
        if (item.kind == kind)
            result.append(item.name);
    }
    return result;
}

QString ProcessDeviceManager::normalizedName(const QString& name) const
{
    return name.trimmed().toCaseFolded();
}

} // namespace lcnc::process
