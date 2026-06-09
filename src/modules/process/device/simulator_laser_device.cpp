#include "modules/process/device/simulator_laser_device.h"

#include "core/logging/logger.h"

namespace lcnc::process {

bool SimulatorLaserDevice::connectDevice(QString* errorMessage)
{
    Q_UNUSED(errorMessage);
    m_connected = true;
    m_state = ProcessDeviceConnectionState::Connected;
    m_lastError.clear();
    LCNC_INFO(lcnc::LogCode::Generic, "process.laser.simulator: connected");
    return true;
}

void SimulatorLaserDevice::disconnectDevice()
{
    m_laserOn = false;
    m_aimingOn = false;
    m_connected = false;
    m_state = ProcessDeviceConnectionState::Disconnected;
    LCNC_INFO(lcnc::LogCode::Generic, "process.laser.simulator: disconnected");
}

QList<ProcessDeviceStatusItem> SimulatorLaserDevice::statusItems() const
{
    return {
        { QStringLiteral("Connected"), m_connected ? QStringLiteral("true") : QStringLiteral("false") },
        { QStringLiteral("LaserOn"), m_laserOn ? QStringLiteral("true") : QStringLiteral("false") },
        { QStringLiteral("AimingOn"), m_aimingOn ? QStringLiteral("true") : QStringLiteral("false") },
        { QStringLiteral("Energy"), QString::number(m_energy, 'g', 12) },
        { QStringLiteral("Frequency"), QString::number(m_frequency, 'g', 12) },
        { QStringLiteral("PulseWidth"), QString::number(m_pulseWidth, 'g', 12) }
    };
}

bool SimulatorLaserDevice::startLaser(QString* errorMessage)
{
    if (!m_connected) {
        if (errorMessage)
            *errorMessage = QStringLiteral("laser simulator is disconnected");
        m_lastError = QStringLiteral("laser simulator is disconnected");
        m_state = ProcessDeviceConnectionState::Error;
        return false;
    }
    m_laserOn = true;
    LCNC_INFO(lcnc::LogCode::Generic, "process.laser.simulator: laser on");
    return true;
}

bool SimulatorLaserDevice::stopLaser(QString* errorMessage)
{
    Q_UNUSED(errorMessage);
    m_laserOn = false;
    LCNC_INFO(lcnc::LogCode::Generic, "process.laser.simulator: laser off");
    return true;
}

bool SimulatorLaserDevice::startAiming(QString* errorMessage)
{
    if (!m_connected) {
        if (errorMessage)
            *errorMessage = QStringLiteral("laser simulator is disconnected");
        m_lastError = QStringLiteral("laser simulator is disconnected");
        m_state = ProcessDeviceConnectionState::Error;
        return false;
    }
    m_aimingOn = true;
    return true;
}

bool SimulatorLaserDevice::stopAiming(QString* errorMessage)
{
    Q_UNUSED(errorMessage);
    m_aimingOn = false;
    return true;
}

bool SimulatorLaserDevice::setEnergy(double value, QString* errorMessage)
{
    Q_UNUSED(errorMessage);
    m_energy = value;
    return true;
}

bool SimulatorLaserDevice::setFrequency(double value, QString* errorMessage)
{
    Q_UNUSED(errorMessage);
    m_frequency = value;
    return true;
}

bool SimulatorLaserDevice::setPulseWidth(double value, QString* errorMessage)
{
    Q_UNUSED(errorMessage);
    m_pulseWidth = value;
    return true;
}

} // namespace lcnc::process
