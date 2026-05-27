#include "modules/process/device/simulator_laser_device.h"

#include "core/logging/logger.h"

namespace lcnc::process {

bool SimulatorLaserDevice::connectDevice(QString* errorMessage)
{
    Q_UNUSED(errorMessage);
    m_connected = true;
    LCNC_INFO(lcnc::LogCode::Generic, "process.laser.simulator: connected");
    return true;
}

void SimulatorLaserDevice::disconnectDevice()
{
    m_laserOn = false;
    m_aimingOn = false;
    m_connected = false;
    LCNC_INFO(lcnc::LogCode::Generic, "process.laser.simulator: disconnected");
}

bool SimulatorLaserDevice::startLaser(QString* errorMessage)
{
    if (!m_connected) {
        if (errorMessage)
            *errorMessage = QStringLiteral("laser simulator is disconnected");
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
