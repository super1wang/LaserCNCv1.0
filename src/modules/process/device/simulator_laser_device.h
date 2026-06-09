#pragma once

#include "modules/process/device/i_laser_device.h"

namespace lcnc::process {

class SimulatorLaserDevice : public ILaserDevice
{
public:
    QString name() const override { return QStringLiteral("Simulator"); }
    bool connectDevice(QString* errorMessage = nullptr) override;
    void disconnectDevice() override;
    bool isConnected() const override { return m_connected; }
    ProcessDeviceConnectionState connectionState() const override { return m_state; }
    QString lastError() const override { return m_lastError; }
    QList<ProcessDeviceStatusItem> statusItems() const override;

    bool startLaser(QString* errorMessage = nullptr) override;
    bool stopLaser(QString* errorMessage = nullptr) override;
    bool startAiming(QString* errorMessage = nullptr) override;
    bool stopAiming(QString* errorMessage = nullptr) override;

    bool setEnergy(double value, QString* errorMessage = nullptr) override;
    bool setFrequency(double value, QString* errorMessage = nullptr) override;
    bool setPulseWidth(double value, QString* errorMessage = nullptr) override;

    double energy() const override { return m_energy; }
    double frequency() const override { return m_frequency; }
    double pulseWidth() const override { return m_pulseWidth; }

private:
    bool m_connected{true};
    ProcessDeviceConnectionState m_state{ProcessDeviceConnectionState::Connected};
    QString m_lastError;
    bool m_laserOn{false};
    bool m_aimingOn{false};
    double m_energy{0.0};
    double m_frequency{0.0};
    double m_pulseWidth{0.0};
};

} // namespace lcnc::process
