#pragma once

#include "modules/process/device/i_process_device.h"

#include <QString>
#include <QStringList>

namespace lcnc::process {

class ILaserDevice : public IProcessDevice
{
public:
    virtual ~ILaserDevice() = default;

    virtual QString name() const = 0;
    QString deviceId() const override { return name(); }
    QString displayName() const override { return name(); }
    ProcessDeviceKind kind() const override { return ProcessDeviceKind::Laser; }
    QStringList capabilities() const override
    {
        return { QStringLiteral("energy"), QStringLiteral("frequency"), QStringLiteral("pulse-width") };
    }
    ProcessDeviceConnectionState connectionState() const override
    {
        return isConnected() ? ProcessDeviceConnectionState::Connected : ProcessDeviceConnectionState::Disconnected;
    }
    QString lastError() const override { return QString(); }
    QList<ProcessDeviceStatusItem> statusItems() const override
    {
        return {
            { QStringLiteral("Energy"), QString::number(energy(), 'g', 12) },
            { QStringLiteral("Frequency"), QString::number(frequency(), 'g', 12) },
            { QStringLiteral("PulseWidth"), QString::number(pulseWidth(), 'g', 12) }
        };
    }

    virtual bool connectDevice(QString* errorMessage = nullptr) = 0;
    virtual void disconnectDevice() = 0;
    virtual bool isConnected() const = 0;

    virtual bool startLaser(QString* errorMessage = nullptr) = 0;
    virtual bool stopLaser(QString* errorMessage = nullptr) = 0;
    virtual bool startAiming(QString* errorMessage = nullptr) = 0;
    virtual bool stopAiming(QString* errorMessage = nullptr) = 0;

    virtual bool setEnergy(double value, QString* errorMessage = nullptr) = 0;
    virtual bool setFrequency(double value, QString* errorMessage = nullptr) = 0;
    virtual bool setPulseWidth(double value, QString* errorMessage = nullptr) = 0;

    virtual double energy() const = 0;
    virtual double frequency() const = 0;
    virtual double pulseWidth() const = 0;
};

} // namespace lcnc::process
