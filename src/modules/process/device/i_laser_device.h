#pragma once

#include <QString>

namespace lcnc::process {

class ILaserDevice
{
public:
    virtual ~ILaserDevice() = default;

    virtual QString name() const = 0;
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
