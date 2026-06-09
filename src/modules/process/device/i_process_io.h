#pragma once

#include "modules/process/device/i_process_device.h"

#include <QString>
#include <QStringList>

namespace lcnc::process {

class IProcessIo : public IProcessDevice
{
public:
    virtual ~IProcessIo() = default;

    virtual QString name() const = 0;
    QString deviceId() const override { return name(); }
    QString displayName() const override { return name(); }
    ProcessDeviceKind kind() const override { return ProcessDeviceKind::Io; }
    QStringList capabilities() const override
    {
        return {
            QStringLiteral("digital-in"),
            QStringLiteral("digital-out"),
            QStringLiteral("analog-in"),
            QStringLiteral("analog-out")
        };
    }
    bool connectDevice(QString* errorMessage = nullptr) override
    {
        Q_UNUSED(errorMessage);
        return true;
    }
    void disconnectDevice() override {}
    bool isConnected() const override { return true; }
    ProcessDeviceConnectionState connectionState() const override
    {
        return ProcessDeviceConnectionState::Connected;
    }
    QString lastError() const override { return QString(); }
    QList<ProcessDeviceStatusItem> statusItems() const override { return {}; }

    virtual bool setDigitalOutput(const QString& channel, bool value, QString* errorMessage = nullptr) = 0;
    virtual bool digitalInput(const QString& channel, bool* value, QString* errorMessage = nullptr) const = 0;
    virtual bool setAnalogOutput(const QString& channel, double value, QString* errorMessage = nullptr) = 0;
    virtual bool analogInput(const QString& channel, double* value, QString* errorMessage = nullptr) const = 0;
};

} // namespace lcnc::process
