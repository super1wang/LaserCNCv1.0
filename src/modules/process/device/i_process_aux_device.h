#pragma once

#include "modules/process/device/i_process_device.h"

#include <QString>
#include <QStringList>

namespace lcnc::process {

class IProcessAuxDevice : public IProcessDevice
{
public:
    virtual ~IProcessAuxDevice() = default;

    virtual QString name() const = 0;
    QString deviceId() const override { return name(); }
    QString displayName() const override { return name(); }
    ProcessDeviceKind kind() const override { return ProcessDeviceKind::Aux; }
    QStringList capabilities() const override { return {}; }
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
    QList<ProcessDeviceStatusItem> statusItems() const override
    {
        return { { QStringLiteral("Enabled"), isEnabled() ? QStringLiteral("true") : QStringLiteral("false") } };
    }

    virtual bool setEnabled(bool enabled, QString* errorMessage = nullptr) = 0;
    virtual bool isEnabled() const = 0;
    virtual QString statusText() const = 0;
};

} // namespace lcnc::process
