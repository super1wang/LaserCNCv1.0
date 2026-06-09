#pragma once

#include "modules/process/device/i_process_io.h"

#include <QHash>
#include <QMutex>

namespace lcnc::process {

class SimulatorProcessIo : public IProcessIo
{
public:
    QString name() const override { return QStringLiteral("SimulatorIO"); }
    bool connectDevice(QString* errorMessage = nullptr) override;
    void disconnectDevice() override;
    bool isConnected() const override;
    ProcessDeviceConnectionState connectionState() const override;
    QString lastError() const override;
    QList<ProcessDeviceStatusItem> statusItems() const override;

    bool setDigitalOutput(const QString& channel, bool value, QString* errorMessage = nullptr) override;
    bool digitalInput(const QString& channel, bool* value, QString* errorMessage = nullptr) const override;
    bool setAnalogOutput(const QString& channel, double value, QString* errorMessage = nullptr) override;
    bool analogInput(const QString& channel, double* value, QString* errorMessage = nullptr) const override;

private:
    QHash<QString, bool> m_digital;
    QHash<QString, double> m_analog;
    mutable QMutex m_mutex;
    bool m_connected{false};
    ProcessDeviceConnectionState m_state{ProcessDeviceConnectionState::Disconnected};
    QString m_lastError;
};

} // namespace lcnc::process
