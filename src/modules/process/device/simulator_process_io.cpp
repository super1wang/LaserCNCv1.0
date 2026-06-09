#include "modules/process/device/simulator_process_io.h"

#include "core/logging/logger.h"

#include <QMutexLocker>

namespace lcnc::process {

bool SimulatorProcessIo::connectDevice(QString* errorMessage)
{
    Q_UNUSED(errorMessage);
    QMutexLocker locker(&m_mutex);
    m_connected = true;
    m_state = ProcessDeviceConnectionState::Connected;
    m_lastError.clear();
    LCNC_INFO(lcnc::LogCode::Generic, "process.io.simulator: connected");
    return true;
}

void SimulatorProcessIo::disconnectDevice()
{
    QMutexLocker locker(&m_mutex);
    m_connected = false;
    m_state = ProcessDeviceConnectionState::Disconnected;
    LCNC_INFO(lcnc::LogCode::Generic, "process.io.simulator: disconnected");
}

bool SimulatorProcessIo::isConnected() const
{
    QMutexLocker locker(&m_mutex);
    return m_connected;
}

ProcessDeviceConnectionState SimulatorProcessIo::connectionState() const
{
    QMutexLocker locker(&m_mutex);
    return m_state;
}

QString SimulatorProcessIo::lastError() const
{
    QMutexLocker locker(&m_mutex);
    return m_lastError;
}

QList<ProcessDeviceStatusItem> SimulatorProcessIo::statusItems() const
{
    QMutexLocker locker(&m_mutex);
    return {
        { QStringLiteral("Connected"), m_connected ? QStringLiteral("true") : QStringLiteral("false") },
        { QStringLiteral("DigitalChannels"), QString::number(m_digital.size()) },
        { QStringLiteral("AnalogChannels"), QString::number(m_analog.size()) }
    };
}

bool SimulatorProcessIo::setDigitalOutput(const QString& channel, bool value, QString* errorMessage)
{
    QMutexLocker locker(&m_mutex);
    if (!m_connected) {
        if (errorMessage)
            *errorMessage = QStringLiteral("IO simulator is disconnected");
        m_lastError = QStringLiteral("IO simulator is disconnected");
        m_state = ProcessDeviceConnectionState::Error;
        return false;
    }
    m_digital.insert(channel.trimmed(), value);
    return true;
}

bool SimulatorProcessIo::digitalInput(const QString& channel, bool* value, QString* errorMessage) const
{
    QMutexLocker locker(&m_mutex);
    if (!value) {
        if (errorMessage)
            *errorMessage = QStringLiteral("digital input value pointer is null");
        return false;
    }
    if (!m_connected) {
        if (errorMessage)
            *errorMessage = QStringLiteral("IO simulator is disconnected");
        return false;
    }
    *value = m_digital.value(channel.trimmed(), false);
    return true;
}

bool SimulatorProcessIo::setAnalogOutput(const QString& channel, double value, QString* errorMessage)
{
    QMutexLocker locker(&m_mutex);
    if (!m_connected) {
        if (errorMessage)
            *errorMessage = QStringLiteral("IO simulator is disconnected");
        m_lastError = QStringLiteral("IO simulator is disconnected");
        m_state = ProcessDeviceConnectionState::Error;
        return false;
    }
    m_analog.insert(channel.trimmed(), value);
    return true;
}

bool SimulatorProcessIo::analogInput(const QString& channel, double* value, QString* errorMessage) const
{
    QMutexLocker locker(&m_mutex);
    if (!value) {
        if (errorMessage)
            *errorMessage = QStringLiteral("analog input value pointer is null");
        return false;
    }
    if (!m_connected) {
        if (errorMessage)
            *errorMessage = QStringLiteral("IO simulator is disconnected");
        return false;
    }
    *value = m_analog.value(channel.trimmed(), 0.0);
    return true;
}

} // namespace lcnc::process
