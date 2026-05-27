#include "modules/process/device/simulator_process_io.h"

namespace lcnc::process {

bool SimulatorProcessIo::setDigitalOutput(const QString& channel, bool value, QString* errorMessage)
{
    Q_UNUSED(errorMessage);
    m_digital.insert(channel.trimmed(), value);
    return true;
}

bool SimulatorProcessIo::digitalInput(const QString& channel, bool* value, QString* errorMessage) const
{
    if (!value) {
        if (errorMessage)
            *errorMessage = QStringLiteral("digital input value pointer is null");
        return false;
    }
    *value = m_digital.value(channel.trimmed(), false);
    return true;
}

bool SimulatorProcessIo::setAnalogOutput(const QString& channel, double value, QString* errorMessage)
{
    Q_UNUSED(errorMessage);
    m_analog.insert(channel.trimmed(), value);
    return true;
}

bool SimulatorProcessIo::analogInput(const QString& channel, double* value, QString* errorMessage) const
{
    if (!value) {
        if (errorMessage)
            *errorMessage = QStringLiteral("analog input value pointer is null");
        return false;
    }
    *value = m_analog.value(channel.trimmed(), 0.0);
    return true;
}

} // namespace lcnc::process
