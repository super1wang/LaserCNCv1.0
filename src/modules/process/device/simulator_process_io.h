#pragma once

#include "modules/process/device/i_process_io.h"

#include <QHash>

namespace lcnc::process {

class SimulatorProcessIo : public IProcessIo
{
public:
    QString name() const override { return QStringLiteral("SimulatorIO"); }
    bool setDigitalOutput(const QString& channel, bool value, QString* errorMessage = nullptr) override;
    bool digitalInput(const QString& channel, bool* value, QString* errorMessage = nullptr) const override;
    bool setAnalogOutput(const QString& channel, double value, QString* errorMessage = nullptr) override;
    bool analogInput(const QString& channel, double* value, QString* errorMessage = nullptr) const override;

private:
    QHash<QString, bool> m_digital;
    QHash<QString, double> m_analog;
};

} // namespace lcnc::process
