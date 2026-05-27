#pragma once

#include <QString>

namespace lcnc::process {

class IProcessIo
{
public:
    virtual ~IProcessIo() = default;

    virtual QString name() const = 0;
    virtual bool setDigitalOutput(const QString& channel, bool value, QString* errorMessage = nullptr) = 0;
    virtual bool digitalInput(const QString& channel, bool* value, QString* errorMessage = nullptr) const = 0;
    virtual bool setAnalogOutput(const QString& channel, double value, QString* errorMessage = nullptr) = 0;
    virtual bool analogInput(const QString& channel, double* value, QString* errorMessage = nullptr) const = 0;
};

} // namespace lcnc::process
