#pragma once

#include <QString>

namespace lcnc::process {

class IProcessAuxDevice
{
public:
    virtual ~IProcessAuxDevice() = default;

    virtual QString name() const = 0;
    virtual bool setEnabled(bool enabled, QString* errorMessage = nullptr) = 0;
    virtual bool isEnabled() const = 0;
    virtual QString statusText() const = 0;
};

} // namespace lcnc::process
