#pragma once

#include "core/logging/logger.h"
#include "modules/process/system/message_code.h"

#include <QString>

#include <string>
#include <type_traits>

namespace lcnc::process {

template <typename Code>
void logDeviceError(Code code, const char* message)
{
    static_assert(std::is_enum_v<Code>);
    LCNC_ERR(LogCode::Generic, "Process device error code={}: {}",
             static_cast<int>(code), message ? message : "");
}

template <typename Code>
void logDeviceError(Code code, const QString& message)
{
    logDeviceError(code, message.toUtf8().constData());
}

template <typename Code>
void logDeviceWarning(Code code, const QString& message)
{
    static_assert(std::is_enum_v<Code>);
    LCNC_WARN(LogCode::Generic, "Process device warning code={}: {}",
              static_cast<int>(code), message.toStdString());
}

template <typename Code>
void logDeviceWarning(Code code, const char* message)
{
    logDeviceWarning(code, QString::fromUtf8(message ? message : ""));
}

} // namespace lcnc::process
