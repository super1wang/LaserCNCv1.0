#pragma once

#include "core/logging/logger.h"

#include <QString>

namespace lcnc::process::legacy {

inline void reportInfo(const QString& message)
{
    LCNC_INFO(lcnc::LogCode::Generic, "process.legacy: {}", message.toStdString());
}

inline void reportWarn(const QString& message)
{
    LCNC_WARN(lcnc::LogCode::Generic, "process.legacy: {}", message.toStdString());
}

inline void reportError(const QString& message)
{
    LCNC_ERROR(lcnc::LogCode::Generic, "process.legacy: {}", message.toStdString());
}

} // namespace lcnc::process::legacy
