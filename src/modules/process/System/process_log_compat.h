#pragma once

#include "core/logging/logger.h"

// Transitional message metadata.  Device code must use the unified Logger;
// these enums remain only because MessageModule still categorizes UI notices.
enum class LogType { System, Operator, Process };
enum class LogLevel { Trace, Debug, Info, Warn, Error, Critical };

#define LCNC_PROCESS_LOG(level, message) ::lcnc::Logger::log(level, ::lcnc::LogCode::Generic, "{}", (message))
#define LOG_SYS_TRACE(message)     LCNC_PROCESS_LOG(spdlog::level::trace, message)
#define LOG_SYS_DEBUG(message)     LCNC_PROCESS_LOG(spdlog::level::debug, message)
#define LOG_SYS_INFO(message)      LCNC_PROCESS_LOG(spdlog::level::info, message)
#define LOG_SYS_WARN(message)      LCNC_PROCESS_LOG(spdlog::level::warn, message)
#define LOG_SYS_ERROR(message)     LCNC_PROCESS_LOG(spdlog::level::err, message)
#define LOG_SYS_CRITICAL(message)  LCNC_PROCESS_LOG(spdlog::level::critical, message)
#define LOG_OPER_TRACE(message)    LOG_SYS_TRACE(message)
#define LOG_OPER_DEBUG(message)    LOG_SYS_DEBUG(message)
#define LOG_OPER_INFO(message)     LOG_SYS_INFO(message)
#define LOG_OPER_WARN(message)     LOG_SYS_WARN(message)
#define LOG_OPER_ERROR(message)    LOG_SYS_ERROR(message)
#define LOG_OPER_CRITICAL(message) LOG_SYS_CRITICAL(message)
#define LOG_PROCESS_TRACE(message) LOG_SYS_TRACE(message)
#define LOG_PROCESS_DEBUG(message) LOG_SYS_DEBUG(message)
#define LOG_PROCESS_INFO(message)  LOG_SYS_INFO(message)
#define LOG_PROCESS_WARN(message)  LOG_SYS_WARN(message)
#define LOG_PROCESS_ERROR(message) LOG_SYS_ERROR(message)
#define LOG_PROCESS_CRITICAL(message) LOG_SYS_CRITICAL(message)
#define LOG_PROCESS_START() ((void)0)
#define LOG_PROCESS_STOP() ((void)0)
#define LOG_PROCESS_REFRESH() ((void)0)
#define LOG_REFRESH() ((void)0)
