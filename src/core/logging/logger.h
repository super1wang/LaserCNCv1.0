#pragma once

#include <QString>
#include <memory>
#include <string>

#include "core/logging/log_codes.h"

// SPDLOG_HEADER_ONLY is defined globally via CMake; we just include the header.
#include <spdlog/spdlog.h>

namespace lcnc {

/**
 * @brief Application-wide logging facade built on spdlog.
 *
 * - Owns one shared spdlog::logger named "lcnc".
 * - Sinks: rotating file (logs/lasercnc.log), MSVC debug (Debug builds),
 *   stdout color (always when console is available).
 * - Severity flush threshold defaults to warn.
 *
 * Use the LCNC_* macros below; do not call spdlog directly outside this file.
 */
class Logger
{
public:
    /**
     * @brief Initialize the global logger. Idempotent.
     * @param logDir  Absolute or relative directory for log files.
     *                Will be created if it does not exist.
     */
    static void init(const QString& logDir);

    /// Flush + drop the logger (called from QApplication shutdown if desired).
    static void shutdown();

    /// Returns the shared logger; creates a stderr-only fallback if init() was
    /// not called (so log lines from early code do not crash).
    static std::shared_ptr<spdlog::logger> get();

    /// QString → std::string helper (UTF-8, used by macros).
    static std::string toUtf8(const QString& s) { return s.toStdString(); }

private:
    Logger() = default;
};

} // namespace lcnc

// ─────────────────────────────────────────────────────────────────────────────
// Convenience macros. The first argument is a lcnc::LogCode; the rest is a
// fmt-style format string + args. Code is logged inside [..] for grep-ability.
// ─────────────────────────────────────────────────────────────────────────────

#define LCNC_LOG(level, code, ...) \
    do { \
        auto _lg = ::lcnc::Logger::get(); \
        if (_lg && _lg->should_log(level)) { \
            _lg->log(level, "[{:04d}] " __VA_ARGS__, static_cast<int>(code)); \
        } \
    } while (0)

#define LCNC_TRACE(code, ...) LCNC_LOG(spdlog::level::trace,    code, __VA_ARGS__)
#define LCNC_DEBUG(code, ...) LCNC_LOG(spdlog::level::debug,    code, __VA_ARGS__)
#define LCNC_INFO(code,  ...) LCNC_LOG(spdlog::level::info,     code, __VA_ARGS__)
#define LCNC_WARN(code,  ...) LCNC_LOG(spdlog::level::warn,     code, __VA_ARGS__)
#define LCNC_ERR(code,   ...) LCNC_LOG(spdlog::level::err,      code, __VA_ARGS__)
#define LCNC_CRIT(code,  ...) LCNC_LOG(spdlog::level::critical, code, __VA_ARGS__)

/// Log a QString payload (UTF-8 converted).
#define LCNC_INFO_Q(code, qstr) LCNC_INFO(code, "{}", ::lcnc::Logger::toUtf8(qstr))
#define LCNC_WARN_Q(code, qstr) LCNC_WARN(code, "{}", ::lcnc::Logger::toUtf8(qstr))
#define LCNC_ERR_Q(code,  qstr) LCNC_ERR (code, "{}", ::lcnc::Logger::toUtf8(qstr))
