#include "core/logging/logger.h"

#include <QDir>

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#ifdef _WIN32
#  include <spdlog/sinks/msvc_sink.h>
#endif

#include <vector>
#include <mutex>

namespace lcnc {

namespace {

constexpr const char* kLoggerName = "lcnc";

std::shared_ptr<spdlog::logger>& logger_storage()
{
    static std::shared_ptr<spdlog::logger> s_logger;
    return s_logger;
}

std::once_flag& fallback_flag()
{
    static std::once_flag s_flag;
    return s_flag;
}

void install_fallback_logger()
{
    // Lightweight fallback if init() was not called yet.
    auto sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    auto lg = std::make_shared<spdlog::logger>(kLoggerName, sink);
    lg->set_level(spdlog::level::info);
    lg->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
    logger_storage() = lg;
}

} // namespace

void Logger::init(const QString& logDir)
{
    // If a real logger was already installed, do nothing (idempotent).
    auto& slot = logger_storage();
    if (slot && slot->name() == kLoggerName) {
        bool isFallback = (slot->sinks().size() == 1);
        if (!isFallback)
            return;
    }

    QDir().mkpath(logDir);
    const QString logFile = QDir(logDir).absoluteFilePath("lasercnc.log");

    std::vector<spdlog::sink_ptr> sinks;

    // Rotating file: 5 MB per file, keep 5 files.
    try {
        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            logFile.toStdString(),
            5 * 1024 * 1024,
            5);
        file_sink->set_level(spdlog::level::trace);
        sinks.push_back(file_sink);
    } catch (const std::exception&) {
        // Fall through; we still get console.
    }

    // Console (visible when launched from a terminal).
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::info);
    sinks.push_back(console_sink);

#ifdef _WIN32
    auto msvc_sink = std::make_shared<spdlog::sinks::msvc_sink_mt>();
    msvc_sink->set_level(spdlog::level::debug);
    sinks.push_back(msvc_sink);
#endif

    auto lg = std::make_shared<spdlog::logger>(kLoggerName, sinks.begin(), sinks.end());
    lg->set_level(spdlog::level::debug);
    lg->flush_on(spdlog::level::warn);
    lg->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%t] [%^%l%$] %v");

    slot = lg;
    LCNC_INFO(LogCode::LoggerInitialized,
              "Logger initialized, log file: {}",
              logFile.toStdString());
}

void Logger::shutdown()
{
    auto& slot = logger_storage();
    if (slot) {
        slot->flush();
        slot.reset();
    }
}

std::shared_ptr<spdlog::logger> Logger::get()
{
    auto& slot = logger_storage();
    if (!slot) {
        std::call_once(fallback_flag(), &install_fallback_logger);
    }
    return slot;
}

} // namespace lcnc
