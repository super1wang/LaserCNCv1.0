#pragma once

#include <QString>

#include "core/kernel/i_service.h"
#include "core/logging/log_codes.h"

#include <spdlog/common.h>   // spdlog::level::level_enum

namespace lcnc {

/**
 * @brief 日志服务接口（薄封装 spdlog）。
 *
 * 模块内部不应直接 include @c spdlog 头：通过本接口转发，便于：
 *   - 单测时换成内存收集器；
 *   - 未来切换为别的日志后端；
 *   - 统一携带 @ref LogCode 错误码以便日志分析。
 *
 * 推荐仍使用 LCNC_INFO/WARN/... 宏（见 logger.h），那些宏直接调用底层
 * spdlog logger，性能最佳；本接口适合于"需要把 logger 作为依赖注入"
 * 的场景（如 Mock）。
 */
class ILoggerService : public IService
{
public:
    /// 单条日志输出。
    virtual void log(spdlog::level::level_enum level,
                     LogCode                    code,
                     const QString&             message) = 0;

    /// 设置全局最低日志级别。
    virtual void setLevel(spdlog::level::level_enum level) = 0;

    /// 强制刷新所有 sink。
    virtual void flush() = 0;
};

} // namespace lcnc
