#pragma once

#include <memory>

#include "core/command/i_command_bus.h"
#include "core/document/i_document_registry.h"
#include "core/kernel/event_bus.h"
#include "core/kernel/service_registry.h"
#include "core/logging/i_logger_service.h"
#include "core/settings/i_settings_service.h"
#include "core/task/i_task_runner.h"

namespace lcnc {

/**
 * @brief 微内核对外接口：模块在 @c init() 中拿到的唯一句柄。
 *
 * 提供两类访问能力：
 *   1. **核心服务直达 getter**（logger/settings/tasks/documents/commands）—
 *      使用频率最高，避免每次写 services().getService<ILoggerService>()；
 *   2. **通用 services() / events()** — 用于查找业务模块自定义注册的服务
 *      （如 ICamFacade）以及类型化发布订阅。
 *
 * 模块 init 阶段持有 @c IKernel& 引用即可；如需在 start/stop 后继续访问，
 * 应保存所需服务的 @c std::weak_ptr ，**不要** 持有 @c IKernel* 。
 */
class IKernel
{
public:
    virtual ~IKernel() = default;

    // ── 通用注册表 ──────────────────────────────────────────────────────
    virtual ServiceRegistry& services() = 0;
    virtual EventBus&        events()   = 0;

    // ── 核心服务快捷方式 ────────────────────────────────────────────────
    virtual ILoggerService&    logger()    = 0;
    virtual ISettingsService&  settings()  = 0;
    virtual ITaskRunner&       tasks()     = 0;
    virtual IDocumentRegistry& documents() = 0;
    virtual ICommandBus&       commands()  = 0;
};

} // namespace lcnc
