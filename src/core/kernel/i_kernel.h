#pragma once

#include <memory>

#include "core/kernel/event_bus.h"
#include "core/kernel/service_registry.h"

namespace lcnc {

/**
 * @brief 微内核对外接口：模块在 @c init() 中拿到的唯一句柄。
 *
 * 提供两类访问能力：
 *   1. **通用服务注册表** @ref services() — 查找业务模块自定义注册的
 *      服务（如 @c ICamFacade）。
 *   2. **事件总线** @ref events() — 用于类型化发布订阅。
 *
 * 核心容器（ProjectManager / GuiApplication / TaskManager / AppSettings /
 * CommandContainer）不走 IService，请通过 @c lcnc::Kernel 具体类的强类型
 * getter 访问（如 @c app() / @c guiApp() / @c taskManager() / @c appSettings() /
 * @c commandContainer()）。
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
};

} // namespace lcnc
