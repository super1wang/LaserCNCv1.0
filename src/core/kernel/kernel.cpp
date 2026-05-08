#include "core/kernel/kernel.h"

#include "core/logging/logger.h"
#include "core/project/lcnc_project_manager.h"
#include "core/settings/app_settings.h"
#include "core/task/task_manager.h"

namespace lcnc {

// 仅允许一个 Kernel 实例 — main() 构造时被设置，析构时清空。
static Kernel* g_kernelCurrent = nullptr;

// ─── Kernel ─────────────────────────────────────────────────────────────────

Kernel::Kernel()
{
    LCNC_DEBUG(LogCode::Generic, "Kernel::Kernel()");
    Q_ASSERT_X(!g_kernelCurrent, "Kernel",
               "second Kernel instance — only one allowed per process");
    g_kernelCurrent = this;
}

Kernel::~Kernel()
{
    LCNC_DEBUG(LogCode::Generic, "Kernel::~Kernel()");
    if (m_modules.size() > 0) {
        // 免底：用户忘了 shutdown
        shutdown();
    }
    // 释放顺序：依赖反向（gui 订阅了 lcnc 信号，故需在 main 中先于
    // Kernel 销毁 GuiApplication；Kernel 不拥有以避免 core 依赖 view）。
    m_taskMgr.reset();
    m_projectMgr.reset();
    m_appSettings.reset();
    if (g_kernelCurrent == this) g_kernelCurrent = nullptr;
}

Kernel* Kernel::tryCurrent() { return g_kernelCurrent; }

Kernel& Kernel::current()
{
    if (!g_kernelCurrent) {
        LCNC_CRIT(LogCode::InternalUnexpectedState,
                  "Kernel::current() called before Kernel was constructed");
        std::terminate();
    }
    return *g_kernelCurrent;
}

void Kernel::registerCoreServices()
{
    LCNC_DEBUG(LogCode::Generic, "Kernel::registerCoreServices begin");

    // 1) AppSettings — 由 Kernel 直接拥有
    m_appSettings = std::make_unique<AppSettings>();

    // 2) ProjectManager — 单项目生命周期和三域文档入口。
    m_projectMgr = std::make_unique<LcncProjectManager>();

    // 3) GuiApplication 不在 Kernel 创建（core 反向 view 依赖），由 main()
    //    在 registerCoreServices 后、bootstrap 前调用 setGuiApp(...)。

    // 4) TaskManager — 独立
    m_taskMgr = std::make_unique<TaskManager>();

    // 5) CommandContainer 由 MainWindow 后期通过 setCommandContainer 提供。

    LCNC_INFO(LogCode::Generic,
              "Kernel: core objects ready (services={})", m_services.size());
}

void Kernel::addModule(std::unique_ptr<IModule> module)
{
    m_modules.addModule(std::move(module));
}

bool Kernel::bootstrap()
{
    LCNC_INFO(LogCode::Generic,
              "Kernel::bootstrap begin (modules={}, services={})",
              m_modules.size(), m_services.size());
    const bool ok = m_modules.startAll(*this);
    LCNC_INFO(LogCode::Generic,
              "Kernel::bootstrap {}", ok ? "succeeded" : "failed");
    return ok;
}

void Kernel::shutdown()
{
    LCNC_INFO(LogCode::Generic, "Kernel::shutdown begin");
    m_modules.stopAll();
    m_services.clear();
    m_cmdContainer = nullptr;
    LCNC_INFO(LogCode::Generic, "Kernel::shutdown end");
}

} // namespace lcnc
