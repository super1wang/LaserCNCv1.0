#pragma once

#include <memory>

#include "core/kernel/i_kernel.h"
#include "core/kernel/module_registry.h"

// 前置声明（这两个类位于全局命名空间，AppSettings 位于 lcnc 命名空间）。
class GuiApplication;
class TaskManager;

namespace lcnc::cam { class MachineWorkspace; }

namespace lcnc {

class AppSettings;
class LcncProjectManager;
class MachineConfigurationService;
namespace kinematics { class MachineCalibrationService; }

/**
 * @brief 微内核实现。
 *
 * 唯一允许的"全局对象"：由 @c main() 创建并独占，其生命周期严格嵌入
 * @c QApplication 内部（QApplication 之后构造，QApplication 退出之前
 * 销毁）。
 *
 * 启动顺序：
 *   1. 构造 Kernel（不做任何重活）；
 *   2. @ref registerCoreServices() — 注册 logger / settings / tasks /
 *      documents / commands 五大核心服务；
 *   3. @ref addModule(...) — 把 CadModule / CamModule / ProcessModule
 *      等业务插件加入；
 *   4. @ref bootstrap() — 触发模块拓扑排序 + init + start；
 *   5. 应用运行；
 *   6. @ref shutdown() — 反向 stop 模块、清空服务。
 *
 * Kernel 只暴露能力，不持有任何业务状态。所有模块/服务可独立替换或裁剪。
 */
class Kernel : public IKernel
{
public:
    Kernel();
    ~Kernel() override;

    // 禁拷贝/移动：Kernel 只能在 main 中按值持有。
    Kernel(const Kernel&)            = delete;
    Kernel& operator=(const Kernel&) = delete;

    // ── IKernel ─────────────────────────────────────────────────────────
    ServiceRegistry&    services() override   { return m_services; }
    EventBus&           events() override     { return m_events; }


    // ── 启动 / 关闭 ─────────────────────────────────────────────────────

    /**
    * @brief 注册全部核心服务：创建并持有 AppSettings/LcncProjectManager/
    *        TaskManager 这些核心对象的实例（GuiApplication 由 main 负责创建
     *        并以 @ref setGuiApp 注入）。
     *
     * 必须在 @ref bootstrap 之前调用。
     */
    void registerCoreServices();

    /**
     * @brief 把命令注册器实例提交给 Kernel；后续模块通过
     *        @ref commandContainer() 访问同一容器。
     *
     * `CommandContainer` 由 MainWindow 构造（它需要 ICommandContext*），
     * 构造完成后调用此函数把容器交给 Kernel。传入 nullptr 可解除绑定。
     */
    void setCommandContainer(class CommandContainer* container) { m_cmdContainer = container; }

    /// 返回当前绑定的 @c CommandContainer 裸指针（未绑定时为 nullptr）。
    class CommandContainer* commandContainer() const { return m_cmdContainer; }

    /**
     * @brief 加入一个业务模块（顺序无关，依赖由 @c ModuleInfo 声明）。
     */
    void addModule(std::unique_ptr<IModule> module);

    /**
     * @brief 启动所有模块（init + start）。
     */
    bool bootstrap();

    /**
     * @brief 反向停止所有模块并清空服务。
     */
    void shutdown();

    // ── 直达 getter（不再有任何 XxxClass::instance() 调用） ──────────────

    /// 返回 Kernel 持有的项目管理器（在 registerCoreServices 后非空）。
    LcncProjectManager* projectManager() const { return m_projectMgr.get(); }
    /// 返回 Kernel 持有的机台工作台（独立参考资产，core 拥有、跨工程常驻；
    /// 在 registerCoreServices 后非空）。CAM 模块借用它做加载/标定等业务。
    lcnc::cam::MachineWorkspace* machineWorkspace() const { return m_machineWorkspace.get(); }
    /// 返回由 main() 创建并交付 Kernel 的图形/视图管理器。
    /// 所有权不在 Kernel（避免 core 反向 include view），在
    /// @ref setGuiApp 调用后非空。
    GuiApplication*  guiApp() const  { return m_guiApp; }
    /// 设置外部拥有的 GuiApplication 裸指针；生命周期须不短于
    /// Kernel 本身（推荐在 @c registerCoreServices 后、任何模块 init 前
    /// 调用）。传入 nullptr 可解除绑定。
    void             setGuiApp(GuiApplication* g) { m_guiApp = g; }
    /// 返回 Kernel 持有的任务管理器（在 registerCoreServices 后非空）。
    TaskManager*     taskManager() const { return m_taskMgr.get(); }
    /// 返回 Kernel 持有的应用设置（在 registerCoreServices 后非空）。
    AppSettings*     appSettings() const { return m_appSettings.get(); }

    // ── 进程级访问（唯一允许的全局入口） ────────────────────────────────

    /**
     * @brief 拿到当前 Kernel 实例。
     *
     * 由 Kernel 构造函数自动设置；析构时清空。同一进程内只允许存在一个
     * Kernel；此方法是其它模块/widget/命令在没有显式 IKernel 引用时
     * 访问全局服务的统一入口（取代了原来散布各处的 XxxClass::instance()）。
     *
     * @return 非空 Kernel*；如果 Kernel 还未构造则返回 nullptr。
     */
    static Kernel* tryCurrent();

    /// 与 @ref tryCurrent 相同但断言非空（找不到时 LCNC_CRIT + abort）。
    static Kernel& current();

    /// 便捷模板：按接口类型从 ServiceRegistry 取出已注册的服务/模块裸指针。
    /// 找不到时返回 nullptr；推荐用于 `Kernel::current().service<CamModule>()` 这类调用。
    template<class T>
    T* service() const noexcept
    {
        auto sp = m_services.getService<T>();
        return sp.get();
    }

private:
    ServiceRegistry m_services;
    EventBus        m_events;
    ModuleRegistry  m_modules;

    // 由 Kernel 直接拥有所有权（取代原来的 self-managing singleton）。
    std::unique_ptr<LcncProjectManager> m_projectMgr;
    std::unique_ptr<lcnc::cam::MachineWorkspace> m_machineWorkspace; ///< 独立机台参考资产（core 拥有）。
    std::unique_ptr<::TaskManager>     m_taskMgr;
    std::unique_ptr<AppSettings>       m_appSettings;
    std::shared_ptr<MachineConfigurationService> m_machineConfig;
    std::shared_ptr<kinematics::MachineCalibrationService> m_machineCalibration;
    // GuiApplication 仅裸指针；所有权在 main()，避免 core 依赖 view。
    ::GuiApplication*                  m_guiApp{nullptr};
    // CommandContainer 仅裸指针；所有权在 MainWindow。
    class CommandContainer*            m_cmdContainer{nullptr};
};

} // namespace lcnc
