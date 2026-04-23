#include "core/kernel/kernel.h"

#include <stdexcept>

#include <QAction>

#include "core/command/commands_api.h"
#include "core/document/lcnc_application.h"
#include "core/logging/logger.h"
#include "core/settings/app_settings.h"
#include "core/task/task_manager.h"
#include "view/gui_application.h"

namespace lcnc {

// 仅允许一个 Kernel 实例 — main() 构造时被设置，析构时清空。
static Kernel* g_kernelCurrent = nullptr;

// ─── 内部适配器：把现有单例包装为 IService ──────────────────────────────────

namespace {

/// ILoggerService → 转发到 lcnc::Logger 静态方法 / spdlog logger。
class LoggerServiceAdapter : public ILoggerService
{
public:
    void log(spdlog::level::level_enum level,
             LogCode                    code,
             const QString&             message) override
    {
        auto lg = Logger::get();
        if (lg && lg->should_log(level)) {
            lg->log(level, "[{:04d}] {}",
                    static_cast<int>(code),
                    message.toStdString());
        }
    }

    void setLevel(spdlog::level::level_enum level) override
    {
        if (auto lg = Logger::get()) lg->set_level(level);
    }

    void flush() override
    {
        if (auto lg = Logger::get()) lg->flush();
    }
};

/// ISettingsService → 转发到 Kernel 拥有的 AppSettings 。
class SettingsServiceAdapter : public ISettingsService
{
public:
    explicit SettingsServiceAdapter(AppSettings* s) : m_settings(s) {}
    AppSettings& app() override { return *m_settings; }
    bool save() override        { return m_settings->saveDefault(); }
private:
    AppSettings* m_settings;
};

/// ITaskRunner → 转发到 Kernel 拥有的 TaskManager。
class TaskRunnerAdapter : public ITaskRunner
{
public:
    explicit TaskRunnerAdapter(TaskManager* t) : m_tasks(t) {}
    int run(const QString& label, TaskJob job) override
    {
        return m_tasks->run(label, std::move(job));
    }
    void requestAbort(int taskId) override
    {
        m_tasks->requestAbort(taskId);
    }
private:
    TaskManager* m_tasks;
};

/// IDocumentRegistry → 转发到 Kernel 拥有的 LcncApplication。
class DocumentRegistryAdapter : public IDocumentRegistry
{
public:
    explicit DocumentRegistryAdapter(LcncApplication* a) : m_app(a) {}
    DocumentId   activeDocumentId()    const override { return m_app->activeDocumentId(); }
    LcncDocument* activeDocument()     const override { return m_app->activeDocument(); }
    LcncDocument* documentById(DocumentId id) const override { return m_app->documentById(id); }
    QList<LcncDocument*> documents()   const override { return m_app->documents(); }
    int           documentCount()      const override { return m_app->documentCount(); }
    DocumentId    machineDocumentId()  const override { return m_app->machineDocumentId(); }
    LcncDocument* machineDocument()    const override { return m_app->machineDocument(); }
    QList<LcncDocument*> workpieceDocuments() const override { return m_app->workpieceDocuments(); }
private:
    LcncApplication* m_app;
};

/// ICommandBus → 转发到 CommandContainer 实例（由 attach 注入）。
class CommandBusAdapter : public ICommandBus
{
public:
    explicit CommandBusAdapter(CommandContainer* c = nullptr) : m_container(c) {}

    void setContainer(CommandContainer* c) { m_container = c; }

    CommandBase* findCommand(const QString& name) const override
    {
        if (!m_container) return nullptr;
        return m_container->findCommand(name);
    }
    QAction* findAction(const QString& name) const override
    {
        if (!m_container) return nullptr;
        return m_container->findAction(name);
    }
    bool invoke(const QString& name) override
    {
        auto* a = findAction(name);
        if (!a) {
            LCNC_WARN(LogCode::InternalUnexpectedState,
                      "ICommandBus::invoke('{}') not found",
                      name.toStdString());
            return false;
        }
        if (!a->isEnabled()) {
            LCNC_DEBUG(LogCode::Generic,
                       "ICommandBus::invoke('{}') disabled",
                       name.toStdString());
            return false;
        }
        a->trigger();
        return true;
    }

private:
    CommandContainer* m_container;
};

} // namespace

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
    // 释放顺序：依赖反向（gui 订阅了 lcnc 信号）
    m_guiApp.reset();
    m_taskMgr.reset();
    m_app.reset();
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

    // 1) Logger 适配（spdlog 已由 main() 提前 Logger::init() 初始化）
    auto logSvc = std::make_shared<LoggerServiceAdapter>();
    m_services.registerService<ILoggerService>(logSvc);
    m_loggerSvc = logSvc.get();

    // 2) AppSettings — 由 Kernel 直接拥有，以后不依赖其 instance() 懒加载
    m_appSettings = std::make_unique<AppSettings>();
    auto setSvc = std::make_shared<SettingsServiceAdapter>(m_appSettings.get());
    m_services.registerService<ISettingsService>(setSvc);
    m_settingsSvc = setSvc.get();

    // 3) LcncApplication（纯文档层）— 依赖 0
    m_app = std::make_unique<LcncApplication>();
    auto docSvc = std::make_shared<DocumentRegistryAdapter>(m_app.get());
    m_services.registerService<IDocumentRegistry>(docSvc);
    m_docsSvc = docSvc.get();

    // 4) GuiApplication — 需要 LcncApplication 已存在（构造时 connect signals）
    m_guiApp = std::make_unique<GuiApplication>();

    // 5) TaskManager — 独立
    m_taskMgr = std::make_unique<TaskManager>();
    auto tskSvc = std::make_shared<TaskRunnerAdapter>(m_taskMgr.get());
    m_services.registerService<ITaskRunner>(tskSvc);
    m_tasksSvc = tskSvc.get();

    // 6) ICommandBus — container 后期由 MainWindow 通过 setCommandContainer 提供
    auto cmdSvc = std::make_shared<CommandBusAdapter>(nullptr);
    m_services.registerService<ICommandBus>(cmdSvc);
    m_cmdsSvc = cmdSvc.get();

    LCNC_INFO(LogCode::Generic,
              "Kernel: core services registered (count={})", m_services.size());
}

void Kernel::setCommandContainer(CommandContainer* container)
{
    LCNC_DEBUG(LogCode::Generic,
               "Kernel::setCommandContainer({})",
               static_cast<const void*>(container));
    auto bus = m_services.getService<ICommandBus>();
    if (auto* adapter = dynamic_cast<CommandBusAdapter*>(bus.get())) {
        adapter->setContainer(container);
    } else {
        LCNC_ERR(LogCode::InternalUnexpectedState,
                 "Kernel::setCommandContainer: ICommandBus adapter missing");
    }
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
    m_loggerSvc   = nullptr;
    m_settingsSvc = nullptr;
    m_tasksSvc    = nullptr;
    m_docsSvc     = nullptr;
    m_cmdsSvc     = nullptr;
    LCNC_INFO(LogCode::Generic, "Kernel::shutdown end");
}

// ─── 核心服务快捷 getter ────────────────────────────────────────────────────

namespace {
template <class T>
[[noreturn]] void throwIfNull(T* p, const char* name)
{
    LCNC_CRIT(LogCode::InternalAssertion,
              "Kernel::{}() called before registerCoreServices()", name);
    throw std::logic_error(std::string{"Kernel core service '"} + name + "' not registered");
    (void)p;
}
} // namespace

ILoggerService& Kernel::logger()
{
    if (!m_loggerSvc) throwIfNull(m_loggerSvc, "logger");
    return *m_loggerSvc;
}
ISettingsService& Kernel::settings()
{
    if (!m_settingsSvc) throwIfNull(m_settingsSvc, "settings");
    return *m_settingsSvc;
}
ITaskRunner& Kernel::tasks()
{
    if (!m_tasksSvc) throwIfNull(m_tasksSvc, "tasks");
    return *m_tasksSvc;
}
IDocumentRegistry& Kernel::documents()
{
    if (!m_docsSvc) throwIfNull(m_docsSvc, "documents");
    return *m_docsSvc;
}
ICommandBus& Kernel::commands()
{
    if (!m_cmdsSvc) throwIfNull(m_cmdsSvc, "commands");
    return *m_cmdsSvc;
}

} // namespace lcnc
