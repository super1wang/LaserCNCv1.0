#include "modules/process/commands/commands_process.h"

#include <QAction>
#include <QFileDialog>
#include <QIcon>
#include <QKeySequence>
#include <QMessageBox>
#include <QObject>
#include <QStandardPaths>

#include "core/kernel/event_bus.h"
#include "core/kernel/kernel.h"
#include "core/logging/logger.h"
#include "core/services/selection_service.h"
#include "modules/process/cutting/process_cutting_plan_service.h"
#include "modules/process/i_process_facade.h"
#include "modules/process/process_module.h"
#include "modules/process/runtime/process_events.h"
#include "modules/process/settings/process_settings_dialog.h"

namespace lcnc::process {

namespace {
/// 从 Kernel 取 Process facade；未注册时返回 nullptr 并打 WARN。
lcnc::IProcessFacade* processFacade()
{
    auto* p = lcnc::Kernel::current().service<lcnc::IProcessFacade>();
    if (!p) {
        LCNC_WARN(lcnc::LogCode::InternalUnexpectedState,
                  "process.cmd: IProcessFacade service not registered");
    }
    return p;
}

ProcessModule* processModule()
{
    auto* module = lcnc::Kernel::current().service<ProcessModule>();
    if (!module) {
        LCNC_WARN(lcnc::LogCode::InternalUnexpectedState,
                  "process.cmd: ProcessModule service not registered");
    }
    return module;
}
} // namespace

// ── CmdNewProcess ────────────────────────────────────────────────────────
CmdNewProcess::CmdNewProcess(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：新建流程
    auto* a = new QAction(QIcon("themeicons:new_process.svg"), tr("Create new process"), this);
    // 中文翻译：清空当前流程树并创建新流程
    a->setStatusTip(tr("Clear the current process tree and create a new process"));
    setAction(a);
}
bool CmdNewProcess::isEnabled() const
{
    auto* p = lcnc::Kernel::current().service<lcnc::IProcessFacade>();
    return p && p->state() != lcnc::ProcessRunState::Running;
}
void CmdNewProcess::execute()
{
    if (auto* p = processFacade()) p->newProcess();
}

// ── CmdLoadProcess ───────────────────────────────────────────────────────
CmdLoadProcess::CmdLoadProcess(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：加载流程
    auto* a = new QAction(QIcon("themeicons:open_process.svg"), tr("Loading process"), this);
    // 中文翻译：从 TOML 文件加载流程树
    a->setStatusTip(tr("Load process tree from TOML file"));
    setAction(a);
}
bool CmdLoadProcess::isEnabled() const
{
    auto* p = lcnc::Kernel::current().service<lcnc::IProcessFacade>();
    return p && p->state() != lcnc::ProcessRunState::Running;
}
void CmdLoadProcess::execute()
{
    auto* p = processFacade();
    if (!p) return;

    const QString filePath = QFileDialog::getOpenFileName(
        nullptr,
        // 中文翻译：加载流程
        tr("Loading process"),
        QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation),
        tr("Process TOML (*.toml)"));
    if (filePath.isEmpty())
        return;
    p->loadProcess(filePath);
}

// ── CmdSaveProcess ───────────────────────────────────────────────────────
CmdSaveProcess::CmdSaveProcess(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：保存流程
    auto* a = new QAction(QIcon("themeicons:save_process.svg"), tr("Save process"), this);
    // 中文翻译：保存当前流程树为 TOML 文件
    a->setStatusTip(tr("Save the current process tree as a TOML file"));
    setAction(a);
}
bool CmdSaveProcess::isEnabled() const
{
    auto* p = lcnc::Kernel::current().service<lcnc::IProcessFacade>();
    return p && p->state() != lcnc::ProcessRunState::Running;
}
void CmdSaveProcess::execute()
{
    auto* p = processFacade();
    if (!p) return;

    const QString filePath = QFileDialog::getSaveFileName(
        nullptr,
        // 中文翻译：保存流程
        tr("Save process"),
        QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation),
        tr("Process TOML (*.toml)"));
    if (filePath.isEmpty())
        return;
    p->saveProcess(filePath);
}

// ── CmdOpenProcessSettings ──────────────────────────────────────────────
CmdOpenProcessSettings::CmdOpenProcessSettings(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：设置
    auto* a = new QAction(QIcon("themeicons:settings.svg"), tr("settings"), this);
    // 中文翻译：打开外设和加工参数设置
    a->setStatusTip(tr("Open peripherals and processing parameter settings"));
    setAction(a);
}
bool CmdOpenProcessSettings::isEnabled() const
{
    return lcnc::Kernel::current().service<ProcessModule>() != nullptr;
}
void CmdOpenProcessSettings::execute()
{
    try {
        auto* mod = lcnc::Kernel::current().service<ProcessModule>();
        if (!mod || !mod->settingsService())
            return;
        ProcessSettingsDialog dlg(
            mod->settingsService(),
            [mod](const ProcessSettingsChangeSet& changes) {
                if (mod)
                    mod->applySettingsChanges(changes);
            });
        dlg.exec();

        // 设置对话框关闭后刷新主界面 IO 栏（showInMain 列可能改过）。
        if (mod)
            mod->refreshIOFromSettings();
    }
    catch (const std::exception& e) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "CmdOpenProcessSettings::execute failed: {}",
                 e.what());
    }
}

// ── CmdRunStart ─────────────────────────────────────────────────────────────
CmdRunStart::CmdRunStart(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：运行
    auto* a = new QAction(QIcon("themeicons:start.svg"), tr("run"), this);
    a->setShortcut(QKeySequence(Qt::Key_F5));
    // 中文翻译：启动加工运行（仿真或控制器）
    a->setStatusTip(tr("Start a machining run (simulation or controller)"));
    setAction(a);
}
bool CmdRunStart::isEnabled() const
{
    auto* p = lcnc::Kernel::current().service<lcnc::IProcessFacade>();
    return p && p->state() != lcnc::ProcessRunState::Running;
}
void CmdRunStart::execute()
{
    if (auto* p = processFacade()) p->runStart();
}

// ── CmdRunPause ─────────────────────────────────────────────────────────────
CmdRunPause::CmdRunPause(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：暂停
    auto* a = new QAction(QIcon("themeicons:pause.svg"), tr("pause"), this);
    // 中文翻译：暂停当前加工运行
    a->setStatusTip(tr("Pause current processing run"));
    setAction(a);
}
bool CmdRunPause::isEnabled() const
{
    auto* p = lcnc::Kernel::current().service<lcnc::IProcessFacade>();
    return p && p->state() == lcnc::ProcessRunState::Running;
}
void CmdRunPause::execute()
{
    if (auto* p = processFacade()) p->runPause();
}

// ── CmdRunStop ──────────────────────────────────────────────────────────────
CmdRunStop::CmdRunStop(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：停止
    auto* a = new QAction(QIcon("themeicons:stop.svg"), tr("stop"), this);
    // 中文翻译：停止当前加工运行
    a->setStatusTip(tr("Stop current processing run"));
    setAction(a);
}
bool CmdRunStop::isEnabled() const
{
    auto* p = lcnc::Kernel::current().service<lcnc::IProcessFacade>();
    if (!p) return false;
    auto s = p->state();
    return s == lcnc::ProcessRunState::Running || s == lcnc::ProcessRunState::Paused;
}
void CmdRunStop::execute()
{
    if (auto* p = processFacade()) p->runStop();
}

// ── CmdResetStop ────────────────────────────────────────────────────────────
CmdResetStop::CmdResetStop(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：停止复位
    auto* a = new QAction(QIcon("themeicons:reset.svg"), tr("Reset stop"), this);
    // 中文翻译：检查设备并恢复空闲状态
    a->setStatusTip(tr("Check devices and recover the idle state"));
    setAction(a);
}
bool CmdResetStop::isEnabled() const
{
    auto* p = lcnc::Kernel::current().service<lcnc::IProcessFacade>();
    return p && (p->state() == lcnc::ProcessRunState::Stopped
                 || p->state() == lcnc::ProcessRunState::Error);
}
void CmdResetStop::execute()
{
    if (auto* p = processFacade()) p->resetStop();
}

// ── CmdHome ─────────────────────────────────────────────────────────────────
CmdHome::CmdHome(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：回零
    auto* a = new QAction(QIcon("themeicons:home.svg"), tr("Return to zero"), this);
    // 中文翻译：按 Z 轴优先顺序回零
    a->setStatusTip(tr("Return to zero according to Z axis priority order"));
    setAction(a);
}
bool CmdHome::isEnabled() const
{
    auto* p = lcnc::Kernel::current().service<lcnc::IProcessFacade>();
    return p && p->state() != lcnc::ProcessRunState::Running;
}
void CmdHome::execute()
{
    if (auto* p = processFacade()) p->home();
}

// ── CmdMoveToLoadingPosition ────────────────────────────────────────────────
CmdMoveToLoadingPosition::CmdMoveToLoadingPosition(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：上料位
    auto* a = new QAction(QIcon("themeicons:move.svg"), tr("Loading position"), this);
    // 中文翻译：移动至设置中定义的上料位
    a->setStatusTip(tr("Move to the loading level defined in the settings"));
    setAction(a);
}
bool CmdMoveToLoadingPosition::isEnabled() const
{
    auto* p = lcnc::Kernel::current().service<lcnc::IProcessFacade>();
    return p && p->isConnected()
             && p->state() == lcnc::ProcessRunState::Idle;
}
void CmdMoveToLoadingPosition::execute()
{
    if (auto* module = processModule()) module->moveToConfiguredPosition(true);
}

// ── CmdMoveToBlankingPosition ───────────────────────────────────────────────
CmdMoveToBlankingPosition::CmdMoveToBlankingPosition(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：下料位
    auto* a = new QAction(QIcon("themeicons:move.svg"), tr("Unloading position"), this);
    // 中文翻译：移动至设置中定义的下料位
    a->setStatusTip(tr("Move to the blanking position defined in the settings"));
    setAction(a);
}
bool CmdMoveToBlankingPosition::isEnabled() const
{
    auto* p = lcnc::Kernel::current().service<lcnc::IProcessFacade>();
    return p && p->isConnected()
             && p->state() == lcnc::ProcessRunState::Idle;
}
void CmdMoveToBlankingPosition::execute()
{
    if (auto* module = processModule()) module->moveToConfiguredPosition(false);
}

// ── CmdConnectDevices ───────────────────────────────────────────────────────
CmdConnectDevices::CmdConnectDevices(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：连接设备
    auto* a = new QAction(QIcon("themeicons:connect.svg"), tr("Connect devices"), this);
    // 中文翻译：异步连接全部已配置外设（运动控制器、激光器等）
    a->setStatusTip(tr("Asynchronously connect all configured peripherals (motion controllers, lasers, etc.)"));
    setAction(a);
}
bool CmdConnectDevices::isEnabled() const
{
    auto* p = lcnc::Kernel::current().service<lcnc::IProcessFacade>();
    return p && !p->isConnected();
}
void CmdConnectDevices::execute()
{
    auto* p = processFacade();
    if (!p) return;
    p->connectAllDevices();
}

// ── CmdDisconnectDevices ────────────────────────────────────────────────────
CmdDisconnectDevices::CmdDisconnectDevices(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：断开设备
    auto* a = new QAction(QIcon("themeicons:disconnect.svg"), tr("Disconnect device"), this);
    // 中文翻译：异步断开全部已连接外设
    a->setStatusTip(tr("Asynchronously disconnect all connected peripherals"));
    setAction(a);
}
bool CmdDisconnectDevices::isEnabled() const
{
    auto* p = lcnc::Kernel::current().service<lcnc::IProcessFacade>();
    return p && p->isConnected();
}
void CmdDisconnectDevices::execute()
{
    if (auto* p = processFacade()) p->disconnectAllDevices();
}

// ── CmdManualAppendSelectedToCuttingOrder ───────────────────────────────────
CmdManualAppendSelectedToCuttingOrder::CmdManualAppendSelectedToCuttingOrder(IAppContext* ctx)
    : CommandBase(ctx)
{
    // 中文翻译：手动设置加工顺序
    auto* a = new QAction(QIcon("themeicons:cutting_plan.svg"), tr("Manually set processing sequence"), this);
    // 中文翻译：将当前选中的轮廓按选择顺序追加到切割链表
    a->setStatusTip(tr("Append the currently selected contour to the cutting list in the order of selection"));
    setAction(a);
}
bool CmdManualAppendSelectedToCuttingOrder::isEnabled() const
{
    return lcnc::Kernel::current().service<ProcessCuttingPlanService>() != nullptr;
}
void CmdManualAppendSelectedToCuttingOrder::execute()
{
    auto selSvc = lcnc::Kernel::current()
                      .services()
                      .getService<lcnc::core::SelectionService>();
    auto plan = lcnc::Kernel::current()
                    .services()
                    .getService<ProcessCuttingPlanService>();
    if (!plan) return;

    QVector<lcnc::cam::ContourId> ids;
    if (selSvc) ids = selSvc->contoursInSelectionOrder();
    if (ids.isEmpty()) {
        QMessageBox::information(
            nullptr,
            // 中文翻译：提示
            QObject::tr("Tips"),
            // 中文翻译：请先在项目树或视图中选中至少一个轮廓。
            QObject::tr("Please first select at least one profile in the project tree or view."));
        return;
    }
    // 已有手动顺序时弹覆盖确认
    if (!plan->manualContourOrder().isEmpty()) {
        const auto choice = QMessageBox::question(
            nullptr,
            // 中文翻译：覆盖切割链表
            QObject::tr("Cover cutting linked list"),
            // 中文翻译：当前切割链表已有 %1 条轮廓，是否清空并以当前选中（%2 条）重新设置？
            QObject::tr("The current cutting list has %1 contours. Do you want to clear them and reset them with the currently selected ones (%2 contours)?")
                .arg(plan->manualContourOrder().size())
                .arg(ids.size()),
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No);
        if (choice != QMessageBox::Yes) return;
        plan->clearManualOrder();
    }
    const int added = plan->appendToManualOrder(ids);
    plan->setSortStrategy(CuttingPlanSortStrategy::Manual);
    LCNC_INFO(lcnc::LogCode::Generic,
              "process.cmd: manualAppend added {} of {} selected contours",
              added, ids.size());
}

// ── CmdAutoSortCuttingOrder ────────────────────────────────────────────────
CmdAutoSortCuttingOrder::CmdAutoSortCuttingOrder(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：自动设置加工顺序
    auto* a = new QAction(QIcon("themeicons:cutting_plan.svg"), tr("Automatically set processing sequence"), this);
    // 中文翻译：若当前已选轮廓，则仅对选中轮廓按当前轴模式自动规划；否则对全部轮廓自动规划
    a->setStatusTip(tr("If the contour is currently selected, only the selected contour will be automatically planned according to the current axis mode; otherwise, all contours will be automatically planned."));
    setAction(a);
}
bool CmdAutoSortCuttingOrder::isEnabled() const
{
    return lcnc::Kernel::current().service<ProcessCuttingPlanService>() != nullptr;
}
void CmdAutoSortCuttingOrder::execute()
{
    auto plan = lcnc::Kernel::current()
                    .services()
                    .getService<ProcessCuttingPlanService>();
    auto* mod = processModule();
    if (!plan || !mod) return;

    // 已有手动顺序时弹覆盖确认（自动排序内部本就会全覆盖 m_manualContourOrder）
    if (!plan->manualContourOrder().isEmpty()) {
        const auto choice = QMessageBox::question(
            nullptr,
            // 中文翻译：覆盖切割链表
            QObject::tr("Cover cutting linked list"),
            // 中文翻译：当前切割链表已有 %1 条轮廓，是否清空并按所选方向重新自动排序？
            QObject::tr("The current cutting list already has %1 contours. Do you want to clear them and re-sort them automatically according to the selected direction?")
                .arg(plan->manualContourOrder().size()),
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No);
        if (choice != QMessageBox::Yes) return;
    }

    QString err;
    if (!plan->applyAutoSort(mod->autoSortAxis(), &err)) {
        // 中文翻译：自动排序失败
        QMessageBox::warning(nullptr, QObject::tr("Automatic sorting failed"),
                             // 中文翻译：未知错误
                             err.isEmpty() ? QObject::tr("unknown error") : err);
    }
}

// ── CmdToggleTravelPath ────────────────────────────────────────────────────
CmdToggleTravelPath::CmdToggleTravelPath(IAppContext* ctx) : CommandBase(ctx)
{
    // 中文翻译：切割路径显示
    auto* a = new QAction(QIcon("themeicons:cutting_plan.svg"), tr("Cutting path display"), this);
    // 中文翻译：在 3D 视图中用虚线显示相邻轮廓间的空程路径
    a->setStatusTip(tr("Shows the free path between adjacent contours as a dashed line in the 3D view"));
    a->setCheckable(true);
    setAction(a);
}
bool CmdToggleTravelPath::isEnabled() const
{
    return lcnc::Kernel::current().service<ProcessModule>() != nullptr;
}
void CmdToggleTravelPath::execute()
{
    auto* mod = processModule();
    if (!mod) return;
    const bool next = !mod->isTravelPathVisible();
    mod->setTravelPathVisible(next);
    if (action()) action()->setChecked(next);
    // 通过 EventBus 通知 CAM 侧的 TravelPathRenderer。
    lcnc::Kernel::current().events().publish(
        lcnc::process::events::TravelPathVisibilityToggled{next});
}

} // namespace lcnc::process
