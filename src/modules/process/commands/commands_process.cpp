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
    auto* a = new QAction(QIcon(":/icons/new_process.svg"), tr("新建流程"), this);
    a->setStatusTip(tr("清空当前流程树并创建新流程"));
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
    auto* a = new QAction(QIcon(":/icons/open_process.svg"), tr("加载流程"), this);
    a->setStatusTip(tr("从 TOML 文件加载流程树"));
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
        tr("加载流程"),
        QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation),
        tr("Process TOML (*.toml)"));
    if (filePath.isEmpty())
        return;
    p->loadProcess(filePath);
}

// ── CmdSaveProcess ───────────────────────────────────────────────────────
CmdSaveProcess::CmdSaveProcess(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/save_process.svg"), tr("保存流程"), this);
    a->setStatusTip(tr("保存当前流程树为 TOML 文件"));
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
        tr("保存流程"),
        QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation),
        tr("Process TOML (*.toml)"));
    if (filePath.isEmpty())
        return;
    p->saveProcess(filePath);
}

// ── CmdOpenProcessSettings ──────────────────────────────────────────────
CmdOpenProcessSettings::CmdOpenProcessSettings(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/settings.svg"), tr("设置"), this);
    a->setStatusTip(tr("打开外设和加工参数设置"));
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
    auto* a = new QAction(QIcon(":/icons/start.svg"), tr("运行"), this);
    a->setShortcut(QKeySequence(Qt::Key_F5));
    a->setStatusTip(tr("启动加工运行（仿真或控制器）"));
    setAction(a);
}
bool CmdRunStart::isEnabled() const
{
    auto* p = lcnc::Kernel::current().service<lcnc::IProcessFacade>();
    return p && p->state() != lcnc::ProcessRunState::EmergencyStop
             && p->state() != lcnc::ProcessRunState::Running;
}
void CmdRunStart::execute()
{
    if (auto* p = processFacade()) p->runStart();
}

// ── CmdRunPause ─────────────────────────────────────────────────────────────
CmdRunPause::CmdRunPause(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/pause.svg"), tr("暂停"), this);
    a->setStatusTip(tr("暂停当前加工运行"));
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
    auto* a = new QAction(QIcon(":/icons/stop.svg"), tr("停止"), this);
    a->setStatusTip(tr("停止当前加工运行"));
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

// ── CmdEmergencyStop ────────────────────────────────────────────────────────
CmdEmergencyStop::CmdEmergencyStop(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/emergency_stop.svg"), tr("急停"), this);
    a->setStatusTip(tr("立即触发急停"));
    setAction(a);
}
bool CmdEmergencyStop::isEnabled() const
{
    auto* p = lcnc::Kernel::current().service<lcnc::IProcessFacade>();
    return p && p->state() != lcnc::ProcessRunState::EmergencyStop;
}
void CmdEmergencyStop::execute()
{
    if (auto* p = processFacade()) p->emergencyStop();
}

// ── CmdResetEmergencyStop ───────────────────────────────────────────────────
CmdResetEmergencyStop::CmdResetEmergencyStop(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/reset.svg"), tr("复位急停"), this);
    a->setStatusTip(tr("解除急停状态并恢复 Idle"));
    setAction(a);
}
bool CmdResetEmergencyStop::isEnabled() const
{
    auto* p = lcnc::Kernel::current().service<lcnc::IProcessFacade>();
    return p && p->state() == lcnc::ProcessRunState::EmergencyStop;
}
void CmdResetEmergencyStop::execute()
{
    if (auto* p = processFacade()) p->resetEmergencyStop();
}

// ── CmdHome ─────────────────────────────────────────────────────────────────
CmdHome::CmdHome(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/home.svg"), tr("回零"), this);
    a->setStatusTip(tr("按 Z 轴优先顺序回零"));
    setAction(a);
}
bool CmdHome::isEnabled() const
{
    auto* p = lcnc::Kernel::current().service<lcnc::IProcessFacade>();
    return p && p->state() != lcnc::ProcessRunState::EmergencyStop
             && p->state() != lcnc::ProcessRunState::Running;
}
void CmdHome::execute()
{
    if (auto* p = processFacade()) p->home();
}

// ── CmdMoveToLoadingPosition ────────────────────────────────────────────────
CmdMoveToLoadingPosition::CmdMoveToLoadingPosition(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/move.svg"), tr("上料位"), this);
    a->setStatusTip(tr("移动至设置中定义的上料位"));
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
    auto* a = new QAction(QIcon(":/icons/move.svg"), tr("下料位"), this);
    a->setStatusTip(tr("移动至设置中定义的下料位"));
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
    auto* a = new QAction(QIcon(":/icons/connect.svg"), tr("连接设备"), this);
    a->setStatusTip(tr("异步连接全部已配置外设（运动控制器、激光器等）"));
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
    auto* a = new QAction(QIcon(":/icons/disconnect.svg"), tr("断开设备"), this);
    a->setStatusTip(tr("异步断开全部已连接外设"));
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
    auto* a = new QAction(QIcon(":/icons/cutting_plan.svg"), tr("手动设置加工顺序"), this);
    a->setStatusTip(tr("将当前选中的轮廓按选择顺序追加到切割链表"));
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
            QObject::tr("提示"),
            QObject::tr("请先在项目树或视图中选中至少一个轮廓。"));
        return;
    }
    // 已有手动顺序时弹覆盖确认
    if (!plan->manualContourOrder().isEmpty()) {
        const auto choice = QMessageBox::question(
            nullptr,
            QObject::tr("覆盖切割链表"),
            QObject::tr("当前切割链表已有 %1 条轮廓，是否清空并以当前选中（%2 条）重新设置？")
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
    auto* a = new QAction(QIcon(":/icons/cutting_plan.svg"), tr("自动设置加工顺序"), this);
    a->setStatusTip(tr("若当前已选轮廓，则仅对选中轮廓按当前轴模式自动规划；否则对全部轮廓自动规划"));
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
            QObject::tr("覆盖切割链表"),
            QObject::tr("当前切割链表已有 %1 条轮廓，是否清空并按所选方向重新自动排序？")
                .arg(plan->manualContourOrder().size()),
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No);
        if (choice != QMessageBox::Yes) return;
    }

    QString err;
    if (!plan->applyAutoSort(mod->autoSortAxis(), &err)) {
        QMessageBox::warning(nullptr, QObject::tr("自动排序失败"),
                             err.isEmpty() ? QObject::tr("未知错误") : err);
    }
}

// ── CmdToggleTravelPath ────────────────────────────────────────────────────
CmdToggleTravelPath::CmdToggleTravelPath(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/cutting_plan.svg"), tr("切割路径显示"), this);
    a->setStatusTip(tr("在 3D 视图中用虚线显示相邻轮廓间的空程路径"));
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
