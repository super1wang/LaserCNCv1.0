#include "modules/process/commands/commands_process.h"

#include <QAction>
#include <QFileDialog>
#include <QIcon>
#include <QInputDialog>
#include <QKeySequence>
#include <QLineEdit>
#include <QObject>
#include <QStandardPaths>

#include "core/kernel/kernel.h"
#include "core/logging/logger.h"
#include "modules/process/i_process_facade.h"
#include "modules/process/process_module.h"
#include "modules/process/ui/settings/settings_dialog.h"

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
    openSettingsDialog();
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

// ── CmdConnectController ────────────────────────────────────────────────────
CmdConnectController::CmdConnectController(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/connect.svg"), tr("连接设备"), this);
    a->setStatusTip(tr("连接到运动控制器"));
    setAction(a);
}
bool CmdConnectController::isEnabled() const
{
    return lcnc::Kernel::current().service<lcnc::IProcessFacade>() != nullptr;
}
void CmdConnectController::execute()
{
    auto* p = processFacade();
    if (!p) return;

    bool ok = false;
    const QString endpoint = QInputDialog::getText(
        nullptr,
        tr("连接控制器"),
        tr("控制器端点 (如 tcp://127.0.0.1:5000):"),
        QLineEdit::Normal,
        QString(),
        &ok);
    if (ok && !endpoint.trimmed().isEmpty())
        p->connectController(endpoint);
}

// ── CmdDisconnectController ─────────────────────────────────────────────────
CmdDisconnectController::CmdDisconnectController(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/disconnect.svg"), tr("断开设备"), this);
    a->setStatusTip(tr("断开当前控制器连接"));
    setAction(a);
}
bool CmdDisconnectController::isEnabled() const
{
    return lcnc::Kernel::current().service<lcnc::IProcessFacade>() != nullptr;
}
void CmdDisconnectController::execute()
{
    if (auto* p = processFacade()) p->disconnectController();
}

} // namespace lcnc::process
