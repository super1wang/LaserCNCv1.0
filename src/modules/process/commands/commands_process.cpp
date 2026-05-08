#include "modules/process/commands/commands_process.h"

#include <QAction>
#include <QIcon>
#include <QInputDialog>
#include <QKeySequence>
#include <QLineEdit>
#include <QObject>

#include "core/kernel/kernel.h"
#include "core/logging/logger.h"
#include "modules/process/i_process_facade.h"

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
} // namespace

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
    a->setStatusTip(tr("各轴回零"));
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
    auto* a = new QAction(QIcon(":/icons/connect.svg"), tr("连接控制器"), this);
    a->setStatusTip(tr("通过对话框输入控制器地址并连接"));
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
        nullptr, tr("连接控制器"), tr("控制器地址:"),
        QLineEdit::Normal,
        QStringLiteral("tcp://127.0.0.1:5000"), &ok);
    if (!ok) return;
    p->connectController(endpoint);
}

// ── CmdDisconnectController ─────────────────────────────────────────────────
CmdDisconnectController::CmdDisconnectController(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/disconnect.svg"), tr("断开连接"), this);
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

// ── CmdToggleSimulationMode ─────────────────────────────────────────────────
CmdToggleSimulationMode::CmdToggleSimulationMode(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/simulate.svg"), tr("仿真模式"), this);
    a->setCheckable(true);
    if (auto* p = lcnc::Kernel::current().service<lcnc::IProcessFacade>()) {
        a->setChecked(p->simulationMode());
        QObject::connect(p->asQObject(), SIGNAL(simulationModeChanged(bool)),
                         a, SLOT(setChecked(bool)));
    }
    a->setStatusTip(tr("仿真模式：不下发指令到控制器"));
    setAction(a);
}
bool CmdToggleSimulationMode::isEnabled() const
{
    auto* p = lcnc::Kernel::current().service<lcnc::IProcessFacade>();
    return p && p->state() == lcnc::ProcessRunState::Idle;
}
void CmdToggleSimulationMode::execute()
{
    auto* p = processFacade();
    if (!p) return;
    p->setSimulationMode(action()->isChecked());
}

} // namespace lcnc::process
