#include "modules/process/commands/commands_process.h"

#include <QAction>
#include <QIcon>
#include <QInputDialog>
#include <QKeySequence>
#include <QLineEdit>

#include "core/kernel/kernel.h"
#include "core/logging/logger.h"
#include "modules/process/process_module.h"

namespace lcnc::process {

namespace {
/// 从 Kernel 取 ProcessModule 裸指针；未注册时返回 nullptr 并打 WARN。
ProcessModule* pm()
{
    auto* p = lcnc::Kernel::current().service<ProcessModule>();
    if (!p) {
        LCNC_WARN(lcnc::LogCode::InternalUnexpectedState,
                  "process.cmd: ProcessModule service not registered");
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
    auto* p = lcnc::Kernel::current().service<ProcessModule>();
    return p && p->state() != ProcessModule::State::EmergencyStop
             && p->state() != ProcessModule::State::Running;
}
void CmdRunStart::execute()
{
    if (auto* p = pm()) p->runStart();
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
    auto* p = lcnc::Kernel::current().service<ProcessModule>();
    return p && p->state() == ProcessModule::State::Running;
}
void CmdRunPause::execute()
{
    if (auto* p = pm()) p->runPause();
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
    auto* p = lcnc::Kernel::current().service<ProcessModule>();
    if (!p) return false;
    auto s = p->state();
    return s == ProcessModule::State::Running || s == ProcessModule::State::Paused;
}
void CmdRunStop::execute()
{
    if (auto* p = pm()) p->runStop();
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
    auto* p = lcnc::Kernel::current().service<ProcessModule>();
    return p && p->state() != ProcessModule::State::EmergencyStop;
}
void CmdEmergencyStop::execute()
{
    if (auto* p = pm()) p->emergencyStop();
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
    auto* p = lcnc::Kernel::current().service<ProcessModule>();
    return p && p->state() == ProcessModule::State::EmergencyStop;
}
void CmdResetEmergencyStop::execute()
{
    if (auto* p = pm()) p->resetEmergencyStop();
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
    auto* p = lcnc::Kernel::current().service<ProcessModule>();
    return p && p->state() != ProcessModule::State::EmergencyStop
             && p->state() != ProcessModule::State::Running;
}
void CmdHome::execute()
{
    if (auto* p = pm()) p->home();
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
    return lcnc::Kernel::current().service<ProcessModule>() != nullptr;
}
void CmdConnectController::execute()
{
    auto* p = pm();
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
    return lcnc::Kernel::current().service<ProcessModule>() != nullptr;
}
void CmdDisconnectController::execute()
{
    if (auto* p = pm()) p->disconnectController();
}

// ── CmdToggleSimulationMode ─────────────────────────────────────────────────
CmdToggleSimulationMode::CmdToggleSimulationMode(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/simulate.svg"), tr("仿真模式"), this);
    a->setCheckable(true);
    if (auto* p = lcnc::Kernel::current().service<ProcessModule>()) {
        a->setChecked(p->simulationMode());
        // 把 ProcessModule 的状态变化反向同步到 QAction，避免脱钩。
        QObject::connect(p, &ProcessModule::simulationModeChanged,
                         a, &QAction::setChecked);
    }
    a->setStatusTip(tr("仿真模式：不下发指令到控制器"));
    setAction(a);
}
bool CmdToggleSimulationMode::isEnabled() const
{
    auto* p = lcnc::Kernel::current().service<ProcessModule>();
    return p && p->state() == ProcessModule::State::Idle;
}
void CmdToggleSimulationMode::execute()
{
    auto* p = pm();
    if (!p) return;
    p->setSimulationMode(action()->isChecked());
}

} // namespace lcnc::process
