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
#include "modules/process/Setting/process_settings_dialog.h"
#include "modules/process/device/process_device_manager.h"
#include "modules/process/ui/device/process_device_manager_dialog.h"

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

void openSettingsDialog(ProcessSettingsDialog::InitialPage page)
{
    auto* module = processModule();
    if (!module)
        return;

    auto* devices = module->deviceManager();
    const QStringList motionControllers = devices
        ? devices->availableMotionControllers()
        : QStringList{ QStringLiteral("SimulatorCMHP") };
    const QStringList laserDevices = devices
        ? devices->availableLaserDevices()
        : QStringList{ QStringLiteral("Simulator") };

    ProcessSettingsDialog dialog(module->settings(), motionControllers, laserDevices, page);
    if (dialog.exec() == QDialog::Accepted)
        module->reloadDeviceSettings();
}

void openDeviceManagerDialog(ProcessDeviceKind initialKind = ProcessDeviceKind::MotionController)
{
    auto* module = processModule();
    if (!module || !module->deviceManager())
        return;

    ProcessDeviceManagerDialog dialog(*module->deviceManager(), module->settings(), initialKind);
    QObject::connect(&dialog, &ProcessDeviceManagerDialog::settingsApplied,
                     module, &ProcessModule::reloadDeviceSettings);
    if (dialog.exec() == QDialog::Accepted)
        module->reloadDeviceSettings();
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

// ── Settings commands ───────────────────────────────────────────────────
CmdOpenProcessSettings::CmdOpenProcessSettings(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/process_param.svg"), tr("加工设置"), this);
    a->setStatusTip(tr("打开加工参数"));
    setAction(a);
}
bool CmdOpenProcessSettings::isEnabled() const
{
    return lcnc::Kernel::current().service<ProcessModule>() != nullptr;
}
void CmdOpenProcessSettings::execute()
{
    openSettingsDialog(ProcessSettingsDialog::InitialPage::Process);
}

CmdOpenMotionSettings::CmdOpenMotionSettings(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/motion_param.svg"), tr("运动参数"), this);
    a->setStatusTip(tr("打开运动控制参数"));
    setAction(a);
}
bool CmdOpenMotionSettings::isEnabled() const
{
    return lcnc::Kernel::current().service<ProcessModule>() != nullptr;
}
void CmdOpenMotionSettings::execute()
{
    openDeviceManagerDialog(ProcessDeviceKind::MotionController);
}

CmdOpenLaserSettings::CmdOpenLaserSettings(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/laser_param.svg"), tr("激光参数"), this);
    a->setStatusTip(tr("打开激光参数"));
    setAction(a);
}
bool CmdOpenLaserSettings::isEnabled() const
{
    return lcnc::Kernel::current().service<ProcessModule>() != nullptr;
}
void CmdOpenLaserSettings::execute()
{
    openDeviceManagerDialog(ProcessDeviceKind::Laser);
}

// ── CmdOpenDeviceManager ──────────────────────────────────────────────────
CmdOpenDeviceManager::CmdOpenDeviceManager(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/connect.svg"), tr("外设管理"), this);
    a->setStatusTip(tr("打开外设管理与调试界面"));
    setAction(a);
}
bool CmdOpenDeviceManager::isEnabled() const
{
    return lcnc::Kernel::current().service<ProcessModule>() != nullptr;
}
void CmdOpenDeviceManager::execute()
{
    openDeviceManagerDialog();
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
    a->setStatusTip(tr("按 Z 轴优先顺序异步回零"));
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
    a->setStatusTip(tr("异步连接当前已启用的全部外设"));
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
    p->connectDevices();
}

// ── CmdDisconnectController ─────────────────────────────────────────────────
CmdDisconnectController::CmdDisconnectController(IAppContext* ctx) : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/disconnect.svg"), tr("断开设备"), this);
    a->setStatusTip(tr("断开当前已连接的全部外设"));
    setAction(a);
}
bool CmdDisconnectController::isEnabled() const
{
    return lcnc::Kernel::current().service<lcnc::IProcessFacade>() != nullptr;
}
void CmdDisconnectController::execute()
{
    if (auto* p = processFacade()) p->disconnectDevices();
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
