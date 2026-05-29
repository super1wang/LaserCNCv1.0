#include "modules/process/process_module.h"

#include "core/kernel/i_kernel.h"
#include "core/kernel/service_registry.h"
#include "core/kinematics/i_motion_controller.h"
#include "core/logging/logger.h"
#include "modules/cam/i_cam_toolpath_provider.h"
#include "modules/process/device/i_laser_device.h"
#include "modules/process/device/i_process_io.h"
#include "modules/process/device/process_device_coordinator.h"
#include "modules/process/device/process_device_manager.h"
#include "modules/process/execution/process_execution_service.h"
#include "modules/process/execution/process_workflow_executor.h"
#include "modules/process/runtime/process_runtime.h"
#include "modules/process/toolpath/process_toolpath_service.h"
#include "modules/process/workflow/process_flow_store.h"
#include "modules/process/workflow/process_workflow_service.h"

#include <QList>
#include <QTimer>

#include <algorithm>
#include <cmath>
#include <fstream>

namespace {

QString defaultStatusText(bool simulationMode, bool connected)
{
    if (simulationMode) {
        return connected
            ? QObject::tr("仿真模式 — 控制器已连接")
            : QObject::tr("仿真模式 — 未连接");
    }

    return connected
        ? QObject::tr("控制器模式 — 待机")
        : QObject::tr("控制器模式 — 未连接");
}

double jogStepForLevel(int speedLevel)
{
    switch (speedLevel) {
    case 0:
        return 0.5;
    case 2:
        return 10.0;
    case 1:
    default:
        return 2.0;
    }
}

bool sameAxisDefinitions(const QList<MachineAxisDef>& lhs,
                         const QList<MachineAxisDef>& rhs)
{
    if (lhs.size() != rhs.size())
        return false;

    for (int i = 0; i < lhs.size(); ++i) {
        const MachineAxisDef& a = lhs.at(i);
        const MachineAxisDef& b = rhs.at(i);
        if (a.name != b.name ||
            a.motionType != b.motionType ||
            a.parentAxis != b.parentAxis ||
            std::abs(a.minVal - b.minVal) > 1e-9 ||
            std::abs(a.maxVal - b.maxVal) > 1e-9) {
            return false;
        }
    }

    return true;
}

double simulatedAxisValue(const MachineAxisDef& axis,
                          double phase,
                          int linearIndex,
                          int rotaryIndex)
{
    const double axisLimit = std::max(std::abs(axis.minVal), std::abs(axis.maxVal));

    if (axis.motionType == MachineAxisDef::Linear) {
        const double amplitude = axisLimit > 1e-6
            ? std::clamp(axisLimit * 0.12, 5.0, 60.0)
            : 20.0;
        const double freq = 0.55 + 0.18 * linearIndex;
        const double wave = std::sin(phase * freq + linearIndex * 0.8);
        if (axis.name == QStringLiteral("Z"))
            return amplitude + wave * amplitude * 0.7;
        return wave * amplitude;
    }

    const double amplitude = axisLimit >= 9000.0
        ? 90.0
        : std::max(10.0, axisLimit * 0.35);
    const double freq = 0.35 + 0.12 * rotaryIndex;
    return std::sin(phase * freq + rotaryIndex * 0.6) * amplitude;
}

} // namespace

// ── IModule ───────────────────────────────────────────────────────────────────────
lcnc::ModuleInfo ProcessModule::info() const
{
    return {
        QStringLiteral("process"),
        QStringLiteral("加工进程模块"),
        QStringLiteral("1.0.0"),
        { QStringLiteral("cam") }
    };
}

bool ProcessModule::init(lcnc::IKernel& kernel)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "ProcessModule::init begin");
    m_kernel = &kernel;
    auto svc = std::shared_ptr<ProcessModule>(this, [](ProcessModule*) {});
    kernel.services().registerService<ProcessModule>(svc);
    auto facade = std::shared_ptr<lcnc::IProcessFacade>(svc, static_cast<lcnc::IProcessFacade*>(this));
    kernel.services().registerService<lcnc::IProcessFacade>(facade);
    m_toolpathProvider = kernel.services().getService<lcnc::cam::ICamToolpathProvider>();

    // 加载持久化设置（首次运行则使用默认值）并同步到运行时状态。
    m_settings.loadDefault();
    m_simulationMode = m_settings.simulationMode();
    m_deviceManager = std::make_unique<lcnc::process::ProcessDeviceManager>();
    m_deviceManager->syncFromSettings(m_settings);
    m_deviceCoordinator = std::make_unique<lcnc::process::ProcessDeviceCoordinator>(*m_deviceManager, this);
    m_runtime = std::make_unique<lcnc::process::ProcessRuntime>(m_settings, *m_deviceManager, this);
    m_toolpathService = std::make_unique<lcnc::process::ProcessToolpathService>(m_toolpathProvider);
    m_workflowService = std::make_unique<lcnc::process::ProcessWorkflowService>(*m_toolpathService, this);
    m_executionService = std::make_unique<lcnc::process::ProcessExecutionService>(*m_deviceCoordinator, this);
    connect(m_runtime.get(), &lcnc::process::ProcessRuntime::runtimeMessage,
            this, &ProcessModule::setStatusMessage);
    connect(m_workflowService.get(), &lcnc::process::ProcessWorkflowService::workflowPrepared,
            this, [this](int contourCount, int commandCount) {
                setStatusMessage(tr("加工指令已准备: %1 个轮廓 / %2 条指令")
                                     .arg(contourCount)
                                     .arg(commandCount));
            });
    connect(m_workflowService.get(), &lcnc::process::ProcessWorkflowService::workflowRejected,
            this, &ProcessModule::setStatusMessage);
    m_workflowExecutor = std::make_unique<lcnc::process::ProcessWorkflowExecutor>(this);
    m_workflowExecutor->setToolpathSnapshotProvider([this] {
        lcnc::process::ProcessToolpathSnapshot snapshot;
        if (!m_toolpathService || !m_toolpathProvider) {
            snapshot.description = tr("未连接 CAM 刀路服务");
            return snapshot;
        }

        const auto camSnapshot = m_toolpathService->refreshSnapshot();
        snapshot.available = camSnapshot.hasEnabledContours();
        snapshot.contourCount = camSnapshot.contours.size();
        snapshot.totalPointCount = camSnapshot.totalPointCount();
        snapshot.description = camSnapshot.description;
        return snapshot;
    });
    connect(m_workflowExecutor.get(), &lcnc::process::ProcessWorkflowExecutor::messageLogged,
            this, &ProcessModule::setStatusMessage);
        connect(m_workflowExecutor.get(), &lcnc::process::ProcessWorkflowExecutor::nodeStateChanged,
            this, [this](const QString&) { emit processFlowChanged(); });
        connect(m_workflowExecutor.get(), &lcnc::process::ProcessWorkflowExecutor::nodeStarted,
            this, [this](const QString&) { emit processFlowChanged(); });
        connect(m_workflowExecutor.get(), &lcnc::process::ProcessWorkflowExecutor::nodeFinished,
            this, [this](const QString&) { emit processFlowChanged(); });
        connect(m_workflowExecutor.get(), &lcnc::process::ProcessWorkflowExecutor::nodeFailed,
            this, [this](const QString&, const QString& message) {
            emit processFlowChanged();
            setState(State::Error, tr("流程节点失败: %1").arg(message));
            });
        connect(m_workflowExecutor.get(), &lcnc::process::ProcessWorkflowExecutor::axisPositionRequested,
            this, &ProcessModule::setAxisPosition);
        connect(m_workflowExecutor.get(), &lcnc::process::ProcessWorkflowExecutor::laserEnergyRequested,
                this, [this](double value) {
                    if (!m_deviceManager || !m_deviceManager->laserDevice())
                        return;
                    QString errorMessage;
                    if (!m_deviceManager->laserDevice()->setEnergy(value, &errorMessage)) {
                        LCNC_WARN(lcnc::LogCode::Generic,
                                  "process.executor: set laser energy failed: {}",
                                  errorMessage.toStdString());
                    }
                });
        connect(m_workflowExecutor.get(), &lcnc::process::ProcessWorkflowExecutor::digitalOutputRequested,
                this, [this](const QString& channel, bool value) {
                    if (!m_deviceManager || !m_deviceManager->processIo())
                        return;
                    QString errorMessage;
                    if (!m_deviceManager->processIo()->setDigitalOutput(channel, value, &errorMessage)) {
                        LCNC_WARN(lcnc::LogCode::Generic,
                                  "process.executor: set digital output failed: {}",
                                  errorMessage.toStdString());
                    }
                });
        connect(m_workflowExecutor.get(), &lcnc::process::ProcessWorkflowExecutor::workflowFinished,
            this, [this] {
            m_simTimer->stop();
            setState(State::Idle, tr("流程运行完成"));
            emit processFlowChanged();
            });
    setStatusMessage(defaultStatusText(m_simulationMode, m_connected));

    QString motionError;
    if (!switchMotionControllerFromSettings(&motionError)) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "ProcessModule: active motion controller setup failed: {}",
                  motionError.toStdString());
    }
    if (m_runtime)
        m_runtime->initialize();

    m_initialized = true;
    LCNC_INFO(lcnc::LogCode::Generic, "ProcessModule init done");
    return true;
}

bool ProcessModule::start()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "ProcessModule::start (no-op)");
    return true;
}

void ProcessModule::stop()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "ProcessModule::stop begin");
    if (!m_initialized) return;
    if (m_motionController)
        m_motionController->stop();
    if (m_kernel)
        m_kernel->services().unregisterService<lcnc::IMotionController>();
    if (m_workflowExecutor)
        m_workflowExecutor->stop();
    m_initialized = false;
    LCNC_INFO(lcnc::LogCode::Generic, "ProcessModule stop done");
}

ProcessModule::ProcessModule(QObject* parent)
    : QObject(parent)
{
    initializeAxisPositions();

    m_simTimer = new QTimer(this);
    m_simTimer->setInterval(100);
    connect(m_simTimer, &QTimer::timeout, this, &ProcessModule::onSimulationTick);

    setStatusMessage(defaultStatusText(m_simulationMode, m_connected));
}

bool ProcessModule::connectController(const QString& endpoint)
{
    if (endpoint.trimmed().isEmpty()) {
        setState(State::Error, tr("控制器地址不能为空"));
        return false;
    }

    const bool wasConnected = m_connected;
    m_connected = true;
    if (!wasConnected)
        emit connectionChanged(true);

    if (m_simulationMode) {
        m_simulationMode = false;
        m_settings.setSimulationMode(false);
        emit simulationModeChanged(false);
    }

    m_settings.setControllerEndpoint(endpoint);
    if (m_motionController)
        m_motionController->start();

    if (m_state == State::Error) {
        m_state = State::Idle;
        emit stateChanged(m_state);
    }

    setStatusMessage(tr("控制器已连接: %1").arg(endpoint));
    return true;
}

void ProcessModule::disconnectController()
{
    const bool wasConnected = m_connected;
    m_connected = false;
    m_simTimer->stop();
    m_state = State::Idle;
    if (wasConnected)
        emit connectionChanged(false);
    emit stateChanged(m_state);
    setStatusMessage(defaultStatusText(m_simulationMode, m_connected));
}

bool ProcessModule::isConnected() const
{
    return m_connected;
}

void ProcessModule::setSimulationMode(bool on)
{
    if (m_simulationMode == on)
        return;

    m_simulationMode = on;
    m_settings.setSimulationMode(on);
    emit simulationModeChanged(on);

    if (on && m_state == State::Error) {
        m_state = State::Idle;
        emit stateChanged(m_state);
    }

    setStatusMessage(defaultStatusText(m_simulationMode, m_connected));
}

bool ProcessModule::simulationMode() const
{
    return m_simulationMode;
}

void ProcessModule::setAxisDefinitions(const QList<MachineAxisDef>& axes)
{
    if (sameAxisDefinitions(m_axisDefinitions, axes))
        return;

    m_axisDefinitions = axes;
    m_simPhase = 0.0;
    initializeAxisPositions();
}

void ProcessModule::jog(const QString& axisName, int direction, int speedLevel)
{
    if (axisName.trimmed().isEmpty() || direction == 0 || m_state == State::EmergencyStop)
        return;

    const double delta = jogStepForLevel(speedLevel) * (direction > 0 ? 1.0 : -1.0);
    if (!m_simulationMode && m_motionController)
        m_motionController->jog(axisName, delta);
    setAxisPosition(axisName, m_axisPositions.value(axisName) + delta);
    setStatusMessage(tr("点动 %1 轴 %2").arg(
        axisName,
        direction > 0 ? tr("正向") : tr("负向")));
}

void ProcessModule::home()
{
    if (m_state == State::EmergencyStop)
        return;

    initializeAxisPositions();
    if (!m_simulationMode && m_motionController)
        m_motionController->home();
    m_simPhase = 0.0;
    setStatusMessage(tr("回零完成"));
}

void ProcessModule::runStart()
{
    if (m_state == State::EmergencyStop) {
        setStatusMessage(tr("急停状态，复位后才能运行"));
        return;
    }

    if (!m_simulationMode && !m_connected) {
        setState(State::Error, tr("未连接控制器，无法运行"));
        return;
    }

    if (m_state == State::Paused) {
        if (m_workflowExecutor)
            m_workflowExecutor->resume();
        if (m_simulationMode)
            m_simTimer->start(std::max(30, static_cast<int>(100.0 / std::max(0.1, m_feedOverride))));
        setState(State::Running, tr("运行继续"));
        return;
    }

    if (m_workflowService && m_toolpathProvider && m_toolpathProvider->hasToolpath()) {
        QString prepareError;
        if (!m_workflowService->prepare(m_settings, &prepareError)) {
            LCNC_WARN(lcnc::LogCode::Generic,
                      "ProcessModule: process workflow prepare failed: {}",
                      prepareError.toStdString());
        }
    }

    if (m_workflowExecutor) {
        QString errorMessage;
        if (!m_workflowExecutor->start(m_processFlowDocument, &errorMessage)) {
            setState(State::Error, tr("流程启动失败: %1").arg(errorMessage));
            return;
        }
        if (m_workflowExecutor->state() == lcnc::process::ProcessWorkflowExecutor::State::Error)
            return;
        if (m_workflowExecutor->state() == lcnc::process::ProcessWorkflowExecutor::State::Idle)
            return;
    }

    if (m_simulationMode) {
        const int intervalMs = std::max(30, static_cast<int>(100.0 / std::max(0.1, m_feedOverride)));
        m_simTimer->start(intervalMs);
    }

    setState(State::Running,
             m_simulationMode ? tr("仿真运行中") : tr("控制器运行中"));
}

void ProcessModule::runPause()
{
    if (m_state != State::Running)
        return;

    m_simTimer->stop();
    if (m_workflowExecutor)
        m_workflowExecutor->pause();
    setState(State::Paused, tr("运行已暂停"));
}

void ProcessModule::runStop()
{
    m_simTimer->stop();
    if (m_workflowExecutor)
        m_workflowExecutor->stop();
    setState(State::Idle,
             m_simulationMode ? tr("仿真已停止") : tr("运行已停止"));
}

void ProcessModule::emergencyStop()
{
    m_simTimer->stop();
    if (m_workflowExecutor)
        m_workflowExecutor->emergencyStop();
    if (m_motionController)
        m_motionController->emergencyStop();
    setState(State::EmergencyStop, tr("急停已触发"));
}

void ProcessModule::resetEmergencyStop()
{
    if (m_state != State::EmergencyStop)
        return;
    setState(State::Idle, defaultStatusText(m_simulationMode, m_connected));
}

void ProcessModule::newProcess()
{
    m_processFlowDocument.clear();
    m_processFlowDocument.markClean();
    emit processFlowChanged();
    setStatusMessage(tr("已新建流程"));
}

bool ProcessModule::loadProcess(const QString& filePath)
{
    if (filePath.trimmed().isEmpty()) {
        setStatusMessage(tr("流程文件路径为空"));
        return false;
    }

    QString errorMessage;
    if (!lcnc::process::ProcessFlowStore::loadFromFile(filePath, m_processFlowDocument, &errorMessage)) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "ProcessModule: load process failed: {}",
                  errorMessage.toStdString());
        setStatusMessage(tr("加载流程失败: %1").arg(errorMessage));
        return false;
    }

    emit processFlowChanged();
    setStatusMessage(tr("已加载流程: %1").arg(filePath));
    return true;
}

bool ProcessModule::saveProcess(const QString& filePath)
{
    if (filePath.trimmed().isEmpty()) {
        setStatusMessage(tr("流程文件路径为空"));
        return false;
    }

    QString errorMessage;
    if (!lcnc::process::ProcessFlowStore::saveToFile(filePath, m_processFlowDocument, &errorMessage)) {
        setStatusMessage(tr("保存流程失败: %1").arg(errorMessage));
        return false;
    }

    m_processFlowDocument.markClean();
    setStatusMessage(tr("已保存流程: %1").arg(filePath));
    return true;
}

ProcessModule::State ProcessModule::state() const
{
    return m_state;
}

QMap<QString, double> ProcessModule::currentAxisPositions() const
{
    return m_axisPositions;
}

void ProcessModule::setAxisPosition(const QString& axisName, double value)
{
    const double oldValue = m_axisPositions.value(axisName, 0.0);
    if (m_axisPositions.contains(axisName) && std::abs(oldValue - value) < 1e-9)
        return;

    m_axisPositions.insert(axisName, value);
    emit axisPositionChanged(axisName, value);
}

void ProcessModule::setAxisPositions(const QMap<QString, double>& positions)
{
    for (auto it = positions.cbegin(); it != positions.cend(); ++it)
        setAxisPosition(it.key(), it.value());
}

void ProcessModule::setFeedOverride(double factor)
{
    const double clamped = std::clamp(factor, 0.0, 2.0);
    if (std::abs(m_feedOverride - clamped) < 1e-9)
        return;

    m_feedOverride = clamped;
    emit feedOverrideChanged(m_feedOverride);

    if (m_simulationMode && m_simTimer->isActive()) {
        const int intervalMs = std::max(30, static_cast<int>(100.0 / std::max(0.1, m_feedOverride)));
        m_simTimer->setInterval(intervalMs);
    }

    setStatusMessage(tr("进给倍率: %1%").arg(qRound(m_feedOverride * 100.0)));
}

double ProcessModule::feedOverride() const
{
    return m_feedOverride;
}

void ProcessModule::reloadDeviceSettings()
{
    if (!m_deviceManager)
        m_deviceManager = std::make_unique<lcnc::process::ProcessDeviceManager>();
    m_deviceManager->syncFromSettings(m_settings);

    QString motionError;
    if (!switchMotionControllerFromSettings(&motionError)) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "ProcessModule: reload motion controller failed: {}",
                  motionError.toStdString());
    }

    if (m_runtime) {
        QString runtimeError;
        if (!m_runtime->refreshSettings(&runtimeError)) {
            LCNC_WARN(lcnc::LogCode::Generic,
                      "ProcessModule: runtime settings refresh failed: {}",
                      runtimeError.toStdString());
        }
    }

    const bool nextSimulationMode = m_settings.simulationMode();
    if (m_simulationMode != nextSimulationMode) {
        m_simulationMode = nextSimulationMode;
        emit simulationModeChanged(m_simulationMode);
    }

    setStatusMessage(tr("设备配置已更新: %1 / %2").arg(
        m_deviceManager->activeMotionController(),
        m_deviceManager->activeLaserDevice()));
}

QString ProcessModule::statusMessage() const
{
    return m_statusMessage;
}

void ProcessModule::onSimulationTick()
{
    if (m_state != State::Running || !m_simulationMode)
        return;

    m_simPhase += 0.08 * std::max(0.1, m_feedOverride);

    int linearIndex = 0;
    int rotaryIndex = 0;
    for (const MachineAxisDef& axis : m_axisDefinitions) {
        if (axis.name == QStringLiteral("BASE"))
            continue;

        const double value = simulatedAxisValue(axis, m_simPhase, linearIndex, rotaryIndex);
        setAxisPosition(axis.name, value);

        if (axis.motionType == MachineAxisDef::Linear)
            ++linearIndex;
        else
            ++rotaryIndex;
    }
}

void ProcessModule::initializeAxisPositions()
{
    if (m_axisDefinitions.isEmpty()) {
        m_axisPositions.clear();
        return;
    }

    QMap<QString, double> nextPositions;
    for (const MachineAxisDef& axis : m_axisDefinitions) {
        if (axis.name == QStringLiteral("BASE"))
            continue;
        nextPositions.insert(axis.name, 0.0);
    }

    m_axisPositions = nextPositions;
    for (auto it = m_axisPositions.cbegin(); it != m_axisPositions.cend(); ++it)
        emit axisPositionChanged(it.key(), it.value());
}

bool ProcessModule::switchMotionControllerFromSettings(QString* errorMessage)
{
    if (!m_deviceManager) {
        if (errorMessage)
            *errorMessage = tr("设备管理器未初始化");
        return false;
    }

    if (m_state == State::Running || m_state == State::Paused || m_state == State::EmergencyStop) {
        if (errorMessage)
            *errorMessage = tr("运行中、暂停或急停状态禁止切换运动控制器");
        return false;
    }

    QString createError;
    auto nextController = m_deviceManager->createMotionController(m_settings, this, &createError);
    if (!nextController) {
        if (errorMessage)
            *errorMessage = createError;
        return false;
    }

    const QString nextId = nextController->id();
    if (m_motionController && m_motionController->id().compare(nextId, Qt::CaseInsensitive) == 0) {
        registerActiveMotionControllerService();
        if (m_deviceCoordinator)
            m_deviceCoordinator->setMotionController(m_motionController.get());
        return true;
    }

    if (m_motionController)
        m_motionController->stop();
    m_motionController = std::move(nextController);

    if (!m_motionController->start()) {
        if (errorMessage)
            *errorMessage = tr("运动控制器启动失败: %1").arg(nextId);
        registerActiveMotionControllerService();
        return false;
    }

    registerActiveMotionControllerService();
    if (m_deviceCoordinator)
        m_deviceCoordinator->setMotionController(m_motionController.get());
    LCNC_INFO(lcnc::LogCode::Generic,
              "ProcessModule: active motion controller '{}' registered",
              nextId.toStdString());
    return true;
}

void ProcessModule::registerActiveMotionControllerService()
{
    if (!m_kernel || !m_motionController)
        return;
    auto motionSvc = std::shared_ptr<lcnc::IMotionController>(
        m_motionController.get(), [](lcnc::IMotionController*) {});
    m_kernel->services().registerService<lcnc::IMotionController>(motionSvc);
}

void ProcessModule::setState(State state, const QString& statusMessage)
{
    if (m_state != state) {
        m_state = state;
        emit stateChanged(m_state);
    }

    if (m_runtime)
        m_runtime->applyFacadeState(state, statusMessage);

    setStatusMessage(statusMessage);
}

void ProcessModule::setStatusMessage(const QString& message)
{
    if (m_statusMessage == message)
        return;

    m_statusMessage = message;
    emit statusMessageChanged(m_statusMessage);
}
