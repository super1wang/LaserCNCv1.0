#include "modules/process/process_module.h"

#include "core/kernel/i_kernel.h"
#include "core/kernel/kernel.h"
#include "core/kernel/service_registry.h"
#include "core/kinematics/machine_configuration_service.h"
#include "core/kinematics/i_motion_controller.h"
#include "core/logging/logger.h"
#include "core/task/task_manager.h"
#include "core/task/task_progress.h"
#include "modules/cam/i_cam_toolpath_provider.h"
#include "modules/process/device/i_laser_device.h"
#include "modules/process/device/i_process_io.h"
#include "modules/process/device/process_device_coordinator.h"
#include "modules/process/device/process_device_manager.h"
#include "modules/process/execution/process_execution_service.h"
#include "modules/process/execution/process_workflow_executor.h"
#include "modules/process/instructions/i_controller_translator.h"
#include "modules/process/monitor/process_monitor_service.h"
#include "modules/process/runtime/process_runtime.h"
#include "modules/process/toolpath/process_toolpath_service.h"
#include "modules/process/workflow/process_flow_store.h"
#include "modules/process/workflow/process_workflow_service.h"

#include <QList>
#include <QFutureWatcher>
#include <QMetaObject>
#include <QPointer>
#include <QTimer>
#include <QtConcurrent>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

namespace {

constexpr int kCuttingProgramBuffer = 9;

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

QString processStateText(lcnc::ProcessRunState state)
{
    switch (state) {
    case lcnc::ProcessRunState::Idle:
        return QObject::tr("空闲");
    case lcnc::ProcessRunState::Running:
        return QObject::tr("运行中");
    case lcnc::ProcessRunState::Paused:
        return QObject::tr("暂停");
    case lcnc::ProcessRunState::Error:
        return QObject::tr("错误");
    case lcnc::ProcessRunState::EmergencyStop:
        return QObject::tr("急停");
    }
    return QObject::tr("未知");
}

QString logLevelForMessage(const QString& message)
{
    if (message.contains(QObject::tr("失败")) ||
        message.contains(QObject::tr("错误")) ||
        message.contains(QObject::tr("无法")) ||
        message.contains(QObject::tr("急停"))) {
        return QStringLiteral("error");
    }
    if (message.contains(QObject::tr("警告")) || message.contains(QStringLiteral("warning"), Qt::CaseInsensitive))
        return QStringLiteral("warn");
    if (message.contains(QObject::tr("流程")) || message.contains(QObject::tr("加工")) || message.contains(QObject::tr("切割")))
        return QStringLiteral("process");
    return QStringLiteral("info");
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

QString settingChannel(const lcnc::ProcessSettings& settings,
                       const QString& key,
                       const QString& fallback)
{
    const QString value = settings.uiSettingValue(key, fallback).trimmed();
    return value.isEmpty() ? fallback : value;
}

QList<lcnc::process::ProcessMonitorOutputChannel> commonDigitalOutputPresets(const lcnc::ProcessSettings& settings)
{
    return {
        { QObject::tr("激光"), settingChannel(settings, QStringLiteral("Setting_IOIndex.lineEdit_DigitalOUT_aLaser"), settings.ioDefaultChannel()) },
        { QObject::tr("吹气"), settingChannel(settings, QStringLiteral("Setting_IOIndex.lineEdit_DigitalOUT_aBlow"), QStringLiteral("DO1")) },
        { QObject::tr("夹头"), settingChannel(settings, QStringLiteral("Setting_IOIndex.lineEdit_DigitalOUT_aChuck"), QStringLiteral("DO2")) },
        { QObject::tr("水冷"), settingChannel(settings, QStringLiteral("Setting_IOIndex.lineEdit_DigitalOUT_aWater"), QStringLiteral("DO3")) },
        { QObject::tr("气泵"), settingChannel(settings, QStringLiteral("Setting_IOIndex.lineEdit_DigitalOUT_aPump"), QStringLiteral("DO4")) }
    };
}

QString outputChannelFor(const lcnc::ProcessSettings& settings, const QString& outputName)
{
    for (const lcnc::process::ProcessMonitorOutputChannel& preset : commonDigitalOutputPresets(settings)) {
        if (preset.name == outputName)
            return preset.channel;
    }
    return outputName.trimmed();
}

QStringList homeOrderForAxes(const QList<MachineAxisDef>& axes)
{
    QStringList order;
    auto appendAxis = [&order](const QString& axis) {
        const QString key = axis.trimmed().toUpper();
        if (!key.isEmpty() && key != QStringLiteral("BASE") && !order.contains(key))
            order.append(key);
    };
    appendAxis(QStringLiteral("Z"));
    for (const MachineAxisDef& axis : axes)
        appendAxis(axis.name);
    return order;
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
    m_machineConfig = kernel.services().getService<lcnc::MachineConfigurationService>();
    if (m_machineConfig) {
        connect(m_machineConfig.get(), &lcnc::MachineConfigurationService::machineConfigurationChanged,
                this, [this] {
                    if (m_machineConfig)
                        setAxisDefinitions(m_machineConfig->axisDefinitions());
                });
        setAxisDefinitions(m_machineConfig->axisDefinitions());
    }

    // 加载持久化设置（首次运行则使用默认值）并同步到运行时状态。
    m_settings.loadDefault();
    for (const lcnc::process::ProcessMonitorOutputChannel& preset : commonDigitalOutputPresets(m_settings))
        m_digitalOutputs.insert(preset.name, false);
    m_simulationMode = m_settings.simulationMode();
    m_deviceManager = std::make_unique<lcnc::process::ProcessDeviceManager>();
    m_deviceManager->syncFromSettings(m_settings);
    m_deviceCoordinator = std::make_unique<lcnc::process::ProcessDeviceCoordinator>(*m_deviceManager, this);
    m_monitorService = std::make_unique<lcnc::process::ProcessMonitorService>(this);
    m_runtime = std::make_unique<lcnc::process::ProcessRuntime>(m_settings, *m_deviceManager, this);
    m_toolpathService = std::make_unique<lcnc::process::ProcessToolpathService>(m_toolpathProvider);
    m_workflowService = std::make_unique<lcnc::process::ProcessWorkflowService>(*m_toolpathService, this);
    m_executionService = std::make_unique<lcnc::process::ProcessExecutionService>(*m_deviceCoordinator, this);
        connect(m_executionService.get(), &lcnc::process::ProcessExecutionService::executionMessage,
            this, &ProcessModule::setStatusMessage);
        connect(m_executionService.get(), &lcnc::process::ProcessExecutionService::executionError,
            this, [this](const QString& message) { setState(State::Error, message); });
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
    m_workflowExecutor->setCuttingExecutor([this](bool dryRun, QString* errorMessage) {
        return executeCuttingCommandBuffer(dryRun, errorMessage);
    });
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
            safeStopProcessOutputs();
            setState(State::Idle, tr("流程运行完成"));
            emit processFlowChanged();
            });

    if (m_monitorService) {
        m_monitorService->setContextProvider([this]() {
            lcnc::process::ProcessMonitorPollContext context;
            context.connected = m_connected;
            context.simulationMode = m_simulationMode;
            context.monitoringEnabled = m_settings.monitorEnabled();
            context.intervalMs = m_settings.monitorIntervalMs();
            context.settings = m_settings.monitorSettings();
            context.cachedAxisPositions = m_axisPositions;
            context.outputChannels = commonDigitalOutputPresets(m_settings);
            context.interlockChannel = settingChannel(m_settings, QStringLiteral("Setting_IOIndex.lineEdit_DigitalIN_aInterLock"), QStringLiteral("0.3"));
            context.safetyLightCurtainChannel = settingChannel(m_settings, QStringLiteral("Setting_IOIndex.lineEdit_DigitalIN_aSafetyLightCurtain"), QStringLiteral("0.7"));
            context.pressureMonitorChannel = settingChannel(m_settings, QStringLiteral("Setting_IOIndex.lineEdit_DigitalIN_aPressureMonitor"), QStringLiteral("0.2"));
            context.waterLeakageChannel = settingChannel(m_settings, QStringLiteral("Setting_IOIndex.lineEdit_DigitalIN_aWaterLeakageMonitor"), QStringLiteral("0.4"));
            context.waterTankChannel = settingChannel(m_settings, QStringLiteral("Setting_IOIndex.lineEdit_DigitalIN_aWaterTankMonitor"), QStringLiteral("0.6"));
            context.waterPressureChannel = settingChannel(m_settings, QStringLiteral("Setting_IOIndex.lineEdit_AnalogIN_aWaterPressure"), QStringLiteral("1"));
            context.waterLevelChannel = settingChannel(m_settings, QStringLiteral("Setting_IOIndex.lineEdit_AnalogIN_aWaterLevel"), QStringLiteral("0"));
            context.motionController = m_motionController.get();
            context.processIo = m_deviceManager ? m_deviceManager->processIo() : nullptr;
            return context;
        });
        connect(m_monitorService.get(), &lcnc::process::ProcessMonitorService::snapshotUpdated,
                this, &ProcessModule::applyMonitorSnapshot);
        connect(m_monitorService.get(), &lcnc::process::ProcessMonitorService::alarmRaised,
                this, &ProcessModule::handleMonitorAlarmRaised);
        connect(m_monitorService.get(), &lcnc::process::ProcessMonitorService::alarmCleared,
                this, [this](const lcnc::process::ProcessMonitorAlarm& alarm) {
                    emit processLogMessage(QStringLiteral("info"), tr("监控恢复: %1").arg(alarm.title));
                    if (m_state == State::Paused && alarm.action == lcnc::ProcessMonitorFaultAction::Pause)
                        setStatusMessage(tr("监控恢复: %1，可继续加工").arg(alarm.title));
                });
        m_monitorService->start();
    }

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
    stopDeviceAcquisition();
    if (m_monitorService)
        m_monitorService->stop();
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

    m_settings.setControllerEndpoint(endpoint);

    if (m_simulationMode) {
        m_simulationMode = false;
        m_settings.setSimulationMode(false);
        emit simulationModeChanged(false);
    }

    return connectDevices();
}

bool ProcessModule::connectDevices()
{
    if (!m_deviceManager) {
        setState(State::Error, tr("设备管理器未初始化"));
        return false;
    }

    // 已连接情况下不再重复连接控制器，避免底层二次连接。
    if (m_connected && m_motionController && m_motionController->isRunning()) {
        setStatusMessage(tr("外设已连接"));
        return true;
    }

    if (m_runtime) {
        QString reason;
        if (!m_runtime->canChangeConfiguration(&reason)) {
            setStatusMessage(reason);
            return false;
        }
    }

    auto* kernel = lcnc::Kernel::tryCurrent();
    auto* taskManager = kernel ? kernel->taskManager() : nullptr;
    if (!taskManager) {
        QStringList errors;
        const bool ok = m_deviceManager->connectAllDevices(&errors);
        const bool wasConnected = m_connected;
        m_connected = ok;
        if (m_connected != wasConnected)
            emit connectionChanged(m_connected);
        if (ok) {
            if (m_state == State::Error) {
                m_state = State::Idle;
                emit stateChanged(m_state);
            }
            startDeviceAcquisition();
            setStatusMessage(tr("外设已连接"));
        } else {
            stopDeviceAcquisition();
            setState(State::Error, tr("外设连接失败: %1").arg(errors.join(QStringLiteral("; "))));
        }
        return ok;
    }

    setStatusMessage(tr("正在连接外设..."));
    QPointer<ProcessModule> self(this);
    taskManager->run(tr("连接外设"), [self](TaskProgress* progress) {
        if (!self || !self->m_deviceManager)
            return;

        QStringList errors;
        bool ok = self->m_deviceManager->connectAllDevices(
            &errors,
            [progress](int current, int total, const QString& step) {
                if (!progress)
                    return;
                progress->setRange(0, std::max(total, 1));
                progress->setValue(current);
                progress->setStepName(step);
            });

        QMetaObject::invokeMethod(self, [self, ok, errors]() {
            if (!self)
                return;
            const bool wasConnected = self->m_connected;
            self->m_connected = ok;
            if (self->m_connected != wasConnected)
                emit self->connectionChanged(self->m_connected);

            if (ok) {
                if (self->m_state == State::Error) {
                    self->m_state = State::Idle;
                    emit self->stateChanged(self->m_state);
                }
                self->startDeviceAcquisition();
                self->setStatusMessage(self->tr("外设已连接"));
            } else {
                self->stopDeviceAcquisition();
                self->setState(State::Error,
                               self->tr("外设连接失败: %1").arg(errors.join(QStringLiteral("; "))));
            }
        }, Qt::QueuedConnection);

        if (!ok)
            throw std::runtime_error(errors.join(QStringLiteral("; ")).toStdString());
    });

    return true;
}

void ProcessModule::disconnectDevices()
{
    stopDeviceAcquisition();
    if (m_deviceManager)
        m_deviceManager->disconnectAllDevices();

    const bool wasConnected = m_connected;
    m_connected = false;
    m_simTimer->stop();
    m_state = State::Idle;
    if (wasConnected)
        emit connectionChanged(false);
    emit stateChanged(m_state);
    setStatusMessage(defaultStatusText(m_simulationMode, m_connected));
}

void ProcessModule::disconnectController()
{
    disconnectDevices();
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
    initializeAxisEnabledStates();
}

void ProcessModule::jog(const QString& axisName, int direction, int speedLevel, double distance)
{
    if (axisName.trimmed().isEmpty() || direction == 0 || m_state == State::EmergencyStop)
        return;

    const QString normalizedAxis = axisName.trimmed().toUpper();
    if (!m_axisEnabled.value(normalizedAxis, true)) {
        setStatusMessage(tr("%1 轴未使能，点动已忽略").arg(normalizedAxis));
        return;
    }

    // 未连接设备时不触发点动，避免运动控制器被点动操作隐式启动连接。
    if (!m_simulationMode && !m_connected) {
        setStatusMessage(tr("设备未连接，点动已忽略"));
        return;
    }

    const double step = distance > 1e-9 ? distance : jogStepForLevel(speedLevel);
    const double delta = step * (direction > 0 ? 1.0 : -1.0);
    if (m_motionController && !m_motionController->jog(normalizedAxis, delta)) {
        setStatusMessage(tr("%1 轴点动失败").arg(normalizedAxis));
        return;
    }
    setAxisPosition(normalizedAxis, m_axisPositions.value(normalizedAxis) + delta);
    setStatusMessage(tr("点动 %1 轴 %2").arg(
        normalizedAxis,
        direction > 0 ? tr("正向") : tr("负向")));
}

void ProcessModule::home()
{
    if (m_state == State::EmergencyStop)
        return;

    const QStringList axes = homeOrderForAxes(m_axisDefinitions);
    if (axes.isEmpty()) {
        setStatusMessage(tr("没有可回零的轴系"));
        return;
    }

    auto homeOneAxis = [this](const QString& axis) -> bool {
        const bool locallyHomed = std::abs(m_axisPositions.value(axis, 0.0)) < 1e-6;
        const bool controllerHomed = m_motionController ? m_motionController->axisHomed(axis) : false;
        bool ok = true;
        if (!locallyHomed && !controllerHomed && m_motionController)
            ok = m_motionController->home(axis);
        setAxisPosition(axis, 0.0);
        return ok;
    };

    auto* kernel = lcnc::Kernel::tryCurrent();
    auto* taskManager = kernel ? kernel->taskManager() : nullptr;
    if (!taskManager) {
        for (const QString& axis : axes)
            homeOneAxis(axis);
        m_simPhase = 0.0;
        setStatusMessage(tr("回零完成"));
        return;
    }

    setStatusMessage(tr("正在按轴系顺序回零..."));
    QPointer<ProcessModule> self(this);
    taskManager->run(tr("轴系回零"), [self, axes](TaskProgress* progress) {
        if (!self)
            return;
        const int total = axes.size();
        for (int index = 0; index < total; ++index) {
            const QString axis = axes.at(index);
            if (progress) {
                progress->setRange(0, total);
                progress->setValue(index);
                progress->setStepName(QObject::tr("回零 %1 轴").arg(axis));
            }
            bool ok = false;
            QMetaObject::invokeMethod(self, [self, axis, &ok]() {
                if (!self)
                    return;
                const bool locallyHomed = std::abs(self->m_axisPositions.value(axis, 0.0)) < 1e-6;
                const bool controllerHomed = self->m_motionController ? self->m_motionController->axisHomed(axis) : false;
                ok = true;
                if (!locallyHomed && !controllerHomed && self->m_motionController)
                    ok = self->m_motionController->home(axis);
                self->setAxisPosition(axis, 0.0);
            }, Qt::BlockingQueuedConnection);

            if (!ok)
                throw std::runtime_error(QObject::tr("%1 轴回零失败").arg(axis).toStdString());
        }
        if (progress) {
            progress->setValue(total);
            progress->setStepName(QObject::tr("回零完成"));
        }
        QMetaObject::invokeMethod(self, [self]() {
            if (!self)
                return;
            self->m_simPhase = 0.0;
            self->setStatusMessage(self->tr("回零完成"));
        }, Qt::QueuedConnection);
    });
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
        if (!m_simulationMode && m_motionController && m_motionController->supportsProgramPause()) {
            QString errorMessage;
            if (!m_motionController->resumeProgram(kCuttingProgramBuffer, &errorMessage)) {
                setState(State::Error, tr("控制器继续失败: %1").arg(errorMessage));
                return;
            }
        }
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

    if (!m_simulationMode && m_motionController && !m_motionController->supportsProgramPause()) {
        if (m_workflowExecutor)
            m_workflowExecutor->stop();
        if (m_motionController)
            m_motionController->emergencyStop();
        safeStopProcessOutputs();
        setState(State::Error, tr("当前控制器不支持暂停，已降级为安全停止"));
        return;
    }

    if (!m_simulationMode && m_motionController) {
        QString errorMessage;
        if (!m_motionController->pauseProgram(kCuttingProgramBuffer, &errorMessage)) {
            setState(State::Error, tr("控制器暂停失败: %1").arg(errorMessage));
            return;
        }
    }

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
    if (m_motionController) {
        m_motionController->emergencyStop();
        if (m_connected || m_simulationMode)
            m_motionController->start();
    }
    safeStopProcessOutputs();
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
    safeStopProcessOutputs();
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

void ProcessModule::setAxisEnabled(const QString& axisName, bool enabled)
{
    const QString normalizedAxis = axisName.trimmed().toUpper();
    if (normalizedAxis.isEmpty())
        return;

    if (m_motionController && !m_motionController->setAxisEnabled(normalizedAxis, enabled)) {
        setStatusMessage(tr("%1 轴使能切换失败").arg(normalizedAxis));
        return;
    }

    if (m_axisEnabled.value(normalizedAxis, true) == enabled && m_axisEnabled.contains(normalizedAxis))
        return;

    m_axisEnabled.insert(normalizedAxis, enabled);
    emit axisEnabledChanged(normalizedAxis, enabled);
    setStatusMessage(enabled
                         ? tr("%1 轴已使能").arg(normalizedAxis)
                         : tr("%1 轴已禁用").arg(normalizedAxis));
}

void ProcessModule::setDigitalOutput(const QString& outputName, bool value)
{
    const QString name = outputName.trimmed();
    if (name.isEmpty())
        return;

    const QString channel = outputChannelFor(m_settings, name);
    QString errorMessage;
    bool ok = false;
    if (m_motionController)
        ok = m_motionController->setDigitalOutput(channel, value, &errorMessage);
    if (!ok && m_deviceManager && m_deviceManager->processIo())
        ok = m_deviceManager->processIo()->setDigitalOutput(channel, value, &errorMessage);

    if (name == tr("激光") && m_deviceManager && m_deviceManager->laserDevice()) {
        QString laserError;
        const bool laserOk = value
            ? m_deviceManager->laserDevice()->startLaser(&laserError)
            : m_deviceManager->laserDevice()->stopLaser(&laserError);
        if (!laserOk) {
            ok = false;
            if (!laserError.isEmpty())
                errorMessage = laserError;
        }
    }

    if (!ok) {
        setStatusMessage(tr("IO 输出 %1 切换失败: %2").arg(name, errorMessage));
        return;
    }

    if (m_digitalOutputs.value(name, false) == value && m_digitalOutputs.contains(name))
        return;

    m_digitalOutputs.insert(name, value);
    emit digitalOutputChanged(name, channel, value);
    setStatusMessage(value ? tr("%1 已打开").arg(name) : tr("%1 已关闭").arg(name));
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

void ProcessModule::initializeAxisEnabledStates()
{
    QMap<QString, bool> nextStates;
    for (const MachineAxisDef& axis : m_axisDefinitions) {
        const QString name = axis.name.trimmed().toUpper();
        if (name.isEmpty() || name == QStringLiteral("BASE"))
            continue;
        const bool enabled = m_axisEnabled.value(name, true);
        nextStates.insert(name, enabled);
        if (m_motionController)
            m_motionController->setAxisEnabled(name, enabled);
    }

    m_axisEnabled = nextStates;
    for (auto it = m_axisEnabled.cbegin(); it != m_axisEnabled.cend(); ++it)
        emit axisEnabledChanged(it.key(), it.value());
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
        if (m_deviceManager)
            m_deviceManager->setMotionController(m_motionController.get());
        if (m_deviceCoordinator)
            m_deviceCoordinator->setMotionController(m_motionController.get());
        initializeAxisEnabledStates();
        return true;
    }

    if (m_motionController)
        m_motionController->stop();
    m_motionController = std::move(nextController);

    // 先注册控制器实例，确保设备管理器始终持有活动运动控制器，
    // 否则连接时会误报“未创建活动运动控制器”。
    registerActiveMotionControllerService();
    if (m_deviceManager)
        m_deviceManager->setMotionController(m_motionController.get());
    if (m_deviceCoordinator)
        m_deviceCoordinator->setMotionController(m_motionController.get());
    initializeAxisEnabledStates();

    // 仅在外设已处于连接状态时（如运行中切换控制器）才建立底层连接，
    // 启动阶段不自动连接，保证连接动作只在用户点击“连接设备”时发生一次。
    if (m_connected && !m_motionController->start()) {
        if (errorMessage)
            *errorMessage = tr("运动控制器启动失败: %1").arg(nextId);
        return false;
    }
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

void ProcessModule::startDeviceAcquisition()
{
    if (!m_monitorService)
        return;
    m_monitorService->requestPoll();
}

void ProcessModule::stopDeviceAcquisition()
{
    if (m_monitorService)
        m_monitorService->requestPoll();
}

void ProcessModule::pollDeviceSnapshotAsync()
{
    if (m_monitorService)
        m_monitorService->requestPoll();
}

void ProcessModule::safeStopProcessOutputs()
{
    if (m_deviceManager && m_deviceManager->laserDevice()) {
        QString laserError;
        if (!m_deviceManager->laserDevice()->stopLaser(&laserError) && !laserError.isEmpty()) {
            LCNC_WARN(lcnc::LogCode::Generic,
                      "ProcessModule: stop laser failed: {}",
                      laserError.toStdString());
        }
    }

    for (const lcnc::process::ProcessMonitorOutputChannel& preset : commonDigitalOutputPresets(m_settings)) {
        QString errorMessage;
        bool resetOk = false;
        if (m_motionController)
            resetOk = m_motionController->setDigitalOutput(preset.channel, false, &errorMessage);
        if (m_deviceManager && m_deviceManager->processIo() &&
            !resetOk && !m_deviceManager->processIo()->setDigitalOutput(preset.channel, false, &errorMessage)) {
            LCNC_WARN(lcnc::LogCode::Generic,
                      "ProcessModule: reset digital output '{}' failed: {}",
                      preset.channel.toStdString(), errorMessage.toStdString());
        }
        if (m_digitalOutputs.value(preset.name, false) || !m_digitalOutputs.contains(preset.name)) {
            m_digitalOutputs.insert(preset.name, false);
            emit digitalOutputChanged(preset.name, preset.channel, false);
        }
    }
}

void ProcessModule::applyMonitorSnapshot(const lcnc::process::ProcessMonitorSnapshot& snapshot)
{
    m_monitorSnapshot = snapshot;
    setAxisPositions(snapshot.axisPositions);
    for (auto it = snapshot.digitalOutputs.cbegin(); it != snapshot.digitalOutputs.cend(); ++it) {
        const QString channel = outputChannelFor(m_settings, it.key());
        if (m_digitalOutputs.value(it.key(), false) == it.value() && m_digitalOutputs.contains(it.key()))
            continue;
        m_digitalOutputs.insert(it.key(), it.value());
        emit digitalOutputChanged(it.key(), channel, it.value());
    }
    if (!snapshot.warnings.isEmpty()) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "ProcessModule: monitor warnings: {}",
                  snapshot.warnings.join(QStringLiteral("; ")).toStdString());
    }
    emit monitorSnapshotChanged(m_monitorSnapshot);
}

void ProcessModule::handleMonitorAlarmRaised(const lcnc::process::ProcessMonitorAlarm& alarm)
{
    const lcnc::ProcessMonitorFaultAction action = alarm.action;
    const QString actionText = [action]() {
        switch (action) {
        case lcnc::ProcessMonitorFaultAction::Continue:
            return ProcessModule::tr("继续加工");
        case lcnc::ProcessMonitorFaultAction::Pause:
            return ProcessModule::tr("进入暂停");
        case lcnc::ProcessMonitorFaultAction::Stop:
            return ProcessModule::tr("停止加工");
        }
        return ProcessModule::tr("进入暂停");
    }();
    const QString message = tr("监控告警: %1，处理方式: %2").arg(alarm.message, actionText);
    emit processLogMessage(QStringLiteral("warn"), message);

    if (m_state != State::Running && m_state != State::Paused) {
        setStatusMessage(message);
        return;
    }

    switch (action) {
    case lcnc::ProcessMonitorFaultAction::Continue:
        setStatusMessage(message);
        break;
    case lcnc::ProcessMonitorFaultAction::Pause:
        runPause();
        if (m_state == State::Paused) {
            setStatusMessage(message);
        }
        break;
    case lcnc::ProcessMonitorFaultAction::Stop:
        if (m_workflowExecutor)
            m_workflowExecutor->stop();
        m_simTimer->stop();
        if (m_motionController)
            m_motionController->emergencyStop();
        safeStopProcessOutputs();
        setState(State::Error, message);
        break;
    }
}

bool ProcessModule::executeCuttingCommandBuffer(bool dryRun, QString* errorMessage)
{
    if (!m_workflowService) {
        if (errorMessage)
            *errorMessage = tr("工作流服务未初始化");
        return false;
    }

    const auto& buffer = m_workflowService->commandBuffer();
    if (buffer.isEmpty()) {
        if (errorMessage)
            *errorMessage = tr("切割命令缓冲为空");
        return false;
    }

    bool softwareSimulatorController = false;
#if !LCNC_PROCESS_HAS_ACS
    softwareSimulatorController = m_motionController
        && m_motionController->id().compare(QStringLiteral("SimulatorCMHP"), Qt::CaseInsensitive) == 0;
#endif

    if (dryRun || m_simulationMode || !m_motionController || softwareSimulatorController) {
        emit processLogMessage(QStringLiteral("process"), tr("以仿真方式执行切割命令: %1 条").arg(buffer.size()));
        return m_executionService
            ? m_executionService->executeDryRun(buffer, errorMessage)
            : false;
    }

    lcnc::process::AcsControllerTranslator translator;
    QStringList lines;
    if (!translator.translate(buffer, &lines, errorMessage))
        return false;

    QString program = lines.join(QChar('\n'));
    if (!program.endsWith(QChar('\n')))
        program.append(QChar('\n'));
    emit processLogMessage(QStringLiteral("process"), tr("下发 ACS 切割程序: %1 行").arg(lines.size()));
    return m_motionController->executeProgram(program, 9, false, 0, errorMessage);
}

void ProcessModule::setState(State state, const QString& statusMessage)
{
    if (m_state != state) {
        m_state = state;
        emit stateChanged(m_state);
        emit processLogMessage(QStringLiteral("state"), tr("状态机切换为 %1").arg(processStateText(m_state)));
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
    emit processLogMessage(logLevelForMessage(m_statusMessage), m_statusMessage);
}
