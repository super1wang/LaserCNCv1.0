#include "modules/process/process_module.h"

#include "core/kernel/i_kernel.h"
#include "core/kernel/kernel.h"
#include "core/kinematics/machine_configuration_service.h"
#include "core/project/lcnc_project_package.h"
#include "core/logging/logger.h"
#include "core/project/lcnc_project_manager.h"
#include "core/task/task_manager.h"
#include "core/task/task_progress.h"
#include "modules/cam/cam_module.h"
#include "modules/cam/i_cam_layer_provider.h"
#include "modules/cam/i_cam_toolpath_provider.h"
#include "core/project/cam/layer_manager.h"
#include "modules/process/cutting/normal_cutting_manager.h"
#include "modules/process/cutting/process_cutting_plan_service.h"
#include "modules/process/execution/process_workflow_executor.h"
#include "modules/process/monitor/process_monitor_service.h"
#include "modules/process/Setting/BuiltinIODefs.h"
#include "modules/process/settings/process_settings_service.h"
#include "modules/process/runtime/device_command_queue.h"
#include "modules/process/runtime/process_axis_utilities.h"
#include "modules/process/steps/process_step_builtin_registration.h"
#include "modules/process/steps/process_step_registry.h"
#include "modules/process/steps/services/legacy_process_services.h"
#include "modules/process/System/Service.h"
#include "modules/process/Tool/ToolFactory.h"
#include "modules/process/workflow/process_flow_store.h"

#include <QList>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QPointer>
#include <QSet>
#include <QStringList>
#include <QTimer>
#include <QFile>
#include <QSaveFile>
#include <QVector>
#include <QElapsedTimer>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <memory>
#include <stdexcept>

#include "toml.hpp"
using toml::table;
using toml::value;

namespace {

/// 一次性监听 TaskManager::taskFinished，匹配到指定 taskId 后自动断开。
void watchTask(QObject* owner, TaskId taskId, std::function<void(bool)> onFinished)
{
    auto connection = std::make_shared<QMetaObject::Connection>();
    *connection = QObject::connect(
        lcnc::Kernel::current().taskManager(),
        &TaskManager::taskFinished,
        owner,
        [taskId, onFinished = std::move(onFinished), connection](TaskId finishedId, bool success) mutable {
            if (finishedId != taskId)
                return;
            QObject::disconnect(*connection);
            onFinished(success);
        });
}

QString defaultStatusText(bool simulationMode, bool connected)
{
    if (simulationMode) {
        return connected
            // 中文翻译：仿真模式 — 控制器已连接
            ? QObject::tr("Simulation mode - controller connected")
            // 中文翻译：仿真模式 — 未连接
            : QObject::tr("Emulation mode - not connected");
    }

    return connected
        // 中文翻译：控制器模式 — 待机
        ? QObject::tr("Controller Mode - Standby")
        // 中文翻译：控制器模式 — 未连接
        : QObject::tr("Controller mode - not connected");
}

QString processStateText(lcnc::ProcessRunState state)
{
    switch (state) {
    case lcnc::ProcessRunState::Idle:
        // 中文翻译：空闲
        return QObject::tr("free");
    case lcnc::ProcessRunState::Running:
        // 中文翻译：运行中
        return QObject::tr("Running");
    case lcnc::ProcessRunState::Paused:
        // 中文翻译：暂停
        return QObject::tr("pause");
    case lcnc::ProcessRunState::Error:
        // 中文翻译：错误
        return QObject::tr("Error");
    case lcnc::ProcessRunState::EmergencyStop:
        // 中文翻译：急停
        return QObject::tr("emergency stop");
    }
    // 中文翻译：未知
    return QObject::tr("unknown");
}

bool stopProcessHardware(const std::shared_ptr<Service>& service)
{
    if (!service)
        return true;
    const auto deviceLock = service->lockDeviceAccess();
    auto* motion = service->GetMotionControl();
    if (motion && motion->IsConnected()) {
        motion->StopMotion();
        motion->StopAllBuffer();
    }

    bool outputsSafe = true;
    const std::array<DigitalOUT, 2> outputs = {DigitalOUT::Laser, DigitalOUT::Blow};
    if (motion && motion->IsConnected()) {
        for (const DigitalOUT output : outputs) {
            if (motion->m_mapDigitalOUT.count(output)
                && !motion->DigitalOutputSet(output, 0)) {
                outputsSafe = false;
            }
        }
    }
    return outputsSafe;
}

struct HardwarePreflightProjection {
    QMap<QString, double> axisPositions;
    QMap<QString, bool> axisEnabled;
};

struct HardwarePreflightChannels {
    QString interlock;
    QString safetyLightCurtain;
    QString pressure;
    QString waterLeakage;
    QString waterTank;
    QString waterPressure;
    QString waterLevel;
};

QString logLevelForMessage(const QString& message)
{
    // 中文翻译：失败
    if (message.contains(QObject::tr("failed")) ||
        // 中文翻译：错误
        message.contains(QObject::tr("Error")) ||
        // 中文翻译：无法
        message.contains(QObject::tr("Unable")) ||
        // 中文翻译：急停
        message.contains(QObject::tr("emergency stop"))) {
        return QStringLiteral("error");
    }
    // 中文翻译：警告
    if (message.contains(QObject::tr("warning")) || message.contains(QStringLiteral("warning"), Qt::CaseInsensitive))
        return QStringLiteral("warn");
    // 中文翻译：流程；加工；切割
    if (message.contains(QObject::tr("process")) || message.contains(QObject::tr("processing")) || message.contains(QObject::tr("cutting")))
        return QStringLiteral("process");
    return QStringLiteral("info");
}

// `simulationMode` normally means pure software playback.  SimulatorCMHP is
// different: it is an ACS Simulator session that executes controller commands
// and must therefore be polled for its live axis positions.
bool usesAcsSimulator(Service* service)
{
#if defined(LCNC_PROCESS_HAS_ACS) && LCNC_PROCESS_HAS_ACS
    if (!service)
        return false;
    const MotionControl* controller = service->GetMotionControl();
    return controller && controller->GetName() == "SimulatorCMHP";
#else
    Q_UNUSED(service);
    return false;
#endif
}

lcnc::ProcessMonitorSettings readMonitorSettings(const lcnc::process::ProcessSettingsService* settings);
QList<lcnc::process::ProcessMonitorOutputChannel> monitorOutputChannels(const lcnc::process::ProcessSettingsService* settings);
QString configuredIoChannel(const lcnc::process::ProcessSettingsService* settings, lcnc::process::ProcessIoBucket bucket, const QString& fallback);
QString enumNameFromTomlChannel(QString channel);

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

double jogVelocityForLevel(int speedLevel)
{
    return (speedLevel == 0) ? 1.0 : (speedLevel == 2 ? 20.0 : 5.0);
}

bool containsEnabledNodeType(const QVector<lcnc::process::ProcessNode>& nodes,
                             lcnc::process::ProcessNodeType type)
{
    for (const auto& node : nodes) {
        if (node.enabled && node.type == type)
            return true;
        if (containsEnabledNodeType(node.children, type))
            return true;
    }
    return false;
}

} // namespace

// ── IModule ───────────────────────────────────────────────────────────────────────
lcnc::ModuleInfo ProcessModule::info() const
{
    return {
        QStringLiteral("process"),
        // 中文翻译：加工进程模块
        QStringLiteral("Process module"),
        QStringLiteral("1.0.0"),
        { QStringLiteral("cam") }
    };
}

bool ProcessModule::init(lcnc::IKernel& kernel)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "ProcessModule::init begin");
    m_kernel = &kernel;
    // The Process store never reads Peripheral.toml/config.toml.  Missing
    // current-schema files are created by the typed repository.
    m_settingsService = std::make_unique<lcnc::process::ProcessSettingsService>();
    if (!m_settingsService->initialize()) {
        LCNC_ERR(lcnc::LogCode::SettingsParseFailed,
                 "process.settings: typed settings initialization failed");
        return false;
    }
    m_runtimeConfiguration.setSimulationMode(m_simulationMode);
    m_service = std::make_shared<Service>(*m_settingsService, m_runtimeConfiguration);
    m_deviceCommandQueue = std::make_unique<lcnc::process::DeviceCommandQueue>();
    if (!m_deviceCommandQueue->start()) {
        LCNC_ERR(lcnc::LogCode::Generic, "Process device command queue did not start");
        return false;
    }
    m_motionStepService = std::make_unique<lcnc::process::LegacyProcessMotionService>(
        m_service.get(), m_deviceCommandQueue.get());
    m_ioStepService = std::make_unique<lcnc::process::LegacyProcessIoService>(
        m_service.get(), m_deviceCommandQueue.get());
    m_cuttingStepService = std::make_unique<lcnc::process::CallbackProcessCuttingService>();
    auto svc = std::shared_ptr<ProcessModule>(this, [](ProcessModule*) {});
    kernel.services().registerService<ProcessModule>(svc);
    auto facade = std::shared_ptr<lcnc::IProcessFacade>(svc, static_cast<lcnc::IProcessFacade*>(this));
    kernel.services().registerService<lcnc::IProcessFacade>(facade);

    // 注册自定义信号类型，跨线程发射 / Qt::QueuedConnection 时需要。
    qRegisterMetaType<DigitalOutputDescriptor>("DigitalOutputDescriptor");
    qRegisterMetaType<QList<DigitalOutputDescriptor>>("QList<DigitalOutputDescriptor>");

    // 注册内置流程步骤插件（执行器只通过插件分发，不再有业务 fallback）。
    auto& stepRegistry = lcnc::process::ProcessStepRegistry::instance();
    stepRegistry.clear();
    stepRegistry.setSettingsService(m_settingsService.get());
    registerBuiltinProcessSteps(stepRegistry);

    // 从 settings 恢复插件启用/禁用状态。
    {
        const table specialTable = m_settingsService->rawTable(lcnc::process::ProcessConfigArea::Workflow);
        if (specialTable.count("ProcessPlugins") && specialTable.at("ProcessPlugins").is_table()) {
            const auto& plugins = specialTable.at("ProcessPlugins").as_table();
            for (const auto& kv : plugins) {
                if (kv.second.is_boolean())
                    stepRegistry.setPluginEnabled(QString::fromStdString(kv.first.data()), kv.second.as_boolean());
            }
        }
    }

    // 接入 CAM 工具路径快照，供 NormalCutting 等步骤查询。
    auto camProvider = kernel.services().getService<lcnc::cam::ICamToolpathProvider>();
    m_cuttingStepService->setSnapshotProvider([camProvider]() {
        lcnc::process::ProcessToolpathSnapshot s;
        if (camProvider && camProvider->hasToolpath()) {
            const auto exp = camProvider->exportToolpathSnapshot();
            s.available = exp.hasEnabledContours();
            s.contourCount = exp.contours.size();
            s.totalPointCount = exp.totalPointCount();
            s.description = exp.description;
        }
        return s;
    });

    // 普通切割管线：CAM 顺序链表 → MotionControl 指令序列；PureSim 由 ticker 驱动模型。
    m_normalCuttingManager = std::make_unique<lcnc::process::NormalCuttingManager>(
        m_service.get(), camProvider, this, m_deviceCommandQueue.get(), this);
    connect(m_normalCuttingManager.get(), &lcnc::process::NormalCuttingManager::logMessage,
            this, &ProcessModule::setStatusMessage);
    connect(m_normalCuttingManager.get(), &lcnc::process::NormalCuttingManager::contourStarted,
            this, [this](int index, int total, const QString& desc) {
                // 中文翻译：切割中 %1/%2: %3
                setStatusMessage(tr("Cutting %1/%2: %3").arg(index).arg(total).arg(desc));
                emit processingProgressChanged(index - 1, total);
            });
    connect(m_normalCuttingManager.get(), &lcnc::process::NormalCuttingManager::contourFinished,
            this, [this](int index, int total) {
                emit processingProgressChanged(index, total);
            });

    // 加工链表服务：项目级图层 → 工具映射、切割顺序、补偿索引等工艺数据。
    m_cuttingPlanService = std::make_unique<lcnc::process::ProcessCuttingPlanService>(this);
    m_cuttingPlanService->setToolpathProvider(camProvider);
    // Phase B：把 CAM 端图层视图注入，图层级状态以 CAM 容器为权威。
    if (auto layerProvider = kernel.services().getService<lcnc::cam::ICamLayerProvider>()) {
        m_cuttingPlanService->setLayerProvider(layerProvider);
        if (auto* mgr = qobject_cast<lcnc::cam::LayerManager*>(layerProvider->notifier())) {
            auto* svc = m_cuttingPlanService.get();
            connect(mgr, &lcnc::cam::LayerManager::layersReset,
                    svc, &lcnc::process::ProcessCuttingPlanService::notifyExternalPlanChanged);
            connect(mgr, &lcnc::cam::LayerManager::layerAdded,
                    svc, [svc](std::uint64_t) { svc->notifyExternalPlanChanged(); });
            connect(mgr, &lcnc::cam::LayerManager::layerRemoved,
                    svc, [svc](std::uint64_t) { svc->notifyExternalPlanChanged(); });
            connect(mgr, &lcnc::cam::LayerManager::layersReordered,
                    svc, &lcnc::process::ProcessCuttingPlanService::notifyExternalPlanChanged);
            connect(mgr, &lcnc::cam::LayerManager::layerPropertyChanged,
                    svc, [svc](std::uint64_t, lcnc::cam::LayerProperty) {
                        svc->notifyExternalPlanChanged();
                    });
            connect(mgr, &lcnc::cam::LayerManager::contourMembershipChanged,
                    svc, &lcnc::process::ProcessCuttingPlanService::notifyExternalPlanChanged);
            connect(mgr, &lcnc::cam::LayerManager::manualContourOrderChanged,
                    svc, &lcnc::process::ProcessCuttingPlanService::notifyExternalManualOrderChanged);
            connect(mgr, &lcnc::cam::LayerManager::sortStrategyChanged,
                    svc, [svc](lcnc::cam::CuttingPlanSortStrategy) {
                        svc->notifyExternalPlanChanged();
                    });
        }
    }
    {
        auto planService = std::shared_ptr<lcnc::process::ProcessCuttingPlanService>(
            m_cuttingPlanService.get(), [](lcnc::process::ProcessCuttingPlanService*) {});
        kernel.services().registerService<lcnc::process::ProcessCuttingPlanService>(planService);
        // 同一实例额外注册为只读 provider 接口，给 CAM 的 TravelPathRenderer 消费。
        auto providerView = std::static_pointer_cast<lcnc::process::IProcessCuttingPlanProvider>(planService);
        kernel.services().registerService<lcnc::process::IProcessCuttingPlanProvider>(providerView);
    }
    // CAM 图层变更（新增/删除/重命名）时自动同步映射表。
    if (auto cam = kernel.services().getService<CamModule>()) {
        connect(cam.get(), &CamModule::toolpathLayersChanged,
                this, [this]() {
                    if (m_cuttingPlanService)
                        m_cuttingPlanService->syncFromCam();
                });
    }
    // 项目文件 IO 已全部下沉到 core：CAM 数据（含 v1 process_cutting_plan.toml 兼容迁移）
    // 由 LcncProjectManager + cam_toolpath_io 统一读写。Process 不再做任何项目文件 IO，
    // 仅在工程重置时清空派生状态、在 open 后从 CAM 容器重新派生切割链表。
    if (auto* pm = lcnc::Kernel::current().projectManager()) {
        lcnc::LcncProjectPackage::setExtension({
            [](const QString& stagingDirectory, const lcnc::LcncProjectManifest& manifest, QString* error) {
                QSaveFile file(QDir(stagingDirectory).filePath(manifest.toolSnapshotPath));
                if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
                    // 中文翻译：无法写入工具快照
                    if (error) *error = QStringLiteral("Unable to write tool snapshot");
                    return false;
                }
                toml::value root(ToolFactory::snapshot());
                if (file.write(QByteArray::fromStdString(toml::format(root))) < 0 || !file.commit()) {
                    // 中文翻译：提交工具快照失败
                    if (error) *error = QStringLiteral("Failed to submit tool snapshot");
                    return false;
                }
                return true;
            },
            [](const QString& stagingDirectory, const lcnc::LcncProjectManifest& manifest, QString* error) {
                const QString path = QDir(stagingDirectory).filePath(manifest.toolSnapshotPath);
                if (!QFileInfo::exists(path)) return true;
                try {
                    const toml::value root = toml::parse(path.toStdString());
                    if (!root.is_table() || !ToolFactory::restoreSnapshot(root.as_table())) {
                        // 中文翻译：项目工具快照无效
                        if (error) *error = QStringLiteral("Project tool snapshot is invalid");
                        return false;
                    }
                    return true;
                } catch (const std::exception& exception) {
                    LCNC_ERR(lcnc::LogCode::Generic,
                             "ProcessModule: failed to parse project tool snapshot: {}",
                             exception.what());
                    // 中文翻译：解析项目工具快照失败: %1
                    if (error) *error = QStringLiteral("Failed to parse project tools snapshot: %1").arg(exception.what());
                    return false;
                }
            }
        });
        connect(pm, &lcnc::LcncProjectManager::projectOpened,
                this, [this](const QString&) {
                    if (m_cuttingPlanService)
                        m_cuttingPlanService->syncFromCam();
                });
        connect(pm, &lcnc::LcncProjectManager::projectReset,
                this, [this]() {
                    if (m_cuttingPlanService)
                        m_cuttingPlanService->clearAll();
                });
    }
    // 启动时先同步一次（CAM 当前已有的图层入表）。
    m_cuttingPlanService->syncFromCam();

    // 把工艺数据服务注入 NormalCuttingManager（buildCuttingList 改走 plan service）。
    m_normalCuttingManager->setCuttingPlanService(m_cuttingPlanService.get());

    m_cuttingStepService->setExecutor(
        [this](const QString& nodeId,
               const QVariantMap& parameters,
               lcnc::process::ProcessInterruptContext* interrupt,
               QString* errorMessage) -> bool {
            if (!m_normalCuttingManager) {
                // 中文翻译：普通切割管理器未初始化
                if (errorMessage) *errorMessage = tr("Normal cutting manager not initialized");
                return false;
            }
            return m_normalCuttingManager->run(nodeId, parameters, interrupt, errorMessage);
        });

    // 设置机台构型（从 MachineConfigurationService）。
    auto machineConfig = kernel.services().getService<lcnc::MachineConfigurationService>();
    if (machineConfig) {
        connect(machineConfig.get(), &lcnc::MachineConfigurationService::machineConfigurationChanged,
                this, [this, cfg = machineConfig.get()] {
                    setAxisDefinitions(cfg->axisDefinitions());
                });
        setAxisDefinitions(machineConfig->axisDefinitions());
    }

    // 工作流执行器。
    m_workflowExecutor = std::make_unique<lcnc::process::ProcessWorkflowExecutor>(this);
    m_stepContext.motion = m_motionStepService.get();
    m_stepContext.io = m_ioStepService.get();
    m_stepContext.cutting = m_cuttingStepService.get();
    QPointer<ProcessModule> stepContextOwner(this);
    m_stepContext.logMessage = [stepContextOwner](const QString& message) {
        QMetaObject::invokeMethod(QCoreApplication::instance(), [stepContextOwner, message] {
            if (stepContextOwner)
                stepContextOwner->setStatusMessage(message);
        }, Qt::QueuedConnection);
    };
    m_stepContext.requestAxisPosition = [stepContextOwner](const QString& axis, double value) {
        QMetaObject::invokeMethod(QCoreApplication::instance(), [stepContextOwner, axis, value] {
            if (stepContextOwner)
                stepContextOwner->setAxisPosition(axis, value);
        }, Qt::QueuedConnection);
    };
    m_stepContext.requestDigitalOutput = [stepContextOwner](const QString& channel, bool value) {
        QMetaObject::invokeMethod(QCoreApplication::instance(), [stepContextOwner, channel, value] {
            if (stepContextOwner)
                stepContextOwner->setDigitalOutput(channel, value);
        }, Qt::QueuedConnection);
    };
    m_workflowExecutor->setStepRegistry(&stepRegistry);
    m_workflowExecutor->setStepContext(&m_stepContext);
    m_workflowExecutor->setDeviceStopper([this](bool emergency) {
        if (!m_deviceCommandQueue)
            return;
        const auto service = m_service;
        QPointer<ProcessModule> self(this);
        const bool queued = m_deviceCommandQueue->submit(
            [service] { return lcnc::process::DeviceCommandResult{stopProcessHardware(service), {}}; },
            TaskPriority::Stop,
            [self, emergency](const lcnc::process::DeviceCommandResult& result) {
                QMetaObject::invokeMethod(QCoreApplication::instance(), [self, emergency, result] {
                    if (!self)
                        return;
                    if (!result.success && emergency)
                        // 中文翻译：急停触发后安全输出复位失败
                        self->setStatusMessage(self->tr("Safety output reset fails after emergency stop is triggered"));
                }, Qt::QueuedConnection);
            });
        if (!queued && emergency)
            // 中文翻译：急停安全停机命令未能排队
            setStatusMessage(tr("Emergency stop safety shutdown command failed to be queued"));
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
                // 中文翻译：流程节点失败: %1
                setState(State::Error, tr("Process node failed: %1").arg(message));
            });
    connect(m_workflowExecutor.get(), &lcnc::process::ProcessWorkflowExecutor::axisPositionRequested,
            this, &ProcessModule::setAxisPosition);
    connect(m_workflowExecutor.get(), &lcnc::process::ProcessWorkflowExecutor::digitalOutputRequested,
            this, [this](const QString& channel, bool value) {
                // 兼容信号路径；供应商调用仍必须进入工作流优先级设备队列。
                if (!m_service || !m_deviceCommandQueue)
                    return;
                QString enumName = channel.trimmed();
                if (enumName.startsWith(QLatin1Char('a')) && enumName.size() >= 2)
                    enumName = enumName.mid(1);
                auto eOut = enum_cast<DigitalOUT>(enumName.toStdString());
                if (!eOut.has_value())
                    return;
                const auto service = m_service;
                m_deviceCommandQueue->submit([service, output = eOut.value(), value] {
                    const auto deviceLock = service->lockDeviceAccess();
                    auto* mc = service->GetMotionControl();
                    if (mc && mc->IsConnected() && mc->m_mapDigitalOUT.count(output))
                        (void)mc->DigitalOutputSet(output, value ? 1 : 0);
                }, TaskPriority::Workflow);
            });
    connect(m_workflowExecutor.get(), &lcnc::process::ProcessWorkflowExecutor::workflowFinished,
            this, [this] {
                m_simTimer->stop();
                clearSafeOutputCache();
                if (!m_deviceCommandQueue) {
                    // 中文翻译：流程完成后设备命令队列不可用
                    setState(State::Error, tr("The device command queue is unavailable after the process is completed"));
                    return;
                }
                const auto service = m_service;
                QPointer<ProcessModule> self(this);
                if (!m_deviceCommandQueue->submit(
                        [service] {
                            const bool success = stopProcessHardware(service);
                            return lcnc::process::DeviceCommandResult{
                                success,
                                // 中文翻译：安全输出复位失败
                                success ? QString() : QObject::tr("Safety output reset failed")};
                        },
                        TaskPriority::Stop,
                        [self](const lcnc::process::DeviceCommandResult& result) {
                            QMetaObject::invokeMethod(QCoreApplication::instance(), [self, result] {
                                if (!self)
                                    return;
                                self->setState(result.success ? State::Idle : State::Error,
                                    result.success
                                        // 中文翻译：流程运行完成
                                        ? self->tr("The process is completed")
                                        // 中文翻译：流程完成后安全输出复位失败: %1
                                        : self->tr("Safety output reset failed after process completion: %1").arg(result.error));
                                emit self->processFlowChanged();
                            }, Qt::QueuedConnection);
                        })) {
                    // 中文翻译：流程完成后安全停机命令未能排队
                    setState(State::Error, tr("Safety shutdown command failed to queue after process completion"));
                }
            });

    // 设备监控服务：联机后周期性读取安全输入/模拟量，并在报警时按设置暂停或停止。
    m_monitorService = std::make_unique<lcnc::process::ProcessMonitorService>(this);
    m_monitorService->setContextProvider([this]() {
        lcnc::process::ProcessMonitorPollContext context;
        context.connected = m_connected;
        context.simulationMode = m_simulationMode;
        context.monitoringEnabled = m_connected && !m_simulationMode;
        context.intervalMs = 500;
        context.settings = readMonitorSettings(m_settingsService.get());
        context.cachedAxisPositions = m_axisPositions;
        context.outputChannels = monitorOutputChannels(m_settingsService.get());
        context.interlockChannel = configuredIoChannel(m_settingsService.get(), lcnc::process::ProcessIoBucket::DigitalInput, QStringLiteral("aInterLock"));
        context.safetyLightCurtainChannel = configuredIoChannel(m_settingsService.get(), lcnc::process::ProcessIoBucket::DigitalInput, QStringLiteral("aSafetyLightCurtain"));
        context.pressureMonitorChannel = configuredIoChannel(m_settingsService.get(), lcnc::process::ProcessIoBucket::DigitalInput, QStringLiteral("aPressureMonitor"));
        context.waterLeakageChannel = configuredIoChannel(m_settingsService.get(), lcnc::process::ProcessIoBucket::DigitalInput, QStringLiteral("aWaterLeakageMonitor"));
        context.waterTankChannel = configuredIoChannel(m_settingsService.get(), lcnc::process::ProcessIoBucket::DigitalInput, QStringLiteral("aWaterTankMonitor"));
        context.waterPressureChannel = configuredIoChannel(m_settingsService.get(), lcnc::process::ProcessIoBucket::AnalogInput, QStringLiteral("aWaterPressure"));
        context.waterLevelChannel = configuredIoChannel(m_settingsService.get(), lcnc::process::ProcessIoBucket::AnalogInput, QStringLiteral("aWaterLevel"));

        Service* service = m_service.get();
        context.readDigital = [service](const QString& channel, bool* value, QString* errorMessage) {
            if (!value)
                return false;
            const auto deviceLock = service ? service->lockDeviceAccess()
                                            : Service::DeviceLock{};
            MotionControl* mc = service ? service->GetMotionControl() : nullptr;
            if (!mc || !mc->IsConnected()) {
                // 中文翻译：运动控制器未连接
                if (errorMessage) *errorMessage = QObject::tr("Motion controller not connected");
                return false;
            }
            const QString enumName = enumNameFromTomlChannel(channel);
            int raw = 0;
            if (auto eIn = enum_cast<DigitalIN>(enumName.toStdString());
                eIn.has_value() && mc->m_mapDigitalIN.count(eIn.value())
                && mc->DigitalInputGet(eIn.value(), raw)) {
                *value = raw != 0;
                return true;
            }
            if (auto eOut = enum_cast<DigitalOUT>(enumName.toStdString());
                eOut.has_value() && mc->m_mapDigitalOUT.count(eOut.value())
                && mc->DigitalOutputGet(eOut.value(), raw)) {
                *value = raw != 0;
                return true;
            }
            // 中文翻译：通道未配置或读取失败: %1
            if (errorMessage) *errorMessage = QObject::tr("Channel not configured or read failed: %1").arg(channel);
            return false;
        };
        context.readAnalog = [service](const QString& channel, double* value, QString* errorMessage) {
            if (!value)
                return false;
            const auto deviceLock = service ? service->lockDeviceAccess()
                                            : Service::DeviceLock{};
            MotionControl* mc = service ? service->GetMotionControl() : nullptr;
            if (!mc || !mc->IsConnected()) {
                // 中文翻译：运动控制器未连接
                if (errorMessage) *errorMessage = QObject::tr("Motion controller not connected");
                return false;
            }
            const QString enumName = enumNameFromTomlChannel(channel);
            if (auto eIn = enum_cast<AnalogIN>(enumName.toStdString());
                eIn.has_value() && mc->m_mapAnalogIN.count(eIn.value())
                && mc->AnalogInputGet(eIn.value(), *value)) {
                return true;
            }
            // 中文翻译：模拟量通道未配置或读取失败: %1
            if (errorMessage) *errorMessage = QObject::tr("Analog channel is not configured or failed to read: %1").arg(channel);
            return false;
        };
        return context;
    });
    connect(m_monitorService.get(), &lcnc::process::ProcessMonitorService::alarmRaised,
            this, [this](const lcnc::process::ProcessMonitorAlarm& alarm) {
                // 中文翻译：加工环境报警: %1
                setStatusMessage(tr("Processing environment alarm: %1").arg(alarm.message));
                if (m_state != State::Running)
                    return;
                if (alarm.action == lcnc::ProcessMonitorFaultAction::Stop)
                    runStop();
                else if (alarm.action == lcnc::ProcessMonitorFaultAction::Pause)
                    runPause();
            });
    connect(m_monitorService.get(), &lcnc::process::ProcessMonitorService::snapshotUpdated,
            this, [this](const lcnc::process::ProcessMonitorSnapshot& snapshot) {
                if (snapshot.hasActiveAlarm())
                    // 中文翻译：加工环境报警: %1
                    setStatusMessage(tr("Processing environment alarm: %1").arg(snapshot.summary));
            });

    setStatusMessage(defaultStatusText(m_simulationMode, m_connected));

    m_initialized = true;
    // 首次发射主界面 IO 描述符，让 WidgetLaserControl 据此搭建 IO 栏。
    emit digitalOutputDescriptorsChanged(mainPanelDigitalOutputs());
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
    ++m_runRequestGeneration;

    // Process workers borrow Service and controller objects.  They must leave
    // before this module tears down monitoring or its device ownership.
    const bool tasksFinished = cancelOwnedTasks(10000);
    if (!tasksFinished) {
        if (m_deviceCommandQueue) {
            const auto service = m_service;
            (void)m_deviceCommandQueue->submitStop([service] {
                (void)stopProcessHardware(service);
            });
        }
        clearSafeOutputCache();
        setState(State::EmergencyStop,
                 // 中文翻译：Process 后台任务取消超时，已请求安全停机并保留设备运行时
                 tr("Process background task cancellation timeout, safe shutdown requested and device runtime preserved"));
        LCNC_ERR(lcnc::LogCode::Generic,
                 "ProcessModule::stop: owned task cancellation timed out");
    }
    m_deviceOperation = DeviceOperation::None;
    stopDeviceMonitoring();
    bool workflowFinished = true;
    if (m_workflowExecutor) {
        m_workflowExecutor->stop();
        workflowFinished = m_workflowExecutor->waitForIdle(10000);
        if (!workflowFinished) {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "ProcessModule::stop: workflow thread did not exit before timeout");
            // 中文翻译：工作流停止超时，设备保持安全停机状态
            setState(State::EmergencyStop, tr("Workflow stop times out, device remains in safe shutdown state"));
        }
    }

    bool deviceQueueFinished = true;
    if (workflowFinished && m_deviceCommandQueue) {
        deviceQueueFinished = m_deviceCommandQueue->shutdown(10000);
        if (!deviceQueueFinished) {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "ProcessModule::stop: device command queue did not stop before timeout");
        }
    }

    // Kernel shutdown is the only unconditional device lifecycle boundary.
    // Do not destroy or disconnect under an unfinished SDK call; the shared
    // Service lease keeps those objects alive until the worker exits.
    if (tasksFinished && workflowFinished && deviceQueueFinished && m_service) {
        const auto deviceLock = m_service->lockDeviceAccess();
        const bool outputsSafe = triggerSafeStopOutputs();
        const bool disconnectOk = m_service->shutdownDevices() && outputsSafe;
        m_connected = false;
        if (!disconnectOk) {
            // 中文翻译：关闭设备时安全停机或断开失败
            setState(State::Error, tr("Safety shutdown or disconnect failure when shutting down equipment"));
            LCNC_ERR(lcnc::LogCode::Generic,
                     "ProcessModule::stop: safe shutdown did not complete");
        }
    } else if (m_service) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "ProcessModule::stop: retaining device runtime because a worker or device queue is still active");
    }
    m_initialized = false;
    lcnc::process::ProcessStepRegistry::instance().clear();
    LCNC_INFO(lcnc::LogCode::Generic, "ProcessModule stop done");
}

ProcessModule::ProcessModule(QObject* parent)
    : QObject(parent)
{
    m_processFlowDocument.resetToDefault();
    m_processFlowDocument.markClean();
    initializeAxisPositions();

    m_simTimer = new QTimer(this);
    m_simTimer->setInterval(100);
    connect(m_simTimer, &QTimer::timeout, this, &ProcessModule::onSimulationTick);

    // 定时器只负责调度；SDK 读取进入 DeviceCommandQueue 的低优先级、
    // 可合并任务，绝不在 GUI 线程或独立轮询线程池运行。
    m_hwStatusTimer = new QTimer(this);
    m_hwStatusTimer->setInterval(150);
    connect(m_hwStatusTimer, &QTimer::timeout, this, &ProcessModule::pollHardwareStatus);

    // 激光器等外设通常为串口，限制为低频且禁止任务堆积。
    m_peripheralStatusTimer = new QTimer(this);
    m_peripheralStatusTimer->setInterval(2000);
    connect(m_peripheralStatusTimer, &QTimer::timeout,
            this, &ProcessModule::pollPeripheralStatus);

    setStatusMessage(defaultStatusText(m_simulationMode, m_connected));
}

ProcessModule::~ProcessModule() = default;

void ProcessModule::trackOwnedTask(TaskId taskId)
{
    if (taskId != kInvalidTaskId)
        m_ownedTaskIds.insert(taskId);
}

void ProcessModule::releaseOwnedTask(TaskId taskId)
{
    m_ownedTaskIds.remove(taskId);
}

bool ProcessModule::cancelOwnedTasks(int timeoutMs)
{
    auto* taskMgr = lcnc::Kernel::current().taskManager();
    if (!taskMgr || m_ownedTaskIds.isEmpty())
        return true;

    const QSet<TaskId> taskIds = m_ownedTaskIds;
    for (TaskId taskId : taskIds) {
        if (taskMgr->isRunning(taskId))
            taskMgr->requestAbort(taskId);
    }

    QElapsedTimer timeout;
    timeout.start();
    bool allFinished = true;
    for (TaskId taskId : taskIds) {
        if (!taskMgr->isRunning(taskId)) {
            releaseOwnedTask(taskId);
            continue;
        }
        const int remainingMs = std::max(0, timeoutMs - static_cast<int>(timeout.elapsed()));
        if (!taskMgr->waitForDone(taskId, remainingMs) && taskMgr->isRunning(taskId)) {
            allFinished = false;
            continue;
        }
        releaseOwnedTask(taskId);
    }
    return allFinished;
}

bool ProcessModule::isConnected() const
{
    return m_connected;
}

void ProcessModule::connectAllDevices()
{
    if (m_deviceOperation != DeviceOperation::None) {
        // 中文翻译：已有设备操作进行中，请等待完成
        setStatusMessage(tr("Existing equipment operation is in progress, please wait for completion"));
        return;
    }
    if (m_connected) {
        // 中文翻译：设备已连接，无需重复连接
        setStatusMessage(tr("The device is already connected, no need to reconnect"));
        return;
    }

    if (!m_deviceCommandQueue || !m_service) {
        // 中文翻译：设备命令队列不可用
        setState(State::Error, tr("Device command queue is unavailable"));
        return;
    }

    // 后台连接任务持有 Service，超时关机时也不会析构仍被 SDK 使用的对象。
    const auto service = m_service;

    if (!m_runtimeConfiguration.hasEnabledAxes() && !m_axisDefinitions.isEmpty()) {
        QStringList axes;
        for (const MachineAxisDef& axis : m_axisDefinitions)
            if (axis.name != QStringLiteral("BASE")) axes.append(axis.name);
        m_runtimeConfiguration.setEnabledAxes(std::move(axes));
    }

    m_deviceOperation = DeviceOperation::Connecting;
    const bool pureSimulation = m_simulationMode;
    QPointer<ProcessModule> self(this);
    const auto reportProgress = [self](int percent, const QString& step) {
        if (!self)
            return;
        QMetaObject::invokeMethod(self, [self, percent, step] {
            if (!self || self->m_deviceOperation != DeviceOperation::Connecting)
                return;
            // 中文翻译：连接设备
            emit self->deviceConnectProgress(QObject::tr("Connect devices"), percent, step);
            // 中文翻译：正在连接设备：%1
            self->setStatusMessage(QObject::tr("Connecting device: %1").arg(step));
        }, Qt::QueuedConnection);
    };
    // 中文翻译：连接设备；已提交连接任务
    emit deviceConnectProgress(tr("Connect devices"), 0, tr("Connection task submitted"));
    // 中文翻译：正在连接设备：等待设备线程
    setStatusMessage(tr("Connecting device: waiting for device thread"));
    if (!m_deviceCommandQueue->submit([service, pureSimulation, self, reportProgress] {
            bool success = true;
            try {
                // 中文翻译：正在创建运动控制器
                reportProgress(10, QObject::tr("Creating motion controller"));
                if (pureSimulation)
                    service->SetMotionControl("Simulator");
                else
                    service->SetMotionControl();

                auto* motionControl = service->GetMotionControl();
                if (!motionControl) {
                    throw std::runtime_error(
                        // 中文翻译：运动控制器实例化失败；当前构建未启用所选控制器
                        QObject::tr("Motion controller instantiation failed; the selected controller is not enabled for the current build").toStdString());
                }
                // 中文翻译：正在连接运动控制器
                reportProgress(25, QObject::tr("Connecting motion controller"));
                if (!motionControl->Connect()) {
                    throw std::runtime_error(
                        // 中文翻译：运动控制器连接失败；禁止回退到纯软件仿真
                        QObject::tr("Motion controller connection failed; falling back to pure software simulation is prohibited").toStdString());
                }
                // 中文翻译：正在初始化运动轴
                reportProgress(50, QObject::tr("Initializing motion axes"));
                motionControl->rebuildAxes();

                // 中文翻译：正在创建激光器
                reportProgress(65, QObject::tr("Creating laser"));
                service->SetLaserDevice();
                auto* laserDevice = service->GetLaserDevice();
                // 中文翻译：正在连接激光器
                reportProgress(75, QObject::tr("Connecting laser"));
                if (!laserDevice || !laserDevice->Connect()) {
                    throw std::runtime_error(
                        // 中文翻译：激光器连接失败；已取消本次设备连接
                        QObject::tr("Laser connection failed; this device connection has been canceled").toStdString());
                }

                // 中文翻译：正在加载设备参数
                reportProgress(88, QObject::tr("Loading device parameters"));
                if (!pureSimulation) {
                    service->SetMotionControlTable();
                    service->SetDigitalTable();
                    service->SetAnalogTable();
                }
                service->SetLaserTable();
                // 中文翻译：设备初始化完成
                reportProgress(100, QObject::tr("Device initialization completed"));
            } catch (const std::exception& exception) {
                success = false;
                LCNC_ERR(lcnc::LogCode::Generic,
                         "ProcessModule::connectAllDevices failed: {}", exception.what());
                (void)service->shutdownDevices();
            } catch (...) {
                success = false;
                LCNC_ERR(lcnc::LogCode::Generic,
                         "ProcessModule::connectAllDevices threw an unknown exception");
                (void)service->shutdownDevices();
            }

            if (self) {
                QMetaObject::invokeMethod(self, [self, success] {
                    if (!self)
                        return;

                    self->m_deviceOperation = DeviceOperation::None;
                    self->m_connected = success;
                    emit self->connectionChanged(success);
                    if (success) {
                        // 中文翻译：设备已连接
                        self->setState(State::Idle, tr("Device is connected"));
                        self->startDeviceMonitoring();
                    } else {
                        // 中文翻译：设备连接失败；未切换到纯软件仿真
                        self->setState(State::Error, tr("Device connection failed; not switched to software-only emulation"));
                    }
                    emit self->deviceConnectFinished(success,
                        // 中文翻译：所有设备连接成功；设备连接失败，请检查设置
                        success ? tr("All devices connected successfully") : tr("Device connection failed, please check settings"));
                }, Qt::QueuedConnection);
            }
        })) {
        m_deviceOperation = DeviceOperation::None;
        // 中文翻译：设备连接命令未能排队
        setState(State::Error, tr("Device connection command failed to queue"));
        // 中文翻译：设备连接命令未能排队
        emit deviceConnectFinished(false, tr("Device connection command failed to queue"));
    }
}

void ProcessModule::disconnectAllDevices()
{
    if (m_deviceOperation != DeviceOperation::None) {
        // 中文翻译：已有设备操作进行中，请等待完成
        setStatusMessage(tr("Existing equipment operation is in progress, please wait for completion"));
        return;
    }
    if (!m_connected) {
        // 中文翻译：设备未连接
        setStatusMessage(tr("Device not connected"));
        return;
    }
    if (m_preflightInFlight) {
        // 中文翻译：加工环境检查尚未结束，请先停止并等待检查退出
        setStatusMessage(tr("The processing environment check has not ended yet. Please stop and wait for the check to exit."));
        return;
    }
    if (m_state == State::Running || m_state == State::Paused
        || (m_workflowExecutor && !m_workflowExecutor->waitForIdle(0))) {
        // 中文翻译：加工流程尚未完全停止，请先停止并等待当前设备步骤退出
        setStatusMessage(tr("The processing process has not stopped completely. Please stop it first and wait for the current equipment step to exit."));
        return;
    }
    if (m_homing) {
        // 中文翻译：回零进行中，请等待完成后再断开设备
        setStatusMessage(tr("Return to zero is in progress, please wait until it is completed before disconnecting the device."));
        return;
    }

    auto* taskMgr = lcnc::Kernel::current().taskManager();
    if (!taskMgr) {
        // 中文翻译：TaskManager 不可用
        setState(State::Error, tr("TaskManager is not available"));
        return;
    }

    // 进入断开流程前，先把上层运行/工作流/仿真等状态收尾，避免在硬件
    // 已断开的情况下还有定时器/工作流继续触发指令。
    if (m_simTimer)
        m_simTimer->stop();
    stopDeviceMonitoring();
    clearSafeOutputCache();

    // 中文翻译：正在断开设备...
    setStatusMessage(tr("Disconnecting device..."));

    const auto service = m_service;

    m_deviceOperation = DeviceOperation::Disconnecting;
    // 中文翻译：断开设备
    const TaskId taskId = taskMgr->run(tr("Disconnect device"),
        [service](TaskProgress* progress) {
            const auto deviceLock = service->lockDeviceAccess();
            const auto throwIfAborted = [progress]() {
                if (progress->isAbortRequested())
                    // 中文翻译：设备断开已取消
                    throw std::runtime_error("Device disconnect canceled");
            };
            progress->setRange(0, 100);

            // 中文翻译：正在安全停止设备...
            progress->setStepName(QObject::tr("Safely stopping the device..."));
            progress->setValue(0);
            if (!stopProcessHardware(service))
                // 中文翻译：设备断开前安全输出复位失败
                throw std::runtime_error("Safety output reset fails before device disconnection");
            progress->setValue(20);
            throwIfAborted();

            // ── Step 1: 断开激光器 ──
            // 中文翻译：正在断开激光器...
            progress->setStepName(QObject::tr("Disconnecting laser..."));
            {
                auto* ld = service->GetLaserDevice();
                if (ld) {
                    ld->Disconnect();
                }
            }
            progress->setValue(50);
            throwIfAborted();

            // ── Step 2: 断开运动控制器 ──
            // 中文翻译：正在断开运动控制器...
            progress->setStepName(QObject::tr("Disconnecting motion controller..."));
            {
                auto* mc = service->GetMotionControl();
                if (mc) {
                    mc->Disconnect();
                }
            }
            progress->setValue(100);
        });

    trackOwnedTask(taskId);
    watchTask(this, taskId, [this, taskId](bool success) {
        releaseOwnedTask(taskId);
        m_deviceOperation = DeviceOperation::None;
        m_connected = false;
        if (m_simTimer)
            m_simTimer->stop();
        m_state = State::Idle;
        emit connectionChanged(false);
        emit stateChanged(m_state);
        setStatusMessage(success
            // 中文翻译：所有设备已断开
            ? tr("All devices are disconnected")
            // 中文翻译：设备断开过程中出现异常，请检查日志
            : tr("An exception occurred during device disconnection, please check the log."));

        if (success) {
            LCNC_INFO(lcnc::LogCode::Generic,
                      "ProcessModule::disconnectAllDevices: completed");
        } else {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "ProcessModule::disconnectAllDevices: encountered errors");
        }

        emit deviceConnectFinished(success,
            // 中文翻译：所有设备已断开；设备断开过程中出现异常
            success ? tr("All devices are disconnected") : tr("An exception occurred during device disconnection"));
    });
}

void ProcessModule::setSimulationMode(bool on)
{
    if (m_simulationMode == on)
        return;

    if (on && m_service && m_service->configuredControllerRequiresDevice()) {
        const QString controller = QString::fromStdString(
            m_service->configuredMotionControllerName());
        setState(State::Error,
                 // 中文翻译：当前已选择控制器 %1，禁止切换到纯软件仿真；请先在设备设置中显式选择 Simulator
                 tr("Controller %1 is currently selected and switching to software-only simulation is prohibited; please explicitly select Simulator in the device settings first")
                     .arg(controller));
        return;
    }

    m_simulationMode = on;
    m_runtimeConfiguration.setSimulationMode(on);
    emit simulationModeChanged(on);

    setStatusMessage(defaultStatusText(m_simulationMode, m_connected));
}

bool ProcessModule::simulationMode() const
{
    return m_simulationMode;
}

void ProcessModule::setAxisDefinitions(const QList<MachineAxisDef>& axes)
{
    if (lcnc::process::sameAxisDefinitions(m_axisDefinitions, axes))
        return;

    m_axisDefinitions = axes;
    m_simPhase = 0.0;
    initializeAxisPositions();
    initializeAxisEnabledStates();

	QStringList runtimeAxes;
	for (const MachineAxisDef& axis : axes)
		if (axis.name != QStringLiteral("BASE")) runtimeAxes.append(axis.name);
	m_runtimeConfiguration.setEnabledAxes(std::move(runtimeAxes));
}

void ProcessModule::jog(const QString& axisName, int direction, int speedLevel, double distance)
{
    if (axisName.trimmed().isEmpty() || direction == 0 || m_state == State::EmergencyStop)
        return;

    const QString normalizedAxis = axisName.trimmed().toUpper();
    if (!m_axisEnabled.value(normalizedAxis, true)) {
        // 中文翻译：%1 轴未使能，点动已忽略
        setStatusMessage(tr("%1 axis is not enabled, jog has been ignored").arg(normalizedAxis));
        return;
    }

    auto eAxis = enum_cast<Axis>(normalizedAxis.toStdString());
    if (!eAxis.has_value()) {
        // 中文翻译：%1 轴未注册，无法点动
        setStatusMessage(tr("%1 axis is not registered and cannot be jogged").arg(normalizedAxis));
        return;
    }
    if (!m_service || !m_deviceCommandQueue) {
        // 中文翻译：设备命令队列不可用
        setStatusMessage(tr("Device command queue is unavailable"));
        return;
    }

    const double step = distance > 1e-9 ? distance : jogStepForLevel(speedLevel);
    const double delta = step * (direction > 0 ? 1.0 : -1.0);
    const double vel = jogVelocityForLevel(speedLevel);
    const auto service = m_service;
    QPointer<ProcessModule> self(this);
    // 中文翻译：点动 %1 轴 %2
    const QString successMessage = tr("Jog %1 axis %2").arg(
        // 中文翻译：正向；负向
        normalizedAxis, direction > 0 ? tr("forward") : tr("Negative"));
    if (!m_deviceCommandQueue->submit(
            [service, axis = eAxis.value(), delta, vel] {
                const auto deviceLock = service->lockDeviceAccess();
                auto* mc = service->GetMotionControl();
                if (!mc || !mc->IsConnected())
                    // 中文翻译：未连接控制器，请先连接设备
                    return lcnc::process::DeviceCommandResult{false, QObject::tr("The controller is not connected, please connect the device first")};
                if (!mc->IsMotorCreated(axis))
                    // 中文翻译：轴未注册
                    return lcnc::process::DeviceCommandResult{false, QObject::tr("Axis not registered")};
                const bool ok = mc->MoveRelative(axis, delta, vel);
                return lcnc::process::DeviceCommandResult{
                    // 中文翻译：相对运动命令失败
                    ok, ok ? QString() : QObject::tr("Relative motion command failed")};
            },
            TaskPriority::Interactive,
            [self, successMessage](const lcnc::process::DeviceCommandResult& result) {
                QMetaObject::invokeMethod(QCoreApplication::instance(), [self, result, successMessage] {
                    if (self)
                        self->setStatusMessage(result.success ? successMessage : result.error);
                }, Qt::QueuedConnection);
            })) {
        // 中文翻译：点动命令未能排队
        setStatusMessage(tr("Jog command failed to queue"));
    }
}

void ProcessModule::moveAxisAbsolute(const QString& axisName, double position, int speedLevel)
{
    if (axisName.trimmed().isEmpty() || m_state == State::EmergencyStop)
        return;

    const QString normalizedAxis = axisName.trimmed().toUpper();
    if (!m_axisEnabled.value(normalizedAxis, true)) {
        // 中文翻译：%1 轴未使能，绝对运动已忽略
        setStatusMessage(tr("%1 axis is not enabled, absolute motion has been ignored").arg(normalizedAxis));
        return;
    }

    auto eAxis = enum_cast<Axis>(normalizedAxis.toStdString());
    if (!eAxis.has_value()) {
        // 中文翻译：%1 轴未注册，无法绝对运动
        setStatusMessage(tr("%1 axis is not registered and cannot move absolutely").arg(normalizedAxis));
        return;
    }
    if (!m_service || !m_deviceCommandQueue) {
        // 中文翻译：设备命令队列不可用
        setStatusMessage(tr("Device command queue is unavailable"));
        return;
    }

    const auto service = m_service;
    const double velocity = jogVelocityForLevel(speedLevel);
    QPointer<ProcessModule> self(this);
    const QString successMessage =
        // 中文翻译：%1 轴移动到 %2
        tr("%1 axis moved to %2").arg(normalizedAxis).arg(position, 0, 'f', 3);
    if (!m_deviceCommandQueue->submit(
            [service, axis = eAxis.value(), position, velocity] {
                const auto deviceLock = service->lockDeviceAccess();
                auto* mc = service->GetMotionControl();
                if (!mc || !mc->IsConnected())
                    // 中文翻译：未连接控制器，请先连接设备
                    return lcnc::process::DeviceCommandResult{false, QObject::tr("The controller is not connected, please connect the device first")};
                if (!mc->IsMotorCreated(axis))
                    // 中文翻译：轴未注册
                    return lcnc::process::DeviceCommandResult{false, QObject::tr("Axis not registered")};
                const bool ok = mc->MoveAbsolute(axis, position, velocity);
                return lcnc::process::DeviceCommandResult{
                    // 中文翻译：绝对运动命令失败
                    ok, ok ? QString() : QObject::tr("Absolute motion command failed")};
            },
            TaskPriority::Interactive,
            [self, successMessage](const lcnc::process::DeviceCommandResult& result) {
                QMetaObject::invokeMethod(QCoreApplication::instance(), [self, result, successMessage] {
                    if (self)
                        self->setStatusMessage(result.success ? successMessage : result.error);
                }, Qt::QueuedConnection);
            })) {
        // 中文翻译：绝对运动命令未能排队
        setStatusMessage(tr("Absolute motion command failed to queue"));
    }
}

void ProcessModule::startContinuousJog(const QString& axisName, int direction, int speedLevel)
{
    if (axisName.trimmed().isEmpty() || direction == 0 || m_state == State::EmergencyStop)
        return;

    const QString normalizedAxis = axisName.trimmed().toUpper();
    if (!m_axisEnabled.value(normalizedAxis, true)) {
        // 中文翻译：%1 轴未使能，连续运动已忽略
        setStatusMessage(tr("%1 axis is not enabled, continuous motion is ignored").arg(normalizedAxis));
        return;
    }

    auto eAxis = enum_cast<Axis>(normalizedAxis.toStdString());
    if (!eAxis.has_value()) {
        // 中文翻译：%1 轴未注册，无法连续运动
        setStatusMessage(tr("%1 axis is not registered and cannot move continuously.").arg(normalizedAxis));
        return;
    }
    if (!m_service || !m_deviceCommandQueue) {
        // 中文翻译：设备命令队列不可用
        setStatusMessage(tr("Device command queue is unavailable"));
        return;
    }

    const auto service = m_service;
    const bool positive = direction > 0;
    const double velocity = jogVelocityForLevel(speedLevel);
    QPointer<ProcessModule> self(this);
    const QString successMessage =
        // 中文翻译：连续点动 %1 轴 %2；正向；负向
        tr("Continuously jog %1 axis %2").arg(normalizedAxis, positive ? tr("forward") : tr("Negative"));
    if (!m_deviceCommandQueue->submit(
            [service, axis = eAxis.value(), positive, velocity] {
                const auto deviceLock = service->lockDeviceAccess();
                auto* mc = service->GetMotionControl();
                if (!mc || !mc->IsConnected())
                    // 中文翻译：未连接控制器，请先连接设备
                    return lcnc::process::DeviceCommandResult{false, QObject::tr("The controller is not connected, please connect the device first")};
                if (!mc->IsMotorCreated(axis))
                    // 中文翻译：轴未注册
                    return lcnc::process::DeviceCommandResult{false, QObject::tr("Axis not registered")};
                const bool ok = mc->Jog(axis, positive, velocity);
                return lcnc::process::DeviceCommandResult{
                    // 中文翻译：连续运动命令失败
                    ok, ok ? QString() : QObject::tr("Continuous motion command failed")};
            },
            TaskPriority::Interactive,
            [self, successMessage](const lcnc::process::DeviceCommandResult& result) {
                QMetaObject::invokeMethod(QCoreApplication::instance(), [self, result, successMessage] {
                    if (self)
                        self->setStatusMessage(result.success ? successMessage : result.error);
                }, Qt::QueuedConnection);
            })) {
        // 中文翻译：连续点动命令未能排队
        setStatusMessage(tr("Continuous jog commands failed to be queued"));
    }
}

void ProcessModule::stopContinuousJog(const QString& axisName)
{
    if (axisName.trimmed().isEmpty())
        return;

    const QString normalizedAxis = axisName.trimmed().toUpper();
    auto eAxis = enum_cast<Axis>(normalizedAxis.toStdString());
    if (!eAxis.has_value() || !m_service || !m_deviceCommandQueue)
        return;

    const auto service = m_service;
    QPointer<ProcessModule> self(this);
    if (!m_deviceCommandQueue->submit(
            [service, axis = eAxis.value()] {
                const auto deviceLock = service->lockDeviceAccess();
                auto* mc = service->GetMotionControl();
                if (!mc || !mc->IsConnected() || !mc->IsMotorCreated(axis))
                    // 中文翻译：轴停止条件不满足
                    return lcnc::process::DeviceCommandResult{false, QObject::tr("Axis stop conditions are not met")};
                const bool ok = mc->StopMotion(axis);
                return lcnc::process::DeviceCommandResult{
                    // 中文翻译：轴停止命令失败
                    ok, ok ? QString() : QObject::tr("Axis stop command failed")};
            },
            TaskPriority::Stop,
            [self, normalizedAxis](const lcnc::process::DeviceCommandResult& result) {
                QMetaObject::invokeMethod(QCoreApplication::instance(), [self, result, normalizedAxis] {
                    if (self)
                        self->setStatusMessage(result.success
                            // 中文翻译：%1 轴连续运动已停止
                            ? self->tr("%1 axis continuous motion has stopped").arg(normalizedAxis)
                            : result.error);
                }, Qt::QueuedConnection);
            })) {
        // 中文翻译：%1 轴停止命令未能排队
        setStatusMessage(tr("%1 axis stop command failed to be queued").arg(normalizedAxis));
    }
}

void ProcessModule::home()
{
    if (m_state == State::EmergencyStop) {
        // 中文翻译：急停状态，无法回零
        setStatusMessage(tr("Emergency stop state, unable to return to zero"));
        return;
    }
    if (m_state == State::Running) {
        // 中文翻译：加工运行中，无法回零
        setStatusMessage(tr("During processing, it is impossible to return to zero."));
        return;
    }
    if (m_homing) {
        // 中文翻译：回零正在进行中
        setStatusMessage(tr("Return to zero in progress"));
        return;
    }
    if (m_deviceOperation != DeviceOperation::None) {
        // 中文翻译：已有设备操作进行中，请等待完成
        setStatusMessage(tr("Existing equipment operation is in progress, please wait for completion"));
        return;
    }

    // 合并预设轴系（按 Z 优先顺序）+ 用户扩展轴系；扩展轴系无对应 Axis 枚举，
    // 实控阶段只能跳过硬件回零，但仍参与位姿清零，与仿真保持一致。
    QStringList axes = lcnc::process::homeOrderForAxes(m_axisDefinitions);
    for (const QString& ext : m_runtimeConfiguration.extensionAxes()) {
        const QString key = ext.trimmed().toUpper();
        if (!key.isEmpty() && key != QStringLiteral("BASE") && !axes.contains(key))
            axes.append(key);
    }
    if (axes.isEmpty()) {
        // 中文翻译：没有可回零的轴系
        setStatusMessage(tr("No axis system that can be homed"));
        return;
    }

    if (!m_deviceCommandQueue || !m_service) {
        // 中文翻译：设备命令队列不可用
        setState(State::Error, tr("Device command queue is unavailable"));
        return;
    }

    m_homing = true;
    m_deviceOperation = DeviceOperation::Homing;
    // 中文翻译：开始回零（共 %1 个轴）
    setStatusMessage(tr("Start zero return (total %1 axes)").arg(axes.size()));

    const bool connected = m_connected;
    const auto service = m_service;
    QPointer<ProcessModule> self(this);

    // The complete sequence stays on the device thread so no axis can be
    // interleaved with a settings update or later migrated device command.
    if (!m_deviceCommandQueue->submit([axes, connected, service, self] {
            bool success = true;
            try {
                MotionControl* hwMc = connected ? service->GetMotionControl() : nullptr;
                for (const QString& axis : axes) {
                    if (!hwMc)
                        continue;

                    auto eAxis = enum_cast<Axis>(axis.toStdString());
                    if (eAxis.has_value() && hwMc->IsMotorCreated(eAxis.value())) {
                        if (!hwMc->Home(eAxis.value())) {
                            success = false;
                            LCNC_ERR(lcnc::LogCode::Generic,
                                     "ProcessModule::home: hardware Home failed for axis {}",
                                     axis.toStdString());
                            break;
                        }
                    } else {
                        LCNC_WARN(lcnc::LogCode::Generic,
                                  "ProcessModule::home: skip hardware home for axis {} "
                                  "(extension or not registered)",
                                  axis.toStdString());
                    }
                }
            } catch (const std::exception& exception) {
                success = false;
                LCNC_ERR(lcnc::LogCode::Generic,
                         "ProcessModule::home failed: {}", exception.what());
            } catch (...) {
                success = false;
                LCNC_ERR(lcnc::LogCode::Generic,
                         "ProcessModule::home threw an unknown exception");
            }

            if (self) {
                QMetaObject::invokeMethod(self, [self, axes, success] {
                    if (!self)
                        return;

                    self->m_homing = false;
                    self->m_deviceOperation = DeviceOperation::None;
                    // The hardware poller may overwrite this with its actual
                    // position shortly afterwards; clear the UI cache now.
                    for (const QString& axis : axes)
                        self->setAxisPosition(axis, 0.0);
                    self->m_simPhase = 0.0;

                    if (success) {
                        // 中文翻译：回零完成 — 共 %1 个轴
                        self->setStatusMessage(tr("Zero return completed - %1 axes in total").arg(axes.size()));
                        LCNC_INFO(lcnc::LogCode::Generic,
                                  "ProcessModule::home: completed for {} axes", axes.size());
                    } else {
                        // 中文翻译：回零失败，请检查日志
                        self->setState(State::Error, tr("Return to zero failed, please check the log"));
                    }
                }, Qt::QueuedConnection);
            }
        })) {
        m_homing = false;
        m_deviceOperation = DeviceOperation::None;
        // 中文翻译：回零命令未能排队
        setState(State::Error, tr("Return to zero command failed to queue"));
    }
}

void ProcessModule::moveToConfiguredPosition(bool loading)
{
    // 中文翻译：上料位；下料位
    const QString positionName = loading ? tr("Loading position") : tr("Unloading position");
    if (m_state != State::Idle) {
        // 中文翻译：当前加工状态不允许移动至%1
        setStatusMessage(tr("The current processing status does not allow moving to %1").arg(positionName));
        return;
    }
    if (m_homing || m_deviceOperation != DeviceOperation::None) {
        // 中文翻译：已有设备操作进行中，请等待完成
        setStatusMessage(tr("Existing equipment operation is in progress, please wait for completion"));
        return;
    }
    if (!m_settingsService || !m_deviceCommandQueue || !m_service) {
        // 中文翻译：设备命令队列或加工设置不可用
        setStatusMessage(tr("Device command queue or processing settings are not available"));
        return;
    }

    struct AxisTarget {
        QString name;
        double position;
    };
    QVector<AxisTarget> targets;
    const QString tableName = loading ? QStringLiteral("LoadingPos")
                                      : QStringLiteral("BlankingPos");
    const QString keyPrefix = loading ? QStringLiteral("fLoadingPos")
                                      : QStringLiteral("fBlankingPos");
    const QString enablePrefix = loading ? QStringLiteral("bLoadingPos")
                                         : QStringLiteral("bBlankingPos");
    for (const MachineAxisDef& axis : m_axisDefinitions) {
        const QString axisName = axis.name.trimmed().toUpper();
        if (axisName.isEmpty() || axisName == QStringLiteral("BASE"))
            continue;
        const bool positionEnabled = m_settingsService->rawValue(
            lcnc::process::ProcessConfigArea::Operations, tableName,
            enablePrefix + axisName, false).toBool();
        if (!positionEnabled)
            continue;
        if (!m_axisEnabled.value(axisName, true)) {
            // 中文翻译：%1 轴未使能，无法移动至%2
            setStatusMessage(tr("%1 axis is not enabled and cannot move to %2").arg(axisName, positionName));
            return;
        }
        const auto eAxis = enum_cast<Axis>(axisName.toStdString());
        if (!eAxis.has_value()) {
            // 中文翻译：%1 轴未注册，无法移动至%2
            setStatusMessage(tr("%1 axis is not registered and cannot be moved to %2").arg(axisName, positionName));
            return;
        }
        const QVariant targetValue = m_settingsService->rawValue(
            lcnc::process::ProcessConfigArea::Operations, tableName,
            keyPrefix + axisName);
        bool targetValid = false;
        const double target = targetValue.toDouble(&targetValid);
        if (!targetValid || !std::isfinite(target)) {
            // 中文翻译：%1 轴的%2未配置有效位置
            setStatusMessage(tr("%1 axis %2 is not configured with a valid position").arg(axisName, positionName));
            return;
        }
        if (target < axis.minVal || target > axis.maxVal) {
            // 中文翻译：%1 轴的%2超出行程范围
            setStatusMessage(tr("%1 axis %2 exceeds the stroke range").arg(axisName, positionName));
            return;
        }
        targets.append({axisName, target});
    }
    if (targets.isEmpty()) {
        // 中文翻译：没有可移动至%1的轴
        setStatusMessage(tr("No axis to move to %1").arg(positionName));
        return;
    }

    m_deviceOperation = DeviceOperation::PresetMove;
    // 中文翻译：正在移动至%1（共 %2 个轴）
    setStatusMessage(tr("Moving to %1 of %2 axes").arg(positionName).arg(targets.size()));
    const auto service = m_service;
    QPointer<ProcessModule> self(this);
    if (!m_deviceCommandQueue->submit(
            [service, targets, positionName] {
                const auto deviceLock = service->lockDeviceAccess();
                auto* mc = service->GetMotionControl();
                if (!mc || !mc->IsConnected())
                    // 中文翻译：未连接控制器，请先连接设备
                    return lcnc::process::DeviceCommandResult{false, QObject::tr("The controller is not connected, please connect the device first")};

                for (const AxisTarget& target : targets) {
                    const auto axis = enum_cast<Axis>(target.name.toStdString());
                    if (!axis.has_value() || !mc->IsMotorCreated(axis.value()))
                        return lcnc::process::DeviceCommandResult{false,
                            // 中文翻译：%1 轴未在控制器中创建
                            QObject::tr("%1 axis was not created in the controller").arg(target.name)};
                    if (!mc->IsEnabled(axis.value()))
                        return lcnc::process::DeviceCommandResult{false,
                            // 中文翻译：%1 轴当前未使能
                            QObject::tr("%1 axis is not currently enabled").arg(target.name)};
                    if (!mc->IsHomed(axis.value()))
                        return lcnc::process::DeviceCommandResult{false,
                            // 中文翻译：%1 轴尚未回零
                            QObject::tr("%1 axis has not returned to zero yet").arg(target.name)};
                    if (mc->IsAxisMoving(axis.value()))
                        return lcnc::process::DeviceCommandResult{false,
                            // 中文翻译：%1 轴正在运动
                            QObject::tr("%1 axis is moving").arg(target.name)};
                }
                for (const AxisTarget& target : targets) {
                    const auto axis = enum_cast<Axis>(target.name.toStdString());
                    if (!mc->MoveAbsolute(axis.value(), target.position, jogVelocityForLevel(1))) {
                        (void)mc->StopMotion();
                        return lcnc::process::DeviceCommandResult{false,
                            // 中文翻译：%1 轴移动至%2失败
                            QObject::tr("%1 axis movement to %2 failed").arg(target.name, positionName)};
                    }
                }
                return lcnc::process::DeviceCommandResult{true, {}};
            },
            TaskPriority::Interactive,
            [self, positionName](const lcnc::process::DeviceCommandResult& result) {
                QMetaObject::invokeMethod(QCoreApplication::instance(), [self, positionName, result] {
                    if (!self)
                        return;
                    self->m_deviceOperation = DeviceOperation::None;
                    if (!result.success) {
                        self->setStatusMessage(result.error);
                        return;
                    }
                    // 中文翻译：已下发移动至%1的命令
                    self->setStatusMessage(self->tr("The command to move to %1 has been issued").arg(positionName));
                }, Qt::QueuedConnection);
            })) {
        m_deviceOperation = DeviceOperation::None;
        // 中文翻译：移动至%1的命令未能排队
        setStatusMessage(tr("The command to move to %1 failed to be queued").arg(positionName));
    }
}

void ProcessModule::startDeviceMonitoring()
{
    const bool acsSimulator = usesAcsSimulator(m_service.get());
    if (!m_connected || (m_simulationMode && !acsSimulator))
        return;
    if (m_hwStatusTimer && !m_hwStatusTimer->isActive())
        m_hwStatusTimer->start();
    if (m_peripheralStatusTimer && !m_peripheralStatusTimer->isActive())
        m_peripheralStatusTimer->start();
    pollHardwareStatus();
    pollPeripheralStatus();
    // ACS Simulator has live motion positions but no real safety IO; do not
    // start the physical monitor service for that simulated session.
    if (m_monitorService && !acsSimulator)
        m_monitorService->start();
}

void ProcessModule::stopDeviceMonitoring()
{
    if (m_hwStatusTimer)
        m_hwStatusTimer->stop();
    if (m_peripheralStatusTimer)
        m_peripheralStatusTimer->stop();
    if (m_monitorService)
        m_monitorService->stop();
    m_hwPollInFlight = false;
    m_peripheralPollInFlight = false;
}

bool ProcessModule::validateProcessingConfiguration(QString* errorMessage)
{
    auto fail = [errorMessage](const QString& message) {
        if (errorMessage)
            *errorMessage = message;
        return false;
    };

    if (m_state != State::Idle)
        // 中文翻译：当前状态机不是空闲待机状态，不能开始加工
        return fail(tr("The current state machine is not in idle standby state and cannot start processing."));

    if (!m_simulationMode) {
        const auto* project = lcnc::Kernel::current().projectManager();
        if (project && !project->session().machineConfigurationCompatible()) {
            // 中文翻译：当前工程的机台构型与本机不匹配；请确认机台、轴映射和安全 IO 后重新保存工程
            return fail(tr("The machine configuration of the current project does not match the machine; please confirm the machine, axis mapping and safety IO before resave the project"));
        }
    }

    const bool hasNormalCutting = containsEnabledNodeType(
        m_processFlowDocument.rootNodes(), lcnc::process::ProcessNodeType::NormalCutting);
    if (hasNormalCutting) {
        if (!m_cuttingPlanService)
            // 中文翻译：切割计划服务未初始化
            return fail(tr("Cutting plan service not initialized"));

        const auto cuttingList = m_cuttingPlanService->buildCuttingList();
        if (cuttingList.isEmpty())
            // 中文翻译：没有可加工轮廓，请先生成刀路并启用需加工特征
            return fail(tr("There is no machinable contour. Please generate a tool path and enable the features to be machined."));

        QSet<QString> availableTools;
        const QStringList toolNames = ToolFactory::toolNames();
        for (const QString& toolName : toolNames)
            availableTools.insert(toolName);
        QStringList missingTools;
        QStringList unknownTools;
        QSet<std::uint64_t> reportedMissingLayers;
        QSet<std::uint64_t> reportedUnknownLayers;
        for (const auto& entry : cuttingList) {
            const QString toolName = entry.toolName.trimmed();
            const QString layerText = entry.layerName.trimmed().isEmpty()
                // 中文翻译：图层 %1
                ? tr("Layer %1").arg(entry.layerId)
                : entry.layerName;
            if (toolName.isEmpty()) {
                if (!reportedMissingLayers.contains(entry.layerId)) {
                    missingTools.append(layerText);
                    reportedMissingLayers.insert(entry.layerId);
                }
                continue;
            }
            if (!availableTools.contains(toolName) && !reportedUnknownLayers.contains(entry.layerId)) {
                unknownTools.append(tr("%1: %2").arg(layerText, toolName));
                reportedUnknownLayers.insert(entry.layerId);
            }
        }
        if (!missingTools.isEmpty())
            // 中文翻译：需加工特征对应图层未配置工具: %1
            return fail(tr("The layer corresponding to the feature to be processed does not have a tool configured: %1").arg(missingTools.join(tr("，"))));
        if (!unknownTools.isEmpty())
            // 中文翻译：图层配置了不存在的工具: %1
            return fail(tr("The layer is configured with a tool that does not exist: %1").arg(unknownTools.join(tr("，"))));
    }

    if (m_simulationMode)
        return true;

    if (!m_connected)
        // 中文翻译：设备未连接，不能开始加工
        return fail(tr("The device is not connected and processing cannot be started."));
    return true;
}

void ProcessModule::runStart()
{
    if (m_state == State::EmergencyStop) {
        // 中文翻译：急停状态，复位后才能运行
        setStatusMessage(tr("Emergency stop state, can only be run after reset."));
        return;
    }
    if (m_preflightInFlight) {
        // 中文翻译：加工环境检查正在进行中
        setStatusMessage(tr("Processing environment inspection in progress"));
        return;
    }

    if (m_state == State::Paused) {
        if (m_workflowExecutor)
            m_workflowExecutor->resume();
        if (m_simulationMode)
            m_simTimer->start(std::max(30, static_cast<int>(100.0 / std::max(0.1, m_feedOverride))));
        // 中文翻译：运行继续
        setState(State::Running, tr("Run continues"));
        return;
    }

    QString environmentError;
    if (!validateProcessingConfiguration(&environmentError)) {
        // 中文翻译：加工环境检查失败: %1
        setState(State::Error, tr("Processing environment check failed: %1").arg(environmentError));
        return;
    }

    if (m_simulationMode) {
        startWorkflowAfterPreflight();
        return;
    }

    if (!m_deviceCommandQueue || !m_service) {
        // 中文翻译：加工环境检查失败: 设备命令队列不可用
        setState(State::Error, tr("Processing environment check failed: Device command queue unavailable"));
        return;
    }

    const QList<MachineAxisDef> axes = m_axisDefinitions;
    const lcnc::ProcessMonitorSettings monitorSettings =
        readMonitorSettings(m_settingsService.get());
    const HardwarePreflightChannels channels{
        configuredIoChannel(m_settingsService.get(),
            lcnc::process::ProcessIoBucket::DigitalInput, QStringLiteral("aInterLock")),
        configuredIoChannel(m_settingsService.get(),
            lcnc::process::ProcessIoBucket::DigitalInput, QStringLiteral("aSafetyLightCurtain")),
        configuredIoChannel(m_settingsService.get(),
            lcnc::process::ProcessIoBucket::DigitalInput, QStringLiteral("aPressureMonitor")),
        configuredIoChannel(m_settingsService.get(),
            lcnc::process::ProcessIoBucket::DigitalInput, QStringLiteral("aWaterLeakageMonitor")),
        configuredIoChannel(m_settingsService.get(),
            lcnc::process::ProcessIoBucket::DigitalInput, QStringLiteral("aWaterTankMonitor")),
        configuredIoChannel(m_settingsService.get(),
            lcnc::process::ProcessIoBucket::AnalogInput, QStringLiteral("aWaterPressure")),
        configuredIoChannel(m_settingsService.get(),
            lcnc::process::ProcessIoBucket::AnalogInput, QStringLiteral("aWaterLevel"))};
    const auto projection = std::make_shared<HardwarePreflightProjection>();
    const auto service = m_service;
    const std::uint64_t requestGeneration = ++m_runRequestGeneration;
    m_preflightInFlight = true;
    // 中文翻译：正在后台检查加工环境...
    setStatusMessage(tr("Checking the processing environment in the background..."));

    QPointer<ProcessModule> self(this);
    if (!m_deviceCommandQueue->submit(
            [service, axes, monitorSettings, channels, projection] {
                const auto fail = [](const QString& error) {
                    return lcnc::process::DeviceCommandResult{false, error};
                };
                const auto deviceLock = service->lockDeviceAccess();
                MotionControl* mc = service->GetMotionControl();
                if (!mc || !mc->IsConnected())
                    // 中文翻译：运动控制器未连接或连接已断开
                    return fail(QObject::tr("Motion controller not connected or disconnected"));
                if (mc->ErrorOccurred())
                    // 中文翻译：运动控制器存在异常，请清除故障后再加工
                    return fail(QObject::tr("There is an abnormality in the motion controller. Please clear the fault before processing."));

                int fault = 0;
                if (!mc->IsAxisStatusNormal(fault))
                    // 中文翻译：运动控制器状态读取失败，请检查控制器连接
                    return fail(QObject::tr("Motion controller status reading failed, please check the controller connection"));
                if (fault != 0) {
                    // 中文翻译：运动控制器故障码: %1，请清除故障后再加工
                    return fail(QObject::tr("Motion controller fault code: %1, please clear the fault before processing")
                                    .arg(fault));
                }

                QStringList disabledAxes;
                QStringList unregisteredAxes;
                for (const MachineAxisDef& axis : axes) {
                    const QString name = axis.name.trimmed().toUpper();
                    if (name.isEmpty() || name == QStringLiteral("BASE"))
                        continue;
                    const auto eAxis = enum_cast<Axis>(name.toStdString());
                    if (!eAxis.has_value())
                        continue;
                    if (!mc->IsMotorCreated(eAxis.value())) {
                        unregisteredAxes.append(name);
                        continue;
                    }
                    double position = 0.0;
                    if (mc->GetActualPos(eAxis.value(), position))
                        projection->axisPositions.insert(name, position);
                    const bool enabled = mc->IsEnabled(eAxis.value());
                    projection->axisEnabled.insert(name, enabled);
                    if (!enabled)
                        disabledAxes.append(name);
                }
                if (!unregisteredAxes.isEmpty()) {
                    // 中文翻译：运动控制器轴系未注册: %1
                    return fail(QObject::tr("Motion controller axis is not registered: %1")
                                    .arg(unregisteredAxes.join(QObject::tr("，"))));
                }
                if (!disabledAxes.isEmpty()) {
                    // 中文翻译：运动控制器轴系未使能: %1
                    return fail(QObject::tr("Motion controller axis is not enabled: %1")
                                    .arg(disabledAxes.join(QObject::tr("，"))));
                }

                LaserDevice* laser = service->GetLaserDevice();
                if (!laser)
                    // 中文翻译：激光器未创建，请先连接设备
                    return fail(QObject::tr("The laser has not been created, please connect the device first"));
                const QString laserName = QString::fromStdString(laser->GetName());
                const bool analogLaser =
                    laserName.compare(QStringLiteral("AnalogControl"), Qt::CaseInsensitive) == 0;
                if (!analogLaser && !laser->IsConnected())
                    // 中文翻译：激光器未连接或连接已断开
                    return fail(QObject::tr("Laser not connected or disconnected"));
                if (!laser->IsInited())
                    // 中文翻译：激光器参数未初始化
                    return fail(QObject::tr("Laser parameters not initialized"));
                if (!analogLaser) {
                    const QString laserFault =
                        QString::fromStdString(laser->GetTroubleshooting()).trimmed();
                    if (!laserFault.isEmpty()
                        && laserFault.compare(QStringLiteral("OK"), Qt::CaseInsensitive) != 0) {
                        // 中文翻译：激光器异常: %1
                        return fail(QObject::tr("Laser exception: %1").arg(laserFault));
                    }
                }

                const auto requireDigitalNormal =
                    [mc, &fail](bool enabled,
                                const QString& title,
                                const QString& channel) {
                        if (!enabled)
                            return lcnc::process::DeviceCommandResult{};
                        if (channel.trimmed().isEmpty())
                            // 中文翻译：%1监控通道未配置
                            return fail(QObject::tr("%1 monitoring channel is not configured").arg(title));
                        const QString enumName = enumNameFromTomlChannel(channel);
                        const auto eIn = enum_cast<DigitalIN>(enumName.toStdString());
                        if (!eIn.has_value() || !mc->m_mapDigitalIN.count(eIn.value()))
                            // 中文翻译：%1状态获取失败: 通道未配置: %2
                            return fail(QObject::tr("%1 status acquisition failed: Channel not configured: %2")
                                            .arg(title, channel));
                        int raw = 0;
                        if (!mc->DigitalInputGet(eIn.value(), raw))
                            // 中文翻译：%1状态获取失败: 通道读取失败: %2
                            return fail(QObject::tr("%1 Status acquisition failed: Channel read failed: %2")
                                            .arg(title, channel));
                        if (raw != 0)
                            // 中文翻译：%1异常
                            return fail(QObject::tr("%1Exception").arg(title));
                        return lcnc::process::DeviceCommandResult{};
                    };

                auto digitalResult = requireDigitalNormal(
                    // 中文翻译：门禁
                    monitorSettings.interLockEnabled, QObject::tr("access control"), channels.interlock);
                if (!digitalResult.success)
                    return digitalResult;
                digitalResult = requireDigitalNormal(
                    monitorSettings.safetyLightCurtainEnabled,
                    // 中文翻译：安全光栅
                    QObject::tr("Safety grating"), channels.safetyLightCurtain);
                if (!digitalResult.success)
                    return digitalResult;
                digitalResult = requireDigitalNormal(
                    monitorSettings.pressureMonitorEnabled,
                    // 中文翻译：气压监控
                    QObject::tr("Air pressure monitoring"), channels.pressure);
                if (!digitalResult.success)
                    return digitalResult;
                digitalResult = requireDigitalNormal(
                    monitorSettings.waterLeakageMonitorEnabled,
                    // 中文翻译：漏水监控
                    QObject::tr("Water leakage monitoring"), channels.waterLeakage);
                if (!digitalResult.success)
                    return digitalResult;
                digitalResult = requireDigitalNormal(
                    monitorSettings.waterTankMonitorEnabled,
                    // 中文翻译：水箱监控
                    QObject::tr("Water tank monitoring"), channels.waterTank);
                if (!digitalResult.success)
                    return digitalResult;

                const auto requireAnalogAtLeast =
                    [mc, &fail](bool enabled,
                                const QString& title,
                                const QString& channel,
                                double threshold,
                                const QString& unit) {
                        if (!enabled)
                            return lcnc::process::DeviceCommandResult{};
                        if (channel.trimmed().isEmpty())
                            // 中文翻译：%1通道未配置
                            return fail(QObject::tr("%1 channel is not configured").arg(title));
                        const QString enumName = enumNameFromTomlChannel(channel);
                        const auto eIn = enum_cast<AnalogIN>(enumName.toStdString());
                        if (!eIn.has_value() || !mc->m_mapAnalogIN.count(eIn.value()))
                            // 中文翻译：%1通道未配置: %2
                            return fail(QObject::tr("Channel %1 is not configured: %2").arg(title, channel));
                        double value = 0.0;
                        if (!mc->AnalogInputGet(eIn.value(), value))
                            // 中文翻译：%1状态获取失败: %2
                            return fail(QObject::tr("%1 status acquisition failed: %2").arg(title, channel));
                        if (value < threshold) {
                            // 中文翻译：%1异常: 当前值 %2 %3，阈值 %4 %3
                            return fail(QObject::tr("%1Exception: Current value %2 %3, threshold %4 %3")
                                .arg(title,
                                     QString::number(value, 'f', 3),
                                     unit,
                                     QString::number(threshold, 'f', 3)));
                        }
                        return lcnc::process::DeviceCommandResult{};
                    };
                const auto pressureResult = requireAnalogAtLeast(
                    monitorSettings.waterPressureMonitorEnabled,
                    // 中文翻译：水压监控
                    QObject::tr("water pressure monitoring"), channels.waterPressure,
                    monitorSettings.waterPressureLimitMpa, QStringLiteral("MPa"));
                if (!pressureResult.success)
                    return pressureResult;
                const auto levelResult = requireAnalogAtLeast(
                    monitorSettings.waterLevelMonitorEnabled,
                    // 中文翻译：水位监控
                    QObject::tr("water level monitoring"), channels.waterLevel,
                    monitorSettings.waterLevelLimitMm, QStringLiteral("mm"));
                if (!levelResult.success)
                    return levelResult;
                return lcnc::process::DeviceCommandResult{};
            },
            TaskPriority::Workflow,
            [self, projection, requestGeneration](
                const lcnc::process::DeviceCommandResult& result) {
                QMetaObject::invokeMethod(QCoreApplication::instance(),
                    [self, projection, requestGeneration, result] {
                        if (!self)
                            return;
                        self->m_preflightInFlight = false;
                        if (requestGeneration != self->m_runRequestGeneration)
                            return;
                        if (!result.success) {
                            self->setState(State::Error,
                                // 中文翻译：加工环境检查失败: %1
                                self->tr("Processing environment check failed: %1").arg(result.error));
                            return;
                        }
                        for (auto it = projection->axisPositions.cbegin();
                             it != projection->axisPositions.cend(); ++it) {
                            self->setAxisPosition(it.key(), it.value());
                        }
                        for (auto it = projection->axisEnabled.cbegin();
                             it != projection->axisEnabled.cend(); ++it) {
                            const bool changed =
                                !self->m_axisEnabled.contains(it.key())
                                || self->m_axisEnabled.value(it.key()) != it.value();
                            self->m_axisEnabled.insert(it.key(), it.value());
                            if (changed)
                                emit self->axisEnabledChanged(it.key(), it.value());
                        }
                        self->startDeviceMonitoring();
                        if (self->m_monitorService)
                            self->m_monitorService->requestPoll();
                        self->startWorkflowAfterPreflight();
                    }, Qt::QueuedConnection);
            })) {
        m_preflightInFlight = false;
        // 中文翻译：加工环境检查命令未能排队
        setState(State::Error, tr("Processing environment check command failed to queue"));
    }
}

void ProcessModule::startWorkflowAfterPreflight()
{
    emit processingRunStarted();
    if (m_workflowExecutor) {
        QString errorMessage;
        if (!m_workflowExecutor->start(m_processFlowDocument, &errorMessage)) {
            // 中文翻译：流程启动失败: %1
            setState(State::Error, tr("Process start failed: %1").arg(errorMessage));
            return;
        }
        if (m_workflowExecutor->state() == lcnc::process::ProcessWorkflowExecutor::State::Error
            || m_workflowExecutor->state() == lcnc::process::ProcessWorkflowExecutor::State::Idle) {
            return;
        }
    }

    if (m_simulationMode) {
        const int intervalMs =
            std::max(30, static_cast<int>(100.0 / std::max(0.1, m_feedOverride)));
        m_simTimer->start(intervalMs);
    }
    setState(State::Running,
             // 中文翻译：仿真运行中；控制器运行中
             m_simulationMode ? tr("Simulation running") : tr("Controller is running"));
}

void ProcessModule::runPause()
{
    if (m_state != State::Running)
        return;

    m_simTimer->stop();
    if (m_workflowExecutor)
        m_workflowExecutor->pause();
    // 普通切割通过工作流 checkpoint 在轮廓边界暂停。不要暂停 ACS buffer：
    // 已下发的当前轮廓需完整执行，恢复时继续使用同一个连续缓冲流。
    // 中文翻译：已请求暂停，当前轮廓完成后暂停
    setState(State::Paused, tr("Pause requested, pause after completion of current contour"));
}

void ProcessModule::runStop()
{
    ++m_runRequestGeneration;
    m_simTimer->stop();
    bool workflowIssuedStop = false;
    if (m_workflowExecutor)
    {
        workflowIssuedStop = m_workflowExecutor->state()
            != lcnc::process::ProcessWorkflowExecutor::State::Idle;
        m_workflowExecutor->stop();
    }
    if (!workflowIssuedStop && m_deviceCommandQueue) {
        const auto service = m_service;
        QPointer<ProcessModule> self(this);
        if (!m_deviceCommandQueue->submit(
                lcnc::process::DeviceCommandQueue::ResultCommand(
                    [service] { return lcnc::process::DeviceCommandResult{stopProcessHardware(service), {}}; }),
                TaskPriority::Stop,
                [self](const lcnc::process::DeviceCommandResult& result) {
                    QMetaObject::invokeMethod(QCoreApplication::instance(), [self, result] {
                        if (self && !result.success)
                            // 中文翻译：运行已停止，但安全输出复位失败
                            self->setStatusMessage(self->tr("Operation stopped but safety output reset failed"));
                    }, Qt::QueuedConnection);
                })) {
            // 中文翻译：停止命令未能排队
            setState(State::Error, tr("Stop command failed to queue"));
            return;
        }
    }
    // 中文翻译：仿真停止请求已提交；停止请求已提交
    setState(State::Idle, m_simulationMode ? tr("Simulation stop request submitted") : tr("Stop request submitted"));
}

void ProcessModule::emergencyStop()
{
    ++m_runRequestGeneration;
    m_simTimer->stop();
    bool workflowIssuedStop = false;
    if (m_workflowExecutor)
    {
        workflowIssuedStop = m_workflowExecutor->state()
            != lcnc::process::ProcessWorkflowExecutor::State::Idle;
        m_workflowExecutor->emergencyStop();
    }
    if (!workflowIssuedStop && m_deviceCommandQueue) {
        const auto service = m_service;
        if (!m_deviceCommandQueue->submit(
                lcnc::process::DeviceCommandQueue::ResultCommand(
                    [service] { return lcnc::process::DeviceCommandResult{stopProcessHardware(service), {}}; }),
                TaskPriority::Stop)) {
            // 中文翻译：急停安全停机命令未能排队
            setStatusMessage(tr("Emergency stop safety shutdown command failed to be queued"));
        }
    }
    setState(State::EmergencyStop,
             // 中文翻译：急停请求已提交
             tr("Emergency stop request has been submitted"));
}

void ProcessModule::resetEmergencyStop()
{
    if (m_state != State::EmergencyStop)
        return;
    setState(State::Idle, defaultStatusText(m_simulationMode, m_connected));
}

void ProcessModule::newProcess()
{
    m_processFlowDocument.resetToDefault();
    m_processFlowDocument.markClean();
    emit processFlowChanged();
    // 中文翻译：已新建流程
    setStatusMessage(tr("New process has been created"));
}

bool ProcessModule::loadProcess(const QString& filePath)
{
    if (filePath.trimmed().isEmpty()) {
        // 中文翻译：流程文件路径为空
        setStatusMessage(tr("Process file path is empty"));
        return false;
    }

    QString errorMessage;
    if (!lcnc::process::ProcessFlowStore::loadFromFile(filePath, m_processFlowDocument, &errorMessage)) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "ProcessModule: load process failed: {}",
                  errorMessage.toStdString());
        // 中文翻译：加载流程失败: %1
        setStatusMessage(tr("Loading process failed: %1").arg(errorMessage));
        return false;
    }

    emit processFlowChanged();
    // 中文翻译：已加载流程: %1
    setStatusMessage(tr("Loaded process: %1").arg(filePath));
    return true;
}

bool ProcessModule::saveProcess(const QString& filePath)
{
    if (filePath.trimmed().isEmpty()) {
        // 中文翻译：流程文件路径为空
        setStatusMessage(tr("Process file path is empty"));
        return false;
    }

    QString errorMessage;
    if (!lcnc::process::ProcessFlowStore::saveToFile(filePath, m_processFlowDocument, &errorMessage)) {
        // 中文翻译：保存流程失败: %1
        setStatusMessage(tr("Save process failed: %1").arg(errorMessage));
        return false;
    }

    m_processFlowDocument.markClean();
    // 中文翻译：已保存流程: %1
    setStatusMessage(tr("Saved process: %1").arg(filePath));
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

    const auto eAxis = enum_cast<Axis>(normalizedAxis.toStdString());
    if (!eAxis.has_value() || !m_service || !m_deviceCommandQueue) {
        // 中文翻译：%1 轴未注册或设备队列不可用
        setStatusMessage(tr("%1 axis is not registered or the device queue is unavailable").arg(normalizedAxis));
        return;
    }

    const auto service = m_service;
    QPointer<ProcessModule> self(this);
    if (!m_deviceCommandQueue->submit(
            [service, axis = eAxis.value(), enabled] {
                const auto deviceLock = service->lockDeviceAccess();
                auto* mc = service->GetMotionControl();
                if (!mc || !mc->IsConnected())
                    // 中文翻译：运动控制器未连接
                    return lcnc::process::DeviceCommandResult{false, QObject::tr("Motion controller not connected")};
                if (!mc->IsMotorCreated(axis))
                    // 中文翻译：轴未注册
                    return lcnc::process::DeviceCommandResult{false, QObject::tr("Axis not registered")};
                const bool ok = mc->SetAxisEnable(axis, enabled);
                return lcnc::process::DeviceCommandResult{
                    // 中文翻译：轴使能切换失败
                    ok, ok ? QString() : QObject::tr("Axis enable switching failed")};
            },
            TaskPriority::Interactive,
            [self, normalizedAxis, enabled](const lcnc::process::DeviceCommandResult& result) {
                QMetaObject::invokeMethod(QCoreApplication::instance(),
                    [self, result, normalizedAxis, enabled] {
                        if (!self)
                            return;
                        if (!result.success) {
                            self->setStatusMessage(
                                // 中文翻译：%1 轴使能切换失败: %2
                                self->tr("%1 Axis enable switching failed: %2").arg(normalizedAxis, result.error));
                            return;
                        }
                        if (self->m_axisEnabled.value(normalizedAxis, true) != enabled
                            || !self->m_axisEnabled.contains(normalizedAxis)) {
                            self->m_axisEnabled.insert(normalizedAxis, enabled);
                            emit self->axisEnabledChanged(normalizedAxis, enabled);
                        }
                        self->setStatusMessage(enabled
                            // 中文翻译：%1 轴已使能
                            ? self->tr("%1 axis is enabled").arg(normalizedAxis)
                            // 中文翻译：%1 轴已禁用
                            : self->tr("%1 axis is disabled").arg(normalizedAxis));
                    }, Qt::QueuedConnection);
            })) {
        // 中文翻译：%1 轴使能命令未能排队
        setStatusMessage(tr("%1 axis enable command failed to be queued").arg(normalizedAxis));
    }
}

void ProcessModule::setDigitalOutput(const QString& outputName, bool value)
{
    const QString name = outputName.trimmed();
    if (name.isEmpty() || !m_service || !m_deviceCommandQueue)
        return;

    const QString channel = name;
    const auto service = m_service;
    QPointer<ProcessModule> self(this);
    if (!m_deviceCommandQueue->submit(
            [service, channel, value] {
                const auto deviceLock = service->lockDeviceAccess();
                auto* mc = service->GetMotionControl();
                if (!mc || !mc->IsConnected())
                    // 中文翻译：运动控制器未连接
                    return lcnc::process::DeviceCommandResult{false, QObject::tr("Motion controller not connected")};
                const bool ok = mc->DigitalOutputSet(channel, value ? 1 : 0);
                return lcnc::process::DeviceCommandResult{
                    // 中文翻译：IO 输出切换失败
                    ok, ok ? QString() : QObject::tr("IO output switching failed")};
            },
            TaskPriority::Interactive,
            [self, name, channel, value](const lcnc::process::DeviceCommandResult& result) {
                QMetaObject::invokeMethod(QCoreApplication::instance(),
                    [self, result, name, channel, value] {
                        if (!self)
                            return;
                        if (!result.success) {
                            self->setStatusMessage(
                                // 中文翻译：IO 输出 %1 切换失败: %2
                                self->tr("IO output %1 switching failed: %2").arg(name, result.error));
                            return;
                        }
                        if (self->m_digitalOutputs.value(name, false) != value
                            || !self->m_digitalOutputs.contains(name)) {
                            self->m_digitalOutputs.insert(name, value);
                            emit self->digitalOutputChanged(name, channel, value);
                        }
                        self->setStatusMessage(value
                            // 中文翻译：%1 已打开
                            ? self->tr("%1 is open").arg(name)
                            // 中文翻译：%1 已关闭
                            : self->tr("%1 is closed").arg(name));
                    }, Qt::QueuedConnection);
            })) {
        // 中文翻译：IO 输出 %1 命令未能排队
        setStatusMessage(tr("IO output %1 command failed to be queued").arg(name));
    }
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

    // 中文翻译：进给倍率: %1%
    setStatusMessage(tr("Feed rate: %1%").arg(qRound(m_feedOverride * 100.0)));
}

double ProcessModule::feedOverride() const
{
    return m_feedOverride;
}

QString ProcessModule::statusMessage() const
{
    return m_statusMessage;
}

void ProcessModule::onSimulationTick()
{
    if (m_state != State::Running || !m_simulationMode)
        return;
    // 普通切割管线运行期间由 PureSimulationToolpathTicker 直接驱动轴位，
    // 这里跳过 Lissajous 正弦波，避免两个驱动源互相覆盖。
    if (m_normalCuttingActive.load())
        return;

    m_simPhase += 0.08 * std::max(0.1, m_feedOverride);

    int linearIndex = 0;
    int rotaryIndex = 0;
    for (const MachineAxisDef& axis : m_axisDefinitions) {
        if (axis.name == QStringLiteral("BASE"))
            continue;

        const double value = lcnc::process::simulatedAxisValue(axis, m_simPhase, linearIndex, rotaryIndex);
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

void ProcessModule::setNormalCuttingActive(bool active)
{
    m_normalCuttingActive.store(active);
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
        // 控制器 SetAxisEnable 由 setAxisEnabled() 在用户操作时下发，并由
        // 硬件状态轮询回灌真实值；这里只维护本地缓存。
    }

    m_axisEnabled = nextStates;
    for (auto it = m_axisEnabled.cbegin(); it != m_axisEnabled.cend(); ++it)
        emit axisEnabledChanged(it.key(), it.value());
}

namespace {

struct HardwareAxisSample
{
    QString name;
    double  pos{0.0};
    bool    enabled{true};
    bool    valid{false};
};

struct HardwareDigitalOutputSample
{
    QString channel;
    QString displayName;
    bool    value{false};
    bool    valid{false};
};

struct HardwareIoBatch
{
    QVector<HardwareAxisSample> axes;
    QVector<HardwareDigitalOutputSample> digitalOutputs;
};

struct PeripheralStatusSample
{
    QString deviceName;
    bool connected{false};
    bool initialized{false};
    QString diagnostic;
    bool valid{false};
};

// 从 settings 一条 IO 子表里读取四个用得着的字段；兼容旧版 array 写法。
struct IOEntry
{
    QString channel;       // toml key
    QString name;
    QString index;
    bool    active{true};
    bool    enabled{true};
    bool    showInMain{false};
};

bool extractIOEntry(const std::string& tomlKey, const value& v, IOEntry& out)
{
    out.channel = QString::fromStdString(tomlKey);
    if (v.is_table()) {
        const auto& t = v.as_table();
        if (t.count("name"))       out.name       = QString::fromStdString(t.at("name").as_string());
        if (t.count("index"))      out.index      = QString::fromStdString(t.at("index").as_string());
        if (t.count("active"))     out.active     = t.at("active").as_boolean();
        if (t.count("enabled"))    out.enabled    = t.at("enabled").as_boolean();
        if (t.count("showInMain")) out.showInMain = t.at("showInMain").as_boolean();
        return true;
    }
    if (v.is_array()) {
        const auto& a = v.as_array();
        if (a.size() >= 1 && a.at(0).is_string()) out.name  = QString::fromStdString(a.at(0).as_string());
        if (a.size() >= 2 && a.at(1).is_string()) out.index = QString::fromStdString(a.at(1).as_string());
        return true;
    }
    return false;
}

bool tableBool(const table& t, const char* group, const char* key, bool fallback = false)
{
    auto groupIt = t.find(group);
    if (groupIt == t.end() || !groupIt->second.is_table())
        return fallback;
    const auto& sub = groupIt->second.as_table();
    auto it = sub.find(key);
    return it != sub.end() && it->second.is_boolean() ? it->second.as_boolean() : fallback;
}

double tableDouble(const table& t, const char* group, const char* key, double fallback = 0.0)
{
    auto groupIt = t.find(group);
    if (groupIt == t.end() || !groupIt->second.is_table())
        return fallback;
    const auto& sub = groupIt->second.as_table();
    auto it = sub.find(key);
    if (it == sub.end())
        return fallback;
    if (it->second.is_floating())
        return it->second.as_floating();
    if (it->second.is_integer())
        return static_cast<double>(it->second.as_integer());
    return fallback;
}

QString configuredIoChannel(const lcnc::process::ProcessSettingsService* settings, lcnc::process::ProcessIoBucket bucket, const QString& fallback)
{
    if (!settings) return fallback;
    for (const auto& channel : settings->ioChannels(bucket))
        if (channel.id == fallback) return channel.enabled ? channel.id : QString();
    return fallback;
}

QString enumNameFromTomlChannel(QString channel)
{
    channel = channel.trimmed();
    if (channel.startsWith(QLatin1Char('a')) && channel.size() >= 2)
        channel = channel.mid(1);
    return channel;
}

lcnc::ProcessMonitorSettings readMonitorSettings(const lcnc::process::ProcessSettingsService* config)
{
    lcnc::ProcessMonitorSettings settings;
    if (!config) return settings;
    const auto v = [config](const QString& table, const QString& key, const QVariant& fallback) {
        return config->rawValue(lcnc::process::ProcessConfigArea::Operations, table, key, fallback);
    };
    settings.interLockEnabled = v("Cutting", "bInterLock", false).toBool();
    settings.safetyLightCurtainEnabled = v("Cutting", "bSafetyLightCurtain", false).toBool();
    settings.pressureMonitorEnabled = v("Gas", "bPressureMonitor", false).toBool();
    settings.waterLeakageMonitorEnabled = v("Water", "bWaterLeakageMonitor", false).toBool();
    settings.waterTankMonitorEnabled = v("Water", "bWaterTankMonitor", false).toBool();
    settings.waterPressureMonitorEnabled = v("Water", "bWaterPressureMonitor", false).toBool();
    settings.waterPressureLimitMpa = v("Water", "fWaterPressureLimit", 1.0).toDouble();
    settings.waterLevelMonitorEnabled = v("Water", "bWaterLevelMonitor", false).toBool();
    settings.waterLevelLimitMm = v("Water", "fWaterLevelLimit", 50.0).toDouble();
    return settings;
}

QList<lcnc::process::ProcessMonitorOutputChannel> monitorOutputChannels(const lcnc::process::ProcessSettingsService* config)
{
    QList<lcnc::process::ProcessMonitorOutputChannel> out;
    if (config)
        for (const auto& channel : config->ioChannels(lcnc::process::ProcessIoBucket::DigitalOutput))
            if (channel.enabled && !channel.name.isEmpty()) out.append({channel.name, channel.id});
    return out;
}

} // namespace

void ProcessModule::pollHardwareStatus()
{
    if (m_hwPollInFlight)
        return;
    if (!m_connected || (m_simulationMode && !usesAcsSimulator(m_service.get())))
        return;
    if (!m_service)
        return;

    // 仅采集已注册的预设轴；BASE 与扩展轴在硬件层无对应 Axis 枚举，跳过。
    QStringList axisNames;
    for (const MachineAxisDef& axis : m_axisDefinitions) {
        if (axis.name == QStringLiteral("BASE"))
            continue;
        axisNames.append(axis.name.trimmed().toUpper());
    }

    // 数字量输出：所有 enabled 的 DigitalOUT 通道，全部扫描，便于硬件端
    // 直接驱动输出时界面也能同步刷新。
    QVector<QPair<QString, QString>> digitalOutputs; // <channel, display>
    if (auto* config = m_settingsService.get())
        for (const auto& channel : config->ioChannels(lcnc::process::ProcessIoBucket::DigitalOutput))
            if (channel.enabled && !channel.name.isEmpty()) digitalOutputs.append(qMakePair(channel.id, channel.name));

    if (axisNames.isEmpty() && digitalOutputs.isEmpty())
        return;

    if (!m_deviceCommandQueue)
        return;
    m_hwPollInFlight = true;
    QPointer<ProcessModule> self(this);
    const auto service = m_service;
    const auto batch = std::make_shared<HardwareIoBatch>();
    const bool queued = m_deviceCommandQueue->submit(
        [axisNames, digitalOutputs, service, batch]() {
        const auto deviceLock = service ? service->lockDeviceAccess()
                                        : Service::DeviceLock{};
        MotionControl* hwMc = service ? service->GetMotionControl() : nullptr;
        if (!hwMc || !hwMc->IsConnected())
            return lcnc::process::DeviceCommandResult{};
        batch->axes.reserve(axisNames.size());
        for (const QString& name : axisNames) {
            HardwareAxisSample sample;
            sample.name = name;
            auto eAxis = enum_cast<Axis>(name.toStdString());
            if (eAxis.has_value() && hwMc->IsMotorCreated(eAxis.value())) {
                double pos = 0.0;
                if (hwMc->GetActualPos(eAxis.value(), pos)) {
                    sample.pos = pos;
                    sample.enabled = hwMc->IsEnabled(eAxis.value());
                    sample.valid = true;
                }
            }
            batch->axes.push_back(sample);
        }
        batch->digitalOutputs.reserve(digitalOutputs.size());
        for (const auto& pair : digitalOutputs) {
            HardwareDigitalOutputSample sample;
            sample.channel = pair.first;
            sample.displayName = pair.second;
            // 通过 toml key（如 "aLaser"）映射到 DigitalOUT 枚举值，避免触发
            // QString 重载里基于显示名查找失败时的 WARN_MC_NONEINDEX 弹窗。
            QString enumName = pair.first;
            if (enumName.startsWith(QLatin1Char('a')) && enumName.size() >= 2)
                enumName = enumName.mid(1);
            auto eOut = enum_cast<DigitalOUT>(enumName.toStdString());
            if (eOut.has_value() && hwMc->m_mapDigitalOUT.count(eOut.value())) {
                int value = 0;
                if (hwMc->DigitalOutputGet(eOut.value(), value)) {
                    sample.value = value != 0;
                    sample.valid = true;
                }
            }
            batch->digitalOutputs.push_back(sample);
        }
        return lcnc::process::DeviceCommandResult{};
    }, TaskPriority::Polling,
    [self, batch](const lcnc::process::DeviceCommandResult& result) {
        QMetaObject::invokeMethod(QCoreApplication::instance(), [self, batch, result] {
            if (!self)
                return;
            self->m_hwPollInFlight = false;
            if (!self->m_initialized || !self->m_connected)
                return;
            if (!result.success) {
                // 中文翻译：设备状态读取失败: %1
                self->setStatusMessage(self->tr("Failed to read device status: %1").arg(result.error));
                return;
            }
            QStringList disabledAxesWhileRunning;
            for (const HardwareAxisSample& s : batch->axes) {
                if (!s.valid)
                    continue;
                self->setAxisPosition(s.name, s.pos);
                const bool present = self->m_axisEnabled.contains(s.name);
                const bool previous = self->m_axisEnabled.value(s.name, true);
                if (!present || previous != s.enabled) {
                    self->m_axisEnabled.insert(s.name, s.enabled);
                    emit self->axisEnabledChanged(s.name, s.enabled);
                }
                if (!s.enabled && self->m_state == State::Running)
                    disabledAxesWhileRunning.append(s.name);
            }
            if (!disabledAxesWhileRunning.isEmpty()) {
                // 中文翻译：加工过程中检测到轴系未使能: %1；已停止流程
                const QString message = self->tr("It was detected during processing that the axis system is not enabled: %1; the process has been stopped")
                    .arg(disabledAxesWhileRunning.join(self->tr("，")));
                LCNC_ERR(lcnc::LogCode::Generic,
                         "process: axis disabled while running: {}",
                         disabledAxesWhileRunning.join(QStringLiteral(",")).toStdString());
                // emergencyStop() 取消当前工作流、抢占下发安全停机命令，确保不会
                // 继续进入下一轮廓；随后显式落到 Error，而不是显示为正常完成。
                if (self->m_workflowExecutor)
                    self->m_workflowExecutor->emergencyStop();
                self->setState(State::Error, message);
            }
            for (const HardwareDigitalOutputSample& s : batch->digitalOutputs) {
                if (!s.valid)
                    continue;
                const bool present = self->m_digitalOutputs.contains(s.displayName);
                const bool previous = self->m_digitalOutputs.value(s.displayName, false);
                if (!present || previous != s.value) {
                    self->m_digitalOutputs.insert(s.displayName, s.value);
                    emit self->digitalOutputChanged(s.displayName, s.channel, s.value);
                }
            }
        }, Qt::QueuedConnection);
    }, QStringLiteral("controller-status"));
    if (!queued)
        m_hwPollInFlight = false;
}

void ProcessModule::pollPeripheralStatus()
{
    if (m_peripheralPollInFlight || !m_connected || m_simulationMode || !m_service)
        return;
    if (!m_deviceCommandQueue)
        return;

    m_peripheralPollInFlight = true;
    QPointer<ProcessModule> self(this);
    const auto service = m_service;
    const auto sample = std::make_shared<PeripheralStatusSample>();
    const bool queued = m_deviceCommandQueue->submit([service, sample] {
        const auto deviceLock = service ? service->lockDeviceAccess()
                                        : Service::DeviceLock{};
        LaserDevice* laser = service ? service->GetLaserDevice() : nullptr;
        if (!laser)
            return lcnc::process::DeviceCommandResult{};

        sample->deviceName = QString::fromStdString(laser->GetName());
        sample->connected = laser->IsConnected();
        sample->initialized = laser->IsInited();
        sample->valid = true;

        // Simulator/AnalogControl have no serial health command.  Real laser
        // protocols are queried only on this 2 s low-frequency worker.
        if (sample->connected && sample->deviceName != QStringLiteral("Simulator")
            && sample->deviceName != QStringLiteral("AnalogControl")) {
            sample->diagnostic = QString::fromStdString(laser->GetTroubleshooting());
        }
        return lcnc::process::DeviceCommandResult{};
    }, TaskPriority::Polling,
    [self, sample](const lcnc::process::DeviceCommandResult& result) {
        QMetaObject::invokeMethod(QCoreApplication::instance(), [self, sample, result] {
            if (!self)
                return;
            self->m_peripheralPollInFlight = false;
            if (!self->m_initialized || !self->m_connected || !result.success || !sample->valid)
                return;
            emit self->peripheralStatusChanged(sample->deviceName,
                                               sample->connected,
                                               sample->initialized,
                                               sample->diagnostic);
            if (!sample->connected || !sample->initialized) {
                const QString diagnostic = sample->diagnostic.isEmpty()
                    // 中文翻译：外设未连接或未初始化
                    ? self->tr("Peripheral is not connected or not initialized") : sample->diagnostic;
                if (diagnostic != self->m_lastPeripheralDiagnostic) {
                    self->m_lastPeripheralDiagnostic = diagnostic;
                    LCNC_WARN(lcnc::LogCode::Generic,
                             "Process peripheral '{}' health check: {}",
                             sample->deviceName.toStdString(), diagnostic.toStdString());
                }
            } else {
                self->m_lastPeripheralDiagnostic.clear();
            }
        }, Qt::QueuedConnection);
    }, QStringLiteral("peripheral-status"));
    if (!queued)
        m_peripheralPollInFlight = false;
}

QList<DigitalOutputDescriptor> ProcessModule::mainPanelDigitalOutputs() const
{
    QList<DigitalOutputDescriptor> out;
    auto* config = m_settingsService.get();
    if (!config) return out;
    for (const auto& entry : config->ioChannels(lcnc::process::ProcessIoBucket::DigitalOutput)) {
        if (!entry.enabled || !entry.showInMain || entry.name.isEmpty()) continue;
        DigitalOutputDescriptor d;
        d.channel = entry.id;
        d.name    = entry.name;
        d.active  = entry.activeHigh;
        out.push_back(d);
    }
    return out;
}

void ProcessModule::refreshIOFromSettings()
{
    // Settings application is queued separately. This GUI-thread method only
    // projects the new descriptors and clears stale display state.
    m_digitalOutputs.clear();
    emit digitalOutputDescriptorsChanged(mainPanelDigitalOutputs());
}

void ProcessModule::applySettingsChanges(const lcnc::process::ProcessSettingsChangeSet& changes)
{
    refreshIOFromSettings();
    if (!m_service || !m_deviceCommandQueue || changes.empty())
        return;

    const auto service = m_service;
    QPointer<ProcessModule> self(this);
    if (!m_deviceCommandQueue->submit([service, changes, self] {
            bool success = true;
            try {
                if (changes.domains.contains(QStringLiteral("devices"))) {
                    service->SetMotionControlTable();
                    service->SetLaserTable();
                }
                if (changes.domains.contains(QStringLiteral("io"))) {
                    service->SetDigitalTable();
                    service->SetAnalogTable();
                }
                if (changes.domains.contains(QStringLiteral("tools")))
                    service->SetToolTable();
                if (changes.domains.contains(QStringLiteral("operations")))
                    service->SetGasTable();
            } catch (const std::exception& exception) {
                success = false;
                LCNC_ERR(lcnc::LogCode::Generic,
                         "Process device settings application failed: {}", exception.what());
            } catch (...) {
                success = false;
                LCNC_ERR(lcnc::LogCode::Generic,
                         "Process device settings application threw an unknown exception");
            }

            if (self) {
                QMetaObject::invokeMethod(self, [self, success] {
                    if (self)
                        self->setStatusMessage(success
                            // 中文翻译：加工设备设置已应用
                            ? tr("Process equipment settings applied")
                            // 中文翻译：加工设备设置应用失败
                            : tr("Processing equipment settings application failed"));
                }, Qt::QueuedConnection);
            }
        })) {
        // 中文翻译：加工设备设置未能排队
        setStatusMessage(tr("Processing equipment setup failed to queue"));
        return;
    }
    // 中文翻译：正在应用加工设备设置
    setStatusMessage(tr("Applying process equipment settings"));
}

void ProcessModule::clearSafeOutputCache()
{
    const QStringList commonOutputs = {
        // 中文翻译：激光；吹气
        QStringLiteral("laser"), QStringLiteral("blow air"),
        // 中文翻译：夹头；水冷；气泵
        QStringLiteral("chuck"), QStringLiteral("water cooling"), QStringLiteral("air pump"),
        QStringLiteral("aLaser"), QStringLiteral("aBlow")
    };
    for (const QString& name : commonOutputs) {
        if (m_digitalOutputs.value(name, false)) {
            m_digitalOutputs.insert(name, false);
            emit digitalOutputChanged(name, name, false);
        }
    }
}

bool ProcessModule::triggerSafeStopOutputs()
{
    const auto deviceLock = m_service ? m_service->lockDeviceAccess()
                                      : Service::DeviceLock{};
    // 任何中断流程的路径都必须强制关闭激光与吹气。
    struct SafeChannel { QString channel; DigitalOUT eIndex; };
    static const QVector<SafeChannel> kSafeChannels = {
        { QStringLiteral("aLaser"), DigitalOUT::Laser },
        { QStringLiteral("aBlow"),  DigitalOUT::Blow  }
    };
    bool outputsSafe = true;
    if (m_service) {
        if (auto* mc = m_service->GetMotionControl()) {
            if (mc->IsConnected()) {
                for (const SafeChannel& sc : kSafeChannels) {
                    // 先 guard 是否已注册，避免触发 WARN_MC_NONEINDEX 弹窗。
                    if (mc->m_mapDigitalOUT.count(sc.eIndex)
                        && !mc->DigitalOutputSet(sc.eIndex, 0)) {
                        outputsSafe = false;
                        LCNC_ERR(lcnc::LogCode::Generic,
                                 "Process safe-stop: failed to reset output '{}'",
                                 sc.channel.toStdString());
                    }
                }
            }
        }
    }
    // 同步刷 UI 缓存。
    for (const SafeChannel& sc : kSafeChannels) {
        if (m_digitalOutputs.value(sc.channel, false)) {
            m_digitalOutputs.insert(sc.channel, false);
            emit digitalOutputChanged(sc.channel, sc.channel, false);
        }
    }
    return outputsSafe;
}

void ProcessModule::setState(State state, const QString& statusMessage)
{
    if (m_state != state) {
        m_state = state;
        emit stateChanged(m_state);
        // 中文翻译：状态机切换为 %1
        emit processLogMessage(QStringLiteral("state"), tr("State machine switches to %1").arg(processStateText(m_state)));
        // 任何流程被打断/异常的状态都强制关闭激光和吹气。
        if (m_state == State::Paused
            || m_state == State::Error
            || m_state == State::EmergencyStop) {
            if (m_deviceCommandQueue) {
                const auto service = m_service;
                if (!m_deviceCommandQueue->submitStop([service] {
                        (void)stopProcessHardware(service);
                    })) {
                    LCNC_ERR(lcnc::LogCode::Generic,
                             "Process safe-stop command could not be queued");
                }
            }
        }
    }

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

// ── Ribbon「加工顺序」状态 ────────────────────────────────────────────────

void ProcessModule::setAutoSortAxis(lcnc::process::AutoSortAxis a)
{
    if (m_autoSortAxis == a) return;
    m_autoSortAxis = a;
    if (m_cuttingPlanService)
        m_cuttingPlanService->setLastAutoSortAxis(a);
}

void ProcessModule::setAutoSortAxisFromText(const QString& text)
{
    setAutoSortAxis(lcnc::process::autoSortAxisFromString(text, m_autoSortAxis));
}

void ProcessModule::setTravelPathVisible(bool on)
{
    m_travelPathVisible = on;
}
