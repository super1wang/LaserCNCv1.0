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
#include "modules/cam/i_cam_contour_sequence_provider.h"
#include "modules/cam/i_cam_toolpath_provider.h"
#include "core/project/cam/layer_manager.h"
#include "modules/process/cutting/normal_cutting_manager.h"
#include "modules/process/cutting/process_cutting_plan_service.h"
#include "modules/process/execution/process_workflow_executor.h"
#include "modules/process/monitor/process_monitor_service.h"
#include "modules/process/setting/builtin_io_defs.h"
#include "modules/process/settings/process_settings_service.h"
#include "modules/process/runtime/device_command_queue.h"
#include "modules/process/runtime/process_axis_utilities.h"
#include "modules/process/steps/process_step_builtin_registration.h"
#include "modules/process/steps/process_step_registry.h"
#include "modules/process/steps/services/process_workflow_services.h"
#include "modules/process/runtime/process_device_runtime.h"
#include "modules/process/runtime/process_preflight_service.h"
#include "modules/process/runtime/process_connection_service.h"
#include "modules/process/runtime/process_manual_motion_service.h"
#include "modules/process/runtime/process_interactive_io_service.h"

#include "magic_enum.hpp"
#include "modules/process/runtime/process_status_service.h"
#include "modules/process/tool/tool_factory.h"
#include "modules/process/workflow/process_workflow_service.h"

#include <QList>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QPointer>
#include <QSet>
#include <QStringList>
#include <QTimer>
#include <QThread>
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
    case lcnc::ProcessRunState::Stopped:
        // 中文翻译：已停止
        return QObject::tr("Stopped");
    case lcnc::ProcessRunState::Error:
        // 中文翻译：错误
        return QObject::tr("Error");
    }
    // 中文翻译：未知
    return QObject::tr("unknown");
}

bool stopProcessHardware(const std::shared_ptr<ProcessDeviceRuntime>& service)
{
    return !service || service->stopMotionAndSafeOutputs();
}

QString logLevelForMessage(const QString& message)
{
    // 中文翻译：失败
    if (message.contains(QObject::tr("failed")) ||
        // 中文翻译：错误
        message.contains(QObject::tr("Error")) ||
        // 中文翻译：无法
        message.contains(QObject::tr("Unable"))) {
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
bool usesAcsSimulator(ProcessDeviceRuntime* service)
{
#if defined(LCNC_PROCESS_HAS_ACS) && LCNC_PROCESS_HAS_ACS
    return service && service->activeMotionControllerName() == "SimulatorCMHP";
#else
    Q_UNUSED(service);
    return false;
#endif
}

lcnc::ProcessMonitorSettings readMonitorSettings(const lcnc::process::ProcessSettingsService* settings);
QList<lcnc::process::ProcessMonitorOutputChannel> monitorOutputChannels(const lcnc::process::ProcessSettingsService* settings);
QString configuredIoChannel(const lcnc::process::ProcessSettingsService* settings, lcnc::process::ProcessIoBucket bucket, const QString& fallback);

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

lcnc::process::NormalCuttingCallbacks makeNormalCuttingCallbacks(ProcessModule* processModule)
{
    const QPointer<ProcessModule> guardedModule(processModule);
    lcnc::process::NormalCuttingCallbacks callbacks;
    callbacks.motionSink.positionObserver = [guardedModule](const QString& axisName, double position) {
        if (!guardedModule)
            return;
        const auto publish = [guardedModule, axisName, position] {
            if (guardedModule)
                guardedModule->setAxisPosition(axisName, position);
        };
        if (guardedModule->thread() == QThread::currentThread()) {
            publish();
            return;
        }
        QMetaObject::invokeMethod(guardedModule.data(), publish, Qt::QueuedConnection);
    };
    callbacks.motionSink.feedOverrideProvider = [guardedModule] {
        if (!guardedModule)
            return 1.0;
        double feedOverride = 1.0;
        const auto read = [guardedModule, &feedOverride] {
            if (guardedModule)
                feedOverride = guardedModule->feedOverride();
        };
        if (guardedModule->thread() == QThread::currentThread())
            read();
        else
            QMetaObject::invokeMethod(guardedModule.data(), read, Qt::BlockingQueuedConnection);
        return feedOverride;
    };
    callbacks.simulationModeProvider = [guardedModule] {
        if (!guardedModule)
            return false;
        bool simulationMode = false;
        const auto read = [guardedModule, &simulationMode] {
            if (guardedModule)
                simulationMode = guardedModule->simulationMode();
        };
        if (guardedModule->thread() == QThread::currentThread())
            read();
        else
            QMetaObject::invokeMethod(guardedModule.data(), read, Qt::BlockingQueuedConnection);
        return simulationMode;
    };
    callbacks.normalCuttingActivityObserver = [guardedModule](bool active) {
        if (!guardedModule)
            return;
        // This setter only stores an atomic flag. Keep this callback non-blocking
        // so module shutdown cannot deadlock while the GUI thread waits for the
        // workflow worker to finish.
        guardedModule->setNormalCuttingActive(active);
    };
    return callbacks;
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
    m_service = std::make_shared<ProcessDeviceRuntime>(*m_settingsService, m_runtimeConfiguration);
    m_deviceCommandQueue = std::make_unique<lcnc::process::DeviceCommandQueue>();
    if (!m_deviceCommandQueue->start()) {
        LCNC_ERR(lcnc::LogCode::Generic, "Process device command queue did not start");
        return false;
    }
    m_preflightService = std::make_shared<lcnc::process::ProcessPreflightService>(
        *m_service, *m_deviceCommandQueue);
    kernel.services().registerService<lcnc::process::ProcessPreflightService>(m_preflightService);
    auto workflowService = std::shared_ptr<lcnc::process::ProcessWorkflowService>(
        m_workflowService.get(), [](lcnc::process::ProcessWorkflowService*) {});
    kernel.services().registerService<lcnc::process::ProcessWorkflowService>(workflowService);
    auto workflowFacade = std::static_pointer_cast<lcnc::process::IProcessWorkflowService>(workflowService);
    kernel.services().registerService<lcnc::process::IProcessWorkflowService>(workflowFacade);
    m_connectionService = std::make_unique<lcnc::process::ProcessConnectionService>(
        *m_service, *m_deviceCommandQueue, this);
    m_manualMotionService = std::make_unique<lcnc::process::ProcessManualMotionService>(
        *m_service, *m_deviceCommandQueue, this);
    m_interactiveIoService = std::make_unique<lcnc::process::ProcessInteractiveIoService>(
        *m_service, *m_deviceCommandQueue, this,
        [this] {
            return !m_stopRecoveryRequired;
        });
    auto connectionService = std::shared_ptr<lcnc::process::ProcessConnectionService>(
        m_connectionService.get(), [](lcnc::process::ProcessConnectionService*) {});
    kernel.services().registerService<lcnc::process::ProcessConnectionService>(connectionService);
    m_statusService = std::make_unique<lcnc::process::ProcessStatusService>(
        *m_service, *m_deviceCommandQueue, this);
    auto statusService = std::shared_ptr<lcnc::process::ProcessStatusService>(
        m_statusService.get(), [](lcnc::process::ProcessStatusService*) {});
    kernel.services().registerService<lcnc::process::ProcessStatusService>(statusService);
    m_motionStepService = std::make_unique<lcnc::process::ProcessMotionWorkflowService>(
        m_service.get(), m_deviceCommandQueue.get());
    m_ioStepService = std::make_unique<lcnc::process::ProcessIoWorkflowService>(
        m_service.get(), m_deviceCommandQueue.get());
    m_cuttingStepService = std::make_unique<lcnc::process::CallbackProcessCuttingService>();
    auto svc = std::shared_ptr<ProcessModule>(this, [](ProcessModule*) {});
    kernel.services().registerService<ProcessModule>(svc);
    auto facade = std::shared_ptr<lcnc::IProcessFacade>(svc, static_cast<lcnc::IProcessFacade*>(this));
    kernel.services().registerService<lcnc::IProcessFacade>(facade);
    connect(m_workflowService.get(), &lcnc::process::ProcessWorkflowService::flowChanged,
            this, &ProcessModule::processFlowChanged);

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
        m_service.get(), camProvider, makeNormalCuttingCallbacks(this), m_deviceCommandQueue.get(), this);
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
            auto* cuttingPlan = m_cuttingPlanService.get();
            connect(mgr, &lcnc::cam::LayerManager::layersReset,
                    cuttingPlan, &lcnc::process::ProcessCuttingPlanService::notifyExternalPlanChanged);
            connect(mgr, &lcnc::cam::LayerManager::layerAdded,
                    cuttingPlan, [cuttingPlan](std::uint64_t) { cuttingPlan->notifyExternalPlanChanged(); });
            connect(mgr, &lcnc::cam::LayerManager::layerRemoved,
                    cuttingPlan, [cuttingPlan](std::uint64_t) { cuttingPlan->notifyExternalPlanChanged(); });
            connect(mgr, &lcnc::cam::LayerManager::layersReordered,
                    cuttingPlan, &lcnc::process::ProcessCuttingPlanService::notifyExternalPlanChanged);
            connect(mgr, &lcnc::cam::LayerManager::layerPropertyChanged,
                    cuttingPlan, [cuttingPlan](std::uint64_t, lcnc::cam::LayerProperty) {
                        cuttingPlan->notifyExternalPlanChanged();
                    });
            connect(mgr, &lcnc::cam::LayerManager::contourMembershipChanged,
                    cuttingPlan, &lcnc::process::ProcessCuttingPlanService::notifyExternalPlanChanged);
            connect(mgr, &lcnc::cam::LayerManager::manualContourOrderChanged,
                    cuttingPlan, &lcnc::process::ProcessCuttingPlanService::notifyExternalPlanChanged);
            connect(mgr, &lcnc::cam::LayerManager::sortStrategyChanged,
                    cuttingPlan, [cuttingPlan](lcnc::cam::CuttingPlanSortStrategy) {
                        cuttingPlan->notifyExternalPlanChanged();
                    });
        }
    }
    if (auto sequenceProvider = kernel.services().getService<lcnc::cam::ICamContourSequenceProvider>())
        m_cuttingPlanService->setContourSequenceProvider(sequenceProvider);
    {
        auto planService = std::shared_ptr<lcnc::process::ProcessCuttingPlanService>(
            m_cuttingPlanService.get(), [](lcnc::process::ProcessCuttingPlanService*) {});
        kernel.services().registerService<lcnc::process::ProcessCuttingPlanService>(planService);
        // CAM receives only the optional read-only tool-height bridge; it never
        // receives a Process-defined contour sequence.
        auto offsetProvider = std::static_pointer_cast<lcnc::cam::ICamToolOffsetProvider>(planService);
        kernel.services().registerService<lcnc::cam::ICamToolOffsetProvider>(offsetProvider);
    }
    // CAM 图层变更（新增/删除/重命名）时自动同步映射表。
    if (auto cam = kernel.services().getService<CamModule>()) {
        connect(cam.get(), &CamModule::cutterCollisionConfigurationChanged,
                m_cuttingPlanService.get(),
                &lcnc::process::ProcessCuttingPlanService::notifyExternalPlanChanged);
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
    connect(m_workflowExecutor.get(), &lcnc::process::ProcessWorkflowExecutor::messageLogged,
            this, &ProcessModule::setStatusMessage);
    connect(m_workflowExecutor.get(), &lcnc::process::ProcessWorkflowExecutor::nodeStateChanged,
            this, [this](const QString&) { m_workflowService->notifyChanged(); });
    connect(m_workflowExecutor.get(), &lcnc::process::ProcessWorkflowExecutor::nodeStarted,
            this, [this](const QString&) { m_workflowService->notifyChanged(); });
    connect(m_workflowExecutor.get(), &lcnc::process::ProcessWorkflowExecutor::nodeFinished,
            this, [this](const QString&) { m_workflowService->notifyChanged(); });
    connect(m_workflowExecutor.get(), &lcnc::process::ProcessWorkflowExecutor::nodeFailed,
            this, [this](const QString&, const QString& message) {
                m_workflowService->notifyChanged();
                // 中文翻译：流程节点失败: %1
                requestStop(StopOutcome::Error, tr("Process node failed: %1").arg(message));
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
                auto eOut = magic_enum::enum_cast<lcnc::process::DigitalOUT>(enumName.toStdString());
                if (!eOut.has_value())
                    return;
                const auto service = m_service;
                m_deviceCommandQueue->submit([service, output = eOut.value(), value] {
                    (void)service->setDigitalOutput(output, value);
                }, TaskPriority::Workflow);
            });
    connect(m_workflowExecutor.get(), &lcnc::process::ProcessWorkflowExecutor::workflowFinished,
            this, [this] {
                requestStop(StopOutcome::Idle, tr("The process is completed"));
                m_workflowService->notifyChanged();
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

        const auto service = m_service;
        // ProcessMonitorService owns a separate low-frequency pool.  It must
        // synchronously enter the device executor rather than using the old
        // cross-thread coordinator lease.  The response objects outlive a
        // timed-out vendor call, so a late completion cannot touch monitor
        // stack memory.
        auto* deviceQueue = m_deviceCommandQueue.get();
        context.readDigital = [service, deviceQueue](const QString& channel, bool* value, QString* errorMessage) {
            struct ReadResult { bool value{false}; QString error; };
            if (!value || !service || !deviceQueue || !deviceQueue->isRunning())
                return false;
            const auto response = std::make_shared<ReadResult>();
            const auto result = deviceQueue->executeAndWait(
                [service, channel, response] {
                    const bool ok = service->readDigitalChannel(channel, &response->value, &response->error);
                    return lcnc::process::DeviceCommandResult{ok, response->error};
                }, TaskPriority::Polling, 400);
            if (!result.success) {
                if (errorMessage) *errorMessage = result.error;
                return false;
            }
            *value = response->value;
            return true;
        };
        context.readAnalog = [service, deviceQueue](const QString& channel, double* value, QString* errorMessage) {
            struct ReadResult { double value{0.0}; QString error; };
            if (!value || !service || !deviceQueue || !deviceQueue->isRunning())
                return false;
            const auto response = std::make_shared<ReadResult>();
            const auto result = deviceQueue->executeAndWait(
                [service, channel, response] {
                    const bool ok = service->readAnalogChannel(channel, &response->value, &response->error);
                    return lcnc::process::DeviceCommandResult{ok, response->error};
                }, TaskPriority::Polling, 400);
            if (!result.success) {
                if (errorMessage) *errorMessage = result.error;
                return false;
            }
            *value = response->value;
            return true;
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
                    requestStop(StopOutcome::Error,
                                tr("Processing environment alarm: %1").arg(alarm.message));
                else if (alarm.action == lcnc::ProcessMonitorFaultAction::Pause)
                    runPause();
            });
    connect(m_monitorService.get(), &lcnc::process::ProcessMonitorService::snapshotUpdated,
            this, [this](const lcnc::process::ProcessMonitorSnapshot& snapshot) {
                if (snapshot.hasActiveAlarm())
                    // 中文翻译：加工环境报警: %1
                    setStatusMessage(tr("Processing environment alarm: %1").arg(snapshot.summary));
            });

    m_statusService->setRequestProvider([this] {
        lcnc::process::ProcessStatusRequest request;
        request.connected = m_connected;
        request.simulationMode = m_simulationMode;
        request.acsSimulator = usesAcsSimulator(m_service.get());
        for (const MachineAxisDef& axis : m_axisDefinitions) {
            if (axis.name != QStringLiteral("BASE"))
                request.axisNames.append(axis.name.trimmed().toUpper());
        }
        if (m_settingsService) {
            for (const auto& channel : m_settingsService->ioChannels(
                     lcnc::process::ProcessIoBucket::DigitalOutput)) {
                if (channel.enabled && !channel.name.isEmpty())
                    request.digitalOutputs.append(qMakePair(channel.id, channel.name));
            }
        }
        return request;
    });
    m_statusService->setHardwareHandler([this](const lcnc::process::DeviceCommandResult& result,
                                               const lcnc::process::DeviceStatusSnapshot& snapshot) {
        applyHardwareStatus(result, snapshot);
    });
    m_statusService->setPeripheralHandler([this](const lcnc::process::DeviceCommandResult& result,
                                                 const lcnc::process::DevicePeripheralSnapshot& snapshot) {
        applyPeripheralStatus(result, snapshot);
    });
    m_statusService->setSafetyMonitoringHandler([this](bool active) {
        if (!m_monitorService)
            return;
        if (active)
            m_monitorService->start();
        else
            m_monitorService->stop();
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
    if (m_deviceCommandQueue)
        m_deviceCommandQueue->beginStopOnly();

    bool workflowFinished = true;
    if (m_workflowExecutor) {
        m_workflowExecutor->stop();
        workflowFinished = m_workflowExecutor->waitForIdle(10000);
        if (!workflowFinished) {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "ProcessModule::stop: workflow thread did not exit before timeout");
            // 中文翻译：工作流停止超时，设备保持安全停机状态
            setState(State::Error, tr("Workflow stop times out, device remains in safe shutdown state"));
        }
    }

    // Process workers borrow ProcessDeviceRuntime and controller objects.  They must leave
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
        setState(State::Error,
                 // 中文翻译：Process 后台任务取消超时，已请求安全停机并保留设备运行时
                 tr("Process background task cancellation timeout, safe shutdown requested and device runtime preserved"));
        LCNC_ERR(lcnc::LogCode::Generic,
                 "ProcessModule::stop: owned task cancellation timed out");
    }
    m_deviceOperation = DeviceOperation::None;

    // No monitor may enqueue another read while the Stop lane performs safe
    // output reset, disconnect, and device-thread-owned destruction.
    stopDeviceMonitoring();

    bool disconnectOk = false;
    bool runtimeReleased = false;
    if (tasksFinished && workflowFinished && m_service && m_deviceCommandQueue) {
        const auto service = m_service;
        const auto teardown = m_deviceCommandQueue->executeAndWait(
            lcnc::process::DeviceCommandQueue::ResultCommand([service] {
                const bool outputsSafe = stopProcessHardware(service);
                const bool disconnected = service->shutdownDevices();
                const bool success = outputsSafe && disconnected;
                return lcnc::process::DeviceCommandResult{
                    success,
                    success
                        ? QString{}
                        : QObject::tr("Safety shutdown or disconnect failure when shutting down equipment")};
            }),
            TaskPriority::Stop,
            10000);
        disconnectOk = teardown.success;
        if (disconnectOk) {
            m_connected = false;
            auto runtimeHolder =
                std::make_shared<std::shared_ptr<ProcessDeviceRuntime>>(std::move(m_service));
            const auto release = m_deviceCommandQueue->executeAndWait(
                lcnc::process::DeviceCommandQueue::ResultCommand([runtimeHolder] {
                    runtimeHolder->reset();
                    return lcnc::process::DeviceCommandResult{};
                }),
                TaskPriority::Stop,
                10000);
            runtimeReleased = release.success;
            if (!release.success
                && release.completion != lcnc::process::DeviceCommandCompletion::TimedOut
                && *runtimeHolder) {
                m_service = std::move(*runtimeHolder);
            }
        } else {
            setState(State::Error,
                     teardown.error.isEmpty()
                         ? tr("Safety shutdown or disconnect failure when shutting down equipment")
                         : teardown.error);
            LCNC_ERR(lcnc::LogCode::Generic,
                     "ProcessModule::stop: safe shutdown did not complete");
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

    // Do not destroy or disconnect under an unfinished SDK call. The shared
    // ProcessDeviceRuntime lease retains those objects after any shutdown timeout.
    if ((!tasksFinished || !workflowFinished || !disconnectOk
         || !runtimeReleased || !deviceQueueFinished)
        && m_service) {
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
    m_workflowService = std::make_unique<lcnc::process::ProcessWorkflowService>(this);
    m_workflowService->createNew();
    initializeAxisPositions();

    m_simTimer = new QTimer(this);
    m_simTimer->setInterval(100);
    connect(m_simTimer, &QTimer::timeout, this, &ProcessModule::onSimulationTick);

    setStatusMessage(defaultStatusText(m_simulationMode, m_connected));
}

ProcessModule::~ProcessModule() = default;

bool ProcessModule::cancelOwnedTasks(int timeoutMs)
{
    auto* taskMgr = lcnc::Kernel::current().taskManager();
    if (!taskMgr || m_taskScope.empty())
        return true;
    return m_taskScope.cancelAndWait(*taskMgr, timeoutMs);
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
    if (!m_connectionService) {
        // 中文翻译：设备命令队列不可用
        setState(State::Error, tr("Device command queue is unavailable"));
        return;
    }

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
        if (!self || self->m_deviceOperation != DeviceOperation::Connecting)
            return;
        // 中文翻译：连接设备
        emit self->deviceConnectProgress(QObject::tr("Connect devices"), percent, step);
        // 中文翻译：正在连接设备：%1
        self->setStatusMessage(QObject::tr("Connecting device: %1").arg(step));
    };
    // 中文翻译：连接设备；已提交连接任务
    emit deviceConnectProgress(tr("Connect devices"), 0, tr("Connection task submitted"));
    // 中文翻译：正在连接设备：等待设备线程
    setStatusMessage(tr("Connecting device: waiting for device thread"));
    const auto ticket = m_connectionService->connect(
        pureSimulation,
        reportProgress,
        [self](const lcnc::process::DeviceCommandResult& result) {
            if (!self)
                return;
            const bool success = result.success;
            self->m_deviceOperation = DeviceOperation::None;
            self->m_connected = success;
            emit self->connectionChanged(success);
            if (success) {
                // 中文翻译：设备已连接
                self->setState(State::Idle, self->tr("Device is connected"));
                self->startDeviceMonitoring();
            } else {
                // 中文翻译：设备连接失败；未切换到纯软件仿真
                self->setState(State::Error,
                               self->tr("Device connection failed; not switched to software-only emulation"));
            }
            emit self->deviceConnectFinished(success,
                // 中文翻译：所有设备连接成功；设备连接失败，请检查设置
                success ? self->tr("All devices connected successfully")
                        : self->tr("Device connection failed, please check settings"));
        });
    if (!ticket.accepted) {
        m_deviceOperation = DeviceOperation::None;
        // 中文翻译：设备连接命令未能排队
        setState(State::Error, tr("Device connection command failed to queue"));
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

    if (m_simTimer)
        m_simTimer->stop();
    stopDeviceMonitoring();
    clearSafeOutputCache();

    // 中文翻译：正在断开设备...
    setStatusMessage(tr("Disconnecting device..."));
    m_deviceOperation = DeviceOperation::Disconnecting;
    const auto ticket = m_connectionService->disconnect(
        [self = QPointer<ProcessModule>(this)](const lcnc::process::DeviceCommandResult& result) {
            if (!self)
                return;
            const bool success = result.success;
            self->m_deviceOperation = DeviceOperation::None;
            self->m_connected = false;
            if (self->m_simTimer)
                self->m_simTimer->stop();
            emit self->connectionChanged(false);
            self->setState(success ? State::Idle : State::Error, success
                // 中文翻译：所有设备已断开
                ? self->tr("All devices are disconnected")
                // 中文翻译：设备断开过程中出现异常，请检查日志
                : self->tr("An exception occurred during device disconnection, please check the log."));
            if (success) {
                LCNC_INFO(lcnc::LogCode::Generic, "ProcessConnectionService: disconnected");
            } else {
                LCNC_ERR(lcnc::LogCode::Generic, "ProcessConnectionService: disconnect failed");
            }
            emit self->deviceConnectFinished(success,
                // 中文翻译：所有设备已断开；设备断开过程中出现异常
                success ? self->tr("All devices are disconnected")
                        : self->tr("An exception occurred during device disconnection"));
        });
    if (!ticket.accepted) {
        m_deviceOperation = DeviceOperation::None;
        // 中文翻译：设备命令队列不可用
        setState(State::Error, tr("Device command queue is unavailable"));
        emit deviceConnectFinished(false, tr("Device command queue is unavailable"));
    }
}

void ProcessModule::setSimulationMode(bool on)
{
    if (m_simulationMode == on)
        return;
    if (on && m_service && m_service->configuredControllerRequiresDevice())
        return;
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
    if (direction == 0 || !m_manualMotionService)
        return;
    const QString normalizedAxis = axisName.trimmed().toUpper();
    const double step = distance > 1e-9 ? distance : jogStepForLevel(speedLevel);
    const double delta = step * (direction > 0 ? 1.0 : -1.0);
    QPointer<ProcessModule> self(this);
    m_manualMotionService->moveRelative(normalizedAxis, delta, jogVelocityForLevel(speedLevel),
        m_axisEnabled.value(normalizedAxis, true), m_stopRecoveryRequired,
        [self](const QString& message) { if (self) self->setStatusMessage(message); });
}

void ProcessModule::moveAxisAbsolute(const QString& axisName, double position, int speedLevel)
{
    if (!m_manualMotionService)
        return;
    const QString normalizedAxis = axisName.trimmed().toUpper();
    QPointer<ProcessModule> self(this);
    m_manualMotionService->moveAbsolute(normalizedAxis, position, jogVelocityForLevel(speedLevel),
        m_axisEnabled.value(normalizedAxis, true), m_stopRecoveryRequired,
        [self](const QString& message) { if (self) self->setStatusMessage(message); });
}

void ProcessModule::startContinuousJog(const QString& axisName, int direction, int speedLevel)
{
    if (direction == 0 || !m_manualMotionService)
        return;
    const QString normalizedAxis = axisName.trimmed().toUpper();
    QPointer<ProcessModule> self(this);
    m_manualMotionService->startContinuous(normalizedAxis, direction > 0, jogVelocityForLevel(speedLevel),
        m_axisEnabled.value(normalizedAxis, true), m_stopRecoveryRequired,
        [self](const QString& message) { if (self) self->setStatusMessage(message); });
}

void ProcessModule::stopContinuousJog(const QString& axisName)
{
    if (!m_manualMotionService)
        return;
    QPointer<ProcessModule> self(this);
    m_manualMotionService->stopContinuous(axisName,
        [self](const QString& message) { if (self) self->setStatusMessage(message); });
}

void ProcessModule::home()
{
    if (m_stopRecoveryRequired) {
        // 中文翻译：停止复位前无法回零
        setStatusMessage(tr("Stop reset is required before returning to zero"));
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
            const auto result = service->homeAxes(axes, connected);
            const bool success = result.success;
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
            return result;
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

    QMap<QString, double> targets;
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
        const auto eAxis = magic_enum::enum_cast<lcnc::process::Axis>(axisName.toStdString());
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
        targets.insert(axisName, target);
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
                return service->moveToPreset(targets, jogVelocityForLevel(1), positionName);
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
    if (m_statusService)
        m_statusService->start();
}

void ProcessModule::stopDeviceMonitoring()
{
    if (m_statusService)
        m_statusService->stop();
}

bool ProcessModule::validateProcessingConfiguration(QString* errorMessage, bool requireIdle)
{
    auto fail = [errorMessage](const QString& message) {
        if (errorMessage)
            *errorMessage = message;
        return false;
    };

    if (requireIdle && m_state != State::Idle)
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
        m_workflowService->document().rootNodes(), lcnc::process::ProcessNodeType::NormalCutting);
    if (hasNormalCutting) {
        if (!m_cuttingPlanService)
            // 中文翻译：切割计划服务未初始化
            return fail(tr("Cutting plan service not initialized"));

        auto* toolpathProvider = lcnc::Kernel::current().service<lcnc::cam::ICamToolpathProvider>();
        auto* machineConfig = lcnc::Kernel::current().service<lcnc::MachineConfigurationService>();
        if (!toolpathProvider || !machineConfig)
            // 中文翻译：机床构型或 CAM 刀路服务未初始化
            return fail(tr("Machine configuration or CAM toolpath service is not initialized"));
        QString machineConfigurationError;
        if (!machineConfig->validateConfiguration(&machineConfigurationError))
            // 中文翻译：机床配置无效：%1
            return fail(tr("The machine configuration is invalid: %1")
                        .arg(machineConfigurationError));
        const lcnc::cam::ToolpathExportSnapshot snapshot = toolpathProvider->exportToolpathSnapshot();
        // 中文翻译：Process 预检使用当前 CAM 快照的加工模式、轴布局和求解器契约。
        LCNC_INFO(lcnc::LogCode::Generic,
                  "process.preflight: CAM snapshot mode='{}' axes={} solver='{}' revision={}",
                  lcnc::machiningModeName(snapshot.machiningMode).toStdString(),
                  snapshot.machineAxisLayout.count,
                  snapshot.solverId.toStdString(),
                  snapshot.revision);
        QString layoutError;
        if (!snapshot.machineAxisLayout.isValid(&layoutError))
            // 中文翻译：刀路轴布局无效: %1
            return fail(tr("The toolpath axis layout is invalid: %1").arg(layoutError));
        if (!machineConfig->supportsMachiningMode(snapshot.machiningMode))
            // 中文翻译：当前机床不支持工程加工模式 %1
            return fail(tr("The current machine does not support project machining mode %1")
                        .arg(lcnc::machiningModeName(snapshot.machiningMode)));
        const lcnc::MachineModeDefinition definition =
            machineConfig->modeDefinition(snapshot.machiningMode);
        if (snapshot.machineAxisLayout != definition.interpolatedAxes
            || snapshot.solverId != definition.solverId
            || snapshot.solverVersion != definition.solverVersion)
            // 中文翻译：刀路求解契约与当前机床模式不匹配，请重新求解机床坐标
            return fail(tr("The toolpath solving contract does not match the current machine mode; please solve the machine coordinates again"));
        if (!m_simulationMode
            && snapshot.machineConfigurationFingerprint != machineConfig->configurationFingerprint())
            // 中文翻译：刀路机床构型指纹不匹配，请重新求解机床坐标
            return fail(tr("The toolpath machine configuration fingerprint does not match; please solve the machine coordinates again"));

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
    if (m_stopInFlight || m_stopRecoveryRequired || m_stopRecoveryInFlight) {
        // 中文翻译：停止或复位尚未完成
        setStatusMessage(tr("Stop or reset has not completed"));
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

    m_activeLockedAxisTargets.clear();
    if (auto* provider = lcnc::Kernel::current().service<lcnc::cam::ICamToolpathProvider>()) {
        const auto snapshot = provider->exportToolpathSnapshot();
        if (auto* machineConfig = lcnc::Kernel::current().service<lcnc::MachineConfigurationService>())
            m_activeLockedAxisTargets = machineConfig->modeDefinition(snapshot.machiningMode).lockedAxisTargets;
    }

    if (m_simulationMode) {
        for (auto it = m_activeLockedAxisTargets.cbegin();
             it != m_activeLockedAxisTargets.cend(); ++it) {
            setAxisPosition(it.key(), it.value());
        }
        startWorkflowAfterPreflight();
        return;
    }

    if (!m_deviceCommandQueue || !m_service) {
        // 中文翻译：加工环境检查失败: 设备命令队列不可用
        setState(State::Error, tr("Processing environment check failed: Device command queue unavailable"));
        return;
    }

    lcnc::process::ProcessPreflightRequest request;
    for (const MachineAxisDef& axis : m_axisDefinitions)
        request.axisNames.append(axis.name);
    request.lockedAxisTargets = m_activeLockedAxisTargets;

    const lcnc::ProcessMonitorSettings monitorSettings =
        readMonitorSettings(m_settingsService.get());
    request.digitalGuards = {
        {monitorSettings.interLockEnabled,
         // 中文翻译：门禁
         tr("access control"),
         configuredIoChannel(m_settingsService.get(),
             lcnc::process::ProcessIoBucket::DigitalInput, QStringLiteral("aInterLock"))},
        {monitorSettings.safetyLightCurtainEnabled,
         // 中文翻译：安全光栅
         tr("Safety grating"),
         configuredIoChannel(m_settingsService.get(),
             lcnc::process::ProcessIoBucket::DigitalInput, QStringLiteral("aSafetyLightCurtain"))},
        {monitorSettings.pressureMonitorEnabled,
         // 中文翻译：气压监控
         tr("Air pressure monitoring"),
         configuredIoChannel(m_settingsService.get(),
             lcnc::process::ProcessIoBucket::DigitalInput, QStringLiteral("aPressureMonitor"))},
        {monitorSettings.waterLeakageMonitorEnabled,
         // 中文翻译：漏水监控
         tr("Water leakage monitoring"),
         configuredIoChannel(m_settingsService.get(),
             lcnc::process::ProcessIoBucket::DigitalInput, QStringLiteral("aWaterLeakageMonitor"))},
        {monitorSettings.waterTankMonitorEnabled,
         // 中文翻译：水箱监控
         tr("Water tank monitoring"),
         configuredIoChannel(m_settingsService.get(),
             lcnc::process::ProcessIoBucket::DigitalInput, QStringLiteral("aWaterTankMonitor"))}};
    request.analogGuards = {
        {monitorSettings.waterPressureMonitorEnabled,
         // 中文翻译：水压监控
         tr("water pressure monitoring"),
         configuredIoChannel(m_settingsService.get(),
             lcnc::process::ProcessIoBucket::AnalogInput, QStringLiteral("aWaterPressure")),
         monitorSettings.waterPressureLimitMpa,
         QStringLiteral("MPa")},
        {monitorSettings.waterLevelMonitorEnabled,
         // 中文翻译：水位监控
         tr("water level monitoring"),
         configuredIoChannel(m_settingsService.get(),
             lcnc::process::ProcessIoBucket::AnalogInput, QStringLiteral("aWaterLevel")),
         monitorSettings.waterLevelLimitMm,
         QStringLiteral("mm")}};

    const std::uint64_t requestGeneration = ++m_runRequestGeneration;
    m_preflightInFlight = true;
    // 中文翻译：正在后台检查加工环境...
    setStatusMessage(tr("Checking the processing environment in the background..."));

    QPointer<ProcessModule> self(this);
    const auto ticket = m_preflightService->request(
        requestGeneration,
        std::move(request),
        [self](std::uint64_t generation,
               const lcnc::process::DeviceCommandResult& result,
               std::shared_ptr<const lcnc::process::ProcessPreflightReport> report) {
            QMetaObject::invokeMethod(QCoreApplication::instance(),
                [self, generation, result, report = std::move(report)] {
                    if (!self)
                        return;
                    self->m_preflightInFlight = false;
                    if (generation != self->m_runRequestGeneration)
                        return;
                    if (!result.success) {
                        self->setState(State::Error,
                            // 中文翻译：加工环境检查失败: %1
                            self->tr("Processing environment check failed: %1").arg(result.error));
                        return;
                    }
                    for (auto it = report->axisPositions.cbegin();
                         it != report->axisPositions.cend(); ++it) {
                        self->setAxisPosition(it.key(), it.value());
                    }
                    for (auto it = report->axisEnabled.cbegin();
                         it != report->axisEnabled.cend(); ++it) {
                        const bool changed =
                            !self->m_axisEnabled.contains(it.key())
                            || self->m_axisEnabled.value(it.key()) != it.value();
                        self->m_axisEnabled.insert(it.key(), it.value());
                        if (changed)
                            emit self->axisEnabledChanged(it.key(), it.value());
                    }
                    self->startDeviceMonitoring();
                    self->startWorkflowAfterPreflight();
                }, Qt::QueuedConnection);
        });
    if (!ticket.accepted) {
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
        if (!m_workflowExecutor->start(m_workflowService->document(), &errorMessage)) {
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
    requestStop(StopOutcome::Stopped, tr("Stop completed and safety outputs reset"));
}

void ProcessModule::requestStop(StopOutcome outcome, const QString& statusMessage)
{
    ++m_runRequestGeneration;
    m_activeLockedAxisTargets.clear();
    // Queue gating exists only while this safe-stop transaction is unfinished.
    // It must not persist merely because the resulting state is Error/Stopped.
    m_stopRecoveryRequired = true;
    if (m_deviceCommandQueue)
        m_deviceCommandQueue->beginStopOnly();
    if (outcome == StopOutcome::Error || !m_stopInFlight) {
        m_pendingStopOutcome = outcome;
        m_pendingStopMessage = statusMessage;
    }
    if (m_stopInFlight)
        return;
    m_simTimer->stop();
    if (m_workflowExecutor)
        m_workflowExecutor->stop();
    if (m_simulationMode) {
        m_stopRecoveryRequired = false;
        if (m_deviceCommandQueue)
            m_deviceCommandQueue->endStopOnly();
        const State finalState = outcome == StopOutcome::Error ? State::Error
            : outcome == StopOutcome::Stopped ? State::Stopped : State::Idle;
        setState(finalState, statusMessage);
        return;
    }
    if (!m_deviceCommandQueue) {
        setState(State::Error, tr("Stop command queue is unavailable"));
        return;
    }

    m_stopInFlight = true;
    const auto service = m_service;
    QPointer<ProcessModule> self(this);
    if (!m_deviceCommandQueue->submit(
            lcnc::process::DeviceCommandQueue::ResultCommand([service] {
                const bool success = stopProcessHardware(service);
                return lcnc::process::DeviceCommandResult{
                    success, success ? QString() : QObject::tr("Safety shutdown failed")};
            }),
            TaskPriority::Stop,
            [self](const lcnc::process::DeviceCommandResult& result) {
                QMetaObject::invokeMethod(QCoreApplication::instance(), [self, result] {
                    if (!self)
                        return;
                    self->m_stopInFlight = false;
                    if (!result.success) {
                        self->m_stopRecoveryRequired = true;
                        self->setState(State::Error,
                            self->tr("Safety shutdown failed while stopping: %1").arg(result.error));
                        return;
                    }
                    self->clearSafeOutputCache();
                    self->m_stopRecoveryRequired = false;
                    if (self->m_deviceCommandQueue)
                        self->m_deviceCommandQueue->endStopOnly();
                    const State finalState = self->m_pendingStopOutcome == StopOutcome::Error
                        ? State::Error : self->m_pendingStopOutcome == StopOutcome::Stopped
                        ? State::Stopped : State::Idle;
                    self->setState(finalState, self->m_pendingStopMessage);
                }, Qt::QueuedConnection);
            })) {
        m_stopInFlight = false;
        m_stopRecoveryRequired = true;
        setState(State::Error, tr("Stop command failed to queue"));
    }
}

void ProcessModule::resetStop()
{
    if (m_stopInFlight || m_stopRecoveryInFlight)
        return;
    if (m_state == State::Error) {
        QString configurationError;
        // 中文翻译：停止复位必须再次检查导致加工无法开始的配置错误。
        if (!validateProcessingConfiguration(&configurationError, false)) {
            // 中文翻译：停止复位配置检查失败: %1
            setState(State::Error,
                     tr("Stop reset configuration check failed: %1").arg(configurationError));
            return;
        }
    }
    if (m_simulationMode) {
        m_stopRecoveryRequired = false;
        if (m_deviceCommandQueue)
            m_deviceCommandQueue->endStopOnly();
        setState(State::Idle, defaultStatusText(m_simulationMode, m_connected));
        return;
    }
    if (!m_deviceCommandQueue || !m_service) {
        setStatusMessage(tr("Stop reset requires an active device executor"));
        return;
    }

    m_stopRecoveryInFlight = true;
    const auto service = m_service;
    QPointer<ProcessModule> self(this);
    if (!m_deviceCommandQueue->submit(
            lcnc::process::DeviceCommandQueue::ResultCommand([service] {
                if (!stopProcessHardware(service))
                    return lcnc::process::DeviceCommandResult{false,
                        QObject::tr("Safety shutdown failed during stop reset")};
                return service->validateContourBoundary();
            }),
            TaskPriority::Stop,
            [self](const lcnc::process::DeviceCommandResult& result) {
                QMetaObject::invokeMethod(QCoreApplication::instance(), [self, result] {
                    if (!self)
                        return;
                    self->m_stopRecoveryInFlight = false;
                    if (!result.success) {
                        self->setState(State::Error, self->tr("Stop reset health check failed: %1")
                                               .arg(result.error));
                        return;
                    }
                    self->clearSafeOutputCache();
                    self->m_stopRecoveryRequired = false;
                    if (self->m_deviceCommandQueue)
                        self->m_deviceCommandQueue->endStopOnly();
                    self->setState(State::Idle,
                                   self->tr("Stop reset completed and device health verified"));
                }, Qt::QueuedConnection);
            })) {
        m_stopRecoveryInFlight = false;
        setState(State::Error, tr("Stop reset command failed to queue"));
    }
}

void ProcessModule::newProcess()
{
    m_workflowService->createNew();
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
    if (!m_workflowService->load(filePath, &errorMessage)) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "ProcessModule: load process failed: {}",
                  errorMessage.toStdString());
        // 中文翻译：加载流程失败: %1
        setStatusMessage(tr("Loading process failed: %1").arg(errorMessage));
        return false;
    }

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
    if (!m_workflowService->save(filePath, &errorMessage)) {
        // 中文翻译：保存流程失败: %1
        setStatusMessage(tr("Save process failed: %1").arg(errorMessage));
        return false;
    }

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
    if (normalizedAxis.isEmpty() || !m_interactiveIoService)
        return;
    QPointer<ProcessModule> self(this);
    m_interactiveIoService->setAxisEnabled(normalizedAxis, enabled,
        [self, normalizedAxis, enabled](const lcnc::process::DeviceCommandResult& result) {
            if (!self)
                return;
            if (!result.success) {
                self->setStatusMessage(self->tr("%1 Axis enable switching failed: %2")
                                       .arg(normalizedAxis, result.error));
                return;
            }
            if (self->m_axisEnabled.value(normalizedAxis, true) != enabled
                || !self->m_axisEnabled.contains(normalizedAxis)) {
                self->m_axisEnabled.insert(normalizedAxis, enabled);
                emit self->axisEnabledChanged(normalizedAxis, enabled);
            }
            self->setStatusMessage(enabled ? self->tr("%1 axis is enabled").arg(normalizedAxis)
                                           : self->tr("%1 axis is disabled").arg(normalizedAxis));
        });
}

void ProcessModule::setDigitalOutput(const QString& outputName, bool value)
{
    const QString name = outputName.trimmed();
    if (name.isEmpty() || !m_interactiveIoService)
        return;
    QPointer<ProcessModule> self(this);
    m_interactiveIoService->setDigitalOutput(name, value,
        [self, name, value](const lcnc::process::DeviceCommandResult& result) {
            if (!self)
                return;
            if (!result.success) {
                self->setStatusMessage(self->tr("IO output %1 switching failed: %2").arg(name, result.error));
                return;
            }
            if (self->m_digitalOutputs.value(name, false) != value
                || !self->m_digitalOutputs.contains(name)) {
                self->m_digitalOutputs.insert(name, value);
                emit self->digitalOutputChanged(name, name, value);
            }
            self->setStatusMessage(value ? self->tr("%1 is open").arg(name)
                                         : self->tr("%1 is closed").arg(name));
        });
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

void ProcessModule::synchronizeAxisFeedback()
{
    // A newly created coordinate panel has no labels to receive earlier
    // signals.  Replay only Process-owned feedback, then request the normal
    // controller polling path for a fresh physical sample.  Opening or
    // closing a project must never invent a machine coordinate.
    // 中文翻译：坐标面板重建后重放 Process 保存的控制器反馈，并立即走正常轮询获取物理坐标；文件操作绝不能生成坐标。
    for (auto it = m_axisPositions.cbegin(); it != m_axisPositions.cend(); ++it)
        emit axisPositionChanged(it.key(), it.value());

    if (m_connected && m_statusService)
        m_statusService->requestHardwarePoll();
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

        // Changing the configured axis envelope (for example while opening a
        // new project) is not a controller homing operation.  Keep the most
        // recent feedback for axes that still exist, otherwise this method
        // publishes false zero positions and temporarily moves the live CAM
        // view back to the machine home posture.
        // 中文翻译：轴定义变化不是回零；保留仍存在轴的最近反馈，不能向 CAM 发布伪造的零位姿态。
        nextPositions.insert(axis.name, m_axisPositions.value(axis.name, 0.0));
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

using HardwareAxisSample = lcnc::process::DeviceAxisStatusSample;
using HardwareDigitalOutputSample = lcnc::process::DeviceDigitalOutputSample;
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

void ProcessModule::applyHardwareStatus(
    const lcnc::process::DeviceCommandResult& result,
    const lcnc::process::DeviceStatusSnapshot& batch)
{
    if (!m_initialized || !m_connected)
        return;
    if (!result.success) {
        // 中文翻译：设备状态读取失败: %1
        setStatusMessage(tr("Failed to read device status: %1").arg(result.error));
        return;
    }

    QStringList disabledAxesWhileRunning;
    QStringList driftingLockedAxes;
    for (const lcnc::process::DeviceAxisStatusSample& sample : batch.axes) {
        if (!sample.valid)
            continue;
        setAxisPosition(sample.name, sample.pos);
        const bool present = m_axisEnabled.contains(sample.name);
        const bool previous = m_axisEnabled.value(sample.name, true);
        if (!present || previous != sample.enabled) {
            m_axisEnabled.insert(sample.name, sample.enabled);
            emit axisEnabledChanged(sample.name, sample.enabled);
        }
        if (!sample.enabled && m_state == State::Running)
            disabledAxesWhileRunning.append(sample.name);
        const auto locked = m_activeLockedAxisTargets.constFind(sample.name);
        if (m_state == State::Running && locked != m_activeLockedAxisTargets.cend()
            && std::abs(sample.pos - locked.value()) > 0.05) {
            driftingLockedAxes.append(
                QStringLiteral("%1 (%2/%3)").arg(sample.name).arg(sample.pos, 0, 'f', 3)
                    .arg(locked.value(), 0, 'f', 3));
        }
    }
    if (!disabledAxesWhileRunning.isEmpty()) {
        // 中文翻译：加工过程中检测到轴系未使能: %1；已停止流程
        const QString message = tr("It was detected during processing that the axis system is not enabled: %1; the process has been stopped")
            .arg(disabledAxesWhileRunning.join(tr("，")));
        LCNC_ERR(lcnc::LogCode::Generic,
                 "process: axis disabled while running: {}",
                 disabledAxesWhileRunning.join(QStringLiteral(",")).toStdString());
        requestStop(StopOutcome::Error, message);
    }
    if (!driftingLockedAxes.isEmpty() && !m_stopInFlight) {
        // 中文翻译：加工期间锁定轴发生位置漂移: %1；已执行安全停机
        const QString message = tr("Locked-axis position drift occurred during processing: %1; safe shutdown was requested")
            .arg(driftingLockedAxes.join(QStringLiteral(", ")));
        LCNC_ERR(lcnc::LogCode::Generic,
                 "process: locked axis drift while running: {}",
                 driftingLockedAxes.join(QStringLiteral(",")).toStdString());
        requestStop(StopOutcome::Error, message);
    }
    for (const lcnc::process::DeviceDigitalOutputSample& sample : batch.digitalOutputs) {
        if (!sample.valid)
            continue;
        const bool present = m_digitalOutputs.contains(sample.displayName);
        const bool previous = m_digitalOutputs.value(sample.displayName, false);
        if (!present || previous != sample.value) {
            m_digitalOutputs.insert(sample.displayName, sample.value);
            emit digitalOutputChanged(sample.displayName, sample.channel, sample.value);
        }
    }
}

void ProcessModule::applyPeripheralStatus(
    const lcnc::process::DeviceCommandResult& result,
    const lcnc::process::DevicePeripheralSnapshot& sample)
{
    if (!m_initialized || !m_connected || !result.success || !sample.valid)
        return;
    emit peripheralStatusChanged(sample.deviceName,
                                 sample.connected,
                                 sample.initialized,
                                 sample.diagnostic);
    if (!sample.connected || !sample.initialized) {
        const QString diagnostic = sample.diagnostic.isEmpty()
            // 中文翻译：外设未连接或未初始化
            ? tr("Peripheral is not connected or not initialized") : sample.diagnostic;
        if (diagnostic != m_lastPeripheralDiagnostic) {
            m_lastPeripheralDiagnostic = diagnostic;
            LCNC_WARN(lcnc::LogCode::Generic,
                      "Process peripheral '{}' health check: {}",
                      sample.deviceName.toStdString(), diagnostic.toStdString());
        }
    } else {
        m_lastPeripheralDiagnostic.clear();
    }
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
                    service->setMotionControlTable();
                    service->setLaserTable();
                }
                if (changes.domains.contains(QStringLiteral("io"))) {
                    service->setDigitalTable();
                    service->setAnalogTable();
                }
                if (changes.domains.contains(QStringLiteral("tools")))
                    service->setToolTable();
                if (changes.domains.contains(QStringLiteral("operations")))
                    service->setGasTable();
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
    const QSet<QString> safeChannels = {
        QStringLiteral("aLaser"), QStringLiteral("aBlow"), QStringLiteral("aBlow2")};
    QHash<QString, QString> channelsByName;
    for (const QString& channel : safeChannels)
        channelsByName.insert(channel, channel);
    if (m_settingsService) {
        for (const auto& channel : m_settingsService->ioChannels(
                 lcnc::process::ProcessIoBucket::DigitalOutput)) {
            if (channel.enabled && safeChannels.contains(channel.id) && !channel.name.isEmpty())
                channelsByName.insert(channel.name, channel.id);
        }
    }

    // Keep every configured display name synchronized with the forced safe IO state.
    for (auto it = channelsByName.cbegin(); it != channelsByName.cend(); ++it) {
        const QString& name = it.key();
        if (m_digitalOutputs.value(name, false)) {
            m_digitalOutputs.insert(name, false);
            emit digitalOutputChanged(name, it.value(), false);
        }
    }
}

void ProcessModule::setState(State state, const QString& statusMessage)
{
    if (!m_runCoordinator.transitionTo(state)) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Illegal Process state transition: {} -> {}",
                 static_cast<int>(m_state),
                 static_cast<int>(state));
        (void)m_runCoordinator.transitionTo(State::Error);
        state = State::Error;
    }

    if (m_state != state) {
        m_state = state;
        emit stateChanged(m_state);
        // 中文翻译：状态机切换为 %1
        emit processLogMessage(QStringLiteral("state"), tr("State machine switches to %1").arg(processStateText(m_state)));
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
