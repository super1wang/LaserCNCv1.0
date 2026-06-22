#include "modules/process/process_module.h"

#include "core/kernel/i_kernel.h"
#include "core/kernel/kernel.h"
#include "core/kinematics/machine_configuration_service.h"
#include "core/logging/logger.h"
#include "core/task/task_manager.h"
#include "core/task/task_progress.h"
#include "modules/cam/i_cam_toolpath_provider.h"
#include "modules/process/execution/process_workflow_executor.h"
#include "modules/process/Setting/BuiltinIODefs.h"
#include "modules/process/Setting/Settings.h"
#include "modules/process/steps/process_step_builtin_registration.h"
#include "modules/process/steps/process_step_registry.h"
#include "modules/process/steps/services/legacy_process_services.h"
#include "modules/process/System/Service.h"
#include "modules/process/workflow/process_flow_store.h"

#include <QList>
#include <QPointer>
#include <QTimer>
#include <QVector>
#include <QElapsedTimer>
#include <QThread>
#include <QFutureWatcher>
#include <QtConcurrent>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

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
    m_service = std::make_unique<Service>();
    m_motionStepService = std::make_unique<lcnc::process::LegacyProcessMotionService>(m_service.get());
    m_ioStepService = std::make_unique<lcnc::process::LegacyProcessIoService>(m_service.get());
    m_cuttingStepService = std::make_unique<lcnc::process::CallbackProcessCuttingService>();
    auto svc = std::shared_ptr<ProcessModule>(this, [](ProcessModule*) {});
    kernel.services().registerService<ProcessModule>(svc);
    auto facade = std::shared_ptr<lcnc::IProcessFacade>(svc, static_cast<lcnc::IProcessFacade*>(this));
    kernel.services().registerService<lcnc::IProcessFacade>(facade);

    // 加载外设/工艺参数（连接控制器与构造轴系/IO 表都依赖这些 toml）。
    // qg_dlgsetting 在打开时会再次加载，幂等。
    SETTINGS->LoadSettings("./Peripheral.toml");
    SETTINGS->LoadSettings("./config.toml");

    // 用静态预设清单填充 settings 中缺失的 IO 子表（已存在的不动），保证
    // 即使旧 toml 没有新字段也能开箱即用。
    seedDefaultIOTables();

    // 注册自定义信号类型，跨线程发射 / Qt::QueuedConnection 时需要。
    qRegisterMetaType<DigitalOutputDescriptor>("DigitalOutputDescriptor");
    qRegisterMetaType<QList<DigitalOutputDescriptor>>("QList<DigitalOutputDescriptor>");

    // 注册内置流程步骤插件（执行器只通过插件分发，不再有业务 fallback）。
    auto& stepRegistry = lcnc::process::ProcessStepRegistry::instance();
    registerBuiltinProcessSteps(stepRegistry);

    // 从 settings 恢复插件启用/禁用状态。
    {
        const table specialTable = SETTINGS->GetTable(SettingSection::Special);
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
    m_cuttingStepService->setExecutor([](bool dryRun, QString* errorMessage) {
        Q_UNUSED(dryRun);
        Q_UNUSED(errorMessage);
        // 真正切割管线后续接 NormalCuttingManager；当前占位返回成功。
        return true;
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
    m_stepContext.logMessage = [this](const QString& message) { setStatusMessage(message); };
    m_stepContext.requestAxisPosition = [this](const QString& axis, double value) { setAxisPosition(axis, value); };
    m_stepContext.requestDigitalOutput = [this](const QString& channel, bool value) { setDigitalOutput(channel, value); };
    m_workflowExecutor->setStepRegistry(&stepRegistry);
    m_workflowExecutor->setStepContext(&m_stepContext);
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
    connect(m_workflowExecutor.get(), &lcnc::process::ProcessWorkflowExecutor::digitalOutputRequested,
            this, [this](const QString& channel, bool value) {
                // 兼容信号路径；正常流程通过 OutputSignalStep + LegacyProcessIoService 走枚举接口。
                if (!m_service)
                    return;
                auto* mc = m_service->GetMotionControl();
                if (!mc || !mc->IsConnected())
                    return;
                QString enumName = channel.trimmed();
                if (enumName.startsWith(QLatin1Char('a')) && enumName.size() >= 2)
                    enumName = enumName.mid(1);
                auto eOut = enum_cast<DigitalOUT>(enumName.toStdString());
                if (eOut.has_value() && mc->m_mapDigitalOUT.count(eOut.value()))
                    mc->DigitalOutputSet(eOut.value(), value ? 1 : 0);
            });
    connect(m_workflowExecutor.get(), &lcnc::process::ProcessWorkflowExecutor::workflowFinished,
            this, [this] {
                m_simTimer->stop();
                safeStopProcessOutputs();
                setState(State::Idle, tr("流程运行完成"));
                emit processFlowChanged();
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
    if (m_hwStatusTimer)
        m_hwStatusTimer->stop();
    m_hwPollInFlight = false;
    if (m_workflowExecutor)
        m_workflowExecutor->stop();
    m_initialized = false;
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

    // 硬件状态轮询定时器：联机后启动，断开/停机时停止。
    m_hwStatusTimer = new QTimer(this);
    m_hwStatusTimer->setInterval(150);
    connect(m_hwStatusTimer, &QTimer::timeout, this, &ProcessModule::pollHardwareStatus);

    setStatusMessage(defaultStatusText(m_simulationMode, m_connected));
}

bool ProcessModule::connectController(const QString& endpoint)
{
    if (endpoint.trimmed().isEmpty()) {
        setState(State::Error, tr("控制器地址不能为空"));
        return false;
    }

    if (m_simulationMode) {
        m_simulationMode = false;
        emit simulationModeChanged(false);
    }

    MotionControl* mc = m_service ? m_service->GetMotionControl() : nullptr;
    if (!mc) {
        m_service->SetMotionControl();
        mc = m_service->GetMotionControl();
    }
    if (!mc || !mc->IsConnected()) {
        if (!mc || !mc->Connect()) {
            setState(State::Error, tr("运动控制器连接失败"));
            return false;
        }
        // 连接后构建轴系（同 connectAllDevices 路径）。
        mc->rebuildAxes();
        mc->SetMotionControlTable();
        mc->SetDigitalTable();
        mc->SetAnalogTable();
    }

    m_connected = true;
    emit connectionChanged(true);
    if (m_hwStatusTimer && !m_hwStatusTimer->isActive())
        m_hwStatusTimer->start();
    setStatusMessage(tr("控制器已连接: %1").arg(endpoint));
    return true;
}

void ProcessModule::disconnectController()
{
    triggerSafeStopOutputs();
    if (m_hwStatusTimer)
        m_hwStatusTimer->stop();
    m_hwPollInFlight = false;
    if (m_service) {
        if (auto* mc = m_service->GetMotionControl())
            mc->Disconnect();
    }

    m_simTimer->stop();
    m_connected = false;
    m_state = State::Idle;
    emit connectionChanged(false);
    emit stateChanged(m_state);
    setStatusMessage(defaultStatusText(m_simulationMode, m_connected));
}

bool ProcessModule::isConnected() const
{
    return m_connected;
}

void ProcessModule::connectAllDevices()
{
    if (m_connected) {
        setStatusMessage(tr("设备已连接，无需重复连接"));
        return;
    }

    auto* taskMgr = lcnc::Kernel::current().taskManager();
    if (!taskMgr) {
        setState(State::Error, tr("TaskManager 不可用"));
        return;
    }

    // 捕获 Service 裸指针（Service 生命周期由 ProcessModule::m_service 保证）
    auto* service = m_service.get();
    const bool simMode = m_simulationMode;

    // 兜底：若 setAxisDefinitions 还未把 AxisGroup 写入 DT（如启动早期连接），
    // 这里依据当前轴定义重算一次，确保 mc->rebuildAxes() 能拿到正确的 IsAxisUse。
    if (DT::getAxisGroup() == 0 && !m_axisDefinitions.isEmpty()) {
        int axisGroup = 0;
        for (const MachineAxisDef& axis : m_axisDefinitions) {
            if (axis.name == QStringLiteral("BASE"))
                continue;
            auto eAxis = enum_cast<Axis>(axis.name.toStdString());
            if (eAxis.has_value())
                axisGroup |= (1 << static_cast<int>(eAxis.value()));
        }
        DT::setAxisGroup(axisGroup);
    }

    const TaskId taskId = taskMgr->run(tr("连接设备"),
        [service, simMode](TaskProgress* progress) {
            progress->setRange(0, 100);

            // ── Step 1: 创建并连接运动控制器 ──
            progress->setStepName(QObject::tr("正在连接运动控制器..."));
            progress->setValue(0);
            {
                service->SetMotionControl();  // 空字符串 → 从 settings 读取 sType
                auto* mc = service->GetMotionControl();
                if (!mc) {
                    throw std::runtime_error(
                        QObject::tr("运动控制器实例化失败").toStdString());
                }
                if (!mc->Connect()) {
                    throw std::runtime_error(
                        QObject::tr("运动控制器连接失败").toStdString());
                }
                // 连接成功后构建轴系：根据 DT::AxisGroup 从 settings 读取每个轴的
                // 参数表，调用 CreateMotor 填充 m_vecMotors / m_mapMotorValue，
                // 此后 IsMotorCreated/MoveRelative/Home 才有数据可用。
                mc->rebuildAxes();
            }
            progress->setValue(30);

            // ── Step 2: 创建并连接激光器 ──
            progress->setStepName(QObject::tr("正在连接激光器..."));
            {
                service->SetLaserDevice();  // 空字符串 → 从 settings 读取 sType
                auto* ld = service->GetLaserDevice();
                if (ld && !ld->Connect()) {
                    // 激光器连接失败不阻断整体流程，但记录错误
                    throw std::runtime_error(
                        QObject::tr("激光器连接失败").toStdString());
                }
            }
            progress->setValue(60);

            // ── Step 3: 下发参数表 ──
            progress->setStepName(QObject::tr("正在下发运动控制器参数..."));
            service->SetMotionControlTable();
            service->SetDigitalTable();
            service->SetAnalogTable();
            progress->setValue(80);

            progress->setStepName(QObject::tr("正在下发激光器参数..."));
            service->SetLaserTable();
            progress->setValue(100);
        });

    watchTask(this, taskId, [this, simMode](bool success) {
        m_connected = success;
        emit connectionChanged(success);

        if (success) {
            // 非仿真模式下关闭仿真标志
            if (simMode) {
                m_simulationMode = false;
                emit simulationModeChanged(false);
            }
            setStatusMessage(tr("设备已连接"));
            // 启动硬件状态轮询：将控制器实际轴位/使能实时刷新到 UI。
            if (m_hwStatusTimer && !m_hwStatusTimer->isActive())
                m_hwStatusTimer->start();
        } else {
            setStatusMessage(tr("设备连接失败"));
        }

        emit deviceConnectFinished(success,
            success ? tr("所有设备连接成功") : tr("设备连接失败，请检查设置"));
    });
}

void ProcessModule::disconnectAllDevices()
{
    if (!m_connected) {
        setStatusMessage(tr("设备未连接"));
        return;
    }
    if (m_state == State::Running) {
        setStatusMessage(tr("加工运行中，请先停止再断开设备"));
        return;
    }
    if (m_homing) {
        setStatusMessage(tr("回零进行中，请等待完成后再断开设备"));
        return;
    }

    auto* taskMgr = lcnc::Kernel::current().taskManager();
    if (!taskMgr) {
        setState(State::Error, tr("TaskManager 不可用"));
        return;
    }

    // 进入断开流程前，先把上层运行/工作流/仿真等状态收尾，避免在硬件
    // 已断开的情况下还有定时器/工作流继续触发指令。
    if (m_simTimer)
        m_simTimer->stop();
    if (m_hwStatusTimer)
        m_hwStatusTimer->stop();
    m_hwPollInFlight = false;
    if (m_workflowExecutor)
        m_workflowExecutor->stop();
    safeStopProcessOutputs();

    setStatusMessage(tr("正在断开设备..."));

    auto* service = m_service.get();

    const TaskId taskId = taskMgr->run(tr("断开设备"),
        [service](TaskProgress* progress) {
            progress->setRange(0, 100);

            // ── Step 1: 断开激光器 ──
            progress->setStepName(QObject::tr("正在断开激光器..."));
            progress->setValue(0);
            {
                auto* ld = service->GetLaserDevice();
                if (ld) {
                    ld->Disconnect();
                }
            }
            progress->setValue(40);

            // ── Step 2: 断开运动控制器 ──
            progress->setStepName(QObject::tr("正在断开运动控制器..."));
            {
                auto* mc = service->GetMotionControl();
                if (mc) {
                    mc->Disconnect();
                }
            }
            progress->setValue(100);
        });

    watchTask(this, taskId, [this](bool success) {
        m_connected = false;
        if (m_simTimer)
            m_simTimer->stop();
        m_state = State::Idle;
        emit connectionChanged(false);
        emit stateChanged(m_state);
        setStatusMessage(success
            ? tr("所有设备已断开")
            : tr("设备断开过程中出现异常，请检查日志"));

        if (success) {
            LCNC_INFO(lcnc::LogCode::Generic,
                      "ProcessModule::disconnectAllDevices: completed");
        } else {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "ProcessModule::disconnectAllDevices: encountered errors");
        }

        emit deviceConnectFinished(success,
            success ? tr("所有设备已断开") : tr("设备断开过程中出现异常"));
    });
}

void ProcessModule::setSimulationMode(bool on)
{
    if (m_simulationMode == on)
        return;

    m_simulationMode = on;
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

	// 桥接机台构型到 DT::AxisGroup
	int axisGroup = 0;
	for (const MachineAxisDef& axis : axes) {
		if (axis.name == QStringLiteral("BASE"))
			continue;
		auto eAxis = enum_cast<Axis>(axis.name.toStdString());
		if (eAxis.has_value())
			axisGroup |= (1 << static_cast<int>(eAxis.value()));
	}
	DT::setAxisGroup(axisGroup);
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

    MotionControl* mc = m_service ? m_service->GetMotionControl() : nullptr;
    if (!mc || !mc->IsConnected()) {
        setStatusMessage(tr("未连接控制器，请先连接设备"));
        return;
    }

    auto eAxis = enum_cast<Axis>(normalizedAxis.toStdString());
    if (!eAxis.has_value() || !mc->IsMotorCreated(eAxis.value())) {
        setStatusMessage(tr("%1 轴未注册，无法点动").arg(normalizedAxis));
        return;
    }

    const double step = distance > 1e-9 ? distance : jogStepForLevel(speedLevel);
    const double delta = step * (direction > 0 ? 1.0 : -1.0);
    // 速度按档位选取（mm/s），仿真器接受任意正值；实控时由参数表 + soft limit 兜底。
    const double vel = (speedLevel == 0) ? 1.0 : (speedLevel == 2 ? 20.0 : 5.0);
    if (!mc->MoveRelative(eAxis.value(), delta, vel)) {
        setStatusMessage(tr("%1 轴点动失败").arg(normalizedAxis));
        return;
    }
    setStatusMessage(tr("点动 %1 轴 %2").arg(
        normalizedAxis,
        direction > 0 ? tr("正向") : tr("负向")));
}

void ProcessModule::home()
{
    if (m_state == State::EmergencyStop) {
        setStatusMessage(tr("急停状态，无法回零"));
        return;
    }
    if (m_state == State::Running) {
        setStatusMessage(tr("加工运行中，无法回零"));
        return;
    }
    if (m_homing) {
        setStatusMessage(tr("回零正在进行中"));
        return;
    }

    // 合并预设轴系（按 Z 优先顺序）+ 用户扩展轴系；扩展轴系无对应 Axis 枚举，
    // 实控阶段只能跳过硬件回零，但仍参与位姿清零，与仿真保持一致。
    QStringList axes = homeOrderForAxes(m_axisDefinitions);
    for (const QString& ext : DT::getExtensionAxes()) {
        const QString key = ext.trimmed().toUpper();
        if (!key.isEmpty() && key != QStringLiteral("BASE") && !axes.contains(key))
            axes.append(key);
    }
    if (axes.isEmpty()) {
        setStatusMessage(tr("没有可回零的轴系"));
        return;
    }

    auto* taskMgr = lcnc::Kernel::current().taskManager();
    if (!taskMgr) {
        setState(State::Error, tr("TaskManager 不可用"));
        return;
    }

    m_homing = true;
    setStatusMessage(tr("开始回零（共 %1 个轴）").arg(axes.size()));

    const bool          connected      = m_connected;
    auto*               service        = m_service.get();

    // 在后台线程依次回零；失败不再继续后续轴，但已成功的轴保持回零状态。
    const TaskId taskId = taskMgr->run(tr("回零"),
        [axes, connected, service]
        (TaskProgress* progress) {
            progress->setRange(0, axes.size());
            progress->setValue(0);

            // 已连接（含仿真器）时调用控制器硬件回零；未连接则仅做 GUI 归零。
            MotionControl* hwMc = (connected && service)
                ? service->GetMotionControl()
                : nullptr;

            for (int i = 0; i < axes.size(); ++i) {
                const QString& axis = axes.at(i);
                progress->setStepName(QObject::tr("正在回零 %1 轴").arg(axis));

                bool ok = true;
                if (hwMc) {
                    auto eAxis = enum_cast<Axis>(axis.toStdString());
                    if (eAxis.has_value() && hwMc->IsMotorCreated(eAxis.value())) {
                        // 已注册的预设轴：调用控制器硬件回零，阻塞直至完成。
                        if (!hwMc->Home(eAxis.value())) {
                            ok = false;
                            LCNC_ERR(lcnc::LogCode::Generic,
                                     "ProcessModule::home: hardware Home failed for axis {}",
                                     axis.toStdString());
                        }
                    } else {
                        // 扩展轴或未注册轴：硬件无法回零，仅记录并继续。
                        LCNC_WARN(lcnc::LogCode::Generic,
                                  "ProcessModule::home: skip hardware home for axis {} "
                                  "(extension or not registered)",
                                  axis.toStdString());
                    }
                }

                progress->setValue(i + 1);
                if (!ok) {
                    throw std::runtime_error(
                        QObject::tr("%1 轴回零失败").arg(axis).toStdString());
                }
            }
        });

    watchTask(this, taskId, [this, axes](bool success) {
        m_homing = false;
        // 无论成功失败，都把 GUI 端的轴位置归零（硬件状态轮询会很快用控制器
        // 实际位置覆盖回来；这里仅同步逻辑缓存避免短暂残留旧值）。
        for (const QString& axis : axes)
            setAxisPosition(axis, 0.0);
        m_simPhase = 0.0;

        if (success) {
            setStatusMessage(tr("回零完成 — 共 %1 个轴").arg(axes.size()));
            LCNC_INFO(lcnc::LogCode::Generic,
                      "ProcessModule::home: completed for {} axes", axes.size());
        } else {
            setState(State::Error, tr("回零失败，请检查日志"));
        }
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
        if (m_workflowExecutor)
            m_workflowExecutor->resume();
        if (m_simulationMode)
            m_simTimer->start(std::max(30, static_cast<int>(100.0 / std::max(0.1, m_feedOverride))));
        setState(State::Running, tr("运行继续"));
        return;
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

    if (!m_simulationMode && m_service) {
        if (auto* mc = m_service->GetMotionControl())
            mc->PauseBuffer(9);
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
    if (m_service) {
        if (auto* mc = m_service->GetMotionControl()) {
            mc->StopMotion();
            mc->StopAllBuffer();
        }
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
    if (m_service) {
        if (auto* mc = m_service->GetMotionControl()) {
            mc->StopMotion();
            mc->StopAllBuffer();
        }
    }
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
    m_processFlowDocument.resetToDefault();
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

    MotionControl* mc = m_service ? m_service->GetMotionControl() : nullptr;
    if (mc && mc->IsConnected()) {
        auto eAxis = enum_cast<Axis>(normalizedAxis.toStdString());
        if (eAxis.has_value() && mc->IsMotorCreated(eAxis.value())) {
            if (!mc->SetAxisEnable(eAxis.value(), enabled)) {
                setStatusMessage(tr("%1 轴使能切换失败").arg(normalizedAxis));
                return;
            }
        }
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

    QString channel = outputName.trimmed();
    bool ok = false;
    if (m_service) {
        if (auto* mc = m_service->GetMotionControl())
            ok = mc->DigitalOutputSet(channel, value ? 1 : 0);
    }

    if (!ok) {
        setStatusMessage(tr("IO 输出 %1 切换失败").arg(name));
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

} // namespace

void ProcessModule::pollHardwareStatus()
{
    if (m_hwPollInFlight)
        return;
    if (!m_connected || m_simulationMode)
        return;
    if (!m_service)
        return;

    MotionControl* hwMc = m_service->GetMotionControl();
    if (!hwMc || !hwMc->IsConnected())
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
    {
        const table digital = SETTINGS->GetTable(SettingSection::Digital);
        if (digital.count("DigitalOUT") && digital.at("DigitalOUT").is_table()) {
            for (const auto& kv : digital.at("DigitalOUT").as_table()) {
                IOEntry entry;
                if (!extractIOEntry(kv.first.data(), kv.second, entry))
                    continue;
                if (!entry.enabled || entry.name.isEmpty())
                    continue;
                digitalOutputs.append(qMakePair(entry.channel, entry.name));
            }
        }
    }

    if (axisNames.isEmpty() && digitalOutputs.isEmpty())
        return;

    m_hwPollInFlight = true;
    QPointer<ProcessModule> self(this);
    auto* watcher = new QFutureWatcher<HardwareIoBatch>(this);
    connect(watcher, &QFutureWatcher<HardwareIoBatch>::finished, this,
            [this, self, watcher]() {
        const HardwareIoBatch batch = watcher->result();
        watcher->deleteLater();
        m_hwPollInFlight = false;
        if (!self)
            return;

        for (const HardwareAxisSample& s : batch.axes) {
            if (!s.valid)
                continue;
            setAxisPosition(s.name, s.pos);
            const bool present = m_axisEnabled.contains(s.name);
            const bool prev = m_axisEnabled.value(s.name, true);
            if (!present || prev != s.enabled) {
                m_axisEnabled.insert(s.name, s.enabled);
                emit axisEnabledChanged(s.name, s.enabled);
            }
        }

        for (const HardwareDigitalOutputSample& s : batch.digitalOutputs) {
            if (!s.valid)
                continue;
            // 缓存按显示名（与 setDigitalOutput / WidgetLaserControl 一致）。
            const bool present = m_digitalOutputs.contains(s.displayName);
            const bool prev = m_digitalOutputs.value(s.displayName, false);
            if (!present || prev != s.value) {
                m_digitalOutputs.insert(s.displayName, s.value);
                emit digitalOutputChanged(s.displayName, s.channel, s.value);
            }
        }
    });

    watcher->setFuture(QtConcurrent::run([axisNames, digitalOutputs, hwMc]() {
        HardwareIoBatch batch;
        batch.axes.reserve(axisNames.size());
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
            batch.axes.push_back(sample);
        }
        batch.digitalOutputs.reserve(digitalOutputs.size());
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
            batch.digitalOutputs.push_back(sample);
        }
        return batch;
    }));
}

namespace {
} // namespace

void ProcessModule::seedDefaultIOTables()
{
    using lcnc::process::BuiltinIODefList;

    auto seed = [](SettingSection section, const BuiltinIODefList& list) {
        table sectionTable = SETTINGS->GetTable(section);
        bool changed = false;
        for (int i = 0; i < list.count; ++i) {
            const auto& def = list.items[i];
            const std::string sub = def.sectionKey;
            if (!sectionTable.count(sub))
                sectionTable[sub] = table{};
            table& bucket = sectionTable[sub].as_table();
            // 已有该 key（不论 array 还是 sub-table）就保留用户值。
            if (bucket.count(def.tomlKey))
                continue;
            table entry;
            entry["name"]        = std::string(def.nameZh);
            entry["index"]       = std::string(def.defaultIndex);
            entry["active"]      = def.defaultActive;
            entry["enabled"]     = def.defaultEnabled;
            entry["showInMain"]  = def.defaultShowInMain;
            entry["builtin"]     = true;
            bucket[def.tomlKey] = entry;
            changed = true;
        }
        if (changed)
            SETTINGS->SetTable(true, section, sectionTable);
    };

    seed(SettingSection::Digital, lcnc::process::builtinDigitalOUT());
    seed(SettingSection::Digital, lcnc::process::builtinDigitalIN());
    seed(SettingSection::Analog,  lcnc::process::builtinAnalogOUT());
    seed(SettingSection::Analog,  lcnc::process::builtinAnalogIN());
}

QList<DigitalOutputDescriptor> ProcessModule::mainPanelDigitalOutputs() const
{
    QList<DigitalOutputDescriptor> out;
    table tableDigital = SETTINGS->GetTable(SettingSection::Digital);
    if (!tableDigital.count("DigitalOUT"))
        return out;
    const auto& bucket = tableDigital.at("DigitalOUT").as_table();
    for (const auto& kv : bucket) {
        IOEntry entry;
        if (!extractIOEntry(kv.first.data(), kv.second, entry))
            continue;
        if (!entry.enabled || !entry.showInMain || entry.name.isEmpty())
            continue;
        DigitalOutputDescriptor d;
        d.channel = entry.channel;
        d.name    = entry.name;
        d.active  = entry.active;
        out.push_back(d);
    }
    return out;
}

void ProcessModule::refreshIOFromSettings()
{
    // 设置对话框关闭后：
    //   1. 把最新 settings 中的 IO 表重新下发到运动控制器（连接状态下立即生效）。
    //   2. 重新发射主界面 IO 描述符列表，让按钮按新 showInMain 重建。
    //   3. 重置 UI 缓存中的输出值，等待下一次轮询从硬件回填实际状态。
    if (m_service) {
        if (auto* mc = m_service->GetMotionControl()) {
            mc->SetDigitalTable();
            mc->SetAnalogTable();
        }
    }
    m_digitalOutputs.clear();
    emit digitalOutputDescriptorsChanged(mainPanelDigitalOutputs());
}

void ProcessModule::safeStopProcessOutputs()
{
    // 兼容旧调用：除了清 UI 缓存外，强制写硬件 0。
    triggerSafeStopOutputs();
    const QStringList commonOutputs = {
        QStringLiteral("激光"), QStringLiteral("吹气"),
        QStringLiteral("夹头"), QStringLiteral("水冷"), QStringLiteral("气泵")
    };
    for (const QString& name : commonOutputs) {
        if (m_digitalOutputs.value(name, false)) {
            m_digitalOutputs.insert(name, false);
            emit digitalOutputChanged(name, name, false);
        }
    }
}

void ProcessModule::triggerSafeStopOutputs()
{
    // 任何中断流程的路径都必须强制关闭激光与吹气。
    struct SafeChannel { QString channel; DigitalOUT eIndex; };
    static const QVector<SafeChannel> kSafeChannels = {
        { QStringLiteral("aLaser"), DigitalOUT::Laser },
        { QStringLiteral("aBlow"),  DigitalOUT::Blow  }
    };
    if (m_service) {
        if (auto* mc = m_service->GetMotionControl()) {
            if (mc->IsConnected()) {
                for (const SafeChannel& sc : kSafeChannels) {
                    // 先 guard 是否已注册，避免触发 WARN_MC_NONEINDEX 弹窗。
                    if (mc->m_mapDigitalOUT.count(sc.eIndex))
                        mc->DigitalOutputSet(sc.eIndex, 0);
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
}

void ProcessModule::setState(State state, const QString& statusMessage)
{
    if (m_state != state) {
        m_state = state;
        emit stateChanged(m_state);
        emit processLogMessage(QStringLiteral("state"), tr("状态机切换为 %1").arg(processStateText(m_state)));
        // 任何流程被打断/异常的状态都强制关闭激光和吹气。
        if (m_state == State::Paused
            || m_state == State::Error
            || m_state == State::EmergencyStop) {
            triggerSafeStopOutputs();
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
