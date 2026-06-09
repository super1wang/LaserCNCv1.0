#include "modules/process/process_module.h"

#include "core/kernel/i_kernel.h"
#include "core/kernel/kernel.h"
#include "core/kinematics/machine_configuration_service.h"
#include "core/logging/logger.h"
#include "core/task/task_manager.h"
#include "core/task/task_progress.h"
#include "modules/process/controllers/simulator_cmhp_motion_controller.h"
#include "modules/process/execution/process_workflow_executor.h"
#include "modules/process/workflow/process_flow_store.h"

#include <QList>
#include <QPointer>
#include <QTimer>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

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
    auto svc = std::shared_ptr<ProcessModule>(this, [](ProcessModule*) {});
    kernel.services().registerService<ProcessModule>(svc);
    auto facade = std::shared_ptr<lcnc::IProcessFacade>(svc, static_cast<lcnc::IProcessFacade*>(this));
    kernel.services().registerService<lcnc::IProcessFacade>(facade);

    // 创建 SimulatorCmhpMotionController 作为仿真运动控制器。
    m_motionController = std::make_unique<lcnc::process::SimulatorCmhpMotionController>(this);

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
                if (m_motionController)
                    m_motionController->setDigitalOutput(channel, value);
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

    if (m_simulationMode) {
        m_simulationMode = false;
        emit simulationModeChanged(false);
    }

    if (m_motionController && !m_motionController->isRunning()) {
        if (!m_motionController->start()) {
            setState(State::Error, tr("运动控制器启动失败"));
            return false;
        }
    }

    m_connected = true;
    emit connectionChanged(true);
    setStatusMessage(tr("控制器已连接: %1").arg(endpoint));
    return true;
}

void ProcessModule::disconnectController()
{
    if (m_motionController)
        m_motionController->stop();

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

    if (m_motionController) {
        for (const QString& axis : axes)
            m_motionController->home(axis);
    }

    for (const QString& axis : axes)
        setAxisPosition(axis, 0.0);

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

    if (!m_simulationMode && m_motionController)
        m_motionController->pauseProgram(9);

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

    QString channel = outputName.trimmed();
    QString errorMessage;
    bool ok = false;
    if (m_motionController)
        ok = m_motionController->setDigitalOutput(channel, value, &errorMessage);

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

void ProcessModule::safeStopProcessOutputs()
{
    // 重置常用数字输出。
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

void ProcessModule::setState(State state, const QString& statusMessage)
{
    if (m_state != state) {
        m_state = state;
        emit stateChanged(m_state);
        emit processLogMessage(QStringLiteral("state"), tr("状态机切换为 %1").arg(processStateText(m_state)));
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
