#include "modules/process/execution/process_workflow_executor.h"

#include "core/logging/logger.h"
#include "modules/process/workflow/process_flow_document.h"

#include <QTimer>

namespace lcnc::process {

ProcessWorkflowExecutor::ProcessWorkflowExecutor(QObject* parent)
    : QObject(parent)
    , m_stepTimer(new QTimer(this))
{
    m_stepTimer->setSingleShot(true);
    connect(m_stepTimer, &QTimer::timeout,
            this, &ProcessWorkflowExecutor::completeCurrentStep);
}

bool ProcessWorkflowExecutor::start(ProcessFlowDocument& document, QString* errorMessage)
{
    if (m_state == State::Running) {
        if (errorMessage)
            *errorMessage = tr("流程正在运行");
        return false;
    }

    if (m_state == State::Paused) {
        resume();
        return true;
    }

    m_plan.clear();
    m_document = &document;
    resetNodeStates(m_document->rootNodes());
    for (const auto& node : document.rootNodes())
        collectNode(node, 0);

    if (m_plan.isEmpty()) {
        const QString message = tr("流程为空，进入空运行仿真");
        LCNC_WARN(lcnc::LogCode::Generic, "process.executor: empty workflow plan");
        emit messageLogged(message);
    } else {
        LCNC_INFO(lcnc::LogCode::Generic,
                  "process.executor: prepared {} enabled workflow steps",
                  m_plan.size());
    }

    Q_UNUSED(errorMessage);
    m_currentIndex = -1;
    setState(State::Running);
    runNextStep();
    return true;
}

void ProcessWorkflowExecutor::setToolpathSnapshotProvider(std::function<ProcessToolpathSnapshot()> provider)
{
    m_toolpathSnapshotProvider = std::move(provider);
}

void ProcessWorkflowExecutor::pause()
{
    if (m_state == State::Running) {
        m_stepTimer->stop();
        if (m_currentIndex >= 0 && m_currentIndex < m_plan.size())
            setNodeState(m_plan.at(m_currentIndex).nodeId, ProcessNodeState::Paused);
        setState(State::Paused);
        emit messageLogged(tr("流程已暂停"));
    }
}

void ProcessWorkflowExecutor::resume()
{
    if (m_state == State::Paused) {
        setState(State::Running);
        if (m_currentIndex >= 0 && m_currentIndex < m_plan.size()) {
            setNodeState(m_plan.at(m_currentIndex).nodeId, ProcessNodeState::Running);
            m_stepTimer->start(durationForStep(m_plan.at(m_currentIndex)));
        } else {
            runNextStep();
        }
        emit messageLogged(tr("流程继续运行"));
    }
}

void ProcessWorkflowExecutor::stop()
{
    if (m_state == State::Idle)
        return;
    m_stepTimer->stop();
    setState(State::Stopped);
    if (m_currentIndex >= 0 && m_currentIndex < m_plan.size())
        setNodeState(m_plan.at(m_currentIndex).nodeId, ProcessNodeState::Stopped);
    setState(State::Idle);
    emit messageLogged(tr("流程已停止"));
}

void ProcessWorkflowExecutor::emergencyStop()
{
    m_stepTimer->stop();
    if (m_currentIndex >= 0 && m_currentIndex < m_plan.size())
        failCurrentStep(tr("急停中断"));
    setState(State::EmergencyStop);
    emit messageLogged(tr("流程急停中断"));
}

void ProcessWorkflowExecutor::collectNode(const ProcessNode& node, int depth)
{
    if (!node.enabled)
        return;

    ProcessExecutionStep step;
    step.nodeId = node.id;
    step.type = node.type;
    step.name = node.name;
    step.parameters = node.parameters;
    step.depth = depth;
    m_plan.append(step);

    for (const auto& child : node.children)
        collectNode(child, depth + 1);
}

void ProcessWorkflowExecutor::resetNodeStates(QVector<ProcessNode>& nodes)
{
    for (ProcessNode& node : nodes) {
        node.state = node.enabled ? ProcessNodeState::Pending : ProcessNodeState::Disabled;
        emit nodeStateChanged(node.id);
        resetNodeStates(node.children);
    }
}

void ProcessWorkflowExecutor::runNextStep()
{
    try {
        if (m_state != State::Running)
            return;

        ++m_currentIndex;
        if (m_currentIndex >= m_plan.size()) {
            setState(State::Idle);
            emit messageLogged(tr("流程运行完成"));
            emit workflowFinished();
            return;
        }

        const ProcessExecutionStep& step = m_plan.at(m_currentIndex);
        setNodeState(step.nodeId, ProcessNodeState::Running);
        emit nodeStarted(step.nodeId);
        QString errorMessage;
        if (!dispatchStepSideEffects(step, &errorMessage)) {
            failCurrentStep(errorMessage);
            return;
        }
        m_stepTimer->start(durationForStep(step));
    } catch (const std::exception& ex) {
        failCurrentStep(QString::fromUtf8(ex.what()));
    } catch (...) {
        failCurrentStep(tr("未知执行异常"));
    }
}

void ProcessWorkflowExecutor::completeCurrentStep()
{
    if (m_state != State::Running)
        return;

    if (m_currentIndex >= 0 && m_currentIndex < m_plan.size()) {
        const QString nodeId = m_plan.at(m_currentIndex).nodeId;
        setNodeState(nodeId, ProcessNodeState::Stopped);
        emit nodeFinished(nodeId);
    }
    runNextStep();
}

void ProcessWorkflowExecutor::failCurrentStep(const QString& message)
{
    QString nodeId;
    if (m_currentIndex >= 0 && m_currentIndex < m_plan.size()) {
        nodeId = m_plan.at(m_currentIndex).nodeId;
        setNodeState(nodeId, ProcessNodeState::Stopped);
    }
    LCNC_ERR(lcnc::LogCode::Generic,
             "process.executor: node '{}' failed: {}",
             nodeId.toStdString(),
             message.toStdString());
    emit nodeFailed(nodeId, message);
    setState(State::Error);
}

void ProcessWorkflowExecutor::setNodeState(const QString& nodeId, ProcessNodeState state)
{
    if (!m_document)
        return;
    ProcessNode* node = m_document->nodeById(nodeId);
    if (!node)
        return;
    node->state = state;
    m_document->markDirty();
    emit nodeStateChanged(nodeId);
}

int ProcessWorkflowExecutor::durationForStep(const ProcessExecutionStep& step) const
{
    switch (step.type) {
    case ProcessNodeType::Wait:
        return qBound(0, step.parameters.value(QStringLiteral("durationMs"), 1000).toInt(), 24 * 60 * 60 * 1000);
    case ProcessNodeType::Axis:
    case ProcessNodeType::AxesMove:
        return 200;
    case ProcessNodeType::Cutting:
    case ProcessNodeType::OverCutting:
        return step.parameters.value(QStringLiteral("dryRun"), true).toBool() ? 300 : 100;
    default:
        return 1;
    }
}

bool ProcessWorkflowExecutor::dispatchStepSideEffects(const ProcessExecutionStep& step, QString* errorMessage)
{
    if (step.type == ProcessNodeType::Axis) {
        emit axisPositionRequested(
            step.parameters.value(QStringLiteral("axis"), QStringLiteral("X")).toString(),
            step.parameters.value(QStringLiteral("position"), 0.0).toDouble());
    } else if (step.type == ProcessNodeType::AxesMove) {
        emit axisPositionRequested(QStringLiteral("X"), step.parameters.value(QStringLiteral("x"), 0.0).toDouble());
        emit axisPositionRequested(QStringLiteral("Y"), step.parameters.value(QStringLiteral("y"), 0.0).toDouble());
        emit axisPositionRequested(QStringLiteral("Z"), step.parameters.value(QStringLiteral("z"), 0.0).toDouble());
    } else if (step.type == ProcessNodeType::Cutting) {
        const double feedRate = step.parameters.value(QStringLiteral("feedRate"), 100.0).toDouble();
        const double laserEnergy = step.parameters.value(QStringLiteral("laserEnergy"), 10.0).toDouble();
        const bool dryRun = step.parameters.value(QStringLiteral("dryRun"), true).toBool();
        if (feedRate <= 0.0) {
            if (errorMessage)
                *errorMessage = tr("切割进给速度必须大于 0");
            return false;
        }
        if (laserEnergy < 0.0) {
            if (errorMessage)
                *errorMessage = tr("激光能量不能小于 0");
            return false;
        }

        const ProcessToolpathSnapshot snapshot = m_toolpathSnapshotProvider
            ? m_toolpathSnapshotProvider()
            : ProcessToolpathSnapshot{};
        if (!dryRun && !snapshot.available) {
            if (errorMessage)
                *errorMessage = tr("非 dry-run 切割需要可用 CAM 刀路");
            return false;
        }
        emit laserEnergyRequested(laserEnergy);
        emit messageLogged(tr("CAM dry-run: %1，轮廓=%2，点数=%3，feed=%4，energy=%5").arg(
            snapshot.description.isEmpty() ? tr("无 CAM 刀路，按节点参数空跑") : snapshot.description,
            QString::number(snapshot.contourCount),
            QString::number(snapshot.totalPointCount),
            QString::number(feedRate),
            QString::number(laserEnergy)));
    } else if (step.type == ProcessNodeType::OverCutting) {
        const double length = step.parameters.value(QStringLiteral("length"), 0.0).toDouble();
        const double feedRate = step.parameters.value(QStringLiteral("feedRate"), 100.0).toDouble();
        if (length < 0.0 || feedRate <= 0.0) {
            if (errorMessage)
                *errorMessage = tr("过切长度不能小于 0，进给速度必须大于 0");
            return false;
        }
        emit messageLogged(tr("过切 dry-run: length=%1, feed=%2").arg(QString::number(length), QString::number(feedRate)));
    } else if (step.type == ProcessNodeType::EnergySwitch) {
        emit laserEnergyRequested(step.parameters.value(QStringLiteral("laserEnergy"), 10.0).toDouble());
    } else if (step.type == ProcessNodeType::IO) {
        const QString action = step.parameters.value(QStringLiteral("action"), QStringLiteral("set")).toString();
        if (action.compare(QStringLiteral("set"), Qt::CaseInsensitive) == 0) {
            emit digitalOutputRequested(
                step.parameters.value(QStringLiteral("channel"), QStringLiteral("DO0")).toString(),
                step.parameters.value(QStringLiteral("value"), 1).toBool());
        }
    }
    return true;
}

void ProcessWorkflowExecutor::setState(State state)
{
    if (m_state == state)
        return;
    m_state = state;
}

} // namespace lcnc::process
