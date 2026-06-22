#include "modules/process/execution/process_workflow_executor.h"

#include "core/logging/logger.h"
#include "modules/process/steps/process_step_context.h"
#include "modules/process/steps/process_step_registry.h"
#include "modules/process/workflow/process_flow_document.h"
#include "modules/process/workflow/process_node_registry.h"

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

void ProcessWorkflowExecutor::setStepRegistry(ProcessStepRegistry* registry)
{
    m_stepRegistry = registry;
}

void ProcessWorkflowExecutor::setStepContext(ProcessStepContext* context)
{
    m_stepContext = context;
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
    if (const auto* descriptor = ProcessNodeRegistry::instance().descriptor(node.type))
        step.executorKey = descriptor->executorKey;
    else
        step.executorKey = processNodeTypeToString(node.type);
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
    if (m_stepRegistry) {
        if (auto plugin = m_stepRegistry->stepByExecutorKey(step.executorKey)) {
            ProcessNodeExecutionRequest request;
            request.nodeId = step.nodeId;
            request.executorKey = step.executorKey;
            request.displayName = step.name;
            request.parameters = step.parameters;
            return plugin->completionDelayMs(request);
        }
    }
    return 1;
}

bool ProcessWorkflowExecutor::dispatchStepSideEffects(const ProcessExecutionStep& step, QString* errorMessage)
{
    if (!m_stepRegistry || !m_stepContext) {
        if (errorMessage)
            *errorMessage = tr("流程步骤运行环境未初始化");
        return false;
    }
    auto plugin = m_stepRegistry->stepByExecutorKey(step.executorKey);
    if (!plugin) {
        if (errorMessage)
            *errorMessage = tr("未注册或已禁用的流程步骤插件: %1").arg(step.executorKey);
        return false;
    }

    ProcessNodeExecutionRequest request;
    request.nodeId = step.nodeId;
    request.executorKey = step.executorKey;
    request.displayName = step.name;
    request.parameters = step.parameters;
    return plugin->execute(request, *m_stepContext, errorMessage);
}

void ProcessWorkflowExecutor::setState(State state)
{
    if (m_state == state)
        return;
    m_state = state;
}

} // namespace lcnc::process
