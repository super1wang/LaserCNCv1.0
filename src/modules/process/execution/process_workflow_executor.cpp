#include "modules/process/execution/process_workflow_executor.h"

#include "core/logging/logger.h"
#include "modules/process/steps/process_step_context.h"
#include "modules/process/steps/process_step_registry.h"
#include "modules/process/workflow/process_flow_document.h"
#include "modules/process/workflow/process_node_registry.h"

#include <QTimer>
#include <QElapsedTimer>
#include <QThread>
#include <QtConcurrent>

namespace lcnc::process {

ProcessWorkflowExecutor::ProcessWorkflowExecutor(QObject* parent)
    : QObject(parent)
    , m_stepTimer(new QTimer(this))
    , m_stepWatcher(new QFutureWatcher<QPair<bool, QString>>(this))
{
    // 工作流步骤拥有独立单线程，不与 CAM、导入或设备轮询竞争通用线程池。
    // GUI 线程只处理流程文档状态和信号投影。
    m_workflowPool.setMaxThreadCount(1);
    m_workflowPool.setExpiryTimeout(-1);
    m_workflowPool.setObjectName(QStringLiteral("ProcessWorkflow"));
    m_stepTimer->setSingleShot(true);
    connect(m_stepTimer, &QTimer::timeout,
            this, &ProcessWorkflowExecutor::completeCurrentStep);
    connect(m_stepWatcher, &QFutureWatcher<QPair<bool, QString>>::finished,
            this, &ProcessWorkflowExecutor::completeStepDispatch);
}

ProcessWorkflowExecutor::~ProcessWorkflowExecutor()
{
    m_token.requestStop();
    m_workflowPool.waitForDone();
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
    m_token.reset();
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
    if (m_stepContext) {
        m_stepContext->interrupt = &m_token;
        m_stepContext->cancellationToken = &m_token;
    }
}

void ProcessWorkflowExecutor::setDeviceStopper(DeviceStopper stopper)
{
    m_deviceStopper = std::move(stopper);
}

void ProcessWorkflowExecutor::pause()
{
    if (m_state == State::Running) {
        // 暂停只在步骤的 checkpoint 生效。对于普通切割，当前完整轮廓已经
        // 下发到控制器后必须自然执行结束；禁止暂停 ACS buffer，否则继续时
        // 无法可靠地恢复同一缓冲程序。
        m_token.requestPause();
        m_stepTimer->stop();
        if (m_currentIndex >= 0 && m_currentIndex < m_plan.size())
            setNodeState(m_plan.at(m_currentIndex).nodeId, ProcessNodeState::Paused);
        setState(State::Paused);
        emit messageLogged(tr("已请求暂停，当前轮廓完成后暂停"));
    }
}

void ProcessWorkflowExecutor::resume()
{
    if (m_state == State::Paused) {
        m_token.requestResume();
        setState(State::Running);
        if (m_currentIndex >= 0 && m_currentIndex < m_plan.size()) {
            setNodeState(m_plan.at(m_currentIndex).nodeId, ProcessNodeState::Running);
            // 三种情况：
            //  (a) 长流程当前还卡在 plugin->execute() 内的 checkpoint() 抽水循环 —
            //      m_dispatching=true。token.paused 已经清零，checkpoint 自然返回，
            //      execute 自动续跑。这里不要再 start step timer，会和 dispatch 撞。
            //  (b) 长流程已经退出 execute()，正等 stepTimer 调 completeCurrentStep —
            //      m_dispatching=false 且 stepTimer 没 active。重启 timer。
            //  (c) 正等 timer 触发的瞬态 — m_dispatching=false 且 timer active。不动。
            if (!m_dispatching && !m_stepTimer->isActive())
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
    m_token.requestStop();
    if (m_deviceStopper)
        m_deviceStopper(false);
    m_stepTimer->stop();
    setState(State::Stopped);
    if (m_currentIndex >= 0 && m_currentIndex < m_plan.size())
        setNodeState(m_plan.at(m_currentIndex).nodeId, ProcessNodeState::Stopped);
    setState(State::Idle);
    emit messageLogged(tr("流程已停止"));
}

bool ProcessWorkflowExecutor::waitForIdle(int timeoutMs)
{
    if (m_stepWatcher->future().isFinished())
        return true;
    QElapsedTimer timer;
    timer.start();
    while (!m_stepWatcher->future().isFinished()) {
        if (timeoutMs >= 0 && timer.elapsed() >= timeoutMs)
            return false;
        QThread::msleep(10);
    }
    return true;
}

void ProcessWorkflowExecutor::emergencyStop()
{
    m_token.requestEmergencyStop();
    if (m_deviceStopper)
        m_deviceStopper(true);
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
        if (!m_stepRegistry || !m_stepContext) {
            failCurrentStep(tr("流程步骤运行环境未初始化"));
            return;
        }
        const auto plugin = m_stepRegistry->stepByExecutorKey(step.executorKey);
        if (!plugin) {
            failCurrentStep(tr("未注册或已禁用的流程步骤插件: %1").arg(step.executorKey));
            return;
        }
        ProcessNodeExecutionRequest request;
        request.nodeId = step.nodeId;
        request.executorKey = step.executorKey;
        request.displayName = step.name;
        request.parameters = step.parameters;
        ProcessStepContext* context = m_stepContext;
        m_dispatching = true;
        m_stepWatcher->setFuture(QtConcurrent::run(&m_workflowPool,
            [plugin, request, context] {
                QString errorMessage;
                const bool ok = plugin->execute(request, *context, &errorMessage);
                return qMakePair(ok, errorMessage);
            }));
    } catch (const std::exception& ex) {
        failCurrentStep(QString::fromUtf8(ex.what()));
    } catch (...) {
        failCurrentStep(tr("未知执行异常"));
    }
}

void ProcessWorkflowExecutor::completeStepDispatch()
{
    m_dispatching = false;
    const QPair<bool, QString> result = m_stepWatcher->result();
    if (!result.first) {
        if (m_token.isStopping()) {
            LCNC_INFO(lcnc::LogCode::Generic,
                      "process.executor: current step interrupted by stop/emergency");
            return;
        }
        failCurrentStep(result.second.isEmpty() ? tr("流程步骤执行失败") : result.second);
        return;
    }
    if (m_state != State::Running)
        return;
    if (m_currentIndex >= 0 && m_currentIndex < m_plan.size())
        m_stepTimer->start(durationForStep(m_plan.at(m_currentIndex)));
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

    // 标记"正在同步派发"——pause 期间 resume 不去重启 step timer，让 plugin 内部的
    // checkpoint() 自然返回；plugin 正常结束后由调用方 runNextStep 切到 stepTimer 路径。
    m_dispatching = true;
    const bool ok = plugin->execute(request, *m_stepContext, errorMessage);
    m_dispatching = false;
    return ok;
}

void ProcessWorkflowExecutor::setState(State state)
{
    if (m_state == state)
        return;
    m_state = state;
}

} // namespace lcnc::process
