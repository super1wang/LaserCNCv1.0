#include "modules/process/execution/process_workflow_executor.h"

#include "core/logging/logger.h"
#include "modules/process/steps/process_step_context.h"
#include "modules/process/steps/process_step_registry.h"
#include "modules/process/workflow/process_flow_document.h"
#include "modules/process/workflow/process_node_registry.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QTimer>
#include <QElapsedTimer>
#include <QThread>
#include <QtConcurrent>

namespace lcnc::process {

namespace {
// 单个循环节点最大展开次数与计划总步数硬上限，防止误填导致内存膨胀。
constexpr int kMaxLoopCount = 99999;
constexpr int kMaxPlanSize = 200000;
} // namespace

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
            // 中文翻译：流程正在运行
            *errorMessage = tr("Process is running");
        return false;
    }

    if (m_state == State::Paused) {
        resume();
        return true;
    }

    m_plan.clear();
    m_document = &document;
    m_ifResults.clear();
    m_variables.clear();
    seedVariablesFromStart(document.rootNodes());
    resetNodeStates(m_document->rootNodes());
    for (const auto& node : document.rootNodes())
        collectNode(node, 0, QStringList{});

    if (m_plan.isEmpty()) {
        // 中文翻译：流程为空，进入空运行仿真
        const QString message = tr("The process is empty and enters dry run simulation.");
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
    m_workflowElapsed.start();
    LCNC_INFO(lcnc::LogCode::Generic,
              "stage=process.workflow event=begin steps={}", m_plan.size());
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
        m_stepContext->variables = &m_variables;
        m_stepContext->ifResults = &m_ifResults;
    }
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
        // 中文翻译：已请求暂停，当前轮廓完成后暂停
        emit messageLogged(tr("Pause requested, pause after completion of current contour"));
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
        // 中文翻译：流程继续运行
        emit messageLogged(tr("The process continues to run"));
    }
}

void ProcessWorkflowExecutor::stop()
{
    if (m_state == State::Idle)
        return;
    if (m_workflowElapsed.isValid()) {
        LCNC_INFO(lcnc::LogCode::Generic,
                  "stage=process.workflow event=end result=stopped elapsed_ms={}",
                  m_workflowElapsed.elapsed());
        m_workflowElapsed.invalidate();
    }
    m_token.requestStop();
    m_stepTimer->stop();
    if (m_state != State::Error) {
        setState(State::Stopped);
        if (m_currentIndex >= 0 && m_currentIndex < m_plan.size())
            setNodeState(m_plan.at(m_currentIndex).nodeId, ProcessNodeState::Stopped);
        setState(State::Idle);
    }
    // 中文翻译：流程已停止
    emit messageLogged(tr("Process has stopped"));
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

void ProcessWorkflowExecutor::collectNode(const ProcessNode& node, int depth, const QStringList& ifOwners)
{
    if (!node.enabled)
        return;

    auto appendStep = [this, &node, depth, &ifOwners] {
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
        step.ifOwnerIds = ifOwners;
        m_plan.append(step);
    };

    if (node.type == ProcessNodeType::Loop) {
        // 循环头只入计划一次（记录/日志），子节点按 loopCount 展开重复入计划，
        // 这样执行器仍走线性步进，pause/stop/resume 语义不变。
        appendStep();
        const int rawCount = node.parameters.value(QStringLiteral("loopCount"), 1).toInt();
        const int count = qBound(1, rawCount, kMaxLoopCount);
        for (int i = 0; i < count; ++i) {
            if (m_plan.size() >= kMaxPlanSize) {
                LCNC_WARN(lcnc::LogCode::Generic,
                          "process.executor: loop expansion capped at {} steps",
                          kMaxPlanSize);
                break;
            }
            for (const auto& child : node.children)
                collectNode(child, depth + 1, ifOwners);
        }
        return;
    }

    if (node.type == ProcessNodeType::If) {
        // If 头入计划一次（执行时求值条件并写入 m_ifResults），其子节点继承外层 If
        // 链 + 本 If 的 id，执行时据此跳过 false 分支。
        appendStep();
        QStringList childIfOwners = ifOwners;
        childIfOwners.append(node.id);
        for (const auto& child : node.children)
            collectNode(child, depth + 1, childIfOwners);
        return;
    }

    appendStep();
    for (const auto& child : node.children)
        collectNode(child, depth + 1, ifOwners);
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

        // 推进到下一个需要分发的步骤；跳过任何外层 If 条件为 false 的节点
        // （If 头执行时已把结果写入 m_ifResults，false 分支的子节点直接略过不分发）。
        while (true) {
            ++m_currentIndex;
            if (m_currentIndex >= m_plan.size()) {
                if (m_workflowElapsed.isValid()) {
                    LCNC_INFO(lcnc::LogCode::Generic,
                              "stage=process.workflow event=end result=success elapsed_ms={}",
                              m_workflowElapsed.elapsed());
                    m_workflowElapsed.invalidate();
                }
                setState(State::Idle);
                // 中文翻译：流程运行完成
                emit messageLogged(tr("The process is completed"));
                emit workflowFinished();
                return;
            }
            if (!shouldSkipDueToIf(m_plan.at(m_currentIndex)))
                break;
        }

        const ProcessExecutionStep& step = m_plan.at(m_currentIndex);
        m_stepElapsed.start();
        LCNC_INFO(lcnc::LogCode::Generic,
                  "stage=process.workflow.step event=begin index={} node='{}' executor='{}' name='{}'",
                  m_currentIndex, step.nodeId.toStdString(),
                  step.executorKey.toStdString(), step.name.toStdString());
        setNodeState(step.nodeId, ProcessNodeState::Running);
        emit nodeStarted(step.nodeId);
        if (!m_stepRegistry || !m_stepContext) {
            // 中文翻译：流程步骤运行环境未初始化
            failCurrentStep(tr("The process step running environment is not initialized"));
            return;
        }
        const auto plugin = m_stepRegistry->stepByExecutorKey(step.executorKey);
        if (!plugin) {
            // 中文翻译：未注册或已禁用的流程步骤插件: %1
            failCurrentStep(tr("Unregistered or disabled process step plugin: %1").arg(step.executorKey));
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
        LCNC_ERR(lcnc::LogCode::Generic,
                 "process.executor: step dispatch threw: {}", ex.what());
        failCurrentStep(QString::fromUtf8(ex.what()));
    } catch (...) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "process.executor: step dispatch threw an unknown exception");
        // 中文翻译：未知执行异常
        failCurrentStep(tr("Unknown execution exception"));
    }
}

void ProcessWorkflowExecutor::completeStepDispatch()
{
    m_dispatching = false;
    const QPair<bool, QString> result = m_stepWatcher->result();
    if (!result.first) {
        if (m_token.isStopping()) {
            LCNC_INFO(lcnc::LogCode::Generic,
                      "process.executor: current step interrupted by stop");
            return;
        }
        // 中文翻译：流程步骤执行失败
        failCurrentStep(result.second.isEmpty() ? tr("Process step execution failed") : result.second);
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
        LCNC_INFO(lcnc::LogCode::Generic,
                  "stage=process.workflow.step event=end result=success index={} node='{}' elapsed_ms={}",
                  m_currentIndex, nodeId.toStdString(),
                  m_stepElapsed.isValid() ? m_stepElapsed.elapsed() : -1);
        m_stepElapsed.invalidate();
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
             "stage=process.workflow.step event=end result=failed index={} node='{}' elapsed_ms={} error='{}'",
             m_currentIndex,
             nodeId.toStdString(),
             m_stepElapsed.isValid() ? m_stepElapsed.elapsed() : -1,
             message.toStdString());
    m_stepElapsed.invalidate();
    if (m_workflowElapsed.isValid()) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "stage=process.workflow event=end result=failed elapsed_ms={} error='{}'",
                 m_workflowElapsed.elapsed(), message.toStdString());
        m_workflowElapsed.invalidate();
    }
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
            // 中文翻译：流程步骤运行环境未初始化
            *errorMessage = tr("The process step running environment is not initialized");
        return false;
    }
    auto plugin = m_stepRegistry->stepByExecutorKey(step.executorKey);
    if (!plugin) {
        if (errorMessage)
            // 中文翻译：未注册或已禁用的流程步骤插件: %1
            *errorMessage = tr("Unregistered or disabled process step plugin: %1").arg(step.executorKey);
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

void ProcessWorkflowExecutor::seedVariablesFromStart(const QVector<ProcessNode>& roots)
{
    const ProcessNode* startNode = nullptr;
    for (const auto& node : roots) {
        if (node.type == ProcessNodeType::Start) {
            startNode = &node;
            break;
        }
    }
    if (!startNode)
        return;

    const QString json = startNode->parameters
                             .value(QStringLiteral("variables"), QStringLiteral("[]"))
                             .toString();
    const QJsonDocument doc = QJsonDocument::fromJson(json.toUtf8());
    if (!doc.isArray()) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "process.executor: Start variables JSON is not an array, ignored");
        return;
    }
    for (const QJsonValue& value : doc.array()) {
        const QJsonObject obj = value.toObject();
        const QString name = obj.value(QStringLiteral("name")).toString();
        if (name.isEmpty())
            continue;
        m_variables.insert(name, obj.value(QStringLiteral("default")).toVariant());
    }
}

bool ProcessWorkflowExecutor::shouldSkipDueToIf(const ProcessExecutionStep& step) const
{
    for (const QString& ifId : step.ifOwnerIds) {
        auto it = m_ifResults.constFind(ifId);
        if (it != m_ifResults.cend() && !it.value())
            return true;
    }
    return false;
}

} // namespace lcnc::process
