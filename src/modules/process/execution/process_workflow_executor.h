#pragma once

#include "modules/process/runtime/process_cancellation_token.h"
#include "modules/process/workflow/process_node.h"

#include <QMap>
#include <QObject>
#include <QStringList>
#include <QVariant>
#include <QVector>
#include <QFutureWatcher>
#include <QThreadPool>
#include <functional>

class QTimer;

namespace lcnc::process {

class ProcessFlowDocument;
class ProcessStepRegistry;
struct ProcessStepContext;

struct ProcessExecutionStep
{
    QString nodeId;
    ProcessNodeType type{ProcessNodeType::Base};
    QString name;
    QVariantMap parameters;
    QString executorKey;
    int depth{0};
    // 该步骤所有外层 If 节点的 id（由内向外）。执行时若任一外层 If 条件为 false，
    // 则跳过此步骤不分发。普通步骤为空。
    QStringList ifOwnerIds;
};

struct ProcessToolpathSnapshot
{
    bool available{false};
    int contourCount{0};
    int totalPointCount{0};
    QString description;
};

class ProcessWorkflowExecutor : public QObject
{
    Q_OBJECT
public:
    enum class State
    {
        Idle,
        Running,
        Paused,
        Stopped,
        Error
    };

    explicit ProcessWorkflowExecutor(QObject* parent = nullptr);
    ~ProcessWorkflowExecutor() override;

    bool start(ProcessFlowDocument& document, QString* errorMessage = nullptr);
    void pause();
    void resume();
    void stop();

    State state() const { return m_state; }
    const QVector<ProcessExecutionStep>& plan() const { return m_plan; }
    void setStepRegistry(ProcessStepRegistry* registry);
    void setStepContext(ProcessStepContext* context);

    ProcessCancellationToken* cancellationToken() { return &m_token; }
    /// Used during module teardown after stop() requested cooperative exit.
    bool waitForIdle(int timeoutMs);

signals:
    void messageLogged(const QString& message);
    void nodeStarted(const QString& nodeId);
    void nodeFinished(const QString& nodeId);
    void nodeFailed(const QString& nodeId, const QString& message);
    void nodeStateChanged(const QString& nodeId);
    void axisPositionRequested(const QString& axisName, double position);
    void laserEnergyRequested(double value);
    void digitalOutputRequested(const QString& channel, bool value);
    void workflowFinished();

private:
    void collectNode(const ProcessNode& node, int depth, const QStringList& ifOwners);
    void resetNodeStates(QVector<ProcessNode>& nodes);
    void runNextStep();
    void completeStepDispatch();
    void completeCurrentStep();
    void failCurrentStep(const QString& message);
    void setNodeState(const QString& nodeId, ProcessNodeState state);
    int durationForStep(const ProcessExecutionStep& step) const;
    bool dispatchStepSideEffects(const ProcessExecutionStep& step, QString* errorMessage);
    void setState(State state);
    void seedVariablesFromStart(const QVector<ProcessNode>& roots);
    bool shouldSkipDueToIf(const ProcessExecutionStep& step) const;

    State m_state{State::Idle};
    QVector<ProcessExecutionStep> m_plan;
    ProcessFlowDocument* m_document{nullptr};
    ProcessStepRegistry* m_stepRegistry{nullptr};
    ProcessStepContext* m_stepContext{nullptr};
    QTimer* m_stepTimer{nullptr};
    QFutureWatcher<QPair<bool, QString>>* m_stepWatcher{nullptr};
    QThreadPool m_workflowPool;
    int m_currentIndex{-1};
    bool m_dispatching{false};   ///< true: 当前正同步运行 plugin->execute()，pause 在 checkpoint 内生效
    ProcessCancellationToken m_token;
    QMap<QString, QVariant> m_variables;  // 流程变量环境（Start 声明播种）
    QMap<QString, bool> m_ifResults;      // If 节点条件结果（nodeId -> bool）
};

} // namespace lcnc::process
