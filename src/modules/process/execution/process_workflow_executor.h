#pragma once

#include "modules/process/runtime/process_cancellation_token.h"
#include "modules/process/workflow/process_node.h"

#include <QObject>
#include <QVector>
#include <functional>

class MotionControl;
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
        EmergencyStop,
        Error
    };

    using ControllerAccessor = std::function<MotionControl*()>;

    explicit ProcessWorkflowExecutor(QObject* parent = nullptr);

    bool start(ProcessFlowDocument& document, QString* errorMessage = nullptr);
    void pause();
    void resume();
    void stop();
    void emergencyStop();

    State state() const { return m_state; }
    const QVector<ProcessExecutionStep>& plan() const { return m_plan; }
    void setStepRegistry(ProcessStepRegistry* registry);
    void setStepContext(ProcessStepContext* context);
    void setControllerAccessor(ControllerAccessor accessor);

    ProcessCancellationToken* cancellationToken() { return &m_token; }

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
    void collectNode(const ProcessNode& node, int depth);
    void resetNodeStates(QVector<ProcessNode>& nodes);
    void runNextStep();
    void completeCurrentStep();
    void failCurrentStep(const QString& message);
    void setNodeState(const QString& nodeId, ProcessNodeState state);
    int durationForStep(const ProcessExecutionStep& step) const;
    bool dispatchStepSideEffects(const ProcessExecutionStep& step, QString* errorMessage);
    void setState(State state);

    State m_state{State::Idle};
    QVector<ProcessExecutionStep> m_plan;
    ProcessFlowDocument* m_document{nullptr};
    ProcessStepRegistry* m_stepRegistry{nullptr};
    ProcessStepContext* m_stepContext{nullptr};
    QTimer* m_stepTimer{nullptr};
    int m_currentIndex{-1};
    bool m_dispatching{false};   ///< true: 当前正同步运行 plugin->execute()，pause 在 checkpoint 内生效
    ProcessCancellationToken m_token;
    ControllerAccessor m_controllerAccessor;
};

} // namespace lcnc::process