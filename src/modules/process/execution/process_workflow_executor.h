#pragma once

#include "modules/process/workflow/process_node.h"

#include <QObject>
#include <QVector>
#include <functional>

class QTimer;

namespace lcnc::process {

class ProcessFlowDocument;

struct ProcessExecutionStep
{
    QString nodeId;
    ProcessNodeType type{ProcessNodeType::Base};
    QString name;
    QVariantMap parameters;
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

    explicit ProcessWorkflowExecutor(QObject* parent = nullptr);

    bool start(ProcessFlowDocument& document, QString* errorMessage = nullptr);
    void pause();
    void resume();
    void stop();
    void emergencyStop();

    State state() const { return m_state; }
    const QVector<ProcessExecutionStep>& plan() const { return m_plan; }
    void setToolpathSnapshotProvider(std::function<ProcessToolpathSnapshot()> provider);
    void setCuttingExecutor(std::function<bool(bool dryRun, QString* errorMessage)> executor);

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
    QTimer* m_stepTimer{nullptr};
    int m_currentIndex{-1};
    std::function<ProcessToolpathSnapshot()> m_toolpathSnapshotProvider;
    std::function<bool(bool dryRun, QString* errorMessage)> m_cuttingExecutor;
};

} // namespace lcnc::process
