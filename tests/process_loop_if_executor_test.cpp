#include "modules/process/execution/process_workflow_executor.h"
#include "modules/process/steps/process_step_builtin_registration.h"
#include "modules/process/steps/process_step_context.h"
#include "modules/process/steps/process_step_registry.h"
#include "modules/process/workflow/process_flow_document.h"
#include "modules/process/workflow/process_node_registry.h"
#include "modules/process/workflow/process_workflow_service.h"

#include <QCoreApplication>
#include <QFile>
#include <QTemporaryDir>
#include <QTimer>
#include <QTextStream>

#include <atomic>

namespace {
int g_outcome = 0; // 0=timeout, 1=finished, 2=failed

class RecordingMotionService final : public lcnc::process::IProcessMotionService
{
public:
    bool moveAxis(const QString&, const QString&, double, double, int, QString*) override
    {
        return false;
    }

    bool moveAxes(const QVariantList&, const QString&, int, QString*) override
    {
        return false;
    }

    bool setAxisPosition(const QVariantList& axes, int timeoutMs, QString*) override
    {
        receivedAxes = axes;
        receivedTimeoutMs = timeoutMs;
        return true;
    }

    bool stopMotion(QString*) override
    {
        return false;
    }

    QVariantList receivedAxes;
    int receivedTimeoutMs{-1};
};

class RecordingCuttingService final : public lcnc::process::IProcessCuttingService
{
public:
    lcnc::process::ProcessToolpathSnapshot toolpathSnapshot() const override
    {
        lcnc::process::ProcessToolpathSnapshot snapshot;
        snapshot.available = true;
        snapshot.contourCount = 2;
        snapshot.totalPointCount = 20;
        snapshot.description = QStringLiteral("integration fixture");
        return snapshot;
    }

    bool executeNormalCutting(const QString& nodeId,
                              const QVariantMap&,
                              lcnc::process::ProcessInterruptContext*,
                              QString*) override
    {
        lastNodeId = nodeId;
        ++executionCount;
        return true;
    }

    std::atomic<int> executionCount{0};
    QString lastNodeId;
};

bool verifyWorkflowPersistence()
{
    QTemporaryDir directory;
    if (!directory.isValid())
        return false;

    lcnc::process::ProcessWorkflowService service;
    int changes = 0;
    QObject::connect(&service, &lcnc::process::ProcessWorkflowService::flowChanged,
                     [&changes] { ++changes; });
    service.createNew();
    if (service.document().rootNodes().size() != 3 || changes != 1)
        return false;

    const QString path = directory.filePath(QStringLiteral("current.toml"));
    QString error;
    if (!service.save(path, &error) || service.document().isDirty())
        return false;
    service.document().resetToDefault();
    if (!service.load(path, &error) || changes != 2)
        return false;

    const QString retainedNodeId = service.document().rootNodes().front().id;
    const QString oldPath = directory.filePath(QStringLiteral("old.toml"));
    QFile oldFile(oldPath);
    if (!oldFile.open(QIODevice::WriteOnly | QIODevice::Text)
        || oldFile.write("[Process]\nschemaVersion = 0\nnodes = []\n") < 0) {
        return false;
    }
    oldFile.close();
    if (service.load(oldPath, &error)
        || service.document().rootNodes().front().id != retainedNodeId
        || changes != 2) {
        return false;
    }
    return true;
}
}

// Drive the executor headlessly through Loop/If workflows. Steps used
// (Start/Stop/Loop/If) need no device services, so the context is minimal.
static bool runCase(lcnc::process::ProcessWorkflowExecutor& executor,
                    lcnc::process::ProcessStepContext& context,
                    lcnc::process::ProcessFlowDocument& doc)
{
    Q_UNUSED(context);
    g_outcome = 0;
    QTimer::singleShot(15000, qApp, [&] { if (!g_outcome) { g_outcome = 3; qApp->quit(); } });
    QString err;
    if (!executor.start(doc, &err))
        return false;
    qApp->exec();
    executor.stop();
    return g_outcome == 1;
}

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    auto& stepRegistry = lcnc::process::ProcessStepRegistry::instance();
    stepRegistry.clear();
    lcnc::process::registerBuiltinProcessSteps(stepRegistry);

    if (!verifyWorkflowPersistence()) {
        QTextStream(stderr) << "Workflow persistence contract failed\n";
        return 1;
    }

    lcnc::process::ProcessWorkflowExecutor executor;
    lcnc::process::ProcessStepContext context;
    context.logMessage = [](const QString&) {};
    executor.setStepRegistry(&stepRegistry);
    executor.setStepContext(&context);

    QObject::connect(&executor, &lcnc::process::ProcessWorkflowExecutor::workflowFinished,
                     [&] { g_outcome = 1; app.quit(); });
    QObject::connect(&executor, &lcnc::process::ProcessWorkflowExecutor::nodeFailed,
                     [&](const QString&, const QString&) { g_outcome = 2; app.quit(); });

    using namespace lcnc::process;

    // Execute the real default Start -> NormalCutting -> Stop workflow through
    // the public cutting-service boundary. This is the headless machining-flow
    // gate; controller-specific SDK dispatch remains in its integration test.
    {
        RecordingCuttingService cutting;
        context.cutting = &cutting;
        ProcessFlowDocument document;
        document.resetToDefault();
        if (!runCase(executor, context, document)
            || cutting.executionCount.load() != 1
            || cutting.lastNodeId.isEmpty()) {
            QTextStream(stderr) << "Default machining workflow did not execute exactly once\n";
            return 1;
        }
        context.cutting = nullptr;
    }

    // The new step must be registered, persist by its stable string id and
    // forward its complete axis table through the motion-service boundary.
    {
        if (processNodeTypeFromString(QStringLiteral("SetAxisPosition"))
                != ProcessNodeType::SetAxisPosition
            || processNodeTypeToString(ProcessNodeType::SetAxisPosition)
                != QStringLiteral("SetAxisPosition")) {
            QTextStream(stderr) << "SetAxisPosition type conversion failed\n";
            return 1;
        }

        const auto step = stepRegistry.step(ProcessNodeType::SetAxisPosition);
        if (!step || step->descriptor().executorKey != QStringLiteral("setAxisPosition")) {
            QTextStream(stderr) << "SetAxisPosition step was not registered\n";
            return 1;
        }

        ProcessNode node = ProcessNodeRegistry::instance().createDefaultNode(
            ProcessNodeType::SetAxisPosition);
        if (!step->summary(node).contains(QStringLiteral("empty"))) {
            QTextStream(stderr) << "SetAxisPosition empty summary is invalid\n";
            return 1;
        }

        QVariantMap x;
        x.insert(QStringLiteral("axis"), QStringLiteral("X"));
        x.insert(QStringLiteral("position"), 0.0);
        QVariantMap y;
        y.insert(QStringLiteral("axis"), QStringLiteral("Y"));
        y.insert(QStringLiteral("position"), 5.0);
        const QVariantList axes{x, y};
        node.parameters.insert(QStringLiteral("axes"), axes);
        node.parameters.insert(QStringLiteral("timeoutMs"), 4321);

        RecordingMotionService motion;
        ProcessStepContext stepContext;
        stepContext.motion = &motion;
        ProcessNodeExecutionRequest request;
        request.parameters = node.parameters;
        QString error;
        if (!step->execute(request, stepContext, &error)
            || motion.receivedAxes != axes
            || motion.receivedTimeoutMs != 4321) {
            QTextStream(stderr) << "SetAxisPosition execution forwarding failed: " << error << '\n';
            return 1;
        }
    }

    // Case 1: Loop(3) containing If(variable, regex=.*) -> If always true.
    {
        ProcessFlowDocument doc;
        doc.rootNodes().append(ProcessNodeRegistry::instance().createDefaultNode(ProcessNodeType::Start));
        ProcessNode loop = ProcessNodeRegistry::instance().createDefaultNode(ProcessNodeType::Loop);
        loop.parameters.insert(QStringLiteral("loopCount"), 3);
        ProcessNode ifNode = ProcessNodeRegistry::instance().createDefaultNode(ProcessNodeType::If);
        ifNode.parameters.insert(QStringLiteral("conditionMode"), QStringLiteral("variable"));
        ifNode.parameters.insert(QStringLiteral("variableName"), QStringLiteral("x"));
        ifNode.parameters.insert(QStringLiteral("regex"), QStringLiteral(".*"));
        loop.children.append(ifNode);
        doc.rootNodes().append(loop);
        doc.rootNodes().append(ProcessNodeRegistry::instance().createDefaultNode(ProcessNodeType::Stop));
        if (!runCase(executor, context, doc)) {
            QTextStream(stderr) << "Case 1 (Loop+If true) failed, outcome=" << g_outcome << '\n';
            return 1;
        }
    }

    // Case 2: If(variable, regex=nomatch) containing Loop(2) -> If false, skip Loop.
    {
        ProcessFlowDocument doc;
        doc.rootNodes().append(ProcessNodeRegistry::instance().createDefaultNode(ProcessNodeType::Start));
        ProcessNode ifNode = ProcessNodeRegistry::instance().createDefaultNode(ProcessNodeType::If);
        ifNode.parameters.insert(QStringLiteral("conditionMode"), QStringLiteral("variable"));
        ifNode.parameters.insert(QStringLiteral("variableName"), QStringLiteral("x"));
        ifNode.parameters.insert(QStringLiteral("regex"), QStringLiteral("nomatch"));
        ProcessNode loop = ProcessNodeRegistry::instance().createDefaultNode(ProcessNodeType::Loop);
        loop.parameters.insert(QStringLiteral("loopCount"), 2);
        ifNode.children.append(loop);
        doc.rootNodes().append(ifNode);
        doc.rootNodes().append(ProcessNodeRegistry::instance().createDefaultNode(ProcessNodeType::Stop));
        if (!runCase(executor, context, doc)) {
            QTextStream(stderr) << "Case 2 (If false skip) failed, outcome=" << g_outcome << '\n';
            return 1;
        }
    }

    // Case 3: If(variable, regex=.*) containing Loop(2) -> If true, run Loop twice.
    {
        ProcessFlowDocument doc;
        doc.rootNodes().append(ProcessNodeRegistry::instance().createDefaultNode(ProcessNodeType::Start));
        ProcessNode ifNode = ProcessNodeRegistry::instance().createDefaultNode(ProcessNodeType::If);
        ifNode.parameters.insert(QStringLiteral("conditionMode"), QStringLiteral("variable"));
        ifNode.parameters.insert(QStringLiteral("variableName"), QStringLiteral("x"));
        ifNode.parameters.insert(QStringLiteral("regex"), QStringLiteral(".*"));
        ProcessNode loop = ProcessNodeRegistry::instance().createDefaultNode(ProcessNodeType::Loop);
        loop.parameters.insert(QStringLiteral("loopCount"), 2);
        ifNode.children.append(loop);
        doc.rootNodes().append(ifNode);
        doc.rootNodes().append(ProcessNodeRegistry::instance().createDefaultNode(ProcessNodeType::Stop));
        if (!runCase(executor, context, doc)) {
            QTextStream(stderr) << "Case 3 (If true + Loop) failed, outcome=" << g_outcome << '\n';
            return 1;
        }
    }

    return 0;
}
