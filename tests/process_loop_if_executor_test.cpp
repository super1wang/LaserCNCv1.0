#include "modules/process/execution/process_workflow_executor.h"
#include "modules/process/steps/process_step_builtin_registration.h"
#include "modules/process/steps/process_step_context.h"
#include "modules/process/steps/process_step_registry.h"
#include "modules/process/workflow/process_flow_document.h"
#include "modules/process/workflow/process_node_registry.h"

#include <QCoreApplication>
#include <QTimer>
#include <QTextStream>

namespace {
int g_outcome = 0; // 0=timeout, 1=finished, 2=failed
}

// Drive the executor headlessly through Loop/If workflows. Steps used
// (Start/Stop/Loop/If) need no device services, so the context is minimal.
static bool runCase(lcnc::process::ProcessWorkflowExecutor& executor,
                    lcnc::process::ProcessStepContext& context,
                    lcnc::process::ProcessFlowDocument& doc)
{
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
