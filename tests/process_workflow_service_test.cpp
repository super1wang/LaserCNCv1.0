#include "modules/process/workflow/process_workflow_service.h"

#include <QCoreApplication>
#include <QFile>
#include <QTemporaryDir>
#include <QTextStream>

namespace {

int fail(const QString& message)
{
    QTextStream(stderr) << message << '\n';
    return 1;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    QTemporaryDir directory;
    if (!directory.isValid())
        return fail(QStringLiteral("Could not create temporary workflow directory"));

    lcnc::process::ProcessWorkflowService service;
    int changes = 0;
    QObject::connect(&service, &lcnc::process::ProcessWorkflowService::flowChanged,
                     [&changes] { ++changes; });

    service.createNew();
    if (service.document().rootNodes().isEmpty() || changes != 1)
        return fail(QStringLiteral("New workflow did not create the default document"));

    const QString workflowPath = directory.filePath(QStringLiteral("current.toml"));
    QString error;
    if (!service.save(workflowPath, &error))
        return fail(QStringLiteral("Could not save current workflow: %1").arg(error));
    if (service.document().isDirty())
        return fail(QStringLiteral("Saved workflow remained dirty"));

    service.document().resetToDefault();
    if (!service.load(workflowPath, &error))
        return fail(QStringLiteral("Could not load current workflow: %1").arg(error));
    if (changes != 2)
        return fail(QStringLiteral("Workflow load did not emit exactly one change"));

    if (service.load(QString(), &error) || error.isEmpty())
        return fail(QStringLiteral("Empty workflow path was accepted"));

    const QString retainedNodeId = service.document().rootNodes().front().id;
    const QString oldWorkflowPath = directory.filePath(QStringLiteral("old.toml"));
    QFile oldWorkflow(oldWorkflowPath);
    if (!oldWorkflow.open(QIODevice::WriteOnly | QIODevice::Text)
        || oldWorkflow.write("[Process]\nschemaVersion = 0\nnodes = []\n") < 0) {
        return fail(QStringLiteral("Could not create old workflow fixture"));
    }
    oldWorkflow.close();
    if (service.load(oldWorkflowPath, &error))
        return fail(QStringLiteral("Old workflow schema was accepted"));
    if (service.document().rootNodes().isEmpty()
        || service.document().rootNodes().front().id != retainedNodeId
        || changes != 2) {
        return fail(QStringLiteral("Failed workflow load changed the active document"));
    }

    service.notifyChanged();
    if (changes != 3)
        return fail(QStringLiteral("Executor workflow notification was not forwarded"));
    return 0;
}
