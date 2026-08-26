#include "modules/process/settings/process_settings_service.h"
#include "modules/process/workflow/process_flow_store.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTemporaryDir>
#include <QTextStream>

#include <algorithm>

namespace {

int fail(const QString& message)
{
    QTextStream(stderr) << message << '\n';
    return 1;
}

bool overwrite(const QString& path, const QByteArray& content)
{
    QFile file(path);
    return file.open(QIODevice::WriteOnly | QIODevice::Truncate)
        && file.write(content) == content.size();
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    QTemporaryDir directory;
    if (!directory.isValid())
        return fail(QStringLiteral("Could not create temporary settings directory"));

    lcnc::process::ProcessSettingsService current(directory.path());
    if (!current.initialize())
        return fail(QStringLiteral("Current Process settings did not initialize"));
    if (current.hasChanges())
        return fail(QStringLiteral("Fresh Process settings incorrectly reported pending changes"));

    const auto initialDefaults = current.initialApproachSettings();
    if (initialDefaults.mode
            != lcnc::process::ProcessInitialApproachMode::Automatic
        || initialDefaults.safetyZ != 0.0) {
        return fail(QStringLiteral("Initial-approach settings defaults are invalid"));
    }
    const auto settingsObjects = current.objects();
    const auto initialObject = std::find_if(
        settingsObjects.cbegin(), settingsObjects.cend(),
        [](const lcnc::process::ParameterObjectDescriptor& object) {
            return object.id == QStringLiteral("initial-approach");
        });
    if (initialObject == settingsObjects.cend())
        return fail(QStringLiteral("Initial-approach settings object is missing"));
    const auto setInitialField = [&current, &initialObject](
                                     const QString& id, const QVariant& value) {
        const auto field = std::find_if(
            initialObject->fields.cbegin(), initialObject->fields.cend(),
            [&id](const lcnc::process::ParameterDescriptor& descriptor) {
                return descriptor.id == id;
            });
        QString error;
        return field != initialObject->fields.cend()
            && current.setFieldValue(*field, initialObject->id, value, &error);
    };
    if (!setInitialField(QStringLiteral("mode"), QStringLiteral("Manual"))
        || !setInitialField(QStringLiteral("safetyZ"), 42.5)) {
        return fail(QStringLiteral("Initial-approach settings could not be edited"));
    }
    const auto initialCommit = current.commit();
    if (!initialCommit.success
        || !initialCommit.changes.domains.contains(QStringLiteral("workflow"))) {
        return fail(QStringLiteral("Initial-approach settings were not committed to workflow"));
    }

    if (!setInitialField(QStringLiteral("safetyZ"), 99.0) || !current.hasChanges())
        return fail(QStringLiteral("Process settings cancel fixture was not modified"));
    const bool cancelSucceeded = current.cancelEdit();
    const bool changesAfterCancel = current.hasChanges();
    const double safetyZAfterCancel = current.initialApproachSettings().safetyZ;
    if (!cancelSucceeded || changesAfterCancel || safetyZAfterCancel != 42.5) {
        return fail(QStringLiteral(
            "Cancel did not restore the last applied Process settings "
            "(success=%1, changed=%2, safetyZ=%3)")
            .arg(cancelSucceeded).arg(changesAfterCancel).arg(safetyZAfterCancel));
    }

    const QString devices = directory.filePath(QStringLiteral("devices.toml"));
    const QByteArray currentData = [&] {
        QFile file(devices);
        if (!file.open(QIODevice::ReadOnly))
            return QByteArray{};
        return file.readAll();
    }();
    if (!currentData.contains("schemaVersion = 2"))
        return fail(QStringLiteral("Process settings writer did not emit schema v2"));

    const QDir toolsDirectory(directory.filePath(QStringLiteral("tools")));
    QStringList toolFiles = toolsDirectory.entryList({QStringLiteral("*.toml")}, QDir::Files);
    toolFiles.removeAll(QStringLiteral("index.toml"));
    const QString persistedToolId = toolFiles.isEmpty()
        ? QString() : QFileInfo(toolFiles.constFirst()).completeBaseName();
    if (toolFiles.size() != 1
        || !overwrite(toolsDirectory.filePath(QStringLiteral("index.toml")),
                      QStringLiteral(
                          "schemaVersion = 2\n"
                          "tools = [{ id = \"%1\", name = \"Default\" }]\n"
                          "[Setting.Tool.ToolIndex]\n"
                          "sToolIndex = \"Default\"\n"
                          "sTool_0 = \"Default\"\n")
                          .arg(persistedToolId).toUtf8())
        || !overwrite(toolsDirectory.filePath(toolFiles.constFirst()),
                      "schemaVersion = 2\n"
                      "[Setting.Tool.Default]\n"
                      "fLineVel = 12.0\n"
                      "fCutAcc = 123.0\n"
                      "fCutJerk = 1234.0\n")) {
        return fail(QStringLiteral("Could not write partial current tool fixture"));
    }

    lcnc::process::ProcessSettingsService currentReload(directory.path());
    if (!currentReload.initialize())
        return fail(QStringLiteral("Current Process settings were rejected"));
    const auto reloadedInitial = currentReload.initialApproachSettings();
    if (reloadedInitial.mode != lcnc::process::ProcessInitialApproachMode::Manual
        || reloadedInitial.safetyZ != 42.5) {
        return fail(QStringLiteral("Initial-approach settings did not round-trip"));
    }

    const QStringList toolNames = currentReload.toolDisplayNames();
    if (toolNames.isEmpty() || toolNames.first() != QStringLiteral("default"))
        return fail(QStringLiteral("Protected default tool was not canonicalized first"));
    QString protectedError;
    if (currentReload.renameTool(QStringLiteral("default"), QStringLiteral("renamed"), &protectedError)
        || currentReload.deleteTool(QStringLiteral("default"), &protectedError)) {
        return fail(QStringLiteral("Protected default tool could be renamed or deleted"));
    }

    const toml::table defaultTool = currentReload.rawTable(
        lcnc::process::ProcessConfigArea::Tools, QStringLiteral("default"));
    const auto idleAcceleration = defaultTool.find("fIdelAcc");
    const auto idleJerk = defaultTool.find("fIdelJerk");
    const auto cuttingAcceleration = defaultTool.find("fCutAcc");
    if (idleAcceleration == defaultTool.end() || idleJerk == defaultTool.end()
        || cuttingAcceleration == defaultTool.end()
        || idleAcceleration->second.as_floating() != 100.0
        || idleJerk->second.as_floating() != 1000.0
        || cuttingAcceleration->second.as_floating() != 123.0) {
        return fail(QStringLiteral("Tool runtime defaults were not materialized"));
    }

    if (!overwrite(devices,
                   "schemaVersion = 1\n"
                   "[Setting.MotionControl]\n"
                   "[Setting.MotionControl.MotionControl]\n"
                   "sType = \"Simulator\"\n")) {
        return fail(QStringLiteral("Could not write old Process settings fixture"));
    }
    lcnc::process::ProcessSettingsService oldSettings(directory.path());
    if (oldSettings.initialize())
        return fail(QStringLiteral("Old Process settings schema was silently accepted or repaired"));

    lcnc::process::ProcessFlowDocument workflow;
    QString error;
    toml::value oldWorkflow(toml::table{});
    oldWorkflow["Process"]["schemaVersion"] = 0;
    oldWorkflow["Process"]["items"] = toml::array{};
    if (lcnc::process::ProcessFlowStore::loadFromToml(oldWorkflow, workflow, &error))
        return fail(QStringLiteral("Old workflow schema was accepted"));

    toml::value currentWorkflow(toml::table{});
    currentWorkflow["Process"]["schemaVersion"] = lcnc::process::ProcessFlowStore::SchemaVersion;
    currentWorkflow["Process"]["nodes"] = toml::array{};
    if (!lcnc::process::ProcessFlowStore::loadFromToml(currentWorkflow, workflow, &error))
        return fail(QStringLiteral("Current workflow schema was rejected: %1").arg(error));

    return 0;
}
