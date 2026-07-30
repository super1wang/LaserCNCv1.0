#include "modules/process/settings/process_settings_service.h"
#include "modules/process/workflow/process_flow_store.h"

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

    const QString devices = directory.filePath(QStringLiteral("devices.toml"));
    const QByteArray currentData = [&] {
        QFile file(devices);
        if (!file.open(QIODevice::ReadOnly))
            return QByteArray{};
        return file.readAll();
    }();
    if (!currentData.contains("schemaVersion = 2"))
        return fail(QStringLiteral("Process settings writer did not emit schema v2"));

    lcnc::process::ProcessSettingsService currentReload(directory.path());
    if (!currentReload.initialize())
        return fail(QStringLiteral("Current Process settings were rejected"));

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
