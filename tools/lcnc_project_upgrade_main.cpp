#include "core/document/lcnc_document.h"
#include "core/logging/logger.h"
#include "core/project/cam/cam_data_manager.h"
#include "core/project/lcnc_project_package.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSaveFile>
#include <QTextStream>

#include "toml.hpp"

#include <sstream>

namespace {

bool readToolSnapshot(const QString& path, QByteArray* contents, QString* error)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (error) *error = QStringLiteral("无法读取工具快照: %1").arg(path);
        return false;
    }
    const QByteArray data = file.readAll();
    try {
        std::istringstream input(data.toStdString());
        const toml::value root = toml::parse(input);
        if (!root.is_table() || root.as_table().empty()) {
            if (error) *error = QStringLiteral("工具快照必须是包含至少一个工具的 TOML 表: %1").arg(path);
            return false;
        }
    } catch (const std::exception& exception) {
        if (error) *error = QStringLiteral("工具快照 TOML 无效: %1").arg(exception.what());
        return false;
    }
    *contents = data;
    return true;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    const QStringList args = app.arguments();
    QTextStream out(stdout), err(stderr);
    if (args.size() != 3 && (args.size() != 5 || args.at(3) != QStringLiteral("--tools"))) {
        err << "Usage: lcnc_project_upgrade <input.lcnc> <output.lcnc> [--tools <tools.toml>]\n";
        return 2;
    }

    const QString input = QFileInfo(args.at(1)).absoluteFilePath();
    const QString output = QFileInfo(args.at(2)).absoluteFilePath();
    if (input.compare(output, Qt::CaseInsensitive) == 0) {
        err << "Refusing to overwrite the input package; choose a separate output path.\n";
        return 2;
    }

    QByteArray toolSnapshot;
    QString error;
    if (args.size() == 5
        && !readToolSnapshot(QFileInfo(args.at(4)).absoluteFilePath(), &toolSnapshot, &error)) {
        err << "Tool snapshot failed: " << error << '\n';
        return 2;
    }

    lcnc::Logger::init(QCoreApplication::applicationDirPath() + QStringLiteral("/logs"));
    lcnc::LcncProjectPackage::setExtension({
        [&toolSnapshot](const QString& stagingDirectory,
                        const lcnc::LcncProjectManifest& manifest,
                        QString* writeError) {
            if (toolSnapshot.isEmpty()) {
                if (writeError) {
                    *writeError = QStringLiteral("旧工程未包含 tools.toml；请使用 --tools <tools.toml> 提供完整工具快照");
                }
                return false;
            }
            QSaveFile file(QDir(stagingDirectory).filePath(manifest.toolSnapshotPath));
            if (!file.open(QIODevice::WriteOnly | QIODevice::Text)
                || file.write(toolSnapshot) != toolSnapshot.size() || !file.commit()) {
                if (writeError) *writeError = QStringLiteral("无法写入升级后的 tools.toml");
                return false;
            }
            return true;
        },
        [&toolSnapshot](const QString& stagingDirectory,
                        const lcnc::LcncProjectManifest& manifest,
                        QString* readError) {
            const QString path = QDir(stagingDirectory).filePath(manifest.toolSnapshotPath);
            if (!QFileInfo::exists(path))
                return true;
            return readToolSnapshot(path, &toolSnapshot, readError);
        }
    });
    auto workpiece = LcncDocument::createStandalone(1, QStringLiteral("Workpiece"));
    auto machine = LcncDocument::createStandalone(2, QStringLiteral("Machine"));
    auto cam = LcncDocument::createStandalone(3, QStringLiteral("CAM"));
    lcnc::cam::CamDataManager camData;
    lcnc::ProjectLoadResult loaded;
    if (!lcnc::LcncProjectPackage::loadForMigration(*workpiece, machine.get(), cam.get(), input, &loaded, &error, &camData)) {
        err << "Load failed: " << error << '\n';
        return 1;
    }

    lcnc::LcncProjectManifest saved;
    if (!lcnc::LcncProjectPackage::save(*workpiece, machine.get(), cam.get(), output,
                                        loaded.manifest, loaded.manifest.saveOptions,
                                        &saved, &error, &camData)) {
        err << "Save failed: " << error << '\n';
        return 1;
    }
    out << "Upgraded project to format " << saved.formatVersion << ": " << output << '\n';
    lcnc::Logger::shutdown();
    return 0;
}
