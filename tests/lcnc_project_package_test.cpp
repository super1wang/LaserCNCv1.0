#include "core/document/lcnc_document.h"
#include "core/project/lcnc_project_package.h"
#include "core/project/lcnc_project_session.h"

#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QSaveFile>
#include <QTemporaryDir>
#include <QTextStream>

#include <JlCompress.h>

namespace {

int fail(const QString& message)
{
    QTextStream(stderr) << message << '\n';
    return 1;
}

bool writeLegacyFixture(const QString& currentPackage,
                        const QString& fixturePackage,
                        int formatVersion,
                        QString* error)
{
    QTemporaryDir staging;
    if (!staging.isValid() || JlCompress::extractDir(currentPackage, staging.path()).isEmpty()) {
        if (error) *error = QStringLiteral("Cannot extract current package for legacy fixture");
        return false;
    }
    QFile::remove(QDir(staging.path()).filePath(QStringLiteral("tools.toml")));
    if (formatVersion == 1) {
        const QString currentXbf = QDir(staging.path()).filePath(QStringLiteral("workpiece.xbf"));
        const QString legacyXbf = QDir(staging.path()).filePath(QStringLiteral("project.xbf"));
        if (!QFile::rename(currentXbf, legacyXbf)) {
            if (error) *error = QStringLiteral("Cannot create v1 XCAF fixture");
            return false;
        }
    }

    QSaveFile manifest(QDir(staging.path()).filePath(QStringLiteral("project.toml")));
    if (!manifest.open(QIODevice::WriteOnly | QIODevice::Text)) {
        if (error) *error = QStringLiteral("Cannot write legacy fixture manifest");
        return false;
    }
    QByteArray contents;
    contents += "schema = \"lcnc.project\"\n";
    contents += "formatVersion = " + QByteArray::number(formatVersion) + "\n";
    contents += "projectName = \"LegacyFixture\"\n";
    contents += "documentName = \"LegacyFixture\"\n";
    contents += "[resources]\n";
    contents += formatVersion == 1 ? "projectXcaf = \"project.xbf\"\n" : "workpieceXcaf = \"workpiece.xbf\"\n";
    contents += "camCacheDirectory = \"cam/cache\"\n";
    contents += "[saveOptions]\nworkpieceModel = true\nmachineModel = false\nparameters = true\ncamData = true\ncamCache = false\n";
    if (manifest.write(contents) != contents.size() || !manifest.commit()) {
        if (error) *error = QStringLiteral("Cannot commit legacy fixture manifest");
        return false;
    }
    if (!JlCompress::compressDir(fixturePackage, staging.path(), true)) {
        if (error) *error = QStringLiteral("Cannot archive legacy fixture");
        return false;
    }
    return true;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    QTemporaryDir temporary;
    if (!temporary.isValid())
        return fail(QStringLiteral("Cannot create project-package test directory"));

    lcnc::LcncProjectSession safetySession;
    safetySession.setMachineConfigurationCompatible(false);
    if (safetySession.machineConfigurationCompatible())
        return fail(QStringLiteral("Machine compatibility safety flag was not retained"));
    safetySession.resetProjectState();
    if (!safetySession.machineConfigurationCompatible())
        return fail(QStringLiteral("New project session did not reset machine compatibility"));

    const QByteArray expectedSnapshot = "[0]\nsName = \"ProjectTool\"\nfLineVel = 600.0\n";
    QByteArray restoredSnapshot;
    const auto installSnapshotExtension = [&]() {
        lcnc::LcncProjectPackage::setExtension({
        [&expectedSnapshot](const QString& stagingDirectory,
                           const lcnc::LcncProjectManifest& manifest,
                           QString* error) {
            QSaveFile file(QDir(stagingDirectory).filePath(manifest.toolSnapshotPath));
            if (!file.open(QIODevice::WriteOnly | QIODevice::Text)
                || file.write(expectedSnapshot) != expectedSnapshot.size() || !file.commit()) {
                if (error) *error = QStringLiteral("Cannot write test tool snapshot");
                return false;
            }
            return true;
        },
        [&restoredSnapshot](const QString& stagingDirectory,
                            const lcnc::LcncProjectManifest& manifest,
                            QString* error) {
            QFile file(QDir(stagingDirectory).filePath(manifest.toolSnapshotPath));
            if (!file.exists())
                return true;
            if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
                if (error) *error = QStringLiteral("Cannot read test tool snapshot");
                return false;
            }
            restoredSnapshot = file.readAll();
            return true;
        }
    });
    };
    installSnapshotExtension();

    auto source = LcncDocument::createStandalone(1, QStringLiteral("Source"));
    const QString packagePath = QDir(temporary.path()).filePath(QStringLiteral("roundtrip.lcnc"));
    QString error;
    if (!lcnc::LcncProjectPackage::save(*source, nullptr, nullptr, packagePath,
                                        lcnc::ProjectSaveOptions{}, &error))
        return fail(QStringLiteral("v4 save failed: %1").arg(error));

    auto target = LcncDocument::createStandalone(2, QStringLiteral("Target"));
    lcnc::ProjectLoadResult loadResult;
    if (!lcnc::LcncProjectPackage::load(*target, nullptr, nullptr, packagePath,
                                        &loadResult, &error))
        return fail(QStringLiteral("v4 load failed: %1").arg(error));
    if (loadResult.manifest.formatVersion != lcnc::LcncProjectManifest::kCurrentFormatVersion
        || restoredSnapshot != expectedSnapshot)
        return fail(QStringLiteral("v4 package did not preserve the required tool snapshot"));

    // A v4 package must never be produced without its declared project resource.
    QFile originalFile(packagePath);
    if (!originalFile.open(QIODevice::ReadOnly))
        return fail(QStringLiteral("Cannot read the existing package before negative save"));
    const QByteArray originalDigest = QCryptographicHash::hash(originalFile.readAll(), QCryptographicHash::Sha256);
    originalFile.close();

    lcnc::LcncProjectPackage::setExtension({});
    if (lcnc::LcncProjectPackage::save(*source, nullptr, nullptr, packagePath,
                                       lcnc::ProjectSaveOptions{}, &error))
        return fail(QStringLiteral("v4 save unexpectedly allowed a missing tool snapshot"));
    if (!originalFile.open(QIODevice::ReadOnly)
        || QCryptographicHash::hash(originalFile.readAll(), QCryptographicHash::Sha256) != originalDigest)
        return fail(QStringLiteral("failed v4 save modified the existing package"));

    // Historical packages are intentionally rejected rather than migrated.
    for (const int legacyVersion : {1, 2, 3}) {
        installSnapshotExtension();
        const QString legacyPath = QDir(temporary.path()).filePath(
            QStringLiteral("legacy-v%1.lcnc").arg(legacyVersion));
        if (!writeLegacyFixture(packagePath, legacyPath, legacyVersion, &error))
            return fail(QStringLiteral("v%1 fixture creation failed: %2").arg(legacyVersion).arg(error));

        auto rejectedWorkpiece = LcncDocument::createStandalone(40 + legacyVersion, QStringLiteral("Rejected"));
        if (lcnc::LcncProjectPackage::load(*rejectedWorkpiece, nullptr, nullptr,
                                            legacyPath, nullptr, &error))
            return fail(QStringLiteral("v%1 package was unexpectedly accepted").arg(legacyVersion));
    }

    return 0;
}
