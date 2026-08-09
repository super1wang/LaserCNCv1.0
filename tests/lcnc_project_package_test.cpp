#include "core/document/lcnc_document.h"
#include "core/project/lcnc_project_package.h"
#include "core/project/lcnc_project_manager.h"
#include "core/project/lcnc_project_session.h"
#include "core/project/cam/cam_data_manager.h"

#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QSaveFile>
#include <QProcess>
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

    {
        lcnc::LcncProjectManager manager;
        const auto originalIds = manager.workspaceIds();
        if (originalIds.size() != 1)
            return fail(QStringLiteral("Project manager did not create its initial workspace"));
        manager.setWorkspaceCloseGuard([](ProjectWorkspaceId) { return false; });
        if (manager.closeWorkspace(originalIds.front())
            || manager.workspace(originalIds.front()) == nullptr) {
            return fail(QStringLiteral("Rejected workspace close released the borrowed document"));
        }
        manager.setDocumentOpenMode(lcnc::DocumentOpenMode::SingleDocument);
        if (manager.beginSingleDocumentOpen() != 0
            || manager.newProject(QStringLiteral("Blocked replacement")) != nullptr
            || manager.workspaceIds() != originalIds) {
            return fail(QStringLiteral("Single-document replacement continued after close rejection"));
        }
        manager.setWorkspaceCloseGuard({});
        if (!manager.closeWorkspace(originalIds.front()))
            return fail(QStringLiteral("Workspace did not close after the guard was cleared"));
    }

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
    lcnc::cam::CamDataManager sourceCam;
    sourceCam.setMachiningMode(lcnc::MachiningMode::RotaryTube4Axis);
    lcnc::MachineAxisLayout sourceLayout;
    sourceLayout.append(QStringLiteral("X"), lcnc::MachineAxisRole::LinearX);
    sourceLayout.append(QStringLiteral("Y"), lcnc::MachineAxisRole::LinearY);
    sourceLayout.append(QStringLiteral("Z"), lcnc::MachineAxisRole::LinearZ);
    sourceLayout.append(QStringLiteral("A"), lcnc::MachineAxisRole::WorkpieceRotary);
    sourceCam.setMachineAxisLayout(sourceLayout);
    sourceCam.setSolverId(QStringLiteral("RotaryTube4Axis"));
    sourceCam.setSolverVersion(lcnc::machiningModeSolverVersion(
        lcnc::MachiningMode::RotaryTube4Axis));
    sourceCam.setSolvedMachineConfigurationFingerprint(QStringLiteral("test-fingerprint"));
    LaserContour persistedContour;
    persistedContour.contourId = 1;
    persistedContour.name = QStringLiteral("v5-contract");
    ToolpathPoint persistedPoint;
    persistedPoint.position = gp_Pnt(1.0, 2.0, 3.0);
    persistedPoint.normal = gp_Dir(0.0, 0.0, 1.0);
    persistedPoint.machineCoord.valid = true;
    persistedPoint.machineCoord.solvedPose.activeMask = 0x0f;
    persistedPoint.machineCoord.solvedPose.valid = true;
    persistedPoint.machineCoord.solvedPose.values = {1.0, 2.0, 3.0, 45.0, 0.0};
    persistedContour.points.push_back(persistedPoint);
    sourceCam.toolpath().contours().push_back(persistedContour);
    sourceCam.ensureToolpathLayers();
    const QString packagePath = QDir(temporary.path()).filePath(QStringLiteral("roundtrip.lcnc"));
    QString error;
    lcnc::LcncProjectManifest manifest;
    if (!lcnc::LcncProjectPackage::save(*source, nullptr, nullptr, packagePath, manifest,
                                        lcnc::ProjectSaveOptions{}, nullptr, &error, &sourceCam))
        return fail(QStringLiteral("v5 save failed: %1").arg(error));

    auto target = LcncDocument::createStandalone(2, QStringLiteral("Target"));
    lcnc::cam::CamDataManager restoredCam;
    lcnc::ProjectLoadResult loadResult;
    if (!lcnc::LcncProjectPackage::load(*target, nullptr, nullptr, packagePath,
                                        &loadResult, &error, &restoredCam))
        return fail(QStringLiteral("v5 load failed: %1").arg(error));
    if (loadResult.manifest.formatVersion != lcnc::LcncProjectManifest::kCurrentFormatVersion
        || restoredSnapshot != expectedSnapshot
        || restoredCam.machiningMode() != lcnc::MachiningMode::RotaryTube4Axis
        || restoredCam.machineAxisLayout() != sourceLayout
        || restoredCam.solvedMachineConfigurationFingerprint() != QStringLiteral("test-fingerprint"))
        return fail(QStringLiteral("v5 package did not preserve the required tool snapshot"));

    // A v5 package must never be produced without its declared project resource.
    QFile originalFile(packagePath);
    if (!originalFile.open(QIODevice::ReadOnly))
        return fail(QStringLiteral("Cannot read the existing package before negative save"));
    const QByteArray originalDigest = QCryptographicHash::hash(originalFile.readAll(), QCryptographicHash::Sha256);
    originalFile.close();

    lcnc::LcncProjectPackage::setExtension({});
    if (lcnc::LcncProjectPackage::save(*source, nullptr, nullptr, packagePath, manifest,
                                       lcnc::ProjectSaveOptions{}, nullptr, &error, &sourceCam))
        return fail(QStringLiteral("v5 save unexpectedly allowed a missing tool snapshot"));
    if (!originalFile.open(QIODevice::ReadOnly)
        || QCryptographicHash::hash(originalFile.readAll(), QCryptographicHash::Sha256) != originalDigest)
        return fail(QStringLiteral("failed v5 save modified the existing package"));

    const QString suppliedTools = QDir(temporary.path()).filePath(QStringLiteral("supplied-tools.toml"));
    {
        QSaveFile toolsFile(suppliedTools);
        if (!toolsFile.open(QIODevice::WriteOnly) || toolsFile.write(expectedSnapshot) != expectedSnapshot.size()
            || !toolsFile.commit())
            return fail(QStringLiteral("Cannot create supplied migration tool snapshot"));
    }

    // Desktop rejects historical packages; the isolated utility creates a new v5 package.
    for (const int legacyVersion : {1, 2, 3, 4}) {
        installSnapshotExtension();
        const QString legacyPath = QDir(temporary.path()).filePath(
            QStringLiteral("legacy-v%1.lcnc").arg(legacyVersion));
        if (!writeLegacyFixture(packagePath, legacyPath, legacyVersion, &error))
            return fail(QStringLiteral("v%1 fixture creation failed: %2").arg(legacyVersion).arg(error));

        auto rejectedWorkpiece = LcncDocument::createStandalone(40 + legacyVersion, QStringLiteral("Rejected"));
        if (lcnc::LcncProjectPackage::load(*rejectedWorkpiece, nullptr, nullptr,
                                            legacyPath, nullptr, &error))
            return fail(QStringLiteral("v%1 package was unexpectedly accepted").arg(legacyVersion));

        const QString upgradedPath = QDir(temporary.path()).filePath(
            QStringLiteral("legacy-v%1-upgraded.lcnc").arg(legacyVersion));
        const QString upgrader = QString::fromUtf8(LCNC_PROJECT_UPGRADE_PATH);
        QProcess process;
        process.start(upgrader, {legacyPath, upgradedPath, QStringLiteral("--tools"), suppliedTools});
        if (!process.waitForFinished(30000) || process.exitStatus() != QProcess::NormalExit
            || process.exitCode() != 0)
            return fail(QStringLiteral("v%1 upgrade failed: %2").arg(legacyVersion)
                        .arg(QString::fromLocal8Bit(process.readAllStandardError())));
        auto upgradedWorkpiece = LcncDocument::createStandalone(50 + legacyVersion, QStringLiteral("Upgraded"));
        lcnc::ProjectLoadResult upgradedResult;
        installSnapshotExtension();
        if (!lcnc::LcncProjectPackage::load(*upgradedWorkpiece, nullptr, nullptr,
                                            upgradedPath, &upgradedResult, &error)
            || upgradedResult.manifest.formatVersion != 5)
            return fail(QStringLiteral("v%1 upgraded package did not load as v5: %2")
                        .arg(legacyVersion).arg(error));
    }

    return 0;
}
