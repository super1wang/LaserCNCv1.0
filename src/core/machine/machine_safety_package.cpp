#include "core/machine/machine_safety_package.h"

#include "core/algorithms/cam/machine_safety_index.h"
#include "core/logging/logger.h"

#include <QCryptographicHash>
#include <QDataStream>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSaveFile>
#include <QSet>
#include <QTemporaryDir>
#include <QTemporaryFile>

#ifdef Q_OS_WIN
#include <windows.h>
#endif

#include <JlCompress.h>
#include <quazip.h>
#include <quazipfileinfo.h>
#include <toml.hpp>

#include <sstream>

namespace {

constexpr auto kManifestName = "machine_safety.toml";

QByteArray fileSha256(const QString& path, QString* errorMessage)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to read machine safety package resource: %1")
                                .arg(path);
        return {};
    }
    QCryptographicHash hash(QCryptographicHash::Sha256);
    if (!hash.addData(&file)) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to hash machine safety package resource: %1")
                                .arg(path);
        return {};
    }
    return hash.result();
}

bool isSafeRelativePath(const QString& path)
{
    const QString normalized = QDir::cleanPath(path.trimmed()).replace('\\', '/');
    return !normalized.isEmpty()
        && normalized != QStringLiteral(".")
        && !QFileInfo(normalized).isAbsolute()
        && normalized != QStringLiteral("..")
        && !normalized.startsWith(QStringLiteral("../"))
        && !normalized.contains(QStringLiteral("/../"));
}

QByteArray safetyConfigurationFingerprint(
    const lcnc::cam_algo::MachineSafetyIndex& index)
{
    QByteArray bytes;
    QDataStream stream(&bytes, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream.setFloatingPointPrecision(QDataStream::DoublePrecision);
    stream << quint32(1) << index.clearanceMm();
    stream << quint32(index.axes().size());
    for (const auto& axis : index.axes()) {
        stream << axis.name << axis.parentAxis << axis.rotary;
        for (double component : axis.direction)
            stream << component;
        stream << axis.minimum << axis.maximum << axis.step << axis.cellCount;
    }
    stream << quint32(index.bodies().size());
    for (const auto& body : index.bodies())
        stream << body.name << body.axisName;
    stream << quint32(index.pairs().size());
    for (const auto& pair : index.pairs())
        stream << pair.firstBody << pair.secondBody;
    return QCryptographicHash::hash(bytes, QCryptographicHash::Sha256);
}

QByteArray packageKey(const lcnc::MachineSafetyPackageManifest& manifest)
{
    QCryptographicHash hash(QCryptographicHash::Sha256);
    const auto add = [&hash](const QByteArray& value) {
        hash.addData(QByteArray::number(value.size()));
        hash.addData(QByteArrayView(":", 1));
        hash.addData(value);
        hash.addData(QByteArrayView(";", 1));
    };
    add(QByteArrayLiteral("lcnc.machine-safety-package/v1"));
    add(manifest.modelSha256);
    add(manifest.safetyIndexSha256);
    add(manifest.indexContentSha256);
    add(manifest.safetyConfigurationSha256);
    add(manifest.runtimeConfigurationSha256);
    return hash.result();
}

bool writeManifest(const QString& path,
                   const lcnc::MachineSafetyPackageManifest& manifest,
                   QString* errorMessage)
{
    toml::value root(toml::table{});
    root["schema"] = manifest.schema.toStdString();
    root["formatVersion"] = manifest.formatVersion;
    root["createdUtc"] = manifest.createdUtc.toStdString();
    root["softwareVersion"] = manifest.softwareVersion.toStdString();
    root["packageKeySha256"] = manifest.packageKeySha256.toHex().toStdString();

    toml::value resources(toml::table{});
    resources["machineModel"] = manifest.modelPath.toStdString();
    resources["safetyIndex"] = manifest.safetyIndexPath.toStdString();
    root["resources"] = resources;

    toml::value fingerprints(toml::table{});
    fingerprints["modelSha256"] = manifest.modelSha256.toHex().toStdString();
    fingerprints["safetyIndexSha256"] = manifest.safetyIndexSha256.toHex().toStdString();
    fingerprints["indexContentSha256"] = manifest.indexContentSha256.toHex().toStdString();
    fingerprints["safetyConfigurationSha256"] =
        manifest.safetyConfigurationSha256.toHex().toStdString();
    fingerprints["runtimeConfigurationSha256"] =
        manifest.runtimeConfigurationSha256.toHex().toStdString();
    root["fingerprints"] = fingerprints;

    const std::string formatted = toml::format(root);
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly)
        || file.write(formatted.data(), static_cast<qint64>(formatted.size()))
            != static_cast<qint64>(formatted.size())
        || !file.commit()) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to write machine safety package manifest: %1")
                                .arg(path);
        return false;
    }
    return true;
}

QString tomlString(const toml::value& table, const char* key)
{
    if (!table.is_table() || !table.contains(key) || !table.at(key).is_string())
        return {};
    return QString::fromStdString(table.at(key).as_string());
}

QByteArray tomlSha256(const toml::value& table, const char* key)
{
    return QByteArray::fromHex(tomlString(table, key).toLatin1());
}

bool readManifest(const QString& path,
                  lcnc::MachineSafetyPackageManifest* manifest,
                  QString* errorMessage)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Machine safety package manifest is missing");
        return false;
    }
    try {
        const QByteArray bytes = file.readAll();
        std::istringstream input(std::string(bytes.constData(),
                                             static_cast<std::size_t>(bytes.size())));
        const toml::value root = toml::parse(input, path.toUtf8().toStdString());
        manifest->schema = tomlString(root, "schema");
        manifest->formatVersion = root.contains("formatVersion")
            && root.at("formatVersion").is_integer()
            ? static_cast<int>(root.at("formatVersion").as_integer()) : 0;
        manifest->createdUtc = tomlString(root, "createdUtc");
        manifest->softwareVersion = tomlString(root, "softwareVersion");
        manifest->packageKeySha256 = tomlSha256(root, "packageKeySha256");
        if (root.contains("resources")) {
            const auto& resources = root.at("resources");
            manifest->modelPath = tomlString(resources, "machineModel");
            manifest->safetyIndexPath = tomlString(resources, "safetyIndex");
        }
        if (root.contains("fingerprints")) {
            const auto& fingerprints = root.at("fingerprints");
            manifest->modelSha256 = tomlSha256(fingerprints, "modelSha256");
            manifest->safetyIndexSha256 = tomlSha256(fingerprints, "safetyIndexSha256");
            manifest->indexContentSha256 = tomlSha256(fingerprints, "indexContentSha256");
            manifest->safetyConfigurationSha256 =
                tomlSha256(fingerprints, "safetyConfigurationSha256");
            manifest->runtimeConfigurationSha256 =
                tomlSha256(fingerprints, "runtimeConfigurationSha256");
        }
    } catch (const std::exception& exception) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Unable to parse machine safety package manifest: {}",
                 exception.what());
        if (errorMessage)
            *errorMessage = QStringLiteral("Invalid machine safety package manifest: %1")
                                .arg(QString::fromUtf8(exception.what()));
        return false;
    }
    if (manifest->schema != QStringLiteral("lcnc.machine-safety-package")
        || manifest->formatVersion
            != lcnc::MachineSafetyPackageManifest::kCurrentFormatVersion
        || !isSafeRelativePath(manifest->modelPath)
        || !isSafeRelativePath(manifest->safetyIndexPath)
        || QFileInfo(manifest->safetyIndexPath).suffix().compare(
               QStringLiteral("lmsi"), Qt::CaseInsensitive) != 0
        || manifest->modelSha256.size() != 32
        || manifest->safetyIndexSha256.size() != 32
        || manifest->indexContentSha256.size() != 32
        || manifest->safetyConfigurationSha256.size() != 32
        || (manifest->runtimeConfigurationSha256.size() != 0
            && manifest->runtimeConfigurationSha256.size() != 32)
        || manifest->packageKeySha256.size() != 32) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Machine safety package manifest is incomplete or unsupported");
        return false;
    }
    return true;
}

bool archiveEntriesAreSafe(const QString& archivePath, QString* errorMessage)
{
    QuaZip archive(archivePath);
    if (!archive.open(QuaZip::mdUnzip)) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to open machine safety package: %1")
                                .arg(archivePath);
        return false;
    }
    int manifestCount = 0;
    int indexCount = 0;
    QSet<QString> normalizedEntries;
    for (bool more = archive.goToFirstFile(); more; more = archive.goToNextFile()) {
        QuaZipFileInfo64 info;
        if (!archive.getCurrentFileInfo(&info)) {
            archive.close();
            if (errorMessage)
                *errorMessage = QStringLiteral("Unable to enumerate machine safety package");
            return false;
        }
        const QString normalized = QDir::cleanPath(info.name).replace('\\', '/');
        if (!isSafeRelativePath(normalized)) {
            archive.close();
            if (errorMessage)
                *errorMessage = QStringLiteral("Machine safety package contains an unsafe path: %1")
                                    .arg(info.name);
            return false;
        }
        if (normalized.endsWith('/'))
            continue;
        const QString entryKey = normalized.toCaseFolded();
        if (normalizedEntries.contains(entryKey)) {
            archive.close();
            if (errorMessage)
                *errorMessage = QStringLiteral(
                    "Machine safety package contains a duplicate resource path: %1")
                                    .arg(info.name);
            return false;
        }
        normalizedEntries.insert(entryKey);
        if (normalized.compare(QString::fromLatin1(kManifestName),
                               Qt::CaseInsensitive) == 0)
            ++manifestCount;
        if (QFileInfo(normalized).suffix().compare(QStringLiteral("lmsi"),
                                                   Qt::CaseInsensitive) == 0)
            ++indexCount;
    }
    archive.close();
    if (manifestCount != 1 || indexCount != 1) {
        if (errorMessage)
            *errorMessage = QStringLiteral(
                "Machine safety package must contain one manifest and exactly one .lmsi file");
        return false;
    }
    return true;
}

bool atomicArchiveDirectory(const QString& sourceDirectory,
                            const QString& packagePath,
                            QString* errorMessage)
{
    const QFileInfo targetInfo(packagePath);
    if (!QDir().mkpath(targetInfo.absolutePath())) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to create machine safety package directory");
        return false;
    }
    QString stagingPath;
    {
        QTemporaryFile staging(QDir(targetInfo.absolutePath()).filePath(
            QStringLiteral(".%1.staging.XXXXXX").arg(targetInfo.fileName())));
        staging.setAutoRemove(false);
        if (!staging.open()) {
            if (errorMessage)
                *errorMessage = QStringLiteral("Unable to create machine safety package staging file");
            return false;
        }
        stagingPath = staging.fileName();
    }
    QFile::remove(stagingPath);
    if (!JlCompress::compressDir(stagingPath, sourceDirectory, true)) {
        QFile::remove(stagingPath);
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to archive machine safety package");
        return false;
    }
#ifdef Q_OS_WIN
    if (!MoveFileExW(reinterpret_cast<LPCWSTR>(stagingPath.utf16()),
                     reinterpret_cast<LPCWSTR>(packagePath.utf16()),
                     MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
        const DWORD systemError = GetLastError();
        QFile::remove(stagingPath);
        if (errorMessage)
            *errorMessage = QStringLiteral(
                "Unable to atomically replace machine safety package (win32=%1)")
                                .arg(systemError);
        return false;
    }
#else
    if (QFileInfo::exists(packagePath) || !QFile::rename(stagingPath, packagePath)) {
        QFile::remove(stagingPath);
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to atomically replace machine safety package");
        return false;
    }
#endif
    return true;
}

} // namespace

namespace lcnc {

bool MachineSafetyPackage::isPackagePath(const QString& path)
{
    return QFileInfo(path).suffix().compare(QStringLiteral("lmsp"),
                                            Qt::CaseInsensitive) == 0;
}

QString MachineSafetyPackage::normalizedPackagePath(const QString& path)
{
    QFileInfo info(path);
    QString result = info.absoluteFilePath();
    if (info.suffix().compare(QStringLiteral("lmsp"), Qt::CaseInsensitive) != 0) {
        result = info.suffix().isEmpty()
            ? result + QStringLiteral(".lmsp")
            : info.absoluteDir().filePath(info.completeBaseName()
                                          + QStringLiteral(".lmsp"));
    }
    return QDir::cleanPath(result);
}

bool MachineSafetyPackage::create(const QString& machineModelPath,
                                  const QString& safetyIndexPath,
                                  const QString& packagePath,
                                  const QByteArray& runtimeConfigurationSha256,
                                  QString* errorMessage)
{
    const QFileInfo modelInfo(machineModelPath);
    const QFileInfo indexInfo(safetyIndexPath);
    if (!modelInfo.isFile() || !indexInfo.isFile()) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Machine model or safety index is missing");
        return false;
    }
    MachineSafetyPackageManifest manifest;
    manifest.createdUtc = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
    manifest.softwareVersion = QString::fromLatin1(LCNC_VERSION_STRING);
    const QString modelSuffix = modelInfo.suffix().trimmed().isEmpty()
        ? QStringLiteral("step") : modelInfo.suffix().toLower();
    manifest.modelPath = QStringLiteral("machine/machine.%1").arg(modelSuffix);

    QString hashError;
    manifest.modelSha256 = fileSha256(machineModelPath, &hashError);
    manifest.safetyIndexSha256 = fileSha256(safetyIndexPath, &hashError);
    if (manifest.modelSha256.isEmpty() || manifest.safetyIndexSha256.isEmpty()) {
        if (errorMessage)
            *errorMessage = hashError;
        return false;
    }
    lcnc::cam_algo::MachineSafetyIndex index;
    QString indexError;
    if (!index.load(safetyIndexPath, &indexError)) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to validate safety index: %1").arg(indexError);
        return false;
    }
    if (index.sourceSha256() != manifest.modelSha256) {
        if (errorMessage)
            *errorMessage = QStringLiteral(
                "Machine model has changed since the safety index was generated");
        return false;
    }
    manifest.indexContentSha256 = index.contentSha256();
    manifest.safetyConfigurationSha256 = safetyConfigurationFingerprint(index);
    manifest.runtimeConfigurationSha256 = runtimeConfigurationSha256;
    manifest.packageKeySha256 = packageKey(manifest);

    QTemporaryDir staging;
    if (!staging.isValid()
        || !QDir().mkpath(QDir(staging.path()).filePath(QStringLiteral("machine")))
        || !QDir().mkpath(QDir(staging.path()).filePath(QStringLiteral("safety")))) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to create machine safety package staging directory");
        return false;
    }
    const QString stagedModel = QDir(staging.path()).filePath(manifest.modelPath);
    const QString stagedIndex = QDir(staging.path()).filePath(manifest.safetyIndexPath);
    if (!QFile::copy(machineModelPath, stagedModel)
        || !QFile::copy(safetyIndexPath, stagedIndex)
        || !writeManifest(QDir(staging.path()).filePath(QString::fromLatin1(kManifestName)),
                          manifest, errorMessage)) {
        if (errorMessage && errorMessage->isEmpty())
            *errorMessage = QStringLiteral("Unable to stage machine safety package resources");
        return false;
    }
    return atomicArchiveDirectory(staging.path(), normalizedPackagePath(packagePath),
                                  errorMessage);
}

bool MachineSafetyPackage::extractAndValidate(
    const QString& packagePath,
    const QString& targetDirectory,
    MachineSafetyPackageLoadResult* result,
    QString* errorMessage)
{
    const QString normalized = normalizedPackagePath(packagePath);
    if (!QFileInfo(normalized).isFile() || !archiveEntriesAreSafe(normalized, errorMessage))
        return false;
    if (!QDir().mkpath(targetDirectory)) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to create machine safety package extraction directory");
        return false;
    }
    const QStringList extracted = JlCompress::extractDir(normalized, targetDirectory);
    if (extracted.isEmpty()) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to extract machine safety package");
        return false;
    }

    MachineSafetyPackageManifest manifest;
    if (!readManifest(QDir(targetDirectory).filePath(QString::fromLatin1(kManifestName)),
                      &manifest, errorMessage))
        return false;
    const QString modelPath = QDir(targetDirectory).filePath(manifest.modelPath);
    const QString indexPath = QDir(targetDirectory).filePath(manifest.safetyIndexPath);
    QString hashError;
    const QByteArray actualModelSha = fileSha256(modelPath, &hashError);
    const QByteArray actualIndexSha = fileSha256(indexPath, &hashError);
    if (actualModelSha != manifest.modelSha256
        || actualIndexSha != manifest.safetyIndexSha256) {
        if (errorMessage)
            *errorMessage = QStringLiteral(
                "Machine model or safety index fingerprint does not match the package manifest");
        return false;
    }
    lcnc::cam_algo::MachineSafetyIndex index;
    QString indexError;
    if (!index.load(indexPath, &indexError)
        || index.sourceSha256() != actualModelSha
        || index.contentSha256() != manifest.indexContentSha256
        || safetyConfigurationFingerprint(index) != manifest.safetyConfigurationSha256
        || packageKey(manifest) != manifest.packageKeySha256) {
        if (errorMessage)
            *errorMessage = indexError.isEmpty()
                ? QStringLiteral("Machine safety package key or embedded index is invalid")
                : QStringLiteral("Unable to load embedded safety index: %1").arg(indexError);
        return false;
    }
    if (result) {
        result->manifest = manifest;
        result->packagePath = normalized;
        result->modelPath = modelPath;
        result->safetyIndexPath = indexPath;
    }
    return true;
}

} // namespace lcnc
