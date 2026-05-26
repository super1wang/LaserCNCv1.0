#include "core/project/lcnc_project_package.h"

#include "core/document/lcnc_document.h"
#include "core/document/xcaf_utils.h"
#include "core/logging/logger.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QProcess>
#include <QTemporaryDir>

#include <BinXCAFDrivers.hxx>
#include <PCDM_ReaderStatus.hxx>
#include <PCDM_StoreStatus.hxx>
#include <TDataStd_Integer.hxx>
#include <TDocStd_Document.hxx>
#include <TDF_LabelSequence.hxx>
#include <TCollection_ExtendedString.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XmlXCAFDrivers.hxx>

namespace {

bool isManifestFile(const QFileInfo& info)
{
    return info.fileName().compare(QStringLiteral("project.toml"), Qt::CaseInsensitive) == 0;
}

bool isArchiveFile(const QFileInfo& info)
{
    return info.suffix().compare(QStringLiteral("lcnc"), Qt::CaseInsensitive) == 0
        && info.exists()
        && info.isFile();
}

QString processError(QProcess& process)
{
    const QString stdErr = QString::fromLocal8Bit(process.readAllStandardError()).trimmed();
    if (!stdErr.isEmpty())
        return stdErr;
    const QString stdOut = QString::fromLocal8Bit(process.readAllStandardOutput()).trimmed();
    return stdOut;
}

bool runPowerShellArchiveCommand(const QString& script,
                                 const QStringList& scriptArgs,
                                 QString* errorMsg)
{
#ifdef Q_OS_WIN
    QProcess process;
    QStringList args;
    args << QStringLiteral("-NoProfile")
         << QStringLiteral("-ExecutionPolicy")
         << QStringLiteral("Bypass")
         << QStringLiteral("-Command")
         << script;
    args << scriptArgs;

    process.start(QStringLiteral("powershell"), args);
    if (!process.waitForStarted(10000)) {
        if (errorMsg)
            *errorMsg = QStringLiteral("无法启动 PowerShell zip 后端");
        LCNC_ERR(lcnc::LogCode::Generic,
                 "PowerShell archive backend failed to start");
        return false;
    }
    process.waitForFinished(-1);
    if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0) {
        const QString message = processError(process);
        if (errorMsg)
            *errorMsg = message;
        LCNC_ERR(lcnc::LogCode::Generic,
                 "PowerShell archive command failed status={} code={} message={}",
                 static_cast<int>(process.exitStatus()),
                 process.exitCode(),
                 message.toStdString());
        return false;
    }
    return true;
#else
    Q_UNUSED(script);
    Q_UNUSED(scriptArgs);
    if (errorMsg)
        *errorMsg = QStringLiteral("当前平台尚未集成 .lcnc zip 后端");
    LCNC_ERR(lcnc::LogCode::Generic,
             "PowerShell archive backend is unavailable on this platform");
    return false;
#endif
}

bool archiveDirectoryToZip(const QString& sourceDir, const QString& archivePath, QString* errorMsg)
{
    const QFileInfo targetInfo(archivePath);
    if (!QDir().mkpath(targetInfo.absolutePath())) {
        if (errorMsg)
            *errorMsg = QStringLiteral("无法创建项目包目录: %1").arg(targetInfo.absolutePath());
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Failed to create package directory '{}'",
                 targetInfo.absolutePath().toStdString());
        return false;
    }

    QTemporaryDir archiveTemp;
    if (!archiveTemp.isValid()) {
        if (errorMsg)
            *errorMsg = QStringLiteral("无法创建 .lcnc 临时压缩目录");
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Failed to create temporary .lcnc archive directory");
        return false;
    }
    const QString tempZip = QDir(archiveTemp.path()).filePath(QStringLiteral("package.zip"));
    const QString script = QStringLiteral(
        "$source=$args[0]; $dest=$args[1]; "
        "Compress-Archive -Path (Join-Path $source '*') -DestinationPath $dest -Force");
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "Archiving .lcnc staging directory source='{}' tempZip='{}' target='{}'",
               sourceDir.toStdString(),
               tempZip.toStdString(),
               archivePath.toStdString());
    if (!runPowerShellArchiveCommand(script, {sourceDir, tempZip}, errorMsg))
        return false;

    if (QFileInfo::exists(archivePath) && !QFile::remove(archivePath)) {
        if (errorMsg)
            *errorMsg = QStringLiteral("无法覆盖已有项目包: %1").arg(archivePath);
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Failed to remove existing package '{}'",
                 archivePath.toStdString());
        return false;
    }
    if (!QFile::copy(tempZip, archivePath)) {
        if (errorMsg)
            *errorMsg = QStringLiteral("无法写入项目包: %1").arg(archivePath);
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Failed to copy temporary package '{}' to '{}'",
                 tempZip.toStdString(),
                 archivePath.toStdString());
        return false;
    }
    return true;
}

bool extractZipToDirectory(const QString& archivePath, const QString& targetDir, QString* errorMsg)
{
    QTemporaryDir archiveTemp;
    if (!archiveTemp.isValid()) {
        if (errorMsg)
            *errorMsg = QStringLiteral("无法创建 .lcnc 临时解压目录");
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Failed to create temporary .lcnc extract directory");
        return false;
    }
    const QString tempZip = QDir(archiveTemp.path()).filePath(QStringLiteral("package.zip"));
    if (!QFile::copy(archivePath, tempZip)) {
        if (errorMsg)
            *errorMsg = QStringLiteral("无法读取项目包: %1").arg(archivePath);
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Failed to copy package '{}' into temporary zip '{}'",
                 archivePath.toStdString(),
                 tempZip.toStdString());
        return false;
    }

    const QString script = QStringLiteral(
        "$source=$args[0]; $dest=$args[1]; "
        "Expand-Archive -Path $source -DestinationPath $dest -Force");
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "Extracting .lcnc archive source='{}' targetDir='{}'",
               archivePath.toStdString(),
               targetDir.toStdString());
    return runPowerShellArchiveCommand(script, {tempZip, targetDir}, errorMsg);
}

lcnc::LcncProjectManifest prepareSaveManifest(const LcncDocument& workpieceDocument,
                                              const QFileInfo& packageInfo,
                                              const lcnc::LcncProjectManifest& manifestTemplate,
                                              const lcnc::ProjectSaveOptions& options)
{
    lcnc::LcncProjectManifest manifest = manifestTemplate;
    const QString fallbackName = workpieceDocument.name().trimmed().isEmpty()
        ? packageInfo.completeBaseName()
        : workpieceDocument.name().trimmed();

    if (manifest.schema.trimmed().isEmpty())
        manifest.schema = QStringLiteral("lcnc.project");
    if (manifest.formatVersion <= 0)
        manifest.formatVersion = lcnc::LcncProjectManifest::kCurrentFormatVersion;
    if (manifest.projectName.trimmed().isEmpty())
        manifest.projectName = fallbackName;
    if (manifest.documentName.trimmed().isEmpty())
        manifest.documentName = manifest.projectName;
    if (manifest.projectXcafPath.trimmed().isEmpty())
        manifest.projectXcafPath = QStringLiteral("project.xbf");
    if (manifest.camCacheDirectory.trimmed().isEmpty())
        manifest.camCacheDirectory = QStringLiteral("cam/cache");

    manifest.saveOptions = options;
    const QString now = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
    if (manifest.createdUtc.trimmed().isEmpty())
        manifest.createdUtc = now;
    manifest.savedUtc = now;
    return manifest;
}

TCollection_ExtendedString occPath(const QString& path)
{
    return TCollection_ExtendedString(QDir::toNativeSeparators(path).toStdWString().c_str());
}

void ensureXcafDrivers()
{
    Handle(XCAFApp_Application) app = XCAFApp_Application::GetApplication();
    BinXCAFDrivers::DefineFormat(app);
    XmlXCAFDrivers::DefineFormat(app);
}

QString labelNameOrFallback(const TDF_Label& label, const QString& fallback)
{
    const QString name = XcafUtils::name(label).trimmed();
    return name.isEmpty() ? fallback : name;
}

LcncDocument::EntityKind entityKindFromLabel(const TDF_Label& label)
{
    Handle(TDataStd_Integer) kindAttr;
    if (!label.FindAttribute(TDataStd_Integer::GetID(), kindAttr))
        return LcncDocument::EntityKind::Workpiece;

    const int value = kindAttr->Get();
    if (value == static_cast<int>(LcncDocument::EntityKind::Machine))
        return LcncDocument::EntityKind::Machine;
    if (value == static_cast<int>(LcncDocument::EntityKind::Cam))
        return LcncDocument::EntityKind::Cam;
    if (value == static_cast<int>(LcncDocument::EntityKind::Auxiliary))
        return LcncDocument::EntityKind::Auxiliary;
    return LcncDocument::EntityKind::Workpiece;
}

void exportEntityKind(const LcncDocument& source,
                      LcncDocument::EntityKind kind,
                      const Handle(XCAFDoc_ShapeTool)& targetShapeTool)
{
    const TDF_LabelSequence labels = source.entityLabels(kind);
    const Handle(XCAFDoc_ShapeTool) sourceShapeTool = source.shapeTool();

    for (int i = 1; i <= labels.Length(); ++i) {
        const TDF_Label sourceLabel = labels.Value(i);
        const TopoDS_Shape shape = sourceShapeTool->GetShape(sourceLabel);
        if (shape.IsNull())
            continue;

        TDF_Label targetLabel = targetShapeTool->NewShape();
        targetShapeTool->SetShape(targetLabel, shape);
        XcafUtils::setName(targetLabel,
                           labelNameOrFallback(sourceLabel, QStringLiteral("Shape_%1").arg(i)));
        TDataStd_Integer::Set(targetLabel, static_cast<Standard_Integer>(kind));
    }
}

bool saveXcafSnapshot(const LcncDocument& workpieceDocument,
                      const LcncDocument* machineDocument,
                      const LcncDocument* camDocument,
                      const QString& xcafPath,
                      const lcnc::ProjectSaveOptions& options,
                      QString* errorMsg)
{
    ensureXcafDrivers();

    Handle(XCAFApp_Application) app = XCAFApp_Application::GetApplication();
    Handle(TDocStd_Document) xdeDoc;
    app->NewDocument("BinXCAF", xdeDoc);
    XCAFDoc_DocumentTool::Set(xdeDoc->Main());

    const Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(xdeDoc->Main());
    if (options.includeWorkpieceModel)
        exportEntityKind(workpieceDocument, LcncDocument::EntityKind::Workpiece, shapeTool);
    if (options.includeMachineModel && machineDocument)
        exportEntityKind(*machineDocument, LcncDocument::EntityKind::Machine, shapeTool);
    if (options.includeCamData && camDocument)
        exportEntityKind(*camDocument, LcncDocument::EntityKind::Cam, shapeTool);
    exportEntityKind(workpieceDocument, LcncDocument::EntityKind::Auxiliary, shapeTool);

    const PCDM_StoreStatus status = app->SaveAs(xdeDoc, occPath(xcafPath));
    if (status != PCDM_SS_OK) {
        if (errorMsg)
            *errorMsg = QStringLiteral("写入项目 XCAF 失败: %1").arg(xcafPath);
        return false;
    }
    return true;
}

bool loadXcafSnapshot(LcncDocument& workpieceDocument,
                      LcncDocument* machineDocument,
                      LcncDocument* camDocument,
                      const QString& xcafPath,
                      QString* errorMsg)
{
    ensureXcafDrivers();

    Handle(XCAFApp_Application) app = XCAFApp_Application::GetApplication();
    Handle(TDocStd_Document) xdeDoc;
    const PCDM_ReaderStatus status = app->Open(occPath(xcafPath), xdeDoc);
    if (status != PCDM_RS_OK || xdeDoc.IsNull()) {
        if (errorMsg)
            *errorMsg = QStringLiteral("读取项目 XCAF 失败: %1").arg(xcafPath);
        return false;
    }

    const Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(xdeDoc->Main());
    TDF_LabelSequence labels;
    shapeTool->GetFreeShapes(labels);
    for (int i = 1; i <= labels.Length(); ++i) {
        const TDF_Label label = labels.Value(i);
        const TopoDS_Shape shape = shapeTool->GetShape(label);
        if (shape.IsNull())
            continue;

        const LcncDocument::EntityKind kind = entityKindFromLabel(label);
        LcncDocument* target = &workpieceDocument;
        if (kind == LcncDocument::EntityKind::Machine && machineDocument)
            target = machineDocument;
        else if (kind == LcncDocument::EntityKind::Cam && camDocument)
            target = camDocument;

        target->addShapeEntity(shape,
                               labelNameOrFallback(label, QStringLiteral("Shape_%1").arg(i)),
                               kind);
    }

    return true;
}

} // namespace

namespace lcnc {

bool LcncProjectPackage::isProjectPath(const QString& path)
{
    const QFileInfo info(path);
    if (isManifestFile(info))
        return true;
    return info.suffix().compare(QStringLiteral("lcnc"), Qt::CaseInsensitive) == 0;
}

QString LcncProjectPackage::packageDirectory(const QString& path)
{
    QFileInfo info(path);
    if (isManifestFile(info))
        return info.absoluteDir().absolutePath();

    QString packagePath = info.absoluteFilePath();
    if (info.suffix().isEmpty() && !info.exists())
        packagePath += QStringLiteral(".lcnc");
    return QDir::cleanPath(packagePath);
}

QString LcncProjectPackage::manifestPath(const QString& path)
{
    return QDir(packageDirectory(path)).filePath(QStringLiteral("project.toml"));
}

QString LcncProjectPackage::projectXcafPath(const QString& path,
                                            const LcncProjectManifest& manifest)
{
    return QDir(packageDirectory(path)).filePath(manifest.projectXcafPath);
}

bool LcncProjectPackage::save(const LcncDocument& document,
                              const QString& path,
                              const ProjectSaveOptions& options,
                              QString* errorMsg)
{
    return save(document, nullptr, nullptr, path, options, errorMsg);
}

bool LcncProjectPackage::save(const LcncDocument& workpieceDocument,
                              const LcncDocument* machineDocument,
                              const LcncDocument* camDocument,
                              const QString& path,
                              const ProjectSaveOptions& options,
                              QString* errorMsg)
{
    LcncProjectManifest manifestTemplate;
    manifestTemplate.sourceFilePath = workpieceDocument.filePath();
    return save(workpieceDocument, machineDocument, camDocument,
                path, manifestTemplate, options, nullptr, errorMsg);
}

bool LcncProjectPackage::save(const LcncDocument& workpieceDocument,
                              const LcncDocument* machineDocument,
                              const LcncDocument* camDocument,
                              const QString& path,
                              const LcncProjectManifest& manifestTemplate,
                              const ProjectSaveOptions& options,
                              LcncProjectManifest* savedManifest,
                              QString* errorMsg)
{
    const QFileInfo targetInfo(path);
    const bool writeArchive = targetInfo.suffix().compare(QStringLiteral("lcnc"), Qt::CaseInsensitive) == 0
        && !targetInfo.isDir();

    QTemporaryDir tempPackage;
    if (writeArchive && !tempPackage.isValid()) {
        if (errorMsg)
            *errorMsg = QStringLiteral("无法创建 .lcnc 临时项目目录");
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Failed to create temporary .lcnc staging directory for '{}'",
                 path.toStdString());
        return false;
    }

    const QString packagePath = writeArchive ? tempPackage.path() : packageDirectory(path);
    QDir packageDir(packagePath);
    if (!packageDir.exists() && !QDir().mkpath(packagePath)) {
        if (errorMsg)
            *errorMsg = QStringLiteral("无法创建项目目录: %1").arg(packagePath);
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Failed to create .lcnc package directory '{}'",
                 packagePath.toStdString());
        return false;
    }

    const QFileInfo packageInfo(writeArchive ? targetInfo.absoluteFilePath() : packagePath);
    LcncProjectManifest manifest = prepareSaveManifest(workpieceDocument, packageInfo, manifestTemplate, options);
    if (!manifest.validate(errorMsg)) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Refusing to save invalid .lcnc manifest for '{}'",
                 path.toStdString());
        return false;
    }

    const QString xcafPath = projectXcafPath(packagePath, manifest);
    if (!saveXcafSnapshot(workpieceDocument, machineDocument, camDocument,
                          xcafPath, options, errorMsg)) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Failed to save .lcnc XCAF snapshot '{}'",
                 xcafPath.toStdString());
        return false;
    }

    const QString cacheDir = packageDir.filePath(manifest.camCacheDirectory);
    if (options.includeCamCache && !QDir().mkpath(cacheDir)) {
        if (errorMsg)
            *errorMsg = QStringLiteral("无法创建 CAM 缓存目录: %1").arg(cacheDir);
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Failed to create .lcnc CAM cache directory '{}'",
                 cacheDir.toStdString());
        return false;
    }

    if (!manifest.save(manifestPath(packagePath))) {
        if (errorMsg)
            *errorMsg = QStringLiteral("写入项目 manifest 失败: %1").arg(manifestPath(packagePath));
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Failed to write .lcnc manifest '{}'",
                 manifestPath(packagePath).toStdString());
        return false;
    }

    if (writeArchive && !archiveDirectoryToZip(packagePath, targetInfo.absoluteFilePath(), errorMsg))
        return false;

    if (savedManifest)
        *savedManifest = manifest;

    LCNC_INFO(lcnc::LogCode::Generic,
              "Saved .lcnc project package '{}' source='{}'",
              (writeArchive ? targetInfo.absoluteFilePath() : packagePath).toStdString(),
              manifest.sourceFilePath.toStdString());
    return true;
}

bool LcncProjectPackage::load(LcncDocument& document,
                              const QString& path,
                              ProjectLoadResult* result,
                              QString* errorMsg)
{
    return load(document, nullptr, nullptr, path, result, errorMsg);
}

bool LcncProjectPackage::load(LcncDocument& workpieceDocument,
                              LcncDocument* machineDocument,
                              LcncDocument* camDocument,
                              const QString& path,
                              ProjectLoadResult* result,
                              QString* errorMsg)
{
    const QFileInfo inputInfo(path);
    const bool readArchive = isArchiveFile(inputInfo);

    QTemporaryDir tempPackage;
    if (readArchive) {
        if (!tempPackage.isValid()) {
            if (errorMsg)
                *errorMsg = QStringLiteral("无法创建 .lcnc 临时解压目录");
            LCNC_ERR(lcnc::LogCode::Generic,
                     "Failed to create temporary extraction directory for '{}'",
                     path.toStdString());
            return false;
        }
        if (!extractZipToDirectory(inputInfo.absoluteFilePath(), tempPackage.path(), errorMsg))
            return false;
    }

    const QString packagePath = readArchive ? tempPackage.path() : packageDirectory(path);
    LcncProjectManifest manifest;
    const QString manifestFile = manifestPath(packagePath);
    if (!manifest.load(manifestFile)) {
        if (errorMsg)
            *errorMsg = QStringLiteral("读取项目 manifest 失败: %1").arg(manifestFile);
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Failed to read .lcnc manifest '{}'",
                 manifestFile.toStdString());
        return false;
    }
    if (!manifest.validate(errorMsg)) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Invalid .lcnc manifest '{}'",
                 manifestFile.toStdString());
        return false;
    }

    const QString xcafPath = projectXcafPath(packagePath, manifest);
    if (!QFileInfo::exists(xcafPath)) {
        if (errorMsg)
            *errorMsg = QStringLiteral("项目缺少 XCAF 数据文件: %1").arg(xcafPath);
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Missing .lcnc XCAF resource '{}'",
                 xcafPath.toStdString());
        return false;
    }
    if (!loadXcafSnapshot(workpieceDocument, machineDocument, camDocument,
                          xcafPath, errorMsg)) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Failed to load .lcnc XCAF snapshot '{}'",
                 xcafPath.toStdString());
        return false;
    }

    if (result) {
        result->manifest = manifest;
        result->packagePath = readArchive ? inputInfo.absoluteFilePath() : packagePath;
        result->documentName = manifest.documentName.isEmpty()
            ? QFileInfo(result->packagePath).completeBaseName()
            : manifest.documentName;
    }

    LCNC_INFO(lcnc::LogCode::Generic,
              "Loaded .lcnc project package '{}'",
              (readArchive ? inputInfo.absoluteFilePath() : packagePath).toStdString());
    return true;
}

} // namespace lcnc