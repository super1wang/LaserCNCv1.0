#include "core/project/lcnc_project_package.h"

#include "core/document/lcnc_document.h"
#include "core/document/xcaf_utils.h"
#include "core/logging/logger.h"
#include "core/project/cam/cam_data_manager.h"
#include "core/project/cam/cam_toolpath_io.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTemporaryDir>

#include <JlCompress.h>

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

// .lcnc 打包/解包后端：QuaZip（替换原 PowerShell Compress-Archive/Expand-Archive）。
// JlCompress::compressDir 以"相对 sourceDir 的路径"写入条目 —— 即包内 project.toml /
// workpiece.xbf / cam/... 位于根目录，与旧后端及 load 端的期望一致。

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

    // 覆盖式写入：先删旧包，避免 QuaZip 追加/残留。
    if (QFileInfo::exists(archivePath) && !QFile::remove(archivePath)) {
        if (errorMsg)
            *errorMsg = QStringLiteral("无法覆盖已有项目包: %1").arg(archivePath);
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Failed to remove existing package '{}'",
                 archivePath.toStdString());
        return false;
    }

    LCNC_DEBUG(lcnc::LogCode::Generic,
               "Archiving .lcnc staging dir source='{}' target='{}' (QuaZip)",
               sourceDir.toStdString(),
               archivePath.toStdString());

    if (!JlCompress::compressDir(archivePath, sourceDir, /*recursive=*/true)) {
        if (errorMsg)
            *errorMsg = QStringLiteral("打包 .lcnc 项目失败: %1").arg(archivePath);
        LCNC_ERR(lcnc::LogCode::Generic,
                 "QuaZip compressDir failed source='{}' target='{}'",
                 sourceDir.toStdString(), archivePath.toStdString());
        QFile::remove(archivePath); // 清理可能的半成品
        return false;
    }
    return true;
}

bool extractZipToDirectory(const QString& archivePath, const QString& targetDir, QString* errorMsg)
{
    if (!QDir().mkpath(targetDir)) {
        if (errorMsg)
            *errorMsg = QStringLiteral("无法创建 .lcnc 解压目录: %1").arg(targetDir);
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Failed to create .lcnc extract directory '{}'",
                 targetDir.toStdString());
        return false;
    }

    LCNC_DEBUG(lcnc::LogCode::Generic,
               "Extracting .lcnc archive source='{}' targetDir='{}' (QuaZip)",
               archivePath.toStdString(),
               targetDir.toStdString());

    const QStringList extracted = JlCompress::extractDir(archivePath, targetDir);
    if (extracted.isEmpty()) {
        if (errorMsg)
            *errorMsg = QStringLiteral("解压 .lcnc 项目失败或包为空: %1").arg(archivePath);
        LCNC_ERR(lcnc::LogCode::Generic,
                 "QuaZip extractDir failed or empty archive='{}'",
                 archivePath.toStdString());
        return false;
    }
    return true;
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
                      int formatVersion,
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
    exportEntityKind(workpieceDocument, LcncDocument::EntityKind::Auxiliary, shapeTool);
    // v3：统一工程文档 —— CAM 轮廓几何(wire)随工程持久化（EntityKind::Cam）；
    // 稠密采样点仍存 cam_toolpath_points.bin。机台不进 XBF（独立参考资产）。
    exportEntityKind(workpieceDocument, LcncDocument::EntityKind::Cam, shapeTool);
    (void)machineDocument;
    (void)camDocument;
    (void)formatVersion;

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
                      int formatVersion,
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
    int migratedMachine = 0;
    int discardedCam = 0;
    for (int i = 1; i <= labels.Length(); ++i) {
        const TDF_Label label = labels.Value(i);
        const TopoDS_Shape shape = shapeTool->GetShape(label);
        if (shape.IsNull())
            continue;

        const LcncDocument::EntityKind kind = entityKindFromLabel(label);
        if (formatVersion >= 3) {
            // v3: 统一工程文档 —— 工件 + CAM 轮廓同存，按存储的 EntityKind 还原。
            workpieceDocument.addShapeEntity(shape,
                labelNameOrFallback(label, QStringLiteral("Shape_%1").arg(i)),
                kind);
        } else if (formatVersion == 2) {
            // v2: 所有 shape 都属工件（只有 workpiece.xbf，无 CAM 几何）。
            workpieceDocument.addShapeEntity(shape,
                labelNameOrFallback(label, QStringLiteral("Shape_%1").arg(i)),
                LcncDocument::EntityKind::Workpiece);
        } else {
            // v1: 按 kind 分发。
            LcncDocument* target = &workpieceDocument;
            LcncDocument::EntityKind targetKind = LcncDocument::EntityKind::Workpiece;
            if (kind == LcncDocument::EntityKind::Machine && machineDocument) {
                target = machineDocument;
                targetKind = LcncDocument::EntityKind::Machine;
                ++migratedMachine;
            } else if (kind == LcncDocument::EntityKind::Cam) {
                // CAM 实体不再需要(Phase C 剥离了 XCAF 镜像)，直接丢弃。
                ++discardedCam;
                continue;
            }
            target->addShapeEntity(shape,
                labelNameOrFallback(label, QStringLiteral("Shape_%1").arg(i)),
                targetKind);
        }
    }
    if (migratedMachine > 0 || discardedCam > 0) {
        LCNC_INFO(lcnc::LogCode::Generic,
                  "loadXcafSnapshot: v1 migration machine={} cam_discarded={}",
                  migratedMachine, discardedCam);
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
    // v2+ 使用 workpiece.xbf；v1 保持 project.xbf
    const QString xbf = manifest.formatVersion >= 2 ? manifest.workpieceXcafPath : manifest.projectXcafPath;
    return QDir(packageDirectory(path)).filePath(xbf);
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
                              QString* errorMsg,
                              lcnc::cam::CamDataManager* camData)
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
                          xcafPath, options, manifest.formatVersion, errorMsg)) {
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

    // 工程核心 CAM 数据写入与几何同一个 staging 目录 —— 对归档(.lcnc zip)而言即写入包内，
    // 随后一起打包；对目录包而言写入包目录。单一事务。
    if (camData) {
        QString camErr;
        if (!lcnc::cam::saveCamToolpath(*camData, packagePath, &camErr))
            LCNC_WARN(lcnc::LogCode::Generic,
                      "Failed to save CAM toolpath into package '{}': {}",
                      packagePath.toStdString(), camErr.toStdString());
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
                              QString* errorMsg,
                              lcnc::cam::CamDataManager* camData)
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
                          xcafPath, manifest.formatVersion, errorMsg)) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Failed to load .lcnc XCAF snapshot '{}'",
                 xcafPath.toStdString());
        return false;
    }

    // 工程核心 CAM 数据从同一解压目录读取（归档 .lcnc 时即包内）——必须在 tempPackage 销毁前完成。
    if (camData) {
        QString camErr;
        if (!lcnc::cam::loadCamToolpath(*camData, packagePath, &camErr))
            LCNC_INFO(lcnc::LogCode::Generic,
                      "No CAM toolpath in package '{}' ({})",
                      packagePath.toStdString(), camErr.toStdString());
        // v1 旧档 process_cutting_plan.toml → CAM 容器的一次性迁移（包内）。
        if (!lcnc::cam::migrateLegacyProcessCuttingPlan(*camData, packagePath, &camErr))
            LCNC_WARN(lcnc::LogCode::Generic,
                      "v1 cutting-plan migration failed in '{}': {}",
                      packagePath.toStdString(), camErr.toStdString());
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