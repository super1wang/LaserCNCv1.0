#include "modules/cad/services/cad_document_io_service.h"

#include "core/document/lcnc_document.h"
#include "core/document/xcaf_utils.h"
#include "core/logging/logger.h"
#include "core/project/lcnc_project_manager.h"
#include "core/project/lcnc_project_package.h"
#include "core/project/project_workspace.h"
#include "core/task/task_progress.h"

#include <BRepBndLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_tool.hxx>
#include <Bnd_Box.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <IGESCAFControl_Reader.hxx>
#include <IGESControl_Reader.hxx>
#include <IMeshTools_Parameters.hxx>
#include <QFileInfo>
#include <STEPCAFControl_Reader.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <Standard_Failure.hxx>
#include <StlAPI_Reader.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDocStd_Document.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Shape.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <algorithm>
#include <exception>
#include <vector>

namespace lcnc::cad {

struct CadImportPayload {
    Handle(TDocStd_Document) xcafDocument;
    TopoDS_Shape fallbackShape;
    QString displayName;

    bool empty() const noexcept {
        return xcafDocument.IsNull() && fallbackShape.IsNull();
    }
};

CadDocumentIoService::CadDocumentIoService(lcnc::LcncProjectManager& projectManager,
                                           TaskManager& taskManager, QObject* parent)
    : QObject(parent), m_projectManager(projectManager), m_taskManager(taskManager) {}

DocumentId CadDocumentIoService::createDocument(const QString& name) const {
    LcncDocument* document = m_projectManager.newProject(name);
    return document ? document->id() : kInvalidDocumentId;
}

bool CadDocumentIoService::saveDocument(LcncDocument* document, const QString& path,
                                        QString* errorMessage) const {
    if (!document) {
        if (errorMessage)
            // 中文翻译：找不到目标文档
            *errorMessage = tr("Target document not found");
        return false;
    }

    // saveProject currently saves the active workspace.  Refuse a mismatched
    // document rather than silently serializing another workspace.
    if (m_projectManager.workpieceDocument() != document) {
        if (errorMessage)
            // 中文翻译：目标文档不是活动工程
            *errorMessage = tr("Target document is not the active project");
        return false;
    }

    const QString targetPath = path.isEmpty() ? document->filePath() : path;
    if (targetPath.isEmpty()) {
        if (errorMessage)
            // 中文翻译：未指定保存路径
            *errorMessage = tr("No save path specified");
        return false;
    }

    return lcnc::LcncProjectPackage::isProjectPath(targetPath)
               ? m_projectManager.saveProject(targetPath, errorMessage)
               : m_projectManager.exportDomainAsStep(lcnc::ProjectDomain::Workpiece, targetPath,
                                                     errorMessage);
}

bool CadDocumentIoService::closeDocument(DocumentId documentId) const {
    for (ProjectWorkspaceId workspaceId : m_projectManager.workspaceIds()) {
        auto* workspace = m_projectManager.workspace(workspaceId);
        if (workspace && workspace->workpieceDocument() &&
            workspace->workpieceDocument()->id() == documentId) {
            return m_projectManager.closeWorkspace(workspaceId);
        }
    }
    return false;
}

CadDocumentIoService::ExportTask
CadDocumentIoService::exportStepAsync(LcncDocument* document, const QString& filePath) const {
    ExportTask task;
    task.error = std::make_shared<QString>();
    if (!document || filePath.isEmpty()) {
        // 中文翻译：找不到目标文档；未指定导出路径
        *task.error = !document ? tr("Target document not found") : tr("No export path specified");
        return task;
    }

    auto shapes = std::make_shared<std::vector<TopoDS_Shape>>();
    const Handle(XCAFDoc_ShapeTool) shapeTool = document->shapeTool();
    TDF_LabelSequence labels;
    shapeTool->GetFreeShapes(labels);
    shapes->reserve(static_cast<std::size_t>(labels.Length()));
    for (int index = 1; index <= labels.Length(); ++index) {
        const TopoDS_Shape shape = shapeTool->GetShape(labels.Value(index));
        if (!shape.IsNull())
            shapes->push_back(shape);
    }
    if (shapes->empty()) {
        // 中文翻译：目标文档没有可导出的形体
        *task.error = tr("The target document contains no geometry to export");
        return task;
    }

    task.id = m_taskManager.run(
        // 中文翻译：导出 STEP: %1
        tr("Export STEP: %1").arg(QFileInfo(filePath).fileName()),
        [shapes, filePath, error = task.error](TaskProgress* progress) {
            progress->setRange(0, 100);
            // 中文翻译：写入 STEP...
            progress->setStepName(QStringLiteral("Write STEP..."));
            if (progress->isAbortRequested())
                throw std::runtime_error("step export cancelled");

            STEPControl_Writer writer;
            for (const TopoDS_Shape& shape : *shapes)
                writer.Transfer(shape, STEPControl_AsIs);
            if (writer.Write(filePath.toUtf8().constData()) != IFSelect_RetDone) {
                // 中文翻译：导出 STEP 失败: %1
                *error = QObject::tr("Export STEP failed: %1").arg(filePath);
                throw std::runtime_error("step export failed");
            }
            if (progress->isAbortRequested())
                throw std::runtime_error("step export cancelled");
            progress->setValue(100);
        });
    return task;
}

CadDocumentIoService::ImportTask
CadDocumentIoService::readImportAsync(const QString& filePath) const {
    ImportTask task;
    task.error = std::make_shared<QString>();
    task.payload = std::make_shared<CadImportPayload>();
    if (filePath.isEmpty() || !QFileInfo::exists(filePath)) {
        // 中文翻译：文件不存在: %1
        *task.error = tr("File does not exist: %1").arg(filePath);
        return task;
    }

    task.id = m_taskManager.run(
        // 中文翻译：读取导入文件: %1
        tr("Read import file: %1").arg(QFileInfo(filePath).fileName()),
        [filePath, payload = task.payload, error = task.error](TaskProgress* progress) {
            if (!CadDocumentIoService::readImport(filePath, progress, payload, error.get()) ||
                !CadDocumentIoService::prepareImportMesh(payload, progress, error.get())) {
                throw std::runtime_error("detached CAD import failed");
            }
            if (progress->isAbortRequested())
                throw std::runtime_error("detached CAD import cancelled");
            progress->setValue(100);
        });
    return task;
}

bool CadDocumentIoService::readImport(const QString& filePath, TaskProgress* progress,
                                      const std::shared_ptr<CadImportPayload>& payload,
                                      QString* errorMessage) {
    if (!payload || (progress && progress->isAbortRequested()))
        return false;
    payload->xcafDocument.Nullify();
    payload->fallbackShape.Nullify();
    payload->displayName = QFileInfo(filePath).baseName();
    const QString suffix = QFileInfo(filePath).suffix().toLower();
    if (progress) {
        progress->setRange(0, 100);
        // 中文翻译：读取 CAD 文件...
        progress->setStepName(QStringLiteral("Read CAD file..."));
        progress->setValue(10);
    }

    try {
        if (suffix == QStringLiteral("stp") || suffix == QStringLiteral("step")) {
            Handle(TDocStd_Document) xdeDocument =
                new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
            XCAFDoc_DocumentTool::Set(xdeDocument->Main());
            STEPCAFControl_Reader reader;
            reader.SetNameMode(Standard_True);
            if (reader.ReadFile(filePath.toUtf8().constData()) == IFSelect_RetDone &&
                reader.Transfer(xdeDocument)) {
                TDF_LabelSequence roots;
                XCAFDoc_DocumentTool::ShapeTool(xdeDocument->Main())->GetFreeShapes(roots);
                if (roots.Length() > 0)
                    payload->xcafDocument = xdeDocument;
            }
            if (payload->xcafDocument.IsNull()) {
                STEPControl_Reader fallback;
                if (fallback.ReadFile(filePath.toUtf8().constData()) != IFSelect_RetDone ||
                    fallback.TransferRoots() <= 0) {
                    if (errorMessage)
                        // 中文翻译：无法读取 STEP 文件: %1
                        *errorMessage = QObject::tr("Unable to read STEP file: %1").arg(filePath);
                    return false;
                }
                payload->fallbackShape = fallback.OneShape();
            }
        } else if (suffix == QStringLiteral("igs") || suffix == QStringLiteral("iges")) {
            Handle(TDocStd_Document) xdeDocument =
                new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
            XCAFDoc_DocumentTool::Set(xdeDocument->Main());
            IGESCAFControl_Reader reader;
            reader.SetNameMode(Standard_True);
            if (reader.ReadFile(filePath.toUtf8().constData()) == IFSelect_RetDone &&
                reader.Transfer(xdeDocument)) {
                TDF_LabelSequence roots;
                XCAFDoc_DocumentTool::ShapeTool(xdeDocument->Main())->GetFreeShapes(roots);
                if (roots.Length() > 0)
                    payload->xcafDocument = xdeDocument;
            }
            if (payload->xcafDocument.IsNull()) {
                IGESControl_Reader fallback;
                if (fallback.ReadFile(filePath.toUtf8().constData()) != IFSelect_RetDone ||
                    fallback.TransferRoots() <= 0) {
                    if (errorMessage)
                        // 中文翻译：无法读取 IGES 文件: %1
                        *errorMessage = QObject::tr("Unable to read IGES file: %1").arg(filePath);
                    return false;
                }
                payload->fallbackShape = fallback.OneShape();
            }
        } else if (suffix == QStringLiteral("stl")) {
            StlAPI_Reader reader;
            reader.Read(payload->fallbackShape, filePath.toUtf8().constData());
        } else if (suffix == QStringLiteral("brep")) {
            BRep_Builder builder;
            BRepTools::Read(payload->fallbackShape, filePath.toUtf8().constData(), builder);
        } else {
            if (errorMessage)
                // 中文翻译：暂不支持的文件格式: %1
                *errorMessage = QObject::tr("File format not supported yet: %1").arg(suffix);
            return false;
        }
    } catch (const Standard_Failure& exception) {
        LCNC_ERR(lcnc::LogCode::Generic, "Detached CAD import failed path='{}': {}",
                 filePath.toStdString(), exception.GetMessageString());
        if (errorMessage)
            // 中文翻译：读取 CAD 文件失败: %1
            *errorMessage = QObject::tr("Failed to read CAD file: %1")
                                .arg(QString::fromUtf8(exception.GetMessageString()));
        return false;
    } catch (const std::exception& exception) {
        LCNC_ERR(lcnc::LogCode::Generic, "Detached CAD import failed path='{}': {}",
                 filePath.toStdString(), exception.what());
        if (errorMessage)
            // 中文翻译：读取 CAD 文件失败: %1
            *errorMessage =
                QObject::tr("Failed to read CAD file: %1").arg(QString::fromUtf8(exception.what()));
        return false;
    } catch (...) {
        LCNC_ERR(lcnc::LogCode::Generic, "Detached CAD import failed path='{}': unknown exception",
                 filePath.toStdString());
        if (errorMessage)
            // 中文翻译：读取 CAD 文件时发生未知错误
            *errorMessage = QObject::tr("An unknown error occurred while reading the CAD file");
        return false;
    }

    if (payload->empty()) {
        if (errorMessage)
            // 中文翻译：CAD 文件未解析出可显示形体: %1
            *errorMessage =
                QObject::tr("The CAD file did not parse a displayable shape: %1").arg(filePath);
        return false;
    }
    if (progress)
        progress->setValue(55);
    return !progress || !progress->isAbortRequested();
}

bool CadDocumentIoService::prepareImportMesh(const std::shared_ptr<CadImportPayload>& payload,
                                             TaskProgress* progress, QString* errorMessage) {
    if (!payload || payload->empty())
        return false;

    std::vector<TopoDS_Shape> shapes;
    if (!payload->xcafDocument.IsNull()) {
        TDF_LabelSequence roots;
        const Handle(XCAFDoc_ShapeTool) shapeTool =
            XCAFDoc_DocumentTool::ShapeTool(payload->xcafDocument->Main());
        shapeTool->GetFreeShapes(roots);
        shapes.reserve(static_cast<std::size_t>(roots.Length()));
        for (int index = 1; index <= roots.Length(); ++index)
            shapes.push_back(shapeTool->GetShape(roots.Value(index)));
    } else {
        shapes.push_back(payload->fallbackShape);
    }

    int completed = 0;
    for (const TopoDS_Shape& shape : shapes) {
        if (progress && progress->isAbortRequested())
            return false;
        if (shape.IsNull())
            continue;
        try {
            Bnd_Box box;
            BRepBndLib::Add(shape, box, Standard_False);
            Standard_Real xMin = 0.0, yMin = 0.0, zMin = 0.0;
            Standard_Real xMax = 0.0, yMax = 0.0, zMax = 0.0;
            if (!box.IsVoid())
                box.Get(xMin, yMin, zMin, xMax, yMax, zMax);
            const double maxSize =
                box.IsVoid() ? 1.0 : std::max({xMax - xMin, yMax - yMin, zMax - zMin});
            BRepTools::Clean(shape);
            IMeshTools_Parameters parameters;
            parameters.InParallel = Standard_True;
            parameters.AllowQualityDecrease = Standard_False;
            parameters.Relative = Standard_False;
            parameters.Deflection = std::clamp(0.0005 * maxSize, 0.005, 0.05);
            parameters.Angle = 5.0 * 3.14159265358979323846 / 180.0;
            BRepMesh_IncrementalMesh mesher(shape, parameters);
            if (!mesher.IsDone()) {
                if (errorMessage)
                    // 中文翻译：模型显示网格生成未完成
                    *errorMessage = QObject::tr("Model shows mesh generation not completed");
                return false;
            }
        } catch (const Standard_Failure& exception) {
            LCNC_ERR(lcnc::LogCode::Generic, "Detached CAD mesh generation failed: {}",
                     exception.GetMessageString());
            if (errorMessage)
                // 中文翻译：模型显示网格生成失败: %1
                *errorMessage = QObject::tr("Model display mesh generation failed: %1")
                                    .arg(QString::fromUtf8(exception.GetMessageString()));
            return false;
        }
        ++completed;
        if (progress)
            progress->setValue(60 + (35 * completed) / std::max(1, int(shapes.size())));
    }
    return true;
}

bool CadDocumentIoService::commitImport(LcncDocument* document,
                                        const std::shared_ptr<CadImportPayload>& payload,
                                        QString* errorMessage) {
    if (!document || !payload || payload->empty()) {
        if (errorMessage)
            // 中文翻译：导入结果或目标文档无效
            *errorMessage = QObject::tr("The import result or target document is invalid");
        return false;
    }

    const int beforeCount = document->entityLabels(LcncDocument::EntityKind::Workpiece).Length();
    try {
        if (!payload->xcafDocument.IsNull()) {
            document->importFromXcaf(payload->xcafDocument, LcncDocument::EntityKind::Workpiece);
        } else {
            document->addShapeEntity(payload->fallbackShape, payload->displayName,
                                     LcncDocument::EntityKind::Workpiece);
        }
        const int afterCount = document->entityLabels(LcncDocument::EntityKind::Workpiece).Length();
        if (afterCount <= beforeCount) {
            if (errorMessage)
                // 中文翻译：导入结果没有可提交的形体
                *errorMessage = QObject::tr("The import result contains no geometry to commit");
            return false;
        }
        return true;
    } catch (const Standard_Failure& exception) {
        LCNC_ERR(lcnc::LogCode::Generic, "CAD import commit failed: {}",
                 exception.GetMessageString());
        if (errorMessage)
            // 中文翻译：提交 CAD 导入失败: %1
            *errorMessage = QObject::tr("Failed to commit CAD import: %1")
                                .arg(QString::fromUtf8(exception.GetMessageString()));
    } catch (const std::exception& exception) {
        LCNC_ERR(lcnc::LogCode::Generic, "CAD import commit failed: {}", exception.what());
        if (errorMessage)
            // 中文翻译：提交 CAD 导入失败: %1
            *errorMessage = QObject::tr("Failed to commit CAD import: %1")
                                .arg(QString::fromUtf8(exception.what()));
    } catch (...) {
        LCNC_ERR(lcnc::LogCode::Generic, "CAD import commit failed: unknown exception");
        if (errorMessage)
            // 中文翻译：提交 CAD 导入时发生未知错误
            *errorMessage = QObject::tr("An unknown error occurred while committing CAD import");
    }
    return false;
}

bool CadDocumentIoService::importStlIntoDetachedDocument(LcncDocument* document,
                                                         const QString& filePath,
                                                         TaskProgress* progress,
                                                         QString* errorMessage) {
    if (!document || (progress && progress->isAbortRequested()))
        return false;
    if (progress) {
        progress->setRange(0, 100);
        // 中文翻译：读取 STL...
        progress->setStepName(QStringLiteral("Read STL..."));
    }
    TopoDS_Shape shape;
    StlAPI_Reader reader;
    reader.Read(shape, filePath.toUtf8().constData());
    if (shape.IsNull()) {
        if (errorMessage)
            // 中文翻译：无法读取 STL 文件: %1
            *errorMessage = QObject::tr("Unable to read STL file: %1").arg(filePath);
        return false;
    }
    // Cancellation is transactional up to this commit point: do not report a
    // cancelled import after it has already changed the target document.
    if (progress && progress->isAbortRequested())
        return false;
    document->addShapeEntity(shape, QFileInfo(filePath).baseName(),
                             LcncDocument::EntityKind::Workpiece);
    if (progress)
        progress->setValue(100);
    return true;
}

bool CadDocumentIoService::importBrepIntoDetachedDocument(LcncDocument* document,
                                                          const QString& filePath,
                                                          TaskProgress* progress,
                                                          QString* errorMessage) {
    if (!document || (progress && progress->isAbortRequested()))
        return false;
    if (progress) {
        progress->setRange(0, 100);
        // 中文翻译：读取 BREP...
        progress->setStepName(QStringLiteral("Read BREP..."));
    }
    TopoDS_Shape shape;
    BRep_Builder builder;
    BRepTools::Read(shape, filePath.toUtf8().constData(), builder);
    if (shape.IsNull()) {
        if (errorMessage)
            // 中文翻译：无法读取 BREP 文件: %1
            *errorMessage = QObject::tr("Unable to read BREP file: %1").arg(filePath);
        return false;
    }
    // See STL import above: commit only after the final cancellation check.
    if (progress && progress->isAbortRequested())
        return false;
    document->addShapeEntity(shape, QFileInfo(filePath).baseName(),
                             LcncDocument::EntityKind::Workpiece);
    if (progress)
        progress->setValue(100);
    return true;
}

bool CadDocumentIoService::importStepIntoDetachedDocument(LcncDocument* document,
                                                          const QString& filePath,
                                                          QString* errorMessage) {
    if (!document)
        return false;

    const int beforeCount = document->entityLabels(LcncDocument::EntityKind::Workpiece).Length();
    Handle(TDocStd_Document) xdeDocument =
        new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
    XCAFDoc_DocumentTool::Set(xdeDocument->Main());
    STEPCAFControl_Reader reader;
    reader.SetNameMode(Standard_True);
    if (reader.ReadFile(filePath.toUtf8().constData()) != IFSelect_RetDone) {
        if (errorMessage)
            // 中文翻译：无法读取 STEP 文件: %1
            *errorMessage = tr("Unable to read STEP file: %1").arg(filePath);
        return false;
    }

    const Standard_Boolean transferOk = reader.Transfer(xdeDocument);
    document->importFromXcaf(xdeDocument, LcncDocument::EntityKind::Workpiece);
    const int importedCount =
        document->entityLabels(LcncDocument::EntityKind::Workpiece).Length() - beforeCount;
    LCNC_DEBUG(lcnc::LogCode::Generic, "STEP XCAF transfer path={} ok={} imported={}",
               filePath.toStdString(), static_cast<bool>(transferOk), importedCount);
    if (importedCount > 0)
        return true;

    STEPControl_Reader fallbackReader;
    if (fallbackReader.ReadFile(filePath.toUtf8().constData()) != IFSelect_RetDone) {
        if (errorMessage)
            // 中文翻译：无法读取 STEP 文件: %1
            *errorMessage = tr("Unable to read STEP file: %1").arg(filePath);
        return false;
    }
    const Standard_Integer transferred = fallbackReader.TransferRoots();
    const TopoDS_Shape shape = fallbackReader.OneShape();
    if (transferred <= 0 || shape.IsNull()) {
        if (errorMessage)
            // 中文翻译：STEP 文件未解析出可显示形体: %1
            *errorMessage = tr("The STEP file did not parse a displayable shape: %1").arg(filePath);
        return false;
    }
    document->addShapeEntity(shape, QFileInfo(filePath).baseName(),
                             LcncDocument::EntityKind::Workpiece);
    LCNC_WARN(
        lcnc::LogCode::Generic,
        "STEP XCAF import produced no entities; used single-shape fallback path={} transferred={}",
        filePath.toStdString(), transferred);
    return true;
}

bool CadDocumentIoService::importIgesIntoDetachedDocument(LcncDocument* document,
                                                          const QString& filePath,
                                                          QString* errorMessage) {
    if (!document)
        return false;

    const int beforeCount = document->entityLabels(LcncDocument::EntityKind::Workpiece).Length();
    Handle(TDocStd_Document) xdeDocument =
        new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
    XCAFDoc_DocumentTool::Set(xdeDocument->Main());
    IGESCAFControl_Reader reader;
    reader.SetNameMode(Standard_True);
    if (reader.ReadFile(filePath.toUtf8().constData()) != IFSelect_RetDone) {
        if (errorMessage)
            // 中文翻译：无法读取 IGES 文件: %1
            *errorMessage = tr("Unable to read IGES file: %1").arg(filePath);
        return false;
    }

    const Standard_Boolean transferOk = reader.Transfer(xdeDocument);
    document->importFromXcaf(xdeDocument, LcncDocument::EntityKind::Workpiece);
    const int importedCount =
        document->entityLabels(LcncDocument::EntityKind::Workpiece).Length() - beforeCount;
    LCNC_DEBUG(lcnc::LogCode::Generic, "IGES XCAF transfer path={} ok={} imported={}",
               filePath.toStdString(), static_cast<bool>(transferOk), importedCount);
    if (importedCount > 0)
        return true;

    IGESControl_Reader fallbackReader;
    if (fallbackReader.ReadFile(filePath.toUtf8().constData()) != IFSelect_RetDone) {
        if (errorMessage)
            // 中文翻译：无法读取 IGES 文件: %1
            *errorMessage = tr("Unable to read IGES file: %1").arg(filePath);
        return false;
    }
    const Standard_Integer transferred = fallbackReader.TransferRoots();
    const TopoDS_Shape shape = fallbackReader.OneShape();
    if (transferred <= 0 || shape.IsNull()) {
        if (errorMessage)
            // 中文翻译：IGES 文件未解析出可显示形体: %1
            *errorMessage =
                tr("The IGES file did not resolve a displayable shape: %1").arg(filePath);
        return false;
    }
    document->addShapeEntity(shape, QFileInfo(filePath).baseName(),
                             LcncDocument::EntityKind::Workpiece);
    LCNC_WARN(
        lcnc::LogCode::Generic,
        "IGES XCAF import produced no entities; used single-shape fallback path={} transferred={}",
        filePath.toStdString(), transferred);
    return true;
}

bool CadDocumentIoService::prepareDisplayMesh(LcncDocument* document, TaskProgress* progress,
                                              QString* errorMessage) {
    if (!document)
        return false;
    const TDF_LabelSequence labels = document->entityLabels(LcncDocument::EntityKind::Workpiece);
    const int count = labels.Length();
    for (int index = 1; index <= count; ++index) {
        if (progress && progress->isAbortRequested())
            return false;
        const TopoDS_Shape shape = XcafUtils::shape(labels.Value(index));
        if (shape.IsNull())
            continue;
        try {
            Bnd_Box box;
            BRepBndLib::Add(shape, box, Standard_False);
            Standard_Real xMin = 0.0, yMin = 0.0, zMin = 0.0;
            Standard_Real xMax = 0.0, yMax = 0.0, zMax = 0.0;
            if (!box.IsVoid())
                box.Get(xMin, yMin, zMin, xMax, yMax, zMax);
            const double maxSize =
                box.IsVoid() ? 1.0 : std::max({xMax - xMin, yMax - yMin, zMax - zMin});
            // GuiDocument deliberately disables AIS auto triangulation for
            // imported workpieces so the presentation uses this prepared mesh.
            // The former 0.4 %-of-model / 20 degree policy made circular
            // workpiece features visibly polygonal on large assemblies.  Drop
            // any serialized or earlier coarse triangulation first, then use
            // bounded display-quality values.  This runs before the document
            // is published to a GUI view, therefore the intentional mesh reset
            // cannot invalidate a live AIS presentation.
            BRepTools::Clean(shape);

            IMeshTools_Parameters params;
            params.InParallel = Standard_True;
            params.AllowQualityDecrease = Standard_False;
            params.Relative = Standard_False;
            params.Deflection = std::clamp(0.0005 * maxSize, 0.005, 0.05);
            params.Angle = 5.0 * 3.14159265358979323846 / 180.0;
            BRepMesh_IncrementalMesh mesher(shape, params);
            if (!mesher.IsDone()) {
                if (errorMessage)
                    // 中文翻译：模型显示网格生成未完成
                    *errorMessage = QObject::tr("Model shows mesh generation not completed");
                return false;
            }
        } catch (const Standard_Failure& exception) {
            LCNC_ERR(lcnc::LogCode::Generic, "CAD display mesh generation failed: {}",
                     exception.GetMessageString());
            if (errorMessage)
                // 中文翻译：模型显示网格生成失败: %1
                *errorMessage = QObject::tr("Model display mesh generation failed: %1")
                                    .arg(QString::fromUtf8(exception.GetMessageString()));
            return false;
        }
        if (progress)
            progress->setValue(60 + (35 * index) / std::max(1, count));
    }
    return true;
}

} // namespace lcnc::cad
