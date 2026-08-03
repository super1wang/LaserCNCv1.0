#include "modules/cad/services/cad_document_io_service.h"

#include "core/document/lcnc_document.h"
#include "core/project/lcnc_project_manager.h"
#include "core/project/lcnc_project_package.h"
#include "core/project/project_workspace.h"
#include "core/task/task_progress.h"

#include <QFileInfo>

#include <IFSelect_ReturnStatus.hxx>
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <STEPControl_Writer.hxx>
#include <StlAPI_Reader.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopoDS_Shape.hxx>
#include <XCAFDoc_ShapeTool.hxx>

namespace lcnc::cad {

CadDocumentIoService::CadDocumentIoService(lcnc::LcncProjectManager& projectManager,
                                           TaskManager& taskManager,
                                           QObject* parent)
    : QObject(parent)
    , m_projectManager(projectManager)
    , m_taskManager(taskManager)
{
}

DocumentId CadDocumentIoService::createDocument(const QString& name) const
{
    LcncDocument* document = m_projectManager.newProject(name);
    return document ? document->id() : kInvalidDocumentId;
}

bool CadDocumentIoService::saveDocument(LcncDocument* document,
                                        const QString& path,
                                        QString* errorMessage) const
{
    if (!document) {
        if (errorMessage)
            // 中文翻译：找不到目标文档
            *errorMessage = tr("Target document not found");
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
        : m_projectManager.exportDomainAsStep(lcnc::ProjectDomain::Workpiece,
                                              targetPath,
                                              errorMessage);
}

bool CadDocumentIoService::closeDocument(DocumentId documentId) const
{
    for (ProjectWorkspaceId workspaceId : m_projectManager.workspaceIds()) {
        auto* workspace = m_projectManager.workspace(workspaceId);
        if (workspace && workspace->workpieceDocument()
            && workspace->workpieceDocument()->id() == documentId) {
            return m_projectManager.closeWorkspace(workspaceId);
        }
    }
    return false;
}

CadDocumentIoService::ExportTask CadDocumentIoService::exportStepAsync(
    LcncDocument* document,
    const QString& filePath) const
{
    ExportTask task;
    task.error = std::make_shared<QString>();
    if (!document || filePath.isEmpty()) {
        // 中文翻译：找不到目标文档；未指定导出路径
        *task.error = !document ? tr("Target document not found")
                                : tr("No export path specified");
        return task;
    }

    task.id = m_taskManager.run(
        // 中文翻译：导出 STEP: %1
        tr("Export STEP: %1").arg(QFileInfo(filePath).fileName()),
        [document, filePath, error = task.error](TaskProgress* progress) {
            progress->setRange(0, 100);
            // 中文翻译：写入 STEP...
            progress->setStepName(QStringLiteral("Write STEP..."));
            if (progress->isAbortRequested())
                throw std::runtime_error("step export cancelled");

            Handle(XCAFDoc_ShapeTool) shapeTool = document->shapeTool();
            TDF_LabelSequence shapes;
            shapeTool->GetFreeShapes(shapes);
            STEPControl_Writer writer;
            for (int index = 1; index <= shapes.Length(); ++index) {
                const TopoDS_Shape shape = shapeTool->GetShape(shapes.Value(index));
                if (!shape.IsNull())
                    writer.Transfer(shape, STEPControl_AsIs);
            }
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

CadDocumentIoService::ImportTask CadDocumentIoService::importStlAsync(
    LcncDocument* document,
    const QString& filePath) const
{
    ImportTask task;
    task.error = std::make_shared<QString>();
    if (!document || filePath.isEmpty()) {
        // 中文翻译：找不到目标文档；文件不存在: %1
        *task.error = !document ? tr("Target document not found")
                                : tr("File does not exist: %1").arg(filePath);
        return task;
    }

    task.id = m_taskManager.run(
        // 中文翻译：导入 STL: %1
        tr("Import STL: %1").arg(QFileInfo(filePath).fileName()),
        [this, document, filePath, error = task.error](TaskProgress* progress) {
            if (!importStlIntoDocument(document, filePath, progress, error.get()))
                throw std::runtime_error("stl import failed");
        });
    return task;
}

bool CadDocumentIoService::importStlIntoDocument(LcncDocument* document,
                                                  const QString& filePath,
                                                  TaskProgress* progress,
                                                  QString* errorMessage) const
{
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
    document->addShapeEntity(shape,
                             QFileInfo(filePath).baseName(),
                             LcncDocument::EntityKind::Workpiece);
    if (progress && progress->isAbortRequested())
        return false;
    if (progress)
        progress->setValue(100);
    return true;
}

CadDocumentIoService::ImportTask CadDocumentIoService::importBrepAsync(
    LcncDocument* document,
    const QString& filePath) const
{
    ImportTask task;
    task.error = std::make_shared<QString>();
    if (!document || filePath.isEmpty()) {
        // 中文翻译：找不到目标文档；文件不存在: %1
        *task.error = !document ? tr("Target document not found")
                                : tr("File does not exist: %1").arg(filePath);
        return task;
    }

    task.id = m_taskManager.run(
        // 中文翻译：导入 BREP: %1
        tr("Import BREP: %1").arg(QFileInfo(filePath).fileName()),
        [this, document, filePath, error = task.error](TaskProgress* progress) {
            if (!importBrepIntoDocument(document, filePath, progress, error.get()))
                throw std::runtime_error("brep import failed");
        });
    return task;
}

bool CadDocumentIoService::importBrepIntoDocument(LcncDocument* document,
                                                   const QString& filePath,
                                                   TaskProgress* progress,
                                                   QString* errorMessage) const
{
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
    document->addShapeEntity(shape,
                             QFileInfo(filePath).baseName(),
                             LcncDocument::EntityKind::Workpiece);
    if (progress && progress->isAbortRequested())
        return false;
    if (progress)
        progress->setValue(100);
    return true;
}

} // namespace lcnc::cad
