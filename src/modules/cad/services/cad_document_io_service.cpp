#include "modules/cad/services/cad_document_io_service.h"

#include "core/document/lcnc_document.h"
#include "core/project/lcnc_project_manager.h"
#include "core/project/lcnc_project_package.h"
#include "core/project/project_workspace.h"
#include "core/task/task_progress.h"

#include <QFileInfo>

#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_Writer.hxx>
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

} // namespace lcnc::cad
