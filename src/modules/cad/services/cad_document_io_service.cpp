#include "modules/cad/services/cad_document_io_service.h"

#include "core/document/lcnc_document.h"
#include "core/project/lcnc_project_manager.h"
#include "core/project/lcnc_project_package.h"
#include "core/project/project_workspace.h"

namespace lcnc::cad {

CadDocumentIoService::CadDocumentIoService(lcnc::LcncProjectManager& projectManager,
                                           QObject* parent)
    : QObject(parent)
    , m_projectManager(projectManager)
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

} // namespace lcnc::cad
