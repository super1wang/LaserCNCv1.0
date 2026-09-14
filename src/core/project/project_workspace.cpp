#include "core/project/project_workspace.h"

#include "core/document/lcnc_document.h"
#include "core/project/cam/cam_data_manager.h"

namespace lcnc {

ProjectWorkspace::ProjectWorkspace(ProjectWorkspaceId id,
                                   std::unique_ptr<LcncDocument> projectDocument,
                                   std::unique_ptr<lcnc::cam::CamDataManager> camData)
    : m_id(id)
    , m_projectDocument(std::move(projectDocument))
    , m_camData(std::move(camData))
{
    syncSessionFromDocuments();
}

ProjectWorkspace::~ProjectWorkspace() = default;

void ProjectWorkspace::bindMachineDocument(LcncDocument* machineDocument)
{
    m_machineDocument = machineDocument;
    syncSessionFromDocuments();
}

void ProjectWorkspace::resetProjectState(const QString& projectName)
{
    if (m_projectDocument) {
        m_projectDocument->setName(projectName);
        m_projectDocument->setFilePath(QString());
    }
    if (m_camData)
        m_camData->clearToolpath();

    m_session.resetProjectState();
    m_session.setProjectName(projectName);
    m_session.setProjectPath(QString());
    syncSessionFromDocuments();
    m_session.clearDirty();
}

void ProjectWorkspace::syncSessionFromDocuments()
{
    m_session.bindDomainDocuments(workpieceDocument(), m_machineDocument, camDocument());
    if (LcncDocument* workpiece = workpieceDocument()) {
        m_session.setProjectName(workpiece->name());
        m_session.setProjectPath(workpiece->filePath());
    }
}

} // namespace lcnc
