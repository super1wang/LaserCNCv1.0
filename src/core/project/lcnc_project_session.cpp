#include "core/project/lcnc_project_session.h"

namespace lcnc {

void WorkpieceProjectState::clear()
{
    displayName.clear();
    sourceFilePath.clear();
}

void MachineProjectState::clear()
{
    modelFilePath.clear();
}

void CamProjectState::clear()
{
    hasRuntimeData = false;
}

void LcncProjectSession::bindDomainDocuments(LcncDocument* workpieceDocument,
                                             LcncDocument* machineDocument,
                                             LcncDocument* camDocument)
{
    m_workpieceDocument = workpieceDocument;
    m_machineDocument = machineDocument;
    m_camDocument = camDocument;
}

LcncDocument* LcncProjectSession::document(ProjectDomain domain) const
{
    switch (domain) {
    case ProjectDomain::Workpiece:
        return m_workpieceDocument;
    case ProjectDomain::Machine:
        return m_machineDocument;
    case ProjectDomain::Cam:
        return m_camDocument;
    case ProjectDomain::Project:
        return nullptr;
    }
    return nullptr;
}

void LcncProjectSession::markDirty(ProjectDomain domain)
{
    m_dirtyFlags |= dirtyFlagForDomain(domain);
}

void LcncProjectSession::clearDirty()
{
    m_dirtyFlags = ProjectDirtyFlags{};
}

void LcncProjectSession::resetProjectState()
{
    m_projectName.clear();
    m_projectPath.clear();
    m_manifest = LcncProjectManifest{};
    m_saveOptions = ProjectSaveOptions{};
    m_workpiece.clear();
    m_machine.clear();
    m_cam.clear();
    clearDirty();
}

ProjectDirtyFlag dirtyFlagForDomain(ProjectDomain domain)
{
    switch (domain) {
    case ProjectDomain::Project:
        return ProjectDirtyFlag::Project;
    case ProjectDomain::Workpiece:
        return ProjectDirtyFlag::Workpiece;
    case ProjectDomain::Machine:
        return ProjectDirtyFlag::Machine;
    case ProjectDomain::Cam:
        return ProjectDirtyFlag::Cam;
    }
    return ProjectDirtyFlag::Project;
}

} // namespace lcnc
