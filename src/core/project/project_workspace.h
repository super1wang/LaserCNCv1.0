#pragma once

#include "core/project/lcnc_project_session.h"

#include <cstdint>
#include <memory>

class LcncDocument;

namespace lcnc::cam { class CamDataManager; }

namespace lcnc {

/**
 * @brief Core-owned data aggregate for one open project workspace.
 *
 * A workspace owns the project XCAF document and dense CAM runtime data. The
 * machine document is a borrowed global reference managed by MachineWorkspace.
 */
class ProjectWorkspace
{
public:
    ProjectWorkspace(ProjectWorkspaceId id,
                     std::unique_ptr<LcncDocument> projectDocument,
                     std::unique_ptr<lcnc::cam::CamDataManager> camData);
    ~ProjectWorkspace();

    ProjectWorkspaceId id() const { return m_id; }
    std::uint64_t generation() const { return m_generation; }

    LcncDocument*       workpieceDocument() { return m_projectDocument.get(); }
    const LcncDocument* workpieceDocument() const { return m_projectDocument.get(); }
    LcncDocument*       camDocument() { return m_projectDocument.get(); }
    const LcncDocument* camDocument() const { return m_projectDocument.get(); }

    lcnc::cam::CamDataManager*       camData() { return m_camData.get(); }
    const lcnc::cam::CamDataManager* camData() const { return m_camData.get(); }

    LcncProjectSession&       session() { return m_session; }
    const LcncProjectSession& session() const { return m_session; }

    void bindMachineDocument(LcncDocument* machineDocument);
    void resetProjectState(const QString& projectName);
    void syncSessionFromDocuments();

private:
    ProjectWorkspaceId m_id{kInvalidProjectWorkspaceId};
    std::uint64_t m_generation{0};
    std::unique_ptr<LcncDocument> m_projectDocument;
    std::unique_ptr<lcnc::cam::CamDataManager> m_camData;
    LcncProjectSession m_session;
    LcncDocument* m_machineDocument{nullptr};
};

} // namespace lcnc
