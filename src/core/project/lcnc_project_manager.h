#pragma once

#include "core/project/lcnc_project_session.h"
#include "core/settings/app_settings.h"

#include <QObject>
#include <QString>
#include <QList>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>

class LcncDocument;

namespace lcnc::cam { class CamDataManager; }

namespace lcnc {

class ProjectWorkspace;

/**
 * @brief Project lifecycle and data-domain coordinator for open workspaces.
 *
 * Owns project workspaces in the core layer. CAD/CAM/Process modules borrow
 * data through active or explicit workspace accessors; they do not own it.
 */
class LcncProjectManager : public QObject
{
    Q_OBJECT
public:
    explicit LcncProjectManager(QObject* parent = nullptr);
    ~LcncProjectManager() override;

    LcncProjectSession& session();
    const LcncProjectSession& session() const;

    void ensureProject();

    DocumentOpenMode documentOpenMode() const { return m_documentOpenMode; }
    void setDocumentOpenMode(DocumentOpenMode mode) { m_documentOpenMode = mode; }

    ProjectWorkspaceId activeWorkspaceId() const { return m_activeWorkspaceId; }
    ProjectWorkspace* activeWorkspace() const;
    ProjectWorkspace* workspace(ProjectWorkspaceId id) const;
    QList<ProjectWorkspaceId> workspaceIds() const;

    std::shared_ptr<ProjectWorkspace> createDetachedWorkspace(const QString& name = QString());
    ProjectWorkspaceId adoptWorkspace(const std::shared_ptr<ProjectWorkspace>& workspace,
                                      bool emitProjectOpened = false);
    /// Gives borrowers a veto point before a workspace releases its documents.
    /// A false result leaves the workspace intact for a later retry.
    using WorkspaceCloseGuard = std::function<bool(ProjectWorkspaceId)>;
    void setWorkspaceCloseGuard(WorkspaceCloseGuard guard);
    bool closeWorkspace(ProjectWorkspaceId id);
    [[nodiscard]] bool closeAllWorkspaces();
    void setActiveWorkspace(ProjectWorkspaceId id);
    std::uint64_t beginSingleDocumentOpen();
    bool isSingleDocumentOpenCurrent(std::uint64_t generation) const;

    LcncDocument* newProject(const QString& name = QString());
    LcncDocument* openProject(const QString& filePath, QString* errorMsg = nullptr);
    bool saveProject(const QString& filePath = QString(), QString* errorMsg = nullptr);

    LcncDocument* importWorkpieceModel(const QString& filePath, QString* errorMsg = nullptr);
    bool exportDomainAsStep(ProjectDomain domain, const QString& filePath, QString* errorMsg = nullptr);
    void clearDomain(ProjectDomain domain);
    void notifyDomainChanged(ProjectDomain domain);
    void notifyDomainChanged(DocumentId documentId);

    LcncDocument* document(ProjectDomain domain) const;
    LcncDocument* domainDocumentById(DocumentId documentId) const;
    DocumentId documentId(ProjectDomain domain) const;
    LcncDocument* workpieceDocument() const;
    LcncDocument* machineDocument() const;
    LcncDocument* camDocument() const;

    /// CAM 运行时数据（轮廓 + 刀路 + 图层 + 工艺参数）是工程核心数据，
    /// 由本管理器在 core 层拥有；CAM 模块仅借用此实例做业务计算与渲染。
    lcnc::cam::CamDataManager* camData() const;
    DocumentId workpieceDocumentId() const;
    DocumentId machineDocumentId() const;
    DocumentId camDocumentId() const;
    bool domainForDocument(DocumentId documentId, ProjectDomain* domain) const;
    bool isDomainDocument(DocumentId documentId, ProjectDomain domain) const;

    /// CAM 的 MachineWorkspace 借用 manager 的 id 序列以保持 docId 全局唯一。
    int reserveDocumentId();
    /// 工厂：构造一个机台 LcncDocument（构造函数为 private，仅 manager 可建）。
    /// 所有权交给调用方（MachineWorkspace），manager 不持有。机台是参考资产，
    /// 不属于工程数据，不随 .lcnc 持久化。
    LcncDocument* createMachineDocument();
    /// 机台是独立参考资产，由 MachineWorkspace 拥有。此处仅登记一个 **非拥有引用**，
    /// 供视图/选择的域路由（GuiDocument::domainForDocument 等）识别机台文档；
    /// manager 不读写机台几何、不持久化、不计入工程脏标记。传 nullptr 解除登记。
    void attachMachineDocument(LcncDocument* borrowed);

signals:
    void workspaceAdded(ProjectWorkspaceId id);
    void workspaceAboutToClose(ProjectWorkspaceId id);
    void workspaceClosed(ProjectWorkspaceId id);
    void activeWorkspaceChanged(ProjectWorkspaceId id);
    void workspaceDomainChanged(ProjectWorkspaceId id, lcnc::ProjectDomain domain);
    void projectReset();
    void projectOpened(const QString& filePath);
    /// Emitted after opening a project saved against a different machine
    /// configuration. The core does not display UI; consumers decide how to
    /// present it while Process uses the session compatibility gate.
    void projectMachineConfigurationMismatch(const QString& filePath,
                                             const QString& expectedFingerprint,
                                             const QString& actualFingerprint);
    void projectSaved(const QString& filePath);
    void domainDataChanged(lcnc::ProjectDomain domain);
    void projectDirtyChanged(bool dirty);

private:
    LcncDocument* createDomainDocument(ProjectDomain domain, const QString& name);
    void resetWorkspace(ProjectWorkspace* workspace, const QString& projectName);
    bool importGeometryFile(LcncDocument* document, const QString& filePath, QString* errorMsg);
    void syncWorkspaceSession(ProjectWorkspace* workspace) const;
    void markDomainDirty(ProjectDomain domain);
    LcncProjectSession& fallbackSession();
    const LcncProjectSession& fallbackSession() const;
    std::shared_ptr<ProjectWorkspace> makeWorkspace(ProjectWorkspaceId id, const QString& name);
    void bindMachineDocumentToWorkspaces();

    std::unique_ptr<LcncDocument> m_machineDocument;       ///< 内部 owned 兜底；attachMachineDocument 后被借用指针取代。
    LcncDocument*                 m_machineBorrowed{nullptr}; ///< CAM 工作台登记的"借用"机台 doc（非拥有视图路由引用）。
    std::map<ProjectWorkspaceId, std::shared_ptr<ProjectWorkspace>> m_workspaces;
    WorkspaceCloseGuard m_workspaceCloseGuard;
    ProjectWorkspaceId m_activeWorkspaceId{kInvalidProjectWorkspaceId};
    DocumentOpenMode m_documentOpenMode{DocumentOpenMode::MultiDocument};
    std::uint64_t m_singleDocumentOpenGeneration{0};
    int m_nextDocumentId{0};
    ProjectWorkspaceId m_nextWorkspaceId{0};
    mutable LcncProjectSession m_emptySession;
};

} // namespace lcnc
