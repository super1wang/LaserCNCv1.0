#pragma once

#include "core/project/lcnc_project_session.h"

#include <QObject>
#include <QString>
#include <memory>

class LcncDocument;

namespace lcnc::cam { class CamDataManager; }

namespace lcnc {

/**
 * @brief Project lifecycle and data-domain coordinator for the single project.
 *
 * Owns the three persistent OCC-backed domain stores and keeps project identity,
 * dirty state, and package IO outside LcncDocument.
 */
class LcncProjectManager : public QObject
{
    Q_OBJECT
public:
    explicit LcncProjectManager(QObject* parent = nullptr);
    ~LcncProjectManager() override;

    LcncProjectSession& session() { return m_session; }
    const LcncProjectSession& session() const { return m_session; }

    void ensureProject();

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
    lcnc::cam::CamDataManager* camData() const { return m_camData.get(); }
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
    void projectReset();
    void projectOpened(const QString& filePath);
    void projectSaved(const QString& filePath);
    void domainDataChanged(lcnc::ProjectDomain domain);
    void projectDirtyChanged(bool dirty);

private:
    LcncDocument* createDomainDocument(ProjectDomain domain, const QString& name);
    void resetProjectDocuments(const QString& projectName);
    bool importGeometryFile(LcncDocument* document, const QString& filePath, QString* errorMsg);
    void syncSessionFromDocuments();
    void markDomainDirty(ProjectDomain domain);

    std::unique_ptr<LcncDocument> m_workpieceDocument;
    std::unique_ptr<LcncDocument> m_machineDocument;       ///< 内部 owned 兜底；attachMachineDocument 后被借用指针取代。
    LcncDocument*                 m_machineBorrowed{nullptr}; ///< Phase D：CAM 工作台注入的"借用" machine doc。
    std::unique_ptr<LcncDocument> m_camDocument;
    std::unique_ptr<lcnc::cam::CamDataManager> m_camData; ///< 工程核心：CAM 运行时数据。
    int m_nextDocumentId{0};
    LcncProjectSession m_session;
};

} // namespace lcnc
