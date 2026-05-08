#pragma once

#include "core/project/lcnc_project_session.h"

#include <QObject>
#include <QString>
#include <memory>

class LcncDocument;

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
    DocumentId workpieceDocumentId() const;
    DocumentId machineDocumentId() const;
    DocumentId camDocumentId() const;
    bool domainForDocument(DocumentId documentId, ProjectDomain* domain) const;
    bool isDomainDocument(DocumentId documentId, ProjectDomain domain) const;

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
    std::unique_ptr<LcncDocument> m_machineDocument;
    std::unique_ptr<LcncDocument> m_camDocument;
    int m_nextDocumentId{0};
    LcncProjectSession m_session;
};

} // namespace lcnc
