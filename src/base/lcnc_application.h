#pragma once

#include <QObject>
#include <QString>
#include <QList>

// Forward declarations
class LcncDocument;

// Unique document identifier within one session
using DocumentId = int;
constexpr DocumentId kInvalidDocumentId = -1;

/**
 * @brief Central application manager for LaserCNC.
 *
 * Owns and manages the lifecycle of all open LcncDocument instances.
 * Bridges the OCC XCAFApp_Application with Qt's signal/slot system.
 */
class LcncApplication : public QObject
{
    Q_OBJECT
public:
    static LcncApplication* instance();

    // ── Document lifecycle ────────────────────────────────────────────────────
    LcncDocument* newDocument(const QString& name = QString());
    LcncDocument* openDocument(const QString& filePath, QString* errorMsg = nullptr);
    bool saveDocument(DocumentId id, const QString& filePath, QString* errorMsg = nullptr);
    void closeDocument(DocumentId id);
    void notifyDocumentModified(DocumentId id);

    // ── Document access ───────────────────────────────────────────────────────
    LcncDocument*        documentById(DocumentId id) const;
    QList<LcncDocument*> documents() const;
    int                  documentCount() const;

    // ── Active document ───────────────────────────────────────────────────────
    DocumentId    activeDocumentId() const { return m_activeId; }
    LcncDocument* activeDocument() const;
    void          setActiveDocument(DocumentId id);

signals:
    void documentAdded(DocumentId id);
    void documentClosed(DocumentId id);
    void documentModified(DocumentId id);
    void activeDocumentChanged(DocumentId id);

private:
    explicit LcncApplication(QObject* parent = nullptr);
    ~LcncApplication() override;

    static LcncApplication* s_instance;

    QList<LcncDocument*> m_documents;
    DocumentId           m_activeId   = kInvalidDocumentId;
    int                  m_nextId     = 0;
};
