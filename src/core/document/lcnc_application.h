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

    // ── Machine workspace ─────────────────────────────────────────────────────
    /// Creates the unique machine document on first call.  Must be called after
    /// GuiApplication has subscribed to documentAdded.
    LcncDocument*        ensureMachineDocument();
    DocumentId           machineDocumentId()              const { return m_machineDocId; }
    LcncDocument*        machineDocument()                const;
    bool                 isMachineDocument(DocumentId id) const { return id == m_machineDocId; }
    /// Returns all documents except the machine workspace document.
    QList<LcncDocument*> workpieceDocuments()             const;

signals:
    void documentAdded(DocumentId id);
    void documentClosed(DocumentId id);
    void documentModified(DocumentId id);
    void activeDocumentChanged(DocumentId id);

public:
    /// Construct directly. Normally only created once by lcnc::Kernel
    /// during registerCoreServices(); ctor sets s_instance.
    explicit LcncApplication(QObject* parent = nullptr);
    ~LcncApplication() override;

private:
    static LcncApplication* s_instance;

    QList<LcncDocument*> m_documents;
    DocumentId           m_activeId     = kInvalidDocumentId;
    DocumentId           m_machineDocId = kInvalidDocumentId;
    int                  m_nextId       = 0;
};
