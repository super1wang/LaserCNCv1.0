#pragma once

#include <QObject>
#include <QList>
#include <QMap>
#include "core/document/lcnc_application.h"

class GuiDocument;
class GraphicsScene;

/**
 * @brief GUI-layer application manager.
 *
 * Mirrors LcncApplication's document list but attaches a GuiDocument (which
 * owns a GraphicsScene) to every open document.  All 3D scene operations
 * should go through the GuiDocument, not directly to the OCC viewer.
 */
class GuiApplication : public QObject
{
    Q_OBJECT
public:

    // ── GUI document access ───────────────────────────────────────────────────
    GuiDocument*         guiDocument(DocumentId id) const;
    QList<GuiDocument*>  guiDocuments() const;
    GuiDocument*         activeGuiDocument() const;    GuiDocument*         machineGuiDocument() const;
signals:
    void guiDocumentAdded(DocumentId id);
    void guiDocumentClosed(DocumentId id);
    void activeGuiDocumentChanged(DocumentId id);

public:
    /// Constructed once by lcnc::Kernel during registerCoreServices, after
    /// LcncApplication is in place. Ctor wires LcncApplication signals.
    explicit GuiApplication(QObject* parent = nullptr);
    ~GuiApplication() override;

private:
    static GuiApplication* s_instance;

    void onDocumentAdded(DocumentId id);
    void onDocumentClosed(DocumentId id);
    void onActiveDocumentChanged(DocumentId id);

    QMap<DocumentId, GuiDocument*> m_guiDocs;
};
