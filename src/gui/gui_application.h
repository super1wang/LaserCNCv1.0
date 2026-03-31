#pragma once

#include <QObject>
#include <QList>
#include <QMap>
#include "base/lcnc_application.h"

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
    static GuiApplication* instance();

    // ── GUI document access ───────────────────────────────────────────────────
    GuiDocument*         guiDocument(DocumentId id) const;
    QList<GuiDocument*>  guiDocuments() const;
    GuiDocument*         activeGuiDocument() const;    GuiDocument*         machineGuiDocument() const;
signals:
    void guiDocumentAdded(DocumentId id);
    void guiDocumentClosed(DocumentId id);
    void activeGuiDocumentChanged(DocumentId id);

private:
    explicit GuiApplication(QObject* parent = nullptr);
    static GuiApplication* s_instance;

    void onDocumentAdded(DocumentId id);
    void onDocumentClosed(DocumentId id);
    void onActiveDocumentChanged(DocumentId id);

    QMap<DocumentId, GuiDocument*> m_guiDocs;
};
