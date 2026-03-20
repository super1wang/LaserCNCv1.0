#pragma once

#include <QObject>
#include <QMap>
#include <Standard_Handle.hxx>
#include <AIS_Shape.hxx>

#include "base/lcnc_application.h"
#include "graphics/graphics_scene.h"

/**
 * @brief GUI-layer wrapper for one open document.
 *
 * Combines a LcncDocument (data) with a GraphicsScene (visual).
 * All shape display operations that need to persist across views
 * are registered here so they can be restored when a view is recreated.
 */
class GuiDocument : public QObject
{
    Q_OBJECT
public:
    explicit GuiDocument(DocumentId id, QObject* parent = nullptr);
    ~GuiDocument() override;

    DocumentId       documentId() const { return m_docId; }
    LcncDocument*    document()   const;
    GraphicsScene*   scene()      const { return m_scene; }

    // ── Shape registration ────────────────────────────────────────────────────
    /// Display a shape and register it for this document.
    Handle(AIS_Shape) displayShape(const TopoDS_Shape& shape,
                                   const QString& name,
                                   bool           fitAll = false);

    /// Erase and de-register the AIS_Shape associated with the given label entry.
    void eraseEntity(const QString& labelEntry);

    /// Erase all and re-populate from the document's XDE tree.
    void rebuildDisplay();

    /// Lookup registered AIS shape by label entry.
    Handle(AIS_Shape) aisShape(const QString& labelEntry) const;

signals:
    void displayUpdated();

private:
    DocumentId     m_docId;
    GraphicsScene* m_scene{nullptr};

    // Maps label entry strings to AIS shapes for managed refresh
    QMap<QString, Handle(AIS_Shape)> m_aisMap;
};
