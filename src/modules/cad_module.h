#pragma once

#include <QObject>
#include <QList>
#include <QStringList>
#include <TDF_Label.hxx>
#include <TopoDS_Shape.hxx>

#include "base/lcnc_application.h"
#include "base/lcnc_document.h"

class GuiDocument;
class GuiApplication;
class TaskManager;
class gp_Vec;
class gp_Ax1;

/**
 * @brief CAD module singleton — manages documents, file I/O, and modeling operations.
 *
 * Responsible for:
 *  - Document lifecycle (new / open / save / close / import / export)
 *  - Managing the "文档" (Document) tab page
 *  - Providing modeling operations (create / move / rotate / delete / boolean / explode)
 *    via ShapeService delegation
 *  - Undo/Redo
 *
 * Ribbon commands should call CadModule APIs instead of implementing business logic directly.
 */
class CadModule : public QObject
{
    Q_OBJECT
public:
    static CadModule* instance();

    // ── Document Management (delegates to LcncApplication) ───────────────
    DocumentId  newDocument(const QString& name = QString());
    DocumentId  openDocument(const QString& filePath);
    DocumentId  importStep(const QString& filePath,
                           DocumentId targetDocId = kInvalidDocumentId);
    DocumentId  importStl(const QString& filePath,
                          DocumentId targetDocId = kInvalidDocumentId);
    bool        saveDocument(DocumentId id, const QString& path = QString());
    void        exportStep(DocumentId id, const QString& filePath);
    void        closeDocument(DocumentId id);
    DocumentId  importFile(const QString& filePath);

    // ── Active Document ──────────────────────────────────────────────────
    DocumentId    activeDocumentId() const;
    LcncDocument* activeDocument() const;
    GuiDocument*  activeGuiDocument() const;
    LcncDocument* documentById(DocumentId id) const;
    GuiDocument*  guiDocument(DocumentId id) const;
    QList<LcncDocument*> workpieceDocuments() const;
    void          setActiveDocument(DocumentId id);

    // ── Selection / Visibility ──────────────────────────────────────────
    void setEntityVisible(DocumentId docId, const QString& entry, bool visible);
    void setSelectedEntries(DocumentId docId, const QStringList& entries);
    QStringList selectedEntries(DocumentId docId) const;
    void syncSelectionFromView(DocumentId docId = kInvalidDocumentId);

    // ── Modeling Operations (delegates to ShapeService + refreshes display) ──
    /// Move a shape by translation vector. Returns true on success.
    bool moveShape(DocumentId docId, const TDF_Label& label, const gp_Vec& translation);
    bool moveShapes(DocumentId docId, const QList<TDF_Label>& labels,
                    const gp_Vec& translation);

    /// Rotate a shape around an axis by angleDeg degrees. Returns true on success.
    bool rotateShape(DocumentId docId, const TDF_Label& label,
                     const gp_Ax1& axis, double angleDeg);
    bool rotateShapes(DocumentId docId, const QList<TDF_Label>& labels,
                      const gp_Ax1& axis, double angleDeg);

    /// Delete a shape from a document by its label entry string.
    bool deleteShape(DocumentId docId, const QString& entry);
    bool deleteShapes(DocumentId docId, const QStringList& entries);

    /// Explode a compound into sub-shapes. Returns child count.
    int  explodeShape(DocumentId docId, const TDF_Label& label, int entityKind);

    /// Add a new shape to the document. Returns the new label.
    TDF_Label createShape(DocumentId docId, const TopoDS_Shape& shape,
                          const QString& name,
                          int entityKind = static_cast<int>(LcncDocument::EntityKind::Workpiece));

    // ── Undo / Redo ─────────────────────────────────────────────────────
    bool canUndo(DocumentId docId) const;
    bool canRedo(DocumentId docId) const;
    void undo(DocumentId docId);
    void redo(DocumentId docId);

signals:
    /// Emitted whenever the document list changes (add/remove).
    void documentListChanged();
    /// Emitted when the active document switches.
    void activeDocumentChanged(DocumentId id);
    /// Emitted when a document's content is modified.
    void documentModified(DocumentId id);
    /// Emitted when a module-level operation fails and should be surfaced by the UI.
    void operationFailed(const QString& title, const QString& message);
    /// Emitted when a workpiece document selection changes via module coordination.
    void selectionChanged(DocumentId id, const QStringList& entries);

private:
    explicit CadModule(QObject* parent = nullptr);

    void refreshDisplay(DocumentId docId);

    static CadModule* s_instance;
};
