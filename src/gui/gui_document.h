#pragma once

#include <QObject>
#include <QMap>
#include <Standard_Handle.hxx>
#include <AIS_Shape.hxx>
#include <V3d_View.hxx>
#include <AIS_InteractiveContext.hxx>
#include <Aspect_NeutralWindow.hxx>

#include "base/lcnc_application.h"
#include "graphics/graphics_scene.h"

class AIS_ViewCube;
class AIS_Trihedron;
class QTimer;

/**
 * @brief GUI-layer wrapper for one open document.
 *
 * Each GuiDocument owns its own V3d_View (created once on first activation,
 * reused on subsequent switches).  This follows the Mayo pattern: the widget
 * provides a shared OS window handle; per-document views are bound to it so
 * switching documents means simply changing which view is being redrawn.
 *
 * Gizmos (ViewCube, RGB Trihedron) are created per-document so they
 * survive document switches without needing to be re-initialised.
 */
class GuiDocument : public QObject
{
    Q_OBJECT
public:
    explicit GuiDocument(DocumentId id, QObject* parent = nullptr);
    ~GuiDocument() override;

    DocumentId     documentId() const { return m_docId; }
    LcncDocument*  document()   const;
    GraphicsScene* scene()      const { return m_scene; }

    // ── Per-document View ─────────────────────────────────────────────────────
    bool hasView() const { return !m_view.IsNull(); }
    const Handle(V3d_View)&               view()    const { return m_view;    }
    const Handle(AIS_InteractiveContext)& context() const;

    /// Called by WidgetOccView when this document becomes active.
    /// Creates the V3d_View the first time; just syncs window size afterwards.
    void attachView(const Handle(Aspect_NeutralWindow)& win, int w, int h);

    void resizeView(int w, int h);
    void fitAll();

    // Gizmo accessors used by WidgetOccView for ViewCube click handling
    const Handle(AIS_ViewCube)& viewCube()  const { return m_viewCube; }
    QTimer*                     animTimer() const { return m_animTimer; }

    // ── Shape registration ────────────────────────────────────────────────────
    Handle(AIS_Shape) displayShape(const TopoDS_Shape& shape,
                                   const QString& name,
                                   bool           fitAll = false);
    void eraseEntity(const QString& labelEntry);
    void rebuildDisplay();
    Handle(AIS_Shape) aisShape(const QString& labelEntry) const;    /// Returns label entries of all currently selected AIS shapes.
    QStringList selectedEntries() const;    // ── Axis transform update ─────────────────────────────────────────────
    /// Recomputes and applies AIS local transforms for all axis-assigned
    /// machine shapes and all mounted workpieces at their current positions.
    void updateAxisTransforms();
signals:
    void displayUpdated();

private:
    void initGizmos();   ///< Called once inside attachView() after m_view is created

    DocumentId     m_docId;
    GraphicsScene* m_scene{nullptr};
    QMap<QString, Handle(AIS_Shape)> m_aisMap;

    // Per-document view state (created once, persistent)
    Handle(V3d_View)      m_view;
    Handle(AIS_ViewCube)  m_viewCube;
    Handle(AIS_Trihedron) m_trihedron;
    QTimer*               m_animTimer{nullptr};
};
