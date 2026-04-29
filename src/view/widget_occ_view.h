#pragma once

#include <QWidget>
#include <Standard_Handle.hxx>
#include <V3d_View.hxx>
#include <AIS_InteractiveContext.hxx>
#include <Aspect_NeutralWindow.hxx>
#include <V3d_TypeOfOrientation.hxx>
#include <AIS_RubberBand.hxx>

#include "view/sketch_overlay_renderer.h"
#include "view/transform_gizmo_renderer.h"

class AIS_Shape;
class GuiDocument;
class GraphicsScene;
class TopoDS_Shape;

/**
 * @brief Qt widget that hosts an OpenCASCADE 3D view.
 *
 * Implements the Mayo-style per-document view pattern:
 *  - One Aspect_NeutralWindow (OS window handle) is created once for this widget.
 *  - Each GuiDocument owns its own V3d_View bound to this window.
 *  - Switching documents means swapping which V3d_View is rendered —
 *    no view teardown/recreation, so camera state is preserved per document.
 *
 * Mouse interaction:
 *  - Left click      → toggle-select shape (additive; click again to deselect)
 *  - Left drag       → rubber-band multi-select
 *  - Right drag      → rotate view
 *  - Middle drag     → pan
 *  - Wheel           → zoom
 *  - Double-left     → fit all
 *  - ESC             → clear all selections
 */
class WidgetOccView : public QWidget
{
    Q_OBJECT
public:
    /**
     * @brief Pick filtering mode used by CAD modeling tools.
     */
    enum class CadSnapMode {
        None,
        Vertex,
        Edge,
        Face
    };

    explicit WidgetOccView(QWidget* parent = nullptr);
    ~WidgetOccView() override;

    /// Activate a GuiDocument.  Creates its V3d_View on first call; just
    /// swaps the active view on subsequent activations.
    void attachDocument(GuiDocument* doc);

    /// Show the default (empty-document) scene.
    void attachDefaultScene(GraphicsScene* scene);

    // ── View accessors ───────────────────────────────────────────────────────
    Handle(V3d_View)               view()      const { return m_view;      }
    Handle(AIS_InteractiveContext) context()   const { return m_context;   }
    GuiDocument*                   activeDoc() const { return m_activeDoc; }

    // ── Navigation helpers ───────────────────────────────────────────────────
    void fitAll();
    void setOrientation(V3d_TypeOfOrientation orient);
    void setDisplayMode(int mode); ///< AIS_WireFrame = 0, AIS_Shaded = 1
    void beginLeadInPick();
    void endLeadInPick();
    void beginFacePick();
    void endFacePick();
    void setGridVisible(bool visible);
    void setGridStep(double stepMm);
    void setGridSnapEnabled(bool enabled);
    void setCadSnapMode(CadSnapMode mode);
    void setCadPreviewShape(const TopoDS_Shape& shape);
    void clearCadPreview();
    /// Show a selectable CAD transform gizmo at a world-space reference point.
    void setTransformGizmo(double centerX, double centerY, double centerZ, double size = 60.0);
    /// Clear the CAD transform gizmo from the view.
    void clearTransformGizmo();
    /// Replace the active CAD sketch overlay displayed in the OCC view.
    void setSketchOverlayItems(const QVector<lcnc::view::SketchOverlayItem>& items);
    /// Clear all CAD sketch overlay objects from the OCC view.
    void clearSketchOverlay();
    bool isLeadInPickActive() const { return m_leadInPickActive; }
    bool isFacePickActive() const { return m_facePickActive; }
    bool isGridVisible() const { return m_gridVisible; }
    bool isGridSnapEnabled() const { return m_gridSnapEnabled; }
    double gridStep() const { return m_gridStep; }
    CadSnapMode cadSnapMode() const { return m_cadSnapMode; }

    QPaintEngine* paintEngine() const override { return nullptr; }

signals:
    void selectionChanged();
    void leadInPickMoved(const QPoint& pos);
    void leadInPickConfirmed(const QPoint& pos);
    void leadInPickCanceled();
    void facePickMoved(const QPoint& pos);
    void facePickConfirmed(const QPoint& pos);
    void facePickCanceled();
    void sketchOverlayPicked(const QString& key);
    void sketchOverlayDragStarted(const QString& key);
    void sketchOverlayDragMoved(const QString& key, double deltaX, double deltaY);
    void sketchOverlayDragFinished(const QString& key, double deltaX, double deltaY);
    void sketchOverlayDragCanceled(const QString& key, double rollbackX, double rollbackY);
    void transformGizmoDragMoved(int operation, int axis, double delta);

protected:
    void resizeEvent(QResizeEvent*) override;
    void paintEvent(QPaintEvent*)   override;
    void mousePressEvent(QMouseEvent*)   override;
    void mouseReleaseEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*)    override;
    void wheelEvent(QWheelEvent*)        override;
    void mouseDoubleClickEvent(QMouseEvent*) override;
    void keyPressEvent(QKeyEvent*)       override;
    void showEvent(QShowEvent*)          override;

private:
    /// Create m_occWindow the first time (requires valid HWND — call only when shown).
    void ensureOccWindow();

    /// Remove any in-progress rubber-band overlay from the current context.
    void clearRubberBand();

    /// Switch the active view/context pair and trigger a redraw.
    void activateView(const Handle(V3d_View)& view,
                      const Handle(AIS_InteractiveContext)& ctx);

    void restoreDefaultSelectionModes();

    void handleSelection(const QPoint& pos);
    void eraseGridObject();
    void syncGridObject();
    bool screenToSketchPlane(const QPoint& pos, int planeKind, double* outX, double* outY) const;
    void resetSketchOverlayDrag();
    void resetTransformGizmoDrag();
    double transformGizmoDelta(const QPoint& currentPos) const;

    // ── State ────────────────────────────────────────────────────────────────
    Handle(Aspect_NeutralWindow)   m_occWindow;    ///< OS window, created once
    Handle(V3d_View)               m_view;         ///< currently rendered view
    Handle(AIS_InteractiveContext) m_context;      ///< currently active context

    GuiDocument*  m_activeDoc{nullptr};            ///< document being shown (may be null)
    GraphicsScene* m_defaultScene{nullptr};        ///< fallback scene (no open docs)
    Handle(V3d_View) m_defaultView;                ///< view for the default scene

    QPoint m_prevPos;          ///< last cursor position (used for pan delta)
    QPoint m_pressPos;          ///< where the left button was pressed
    bool   m_rotating{false};
    bool   m_panning{false};
    bool   m_rubberBanding{false};
    bool   m_sketchOverlayDragging{false};
    bool   m_transformGizmoDragging{false};
    bool   m_leadInPickActive{false};
    bool   m_facePickActive{false};
    bool   m_gridVisible{false};
    bool   m_gridSnapEnabled{false};
    double m_gridStep{10.0};
    CadSnapMode m_cadSnapMode{CadSnapMode::Face};
    Handle(AIS_RubberBand) m_rubberBand;  ///< created lazily; displayed only during drag
    Handle(AIS_Shape) m_gridObject;        ///< visual CAD construction grid for the active context
    Handle(AIS_Shape) m_cadPreviewObject;  ///< transient CAD feature preview, never committed to document
    lcnc::view::TransformGizmoRenderer m_transformGizmoRenderer;
    lcnc::view::SketchOverlayRenderer m_sketchOverlayRenderer;
    QPoint m_transformGizmoLastPos;
    bool m_transformGizmoPressed{false};
    int m_transformGizmoOperation{0};
    int m_transformGizmoAxis{0};
    QString m_pressedSketchOverlayKey;
    int m_sketchOverlayDragPlane{0};
    double m_sketchDragLastX{0.0};
    double m_sketchDragLastY{0.0};
    double m_sketchDragTotalX{0.0};
    double m_sketchDragTotalY{0.0};
};
