#pragma once

#include <QWidget>
#include <Standard_Handle.hxx>
#include <V3d_View.hxx>
#include <AIS_InteractiveContext.hxx>
#include <Aspect_NeutralWindow.hxx>
#include <V3d_TypeOfOrientation.hxx>

class GuiDocument;
class GraphicsScene;

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
 *  - Left drag   → rotate
 *  - Middle drag → pan
 *  - Wheel       → zoom
 *  - Double-left → fit all
 */
class WidgetOccView : public QWidget
{
    Q_OBJECT
public:
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

    QPaintEngine* paintEngine() const override { return nullptr; }

signals:
    void selectionChanged();

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

    /// Switch the active view/context pair and trigger a redraw.
    void activateView(const Handle(V3d_View)& view,
                      const Handle(AIS_InteractiveContext)& ctx);

    void handleSelection(const QPoint& pos);

    // ── State ────────────────────────────────────────────────────────────────
    Handle(Aspect_NeutralWindow)   m_occWindow;    ///< OS window, created once
    Handle(V3d_View)               m_view;         ///< currently rendered view
    Handle(AIS_InteractiveContext) m_context;      ///< currently active context

    GuiDocument*  m_activeDoc{nullptr};            ///< document being shown (may be null)
    GraphicsScene* m_defaultScene{nullptr};        ///< fallback scene (no open docs)
    Handle(V3d_View) m_defaultView;                ///< view for the default scene

    QPoint m_prevPos;
    bool   m_rotating{false};
    bool   m_panning{false};
};
