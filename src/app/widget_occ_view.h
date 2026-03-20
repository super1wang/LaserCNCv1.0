#pragma once

#include <QWidget>
#include <Standard_Handle.hxx>
#include <V3d_View.hxx>
#include <AIS_InteractiveContext.hxx>
#include <V3d_TypeOfOrientation.hxx>

class GraphicsScene;
class AIS_ViewCube;

/**
 * @brief Qt widget that hosts an OpenCASCADE 3D view.
 *
 * Uses the native window handle (HWND on Windows) as the OCC rendering
 * surface.  Qt painting is completely disabled on this widget so OCC can
 * draw directly.
 *
 * Mouse interaction:
 *  - Left drag  → rotate
 *  - Middle drag → pan
 *  - Wheel      → zoom
 *  - Right click → context menu
 *  - Double-click left → fit all
 */
class WidgetOccView : public QWidget
{
    Q_OBJECT
public:
    explicit WidgetOccView(QWidget* parent = nullptr);
    ~WidgetOccView() override;

    /// Attach this view to an existing scene (must be called once after construction).
    void attachScene(GraphicsScene* scene);

    // ── View accessors ────────────────────────────────────────────────────────
    Handle(V3d_View)            view()    const { return m_view;    }
    Handle(AIS_InteractiveContext) context() const { return m_context; }
    GraphicsScene*              scene()   const { return m_scene; }

    // ── Navigation helpers ────────────────────────────────────────────────────
    void fitAll();
    void setOrientation(V3d_TypeOfOrientation orient);
    void setDisplayMode(int mode);   ///< AIS_WireFrame = 0, AIS_Shaded = 1

    // ── Override to disable Qt painting ─────────────────────────────────────
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
    void initOccView();
    void handleSelection(const QPoint& pos);
    void initOverlayGizmos();

    GraphicsScene*              m_scene{nullptr};
    Handle(V3d_View)            m_view;
    Handle(AIS_InteractiveContext) m_context;
    Handle(AIS_ViewCube)        m_viewCube;
    bool                        m_viewInitialised{false};

    // Mouse state
    QPoint m_prevPos;
    bool   m_rotating{false};
    bool   m_panning{false};
};
