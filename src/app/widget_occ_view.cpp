#include "app/widget_occ_view.h"
#include "graphics/graphics_scene.h"

#include <QResizeEvent>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>

// OCC view and window includes
#include <Aspect_NeutralWindow.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <AIS_ViewCube.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <Graphic3d_TransformPers.hxx>
#include <Aspect_TypeOfTriedronPosition.hxx>

WidgetOccView::WidgetOccView(QWidget* parent)
    : QWidget(parent)
{
    // Tell Qt not to paint over us; OCC handles all drawing.
    setAttribute(Qt::WA_NativeWindow,       true);
    setAttribute(Qt::WA_NoSystemBackground, true);
    setAttribute(Qt::WA_PaintOnScreen,      true);
    setAutoFillBackground(false);

    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
    setMinimumSize(400, 300);
}

WidgetOccView::~WidgetOccView() = default;

void WidgetOccView::attachScene(GraphicsScene* scene)
{
    if (scene == m_scene)
        return;

    // When switching document scenes, recreate view/context pair.
    if (m_viewInitialised) {
        m_viewInitialised = false;
        m_view.Nullify();
        m_viewCube.Nullify();
    }

    m_scene   = scene;
    m_context = scene ? scene->context() : Handle(AIS_InteractiveContext)();

    if (scene && isVisible())
        initOccView();
}

void WidgetOccView::showEvent(QShowEvent* e)
{
    QWidget::showEvent(e);
    if (m_scene && !m_viewInitialised)
        initOccView();
}

void WidgetOccView::initOccView()
{
    if (!m_scene || m_viewInitialised) return;

    // Create the OCC window backed by this widget's native HWND
    Handle(Aspect_NeutralWindow) occWin = new Aspect_NeutralWindow();
    occWin->SetNativeHandle(reinterpret_cast<Aspect_Drawable>(winId()));
    occWin->SetSize(width(), height());

    m_view = m_scene->viewer()->CreateView();
    m_view->SetWindow(occWin);
    if (!occWin->IsMapped())
        occWin->Map();

    m_view->SetBgGradientColors(
        Quantity_Color(0.30, 0.35, 0.42, Quantity_TOC_RGB),
        Quantity_Color(0.12, 0.15, 0.20, Quantity_TOC_RGB),
        Aspect_GFM_VER, false);

    m_view->MustBeResized();
    m_view->ZFitAll();
    initOverlayGizmos();
    m_view->Redraw();

    m_viewInitialised = true;
}

void WidgetOccView::initOverlayGizmos()
{
    if (m_view.IsNull() || m_context.IsNull())
        return;

    // Left-bottom coordinate axes indicator (native triedron)
    m_view->TriedronDisplay(Aspect_TOTP_LEFT_LOWER,
                            Quantity_Color(0.85, 0.92, 1.00, Quantity_TOC_RGB),
                            0.08,
                            V3d_ZBUFFER);

    // Right-top view cube (silver gray color)
    m_viewCube = new AIS_ViewCube();
    m_viewCube->SetSize(70.0);
    // Silver gray color: RGB(192, 192, 192) normalized to 0-1 range
    m_viewCube->SetBoxColor(Quantity_Color(192.0/255.0, 192.0/255.0, 192.0/255.0, Quantity_TOC_RGB));
    m_viewCube->SetTextColor(Quantity_Color(0.2, 0.2, 0.2, Quantity_TOC_RGB));
    m_viewCube->SetTransparency(0.05);
    Handle(Graphic3d_TransformPers) pers =
        new Graphic3d_TransformPers(Graphic3d_TMF_TriedronPers,
                                    Aspect_TOTP_RIGHT_UPPER,
                                    Graphic3d_Vec2i(110, 110));
    m_viewCube->SetTransformPersistence(pers);
    m_context->Display(m_viewCube, false);
}

// ── Qt event overrides ─────────────────────────────────────────────────────────

void WidgetOccView::resizeEvent(QResizeEvent* e)
{
    QWidget::resizeEvent(e);
    if (m_viewInitialised) {
        Handle(Aspect_NeutralWindow)::DownCast(m_view->Window())
            ->SetSize(e->size().width(), e->size().height());
        m_view->MustBeResized();
        m_view->Redraw();
    }
}

void WidgetOccView::paintEvent(QPaintEvent*)
{
    if (m_viewInitialised)
        m_view->Redraw();
}

void WidgetOccView::mousePressEvent(QMouseEvent* e)
{
    if (!m_viewInitialised || m_view.IsNull()) return;
    m_prevPos = e->pos();

    if (e->button() == Qt::LeftButton) {
        m_rotating = true;
        m_view->StartRotation(e->pos().x(), e->pos().y());
    }
    else if (e->button() == Qt::MiddleButton) {
        m_panning = true;
    }
}

void WidgetOccView::mouseReleaseEvent(QMouseEvent* e)
{
    if (!m_viewInitialised || m_view.IsNull()) return;
    if (e->button() == Qt::LeftButton) {
        m_rotating = false;
        // Selection on click (not drag)
        QPoint delta = e->pos() - m_prevPos;
        if (delta.manhattanLength() < 4)
            handleSelection(e->pos());
    }
    else if (e->button() == Qt::MiddleButton) {
        m_panning = false;
    }
}

void WidgetOccView::mouseMoveEvent(QMouseEvent* e)
{
    if (!m_viewInitialised) return;

    int dx = e->pos().x() - m_prevPos.x();
    int dy = e->pos().y() - m_prevPos.y();

    if (m_rotating) {
        m_view->Rotation(e->pos().x(), e->pos().y());
    }
    else if (m_panning) {
        m_view->Pan(dx, -dy);
    }
    else {
        // Dynamic highlight
        m_context->MoveTo(e->pos().x(), e->pos().y(), m_view, true);
    }

    m_prevPos = e->pos();
    m_view->Redraw();
}

void WidgetOccView::wheelEvent(QWheelEvent* e)
{
    if (!m_viewInitialised) return;
    const double factor = (e->angleDelta().y() > 0) ? 1.1 : 0.9;
    m_view->SetZoom(factor, true);
    m_view->Redraw();
}

void WidgetOccView::mouseDoubleClickEvent(QMouseEvent* e)
{
    if (e->button() == Qt::LeftButton)
        fitAll();
}

void WidgetOccView::keyPressEvent(QKeyEvent* e)
{
    if (!m_viewInitialised) { QWidget::keyPressEvent(e); return; }

    switch (e->key()) {
    case Qt::Key_F:       fitAll(); break;
    case Qt::Key_1:       setOrientation(V3d_Xpos); break;
    case Qt::Key_2:       setOrientation(V3d_Ypos); break;
    case Qt::Key_3:       setOrientation(V3d_Zpos); break;
    case Qt::Key_0:       setOrientation(V3d_XposYnegZpos); break;
    default: QWidget::keyPressEvent(e); return;
    }
}

void WidgetOccView::handleSelection(const QPoint& pos)
{
    if (!m_viewInitialised) return;
    m_context->Select(true);
    emit selectionChanged();
    m_view->Redraw();
}

// ── Public helpers ─────────────────────────────────────────────────────────────
void WidgetOccView::fitAll()
{
    if (!m_viewInitialised) return;
    m_view->FitAll(0.01, true);
    m_view->ZFitAll();
    m_view->Redraw();
}

void WidgetOccView::setOrientation(V3d_TypeOfOrientation orient)
{
    if (!m_viewInitialised) return;
    m_view->SetProj(orient);
    fitAll();
}

void WidgetOccView::setDisplayMode(int mode)
{
    if (!m_context.IsNull())
        m_context->SetDisplayMode(mode, true);
}
