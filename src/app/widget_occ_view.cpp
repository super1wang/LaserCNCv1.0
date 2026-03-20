#include "app/widget_occ_view.h"
#include "gui/gui_document.h"
#include "graphics/graphics_scene.h"

#include <QResizeEvent>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QTimer>

#include <Aspect_NeutralWindow.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>
#include <AIS_InteractiveContext.hxx>
#include <AIS_ViewCube.hxx>
#include <SelectMgr_EntityOwner.hxx>

WidgetOccView::WidgetOccView(QWidget* parent)
    : QWidget(parent)
{
    setAttribute(Qt::WA_NativeWindow,       true);
    setAttribute(Qt::WA_NoSystemBackground, true);
    setAttribute(Qt::WA_PaintOnScreen,      true);
    setAutoFillBackground(false);
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
    setMinimumSize(400, 300);
}

WidgetOccView::~WidgetOccView() = default;

// ── Internal helpers ──────────────────────────────────────────────────────────

void WidgetOccView::ensureOccWindow()
{
    if (!m_occWindow.IsNull()) return;
    m_occWindow = new Aspect_NeutralWindow();
    m_occWindow->SetNativeHandle(reinterpret_cast<Aspect_Drawable>(winId()));
    m_occWindow->SetSize(width(), height());
    if (!m_occWindow->IsMapped())
        m_occWindow->Map();
}

void WidgetOccView::activateView(const Handle(V3d_View)& view,
                                  const Handle(AIS_InteractiveContext)& ctx)
{
    m_view    = view;
    m_context = ctx;
    if (!m_view.IsNull()) {
        m_occWindow->SetSize(width(), height());
        m_view->MustBeResized();
        m_view->Redraw();
    }
}

// ── Public API ────────────────────────────────────────────────────────────────

void WidgetOccView::attachDocument(GuiDocument* doc)
{
    if (!doc) return;
    // Already showing this exact document and view?  Nothing to do.
    if (doc == m_activeDoc && !m_view.IsNull()) return;

    // Stop any in-progress ViewCube animation on the outgoing document
    if (m_activeDoc && m_activeDoc->animTimer())
        m_activeDoc->animTimer()->stop();

    m_activeDoc = doc;

    // Defer actual view setup until the widget has a valid HWND
    if (!isVisible()) return;

    ensureOccWindow();
    doc->attachView(m_occWindow, width(), height());
    activateView(doc->view(), doc->context());
}

void WidgetOccView::attachDefaultScene(GraphicsScene* scene)
{
    // Stop animation on the outgoing document
    if (m_activeDoc && m_activeDoc->animTimer())
        m_activeDoc->animTimer()->stop();

    m_activeDoc    = nullptr;
    m_defaultScene = scene;

    if (!scene) {
        m_view.Nullify();
        m_context.Nullify();
        return;
    }

    // Defer if widget is not yet shown
    if (!isVisible()) return;

    ensureOccWindow();

    if (m_defaultView.IsNull()) {
        m_defaultView = scene->viewer()->CreateView();
        m_defaultView->SetWindow(m_occWindow);
        if (!m_occWindow->IsMapped()) m_occWindow->Map();
        m_defaultView->SetBgGradientColors(
            Quantity_Color(0.30, 0.35, 0.42, Quantity_TOC_RGB),
            Quantity_Color(0.12, 0.15, 0.20, Quantity_TOC_RGB),
            Aspect_GFM_VER, false);
        m_defaultView->MustBeResized();
        m_defaultView->ZFitAll();
    }

    activateView(m_defaultView, scene->context());
}

// ── Qt event overrides ────────────────────────────────────────────────────────

void WidgetOccView::showEvent(QShowEvent* e)
{
    QWidget::showEvent(e);
    // The HWND is now valid — complete any deferred attach
    if (m_activeDoc)
        attachDocument(m_activeDoc);
    else if (m_defaultScene)
        attachDefaultScene(m_defaultScene);
}

void WidgetOccView::resizeEvent(QResizeEvent* e)
{
    QWidget::resizeEvent(e);
    if (!m_view.IsNull() && !m_occWindow.IsNull()) {
        m_occWindow->SetSize(e->size().width(), e->size().height());
        m_view->MustBeResized();
        m_view->Redraw();
    }
}

void WidgetOccView::paintEvent(QPaintEvent*)
{
    if (!m_view.IsNull()) m_view->Redraw();
}

void WidgetOccView::mousePressEvent(QMouseEvent* e)
{
    if (m_view.IsNull() || m_context.IsNull()) return;
    m_prevPos = e->pos();

    if (e->button() == Qt::LeftButton) {
        m_context->MoveTo(e->pos().x(), e->pos().y(), m_view, false);
        if (m_context->HasDetected()) {
            m_rotating = false; // clicked an interactive — don't start orbit
        } else {
            m_rotating = true;
            m_view->StartRotation(e->pos().x(), e->pos().y());
        }
    } else if (e->button() == Qt::MiddleButton) {
        m_panning = true;
    }
}

void WidgetOccView::mouseReleaseEvent(QMouseEvent* e)
{
    if (m_view.IsNull()) return;
    if (e->button() == Qt::LeftButton) {
        m_rotating = false;
        QPoint delta = e->pos() - m_prevPos;
        if (delta.manhattanLength() < 4)
            handleSelection(e->pos());
    } else if (e->button() == Qt::MiddleButton) {
        m_panning = false;
    }
}

void WidgetOccView::mouseMoveEvent(QMouseEvent* e)
{
    if (m_view.IsNull() || m_context.IsNull()) return;

    int dx = e->pos().x() - m_prevPos.x();
    int dy = e->pos().y() - m_prevPos.y();

    if (m_rotating)
        m_view->Rotation(e->pos().x(), e->pos().y());
    else if (m_panning)
        m_view->Pan(dx, -dy);
    else
        m_context->MoveTo(e->pos().x(), e->pos().y(), m_view, true);

    m_prevPos = e->pos();
    m_view->Redraw();
}

void WidgetOccView::wheelEvent(QWheelEvent* e)
{
    if (m_view.IsNull()) return;
    const double factor = (e->angleDelta().y() > 0) ? 1.1 : 0.9;
    m_view->SetZoom(factor, true);
    m_view->Redraw();
}

void WidgetOccView::mouseDoubleClickEvent(QMouseEvent* e)
{
    if (e->button() == Qt::LeftButton) fitAll();
}

void WidgetOccView::keyPressEvent(QKeyEvent* e)
{
    if (m_view.IsNull()) { QWidget::keyPressEvent(e); return; }

    switch (e->key()) {
    case Qt::Key_F: fitAll(); break;
    case Qt::Key_1: setOrientation(V3d_Xpos);          break;
    case Qt::Key_2: setOrientation(V3d_Ypos);          break;
    case Qt::Key_3: setOrientation(V3d_Zpos);          break;
    case Qt::Key_0: setOrientation(V3d_XposYnegZpos);  break;
    default: QWidget::keyPressEvent(e); return;
    }
}

// ── Selection / ViewCube click ────────────────────────────────────────────────

void WidgetOccView::handleSelection(const QPoint& pos)
{
    if (m_view.IsNull() || m_context.IsNull()) return;

    m_context->MoveTo(pos.x(), pos.y(), m_view, false);

    Handle(SelectMgr_EntityOwner) owner = m_context->DetectedOwner();
    if (!owner.IsNull()) {
        Handle(AIS_ViewCubeOwner) cubeOwner =
            Handle(AIS_ViewCubeOwner)::DownCast(owner);
        if (!cubeOwner.IsNull() && m_activeDoc && !m_activeDoc->viewCube().IsNull()) {
            // Call StartAnimation() directly — HandleClick() may be guarded by
            // an internal flag (myToAutoStartAnim) and silently return early.
            m_activeDoc->viewCube()->StartAnimation(cubeOwner);
            if (m_activeDoc->animTimer())
                m_activeDoc->animTimer()->start();
            return;
        }
    }

    m_context->Select(true);
    emit selectionChanged();
    m_view->Redraw();
}

// ── Public navigation helpers ─────────────────────────────────────────────────

void WidgetOccView::fitAll()
{
    if (m_view.IsNull()) return;
    m_view->FitAll(0.01, true);
    m_view->ZFitAll();
    m_view->Redraw();
}

void WidgetOccView::setOrientation(V3d_TypeOfOrientation orient)
{
    if (m_view.IsNull()) return;
    m_view->SetProj(orient);
    fitAll();
}

void WidgetOccView::setDisplayMode(int mode)
{
    if (!m_context.IsNull())
        m_context->SetDisplayMode(mode, true);
}

