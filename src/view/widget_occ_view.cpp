#include "view/widget_occ_view.h"
#include "view/gui_document.h"
#include "view/graphics_scene.h"
#include "core/logging/logger.h"

#include <QResizeEvent>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QTimer>

#include <Aspect_NeutralWindow.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <AIS_ViewCube.hxx>
#include <SelectMgr_EntityOwner.hxx>
#include <TopAbs_ShapeEnum.hxx>

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

void WidgetOccView::clearRubberBand()
{
    m_rubberBanding = false;

    if (!m_context.IsNull() && !m_rubberBand.IsNull())
        m_context->Erase(m_rubberBand, false);

    m_rubberBand.Nullify();
}

void WidgetOccView::activateView(const Handle(V3d_View)& view,
                                  const Handle(AIS_InteractiveContext)& ctx)
{
    if (m_leadInPickActive) {
        endLeadInPick();
        emit leadInPickCanceled();
    }

    if (m_facePickActive) {
        endFacePick();
        emit facePickCanceled();
    }

    clearRubberBand();
    m_view    = view;
    m_context = ctx;
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "WidgetOccView::activateView activeDoc={} viewNull={} ctxNull={}",
               m_activeDoc ? m_activeDoc->documentId() : kInvalidDocumentId,
               m_view.IsNull(),
               m_context.IsNull());
    if (!m_view.IsNull()) {
        m_occWindow->SetSize(width(), height());
        m_view->MustBeResized();
        m_view->Redraw();
    }
}

void WidgetOccView::restoreDefaultSelectionModes()
{
    if (m_context.IsNull())
        return;

    m_context->Deactivate();

    if (m_activeDoc) {
        m_activeDoc->setEntitySelectionMode(0);
        if (!m_activeDoc->viewCube().IsNull())
            m_context->Activate(m_activeDoc->viewCube(), 0, Standard_False);
    }
}

// ── Public API ────────────────────────────────────────────────────────────────

void WidgetOccView::beginLeadInPick()
{
    if (m_view.IsNull() || m_context.IsNull())
        return;

    if (m_facePickActive) {
        endFacePick();
        emit facePickCanceled();
    }

    clearRubberBand();
    m_rotating = false;
    m_panning = false;
    m_leadInPickActive = true;
    setCursor(Qt::CrossCursor);
    setFocus(Qt::OtherFocusReason);
}

void WidgetOccView::endLeadInPick()
{
    if (!m_leadInPickActive)
        return;

    m_leadInPickActive = false;
    unsetCursor();
}

void WidgetOccView::beginFacePick()
{
    if (m_view.IsNull() || m_context.IsNull() || !m_activeDoc)
        return;

    clearRubberBand();
    m_rotating = false;
    m_panning = false;
    m_facePickActive = true;

    if (m_leadInPickActive) {
        endLeadInPick();
        emit leadInPickCanceled();
    }

    m_context->Deactivate();
    m_activeDoc->setEntitySelectionMode(AIS_Shape::SelectionMode(TopAbs_FACE));
    setCursor(Qt::CrossCursor);
    setFocus(Qt::OtherFocusReason);
}

void WidgetOccView::endFacePick()
{
    if (!m_facePickActive)
        return;

    m_facePickActive = false;
    restoreDefaultSelectionModes();
    unsetCursor();
}

void WidgetOccView::attachDocument(GuiDocument* doc)
{
    if (!doc) return;
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "WidgetOccView::attachDocument docId={} visible={} alreadyActive={}",
               doc->documentId(), isVisible(), doc == m_activeDoc && !m_view.IsNull());
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

    clearRubberBand();

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
    m_prevPos  = e->pos();
    m_pressPos = e->pos();

    if (m_leadInPickActive) {
        if (e->button() == Qt::LeftButton)
            clearRubberBand();
        return;
    }

    if (m_facePickActive) {
        if (e->button() == Qt::LeftButton)
            clearRubberBand();
        return;
    }

    if (e->button() == Qt::LeftButton)
        clearRubberBand();

    if (e->button() == Qt::RightButton) {
        // Right button: start view rotation
        m_rotating = true;
        m_view->StartRotation(e->pos().x(), e->pos().y());
    } else if (e->button() == Qt::MiddleButton) {
        m_panning = true;
    }
    // Left button: rubber-band drag / toggle-select are handled in
    // mouseMoveEvent / mouseReleaseEvent respectively.
}

void WidgetOccView::mouseReleaseEvent(QMouseEvent* e)
{
    if (m_view.IsNull() || m_context.IsNull()) return;

    if (m_leadInPickActive) {
        if (e->button() == Qt::RightButton) {
            emit leadInPickCanceled();
            return;
        }

        if (e->button() == Qt::LeftButton
            && (e->pos() - m_pressPos).manhattanLength() < 4) {
            emit leadInPickConfirmed(e->pos());
        }
        return;
    }

    if (m_facePickActive) {
        if (e->button() == Qt::RightButton) {
            emit facePickCanceled();
            return;
        }

        if (e->button() == Qt::LeftButton
            && (e->pos() - m_pressPos).manhattanLength() < 4) {
            emit facePickConfirmed(e->pos());
        }
        return;
    }

    if (e->button() == Qt::LeftButton) {
        if (m_rubberBanding) {
            // Finish rubber-band: hide band, select shapes inside rectangle.
            const QRect selRect = QRect(m_pressPos, e->pos()).normalized();
            clearRubberBand();
            m_context->Select(selRect.left(),  selRect.top(),
                              selRect.right(), selRect.bottom(),
                              m_view, Standard_True);
            emit selectionChanged();
            m_view->Redraw();
        } else {
            // Short click (no drag) → toggle-select under cursor.
            if ((e->pos() - m_pressPos).manhattanLength() < 4)
                handleSelection(e->pos());
        }
    } else if (e->button() == Qt::RightButton) {
        m_rotating = false;
    } else if (e->button() == Qt::MiddleButton) {
        m_panning = false;
    }
}

void WidgetOccView::mouseMoveEvent(QMouseEvent* e)
{
    if (m_view.IsNull() || m_context.IsNull()) return;

    if (m_leadInPickActive) {
        emit leadInPickMoved(e->pos());
        m_prevPos = e->pos();
        return;
    }

    if (m_facePickActive) {
        emit facePickMoved(e->pos());
        m_prevPos = e->pos();
        return;
    }

    if (e->buttons() & Qt::LeftButton) {
        const QPoint delta = e->pos() - m_pressPos;

        if (!m_rubberBanding && delta.manhattanLength() >= 4) {
            // Threshold exceeded — start rubber-band display.
            m_rubberBanding = true;
            m_rubberBand = new AIS_RubberBand(
                Quantity_Color(0.25, 0.70, 1.0, Quantity_TOC_RGB),  // border: blue
                Aspect_TOL_SOLID,
                Quantity_Color(0.25, 0.70, 1.0, Quantity_TOC_RGB),  // fill: same
                0.80,  // 80% transparent (20% opaque) fill
                1.5);  // border width

            const int h = height();
            m_rubberBand->SetRectangle(
                qMin(m_pressPos.x(), e->pos().x()),
                h - qMax(m_pressPos.y(), e->pos().y()),
                qMax(m_pressPos.x(), e->pos().x()),
                h - qMin(m_pressPos.y(), e->pos().y()));
            m_context->Display(m_rubberBand, 0, -1, false);
            m_view->Redraw();

        } else if (m_rubberBanding) {
            // Update rectangle during drag.
            if (m_rubberBand.IsNull())
                return;
            const int h = height();
            m_rubberBand->SetRectangle(
                qMin(m_pressPos.x(), e->pos().x()),
                h - qMax(m_pressPos.y(), e->pos().y()),
                qMax(m_pressPos.x(), e->pos().x()),
                h - qMin(m_pressPos.y(), e->pos().y()));
            m_context->Redisplay(m_rubberBand, false);
            m_view->Redraw();
        }

    } else if (m_rotating) {
        m_view->Rotation(e->pos().x(), e->pos().y());
        m_view->Redraw();
    } else if (m_panning) {
        const int dx = e->pos().x() - m_prevPos.x();
        const int dy = e->pos().y() - m_prevPos.y();
        m_view->Pan(dx, -dy);
        m_view->Redraw();
    } else {
        // Hover highlight (no button held)
        m_context->MoveTo(e->pos().x(), e->pos().y(), m_view, true);
        m_view->Redraw();
    }

    m_prevPos = e->pos();
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
    if (m_leadInPickActive || m_facePickActive)
        return;

    if (e->button() == Qt::LeftButton) fitAll();
}

void WidgetOccView::keyPressEvent(QKeyEvent* e)
{
    if (m_view.IsNull()) { QWidget::keyPressEvent(e); return; }

    if (m_leadInPickActive && e->key() == Qt::Key_Escape) {
        emit leadInPickCanceled();
        return;
    }

    if (m_facePickActive && e->key() == Qt::Key_Escape) {
        emit facePickCanceled();
        return;
    }

    switch (e->key()) {
    case Qt::Key_Escape:
        if (!m_context.IsNull()) {
            m_context->ClearSelected(Standard_True);
            emit selectionChanged();
            m_view->Redraw();
        }
        break;
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
        // ViewCube click — start orientation animation
        Handle(AIS_ViewCubeOwner) cubeOwner =
            Handle(AIS_ViewCubeOwner)::DownCast(owner);
        if (!cubeOwner.IsNull() && m_activeDoc && !m_activeDoc->viewCube().IsNull()) {
            m_activeDoc->viewCube()->StartAnimation(cubeOwner);
            if (m_activeDoc->animTimer())
                m_activeDoc->animTimer()->start();
            return;
        }
    }

    // ShiftSelect = XOR / toggle: adds to selection if not selected,
    // removes from selection if already selected.  Nothing changes when
    // clicking on empty space — use ESC to clear all.
    m_context->ShiftSelect(Standard_True);
    emit selectionChanged();
    m_view->Redraw();
}

// ── Public navigation helpers ─────────────────────────────────────────────────

void WidgetOccView::fitAll()
{
    if (m_view.IsNull()) return;
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "WidgetOccView::fitAll activeDoc={}",
               m_activeDoc ? m_activeDoc->documentId() : kInvalidDocumentId);
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

