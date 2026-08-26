#include "view/widget_occ_view.h"
#include "view/gui_document.h"
#include "view/graphics_scene.h"
#include "core/logging/logger.h"

#include <QResizeEvent>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QTimer>
#include <QtGlobal>

#include <Aspect_NeutralWindow.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <AIS_ViewCube.hxx>
#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <NCollection_Vec2.hxx>
#include <Quantity_Color.hxx>
#include <SelectMgr_EntityOwner.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

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

void WidgetOccView::resetSketchOverlayDrag()
{
    m_sketchOverlayDragging = false;
    m_pressedSketchOverlayKey.clear();
    m_sketchOverlayDragPlane = 0;
    m_sketchDragLastX = 0.0;
    m_sketchDragLastY = 0.0;
    m_sketchDragTotalX = 0.0;
    m_sketchDragTotalY = 0.0;
}

void WidgetOccView::resetTransformGizmoDrag()
{
    m_transformGizmoDragging = false;
    m_transformGizmoPressed = false;
    m_transformGizmoOperation = 0;
    m_transformGizmoAxis = 0;
    m_transformGizmoLastPos = QPoint();
}

double WidgetOccView::transformGizmoDelta(const QPoint& currentPos) const
{
    const QPoint pixelDelta = currentPos - m_transformGizmoLastPos;
    if (m_transformGizmoOperation == 1) {
        const double angleDelta = static_cast<double>(pixelDelta.x() + pixelDelta.y()) * 0.5;
        return qBound(-90.0, angleDelta, 90.0);
    }

    double axisX = 0.0;
    double axisY = 0.0;
    if (!m_transformGizmoRenderer.axisScreenVector(m_view, m_transformGizmoAxis, &axisX, &axisY))
        return 0.0;

    const double length = std::sqrt(axisX * axisX + axisY * axisY);
    if (length < 1.0e-9)
        return 0.0;
    const double unitX = axisX / length;
    const double unitY = axisY / length;
    const double projectedPixels = pixelDelta.x() * unitX + pixelDelta.y() * unitY;
    const double modelPerPixel = m_view.IsNull() ? 0.25 : qMax(0.001, m_view->Convert(1));
    return projectedPixels * modelPerPixel;
}

void WidgetOccView::eraseGridObject()
{
    if (!m_context.IsNull() && !m_gridObject.IsNull())
        m_context->Erase(m_gridObject, false);
    m_gridObject.Nullify();
}

void WidgetOccView::clearCadPreview()
{
    if (!m_context.IsNull() && !m_cadPreviewObject.IsNull())
        m_context->Erase(m_cadPreviewObject, false);
    m_cadPreviewObject.Nullify();
    if (!m_context.IsNull())
        m_context->UpdateCurrentViewer();
    if (!m_view.IsNull())
        m_view->Redraw();
}

void WidgetOccView::setTransformGizmo(double centerX, double centerY, double centerZ, double size)
{
    lcnc::view::TransformGizmoState state;
    state.centerX = centerX;
    state.centerY = centerY;
    state.centerZ = centerZ;
    state.size = qMax(1.0, size);
    state.visible = true;
    m_transformGizmoRenderer.setState(state);
    if (!m_context.IsNull()) {
        m_transformGizmoRenderer.render(m_context, true);
        if (!m_view.IsNull())
            m_view->Redraw();
    }
}

void WidgetOccView::clearTransformGizmo()
{
    resetTransformGizmoDrag();
    m_transformGizmoRenderer.clearObjects(m_context, false);
    m_transformGizmoRenderer.clear();
    if (!m_context.IsNull())
        m_context->UpdateCurrentViewer();
    if (!m_view.IsNull())
        m_view->Redraw();
}

void WidgetOccView::setSketchOverlayItems(const QVector<lcnc::view::SketchOverlayItem>& items)
{
    m_sketchOverlayRenderer.setItems(items);
    if (!m_context.IsNull()) {
        m_sketchOverlayRenderer.render(m_context, true);
        if (!m_view.IsNull())
            m_view->Redraw();
    }
}

void WidgetOccView::clearSketchOverlay()
{
    m_sketchOverlayRenderer.clearObjects(m_context, false);
    m_sketchOverlayRenderer.clear();
    if (!m_context.IsNull())
        m_context->UpdateCurrentViewer();
    if (!m_view.IsNull())
        m_view->Redraw();
}

void WidgetOccView::syncGridObject()
{
    if (m_context.IsNull())
        return;

    eraseGridObject();
    if (!m_gridVisible) {
        m_context->UpdateCurrentViewer();
        if (!m_view.IsNull())
            m_view->Redraw();
        return;
    }

    TopoDS_Compound grid;
    BRep_Builder builder;
    builder.MakeCompound(grid);

    const double step = qMax(0.001, m_gridStep);
    const int lineCount = 20;
    const double halfSize = step * lineCount;
    for (int i = -lineCount; i <= lineCount; ++i) {
        const double offset = step * i;
        BRepBuilderAPI_MakeEdge xLine(gp_Pnt(-halfSize, offset, 0.0),
                                      gp_Pnt( halfSize, offset, 0.0));
        if (xLine.IsDone()) {
            const TopoDS_Shape edgeShape = xLine.Edge();
            builder.Add(grid, edgeShape);
        }
        BRepBuilderAPI_MakeEdge yLine(gp_Pnt(offset, -halfSize, 0.0),
                                      gp_Pnt(offset,  halfSize, 0.0));
        if (yLine.IsDone()) {
            const TopoDS_Shape edgeShape = yLine.Edge();
            builder.Add(grid, edgeShape);
        }
    }

    m_gridObject = new AIS_Shape(grid);
    m_context->Display(m_gridObject, AIS_WireFrame, -1, false);
    m_context->SetColor(m_gridObject, Quantity_Color(0.40, 0.45, 0.50, Quantity_TOC_RGB), false);
    m_context->Deactivate(m_gridObject);
    m_context->UpdateCurrentViewer();
    if (!m_view.IsNull())
        m_view->Redraw();
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
    resetSketchOverlayDrag();
    resetTransformGizmoDrag();
    m_sketchOverlayRenderer.clearObjects(m_context, false);
    m_transformGizmoRenderer.clearObjects(m_context, false);
    clearCadPreview();
    eraseGridObject();
    m_view    = view;
    m_context = ctx;
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "WidgetOccView::activateView activeDoc={} viewNull={} ctxNull={}",
               static_cast<void*>(m_activeDoc),
               m_view.IsNull(),
               m_context.IsNull());
    if (!m_view.IsNull()) {
        m_occWindow->SetSize(width(), height());
        m_view->MustBeResized();
        applyCadSnapSelectionMode();
        syncGridObject();
        m_transformGizmoRenderer.render(m_context, false);
        m_sketchOverlayRenderer.render(m_context, false);
        m_view->Redraw();
    }
}

void WidgetOccView::restoreDefaultSelectionModes()
{
    applyCadSnapSelectionMode();
}

void WidgetOccView::applyCadSnapSelectionMode()
{
    if (m_context.IsNull())
        return;

    m_context->Deactivate();

    if (m_activeDoc) {
        if (!m_modelSelectionEnabled) {
            if (!m_activeDoc->viewCube().IsNull())
                m_context->Activate(m_activeDoc->viewCube(), 0, Standard_False);
            return;
        }
        int selectionMode = 0;
        switch (m_cadSnapMode) {
        case CadSnapMode::Vertex:
            selectionMode = AIS_Shape::SelectionMode(TopAbs_VERTEX);
            break;
        case CadSnapMode::Edge:
            selectionMode = AIS_Shape::SelectionMode(TopAbs_EDGE);
            break;
        case CadSnapMode::Face:
            selectionMode = AIS_Shape::SelectionMode(TopAbs_FACE);
            break;
        case CadSnapMode::None:
            break;
        }
        m_activeDoc->setEntitySelectionMode(selectionMode);
        if (!m_activeDoc->viewCube().IsNull())
            m_context->Activate(m_activeDoc->viewCube(), 0, Standard_False);
    }
}

// ── Public API ────────────────────────────────────────────────────────────────

void WidgetOccView::beginLeadInPick()
{
    if (!m_modelSelectionEnabled || m_view.IsNull() || m_context.IsNull())
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
    if (!m_modelSelectionEnabled || m_view.IsNull() || m_context.IsNull() || !m_activeDoc)
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

void WidgetOccView::setGridVisible(bool visible)
{
    if (m_gridVisible == visible)
        return;
    m_gridVisible = visible;
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "WidgetOccView::setGridVisible visible={}", visible);
    syncGridObject();
}

void WidgetOccView::setGridStep(double stepMm)
{
    const double clamped = qMax(0.001, stepMm);
    if (qFuzzyCompare(m_gridStep, clamped))
        return;
    m_gridStep = clamped;
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "WidgetOccView::setGridStep step={}", m_gridStep);
    syncGridObject();
}

void WidgetOccView::setGridSnapEnabled(bool enabled)
{
    if (m_gridSnapEnabled == enabled)
        return;
    m_gridSnapEnabled = enabled;
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "WidgetOccView::setGridSnapEnabled enabled={}", enabled);
}

void WidgetOccView::setCadSnapMode(CadSnapMode mode)
{
    const bool changed = m_cadSnapMode != mode;
    m_cadSnapMode = mode;
    if (changed) {
        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "WidgetOccView::setCadSnapMode mode={}", static_cast<int>(mode));
    }
    // QAction can be checked already when the user selects it; apply even in
    // that case so the ribbon state and the OCC selection filter cannot drift.
    applyCadSnapSelectionMode();
}

void WidgetOccView::setModelSelectionEnabled(bool enabled)
{
    if (m_modelSelectionEnabled == enabled)
        return;
    m_modelSelectionEnabled = enabled;
    if (!enabled) {
        if (m_leadInPickActive) {
            endLeadInPick();
            emit leadInPickCanceled();
        }
        if (m_facePickActive) {
            endFacePick();
            emit facePickCanceled();
        }
        clearRubberBand();
        resetSketchOverlayDrag();
        resetTransformGizmoDrag();
        if (!m_context.IsNull()) {
            m_context->ClearSelected(Standard_False);
            emit selectionChanged();
        }
    }
    applyCadSnapSelectionMode();
    if (!m_view.IsNull())
        m_view->Redraw();
}

bool WidgetOccView::screenToSketchPlane(const QPoint& pos,
                                        int planeKind,
                                        double* outX,
                                        double* outY) const
{
    if (m_view.IsNull() || !outX || !outY)
        return false;

    Standard_Real px = 0.0;
    Standard_Real py = 0.0;
    Standard_Real pz = 0.0;
    Standard_Real vx = 0.0;
    Standard_Real vy = 0.0;
    Standard_Real vz = 0.0;
    m_view->ConvertWithProj(pos.x(), pos.y(), px, py, pz, vx, vy, vz);

    double denom = 0.0;
    double t = 0.0;
    switch (planeKind) {
    case 1: // YZ: world X = 0, local = (Y, Z)
        denom = vx;
        if (std::abs(denom) < 1.0e-9)
            return false;
        t = -px / denom;
        *outX = py + vy * t;
        *outY = pz + vz * t;
        break;
    case 2: // ZX: world Y = 0, local = (Z, X)
        denom = vy;
        if (std::abs(denom) < 1.0e-9)
            return false;
        t = -py / denom;
        *outX = pz + vz * t;
        *outY = px + vx * t;
        break;
    case 0:
    default: // XY: world Z = 0, local = (X, Y)
        denom = vz;
        if (std::abs(denom) < 1.0e-9)
            return false;
        t = -pz / denom;
        *outX = px + vx * t;
        *outY = py + vy * t;
        break;
    }

    if (m_gridSnapEnabled) {
        const double step = qMax(0.001, m_gridStep);
        *outX = std::round(*outX / step) * step;
        *outY = std::round(*outY / step) * step;
    }
    return true;
}

void WidgetOccView::setCadPreviewShape(const TopoDS_Shape& shape)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "WidgetOccView::setCadPreviewShape shapeNull={}", shape.IsNull());
    if (m_context.IsNull())
        return;

    clearCadPreview();
    if (shape.IsNull())
        return;

    m_cadPreviewObject = new AIS_Shape(shape);
    m_context->Display(m_cadPreviewObject, AIS_Shaded, -1, false);
    m_context->SetColor(m_cadPreviewObject,
                        Quantity_Color(0.20, 0.72, 0.95, Quantity_TOC_RGB),
                        false);
    m_context->SetTransparency(m_cadPreviewObject, 0.55, false);
    m_context->Deactivate(m_cadPreviewObject);
    m_context->UpdateCurrentViewer();
    if (!m_view.IsNull())
        m_view->Redraw();
}

void WidgetOccView::attachDocument(GuiDocument* doc)
{
    if (!doc) return;
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "WidgetOccView::attachDocument doc={} visible={} alreadyActive={}",
               static_cast<void*>(doc), isVisible(), doc == m_activeDoc && !m_view.IsNull());
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
    scene->logOpenGlContextState("default");
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

    if (e->button() == Qt::LeftButton) {
        clearRubberBand();
        m_context->MoveTo(e->pos().x(), e->pos().y(), m_view, false);
        if (!m_modelSelectionEnabled)
            return;
        int gizmoOperation = 0;
        int gizmoAxis = 0;
        if (m_transformGizmoRenderer.detectedPart(m_context, &gizmoOperation, &gizmoAxis)) {
            m_transformGizmoPressed = true;
            m_transformGizmoDragging = false;
            m_transformGizmoOperation = gizmoOperation;
            m_transformGizmoAxis = gizmoAxis;
            m_transformGizmoLastPos = e->pos();
            return;
        }
        const QString overlayKey = m_sketchOverlayRenderer.detectedKey(m_context);
        if (!overlayKey.isEmpty() && m_sketchOverlayRenderer.isDraggable(overlayKey)) {
            m_pressedSketchOverlayKey = overlayKey;
            m_sketchOverlayDragPlane = m_sketchOverlayRenderer.planeForKey(overlayKey);
            if (!screenToSketchPlane(e->pos(), m_sketchOverlayDragPlane,
                                     &m_sketchDragLastX, &m_sketchDragLastY)) {
                resetSketchOverlayDrag();
            }
        }
    }

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
        if (m_transformGizmoPressed) {
            resetTransformGizmoDrag();
            return;
        }

        if (!m_pressedSketchOverlayKey.isEmpty()) {
            const QString key = m_pressedSketchOverlayKey;
            const bool wasDragging = m_sketchOverlayDragging;
            const double totalX = m_sketchDragTotalX;
            const double totalY = m_sketchDragTotalY;
            resetSketchOverlayDrag();
            if (wasDragging)
                emit sketchOverlayDragFinished(key, totalX, totalY);
            else if ((e->pos() - m_pressPos).manhattanLength() < 4)
                handleSelection(e->pos());
            return;
        }

        if (m_rubberBanding) {
            // Finish rubber-band: hide band, select shapes inside rectangle.
            const QRect selRect = QRect(m_pressPos, e->pos()).normalized();
            clearRubberBand();
            m_context->SelectRectangle(
                NCollection_Vec2<int>(selRect.left(), selRect.top()),
                NCollection_Vec2<int>(selRect.right(), selRect.bottom()),
                m_view);
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

    // Convert 返回鼠标屏幕位置在当前 OCC 视图投影平面上的世界坐标。
    // 无论当前处于拾取、旋转或平移状态，状态栏都应反映鼠标所在位置。
    Standard_Real x = 0.0;
    Standard_Real y = 0.0;
    Standard_Real z = 0.0;
    m_view->Convert(e->pos().x(), e->pos().y(), x, y, z);
    emit cursorPositionChanged(x, y, z);

    if (m_leadInPickActive) {
        emit leadInPickMoved(e->pos());
        m_prevPos = e->pos();
        return;
    }

    if (m_facePickActive) {
        // Face-pick mode uses a dedicated face selection mode, but it still
        // needs MoveTo() to update OCCT's dynamic highlight under the cursor.
        // Previously this branch emitted an unconnected signal and returned,
        // leaving the operator with no visual confirmation before clicking.
        m_context->MoveTo(e->pos().x(), e->pos().y(), m_view, Standard_True);
        emit facePickMoved(e->pos());
        m_view->Redraw();
        m_prevPos = e->pos();
        return;
    }

    if (!m_modelSelectionEnabled && (e->buttons() & Qt::LeftButton)) {
        // Laser-processing view keeps camera gestures and ViewCube clicks, but
        // never starts rectangle selection or any draggable model overlay.
        m_prevPos = e->pos();
        return;
    }

    if (e->buttons() & Qt::LeftButton) {
        if (m_transformGizmoPressed) {
            const QPoint delta = e->pos() - m_pressPos;
            if (!m_transformGizmoDragging && delta.manhattanLength() >= 4)
                m_transformGizmoDragging = true;

            if (m_transformGizmoDragging) {
                const double amount = transformGizmoDelta(e->pos());
                if (std::abs(amount) > 1.0e-9) {
                    emit transformGizmoDragMoved(m_transformGizmoOperation,
                                                 m_transformGizmoAxis,
                                                 amount);
                    m_transformGizmoLastPos = e->pos();
                }
            }
            m_prevPos = e->pos();
            return;
        }

        if (!m_pressedSketchOverlayKey.isEmpty()) {
            double currentX = 0.0;
            double currentY = 0.0;
            if (!screenToSketchPlane(e->pos(), m_sketchOverlayDragPlane, &currentX, &currentY)) {
                m_prevPos = e->pos();
                return;
            }

            const QPoint delta = e->pos() - m_pressPos;
            if (!m_sketchOverlayDragging && delta.manhattanLength() >= 4) {
                m_sketchOverlayDragging = true;
                emit sketchOverlayDragStarted(m_pressedSketchOverlayKey);
            }

            if (m_sketchOverlayDragging) {
                const double dx = currentX - m_sketchDragLastX;
                const double dy = currentY - m_sketchDragLastY;
                if (std::abs(dx) > 1.0e-9 || std::abs(dy) > 1.0e-9) {
                    m_sketchDragTotalX += dx;
                    m_sketchDragTotalY += dy;
                    m_sketchDragLastX = currentX;
                    m_sketchDragLastY = currentY;
                    emit sketchOverlayDragMoved(m_pressedSketchOverlayKey, dx, dy);
                }
            }
            m_prevPos = e->pos();
            return;
        }

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

    if (m_sketchOverlayDragging) {
        emit sketchOverlayDragCanceled(m_pressedSketchOverlayKey,
                                       -m_sketchDragTotalX,
                                       -m_sketchDragTotalY);
        resetSketchOverlayDrag();
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

    if (!m_modelSelectionEnabled)
        return;

    const QString overlayKey = m_sketchOverlayRenderer.detectedKey(m_context);
    if (!overlayKey.isEmpty()) {
        emit sketchOverlayPicked(overlayKey);
        m_view->Redraw();
        return;
    }

    // ShiftSelect = XOR / toggle: adds to selection if not selected,
    // removes from selection if already selected.  Nothing changes when
    // clicking on empty space — use ESC to clear all.
    m_context->SelectDetected(AIS_SelectionScheme_XOR);
    emit selectionChanged();
    m_view->Redraw();
}

// ── Public navigation helpers ─────────────────────────────────────────────────

void WidgetOccView::fitAll()
{
    if (m_view.IsNull()) return;
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "WidgetOccView::fitAll activeDoc={}",
               static_cast<void*>(m_activeDoc));
    if (m_activeDoc) {
        m_activeDoc->fitAll();
        return;
    }
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

