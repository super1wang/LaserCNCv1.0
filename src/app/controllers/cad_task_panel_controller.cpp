#include "app/controllers/cad_task_panel_controller.h"

#include "core/logging/logger.h"
#include "modules/cad/cad_module.h"
#include "modules/cad/selection/cad_selection_resolver.h"
#include "modules/cad/ui/widget_cad_task_panel.h"
#include "view/widget_occ_view.h"

#include <QColor>
#include <QVariantMap>

#include <utility>

namespace lcnc::app {
namespace {

QString primitiveToolId(int primitiveIndex)
{
    switch (primitiveIndex) {
    case 1: return QStringLiteral("cad.primitive.cylinder");
    case 2: return QStringLiteral("cad.primitive.sphere");
    case 3: return QStringLiteral("cad.primitive.cone");
    case 4: return QStringLiteral("cad.primitive.torus");
    default: return QStringLiteral("cad.primitive.box");
    }
}

QString featureToolId(int featureIndex)
{
    switch (featureIndex) {
    case 1: return QStringLiteral("cad.feature.revolve");
    case 2: return QStringLiteral("cad.feature.sweep");
    default: return QStringLiteral("cad.feature.extrude");
    }
}

QVariantMap primitiveParams(double sizeX, double sizeY, double sizeZ, double radius1, double radius2)
{
    return {{QStringLiteral("sizeX"), sizeX}, {QStringLiteral("sizeY"), sizeY},
            {QStringLiteral("sizeZ"), sizeZ}, {QStringLiteral("radius1"), radius1},
            {QStringLiteral("radius2"), radius2}};
}

QVariantMap featureParams(double length, double angleDeg)
{
    return {{QStringLiteral("length"), length}, {QStringLiteral("angle"), angleDeg}};
}

CadModule::TransformParameters transformParams(double translateX, double translateY,
                                                double translateZ, double rotateX,
                                                double rotateY, double rotateZ,
                                                int referenceMode)
{
    CadModule::TransformParameters params;
    params.translateX = translateX;
    params.translateY = translateY;
    params.translateZ = translateZ;
    params.rotateX = rotateX;
    params.rotateY = rotateY;
    params.rotateZ = rotateZ;
    params.referenceMode = referenceMode;
    return params;
}

bool isActiveSketchOverlayKey(const QString& key)
{
    return key.startsWith(QStringLiteral("__sketch_active_element_"))
        || key.startsWith(QStringLiteral("__sketch_active_handle_"));
}

} // namespace

CadTaskPanelController::CadTaskPanelController(lcnc::cad::ui::WidgetCadTaskPanel* panel,
                                               CadModule* cad,
                                               ViewProvider viewProvider,
                                               ActivePredicate cadContextActive)
    : m_panel(panel)
    , m_cad(cad)
    , m_viewProvider(std::move(viewProvider))
    , m_cadContextActive(std::move(cadContextActive))
{
}

WidgetOccView* CadTaskPanelController::view() const
{
    return m_viewProvider ? m_viewProvider() : nullptr;
}

bool CadTaskPanelController::isCadContextActive() const
{
    return m_cadContextActive && m_cadContextActive();
}

void CadTaskPanelController::updatePrimitivePreview()
{
    WidgetOccView* occView = view();
    if (!m_panel || !m_cad || !occView || !m_panel->isPrimitivePageActive()
        || !m_panel->isPreviewEnabled())
        return;

    TopoDS_Shape previewShape;
    QString error;
    if (!m_cad->previewTool(primitiveToolId(m_panel->primitiveIndex()),
                            primitiveParams(m_panel->primitiveSizeX(), m_panel->primitiveSizeY(),
                                            m_panel->primitiveSizeZ(), m_panel->primitiveRadius1(),
                                            m_panel->primitiveRadius2()),
                            &previewShape, &error)) {
        occView->clearCadPreview();
        LCNC_DEBUG(lcnc::LogCode::Generic, "CAD primitive preview skipped: {}", error.toStdString());
        return;
    }
    occView->setCadPreviewShape(previewShape);
}

void CadTaskPanelController::updateFeaturePreview()
{
    WidgetOccView* occView = view();
    if (!m_panel || !m_cad || !occView || !m_panel->isFeaturePageActive()
        || !m_panel->isPreviewEnabled())
        return;

    TopoDS_Shape previewShape;
    QString error;
    if (!m_cad->previewTool(featureToolId(m_panel->featureIndex()),
                            featureParams(m_panel->featureLength(), m_panel->featureAngle()),
                            &previewShape, &error)) {
        occView->clearCadPreview();
        LCNC_DEBUG(lcnc::LogCode::Generic, "CAD feature preview skipped: {}", error.toStdString());
        return;
    }
    occView->setCadPreviewShape(previewShape);
}

void CadTaskPanelController::updateTransformPreview()
{
    WidgetOccView* occView = view();
    if (!m_panel || !m_cad || !occView || !m_panel->isTransformPageActive())
        return;

    TopoDS_Shape previewShape;
    QString error;
    double refX = 0.0, refY = 0.0, refZ = 0.0;
    const auto params = transformParams(m_panel->transformTranslateX(), m_panel->transformTranslateY(),
                                        m_panel->transformTranslateZ(), m_panel->transformRotateX(),
                                        m_panel->transformRotateY(), m_panel->transformRotateZ(),
                                        m_panel->transformReferenceMode());
    if (!m_cad->buildTransformPreview(params, &previewShape, &refX, &refY, &refZ, &error)) {
        occView->clearCadPreview();
        occView->clearTransformGizmo();
        LCNC_DEBUG(lcnc::LogCode::Generic, "CAD transform preview skipped: {}", error.toStdString());
        return;
    }
    occView->setTransformGizmo(refX, refY, refZ);
    if (m_panel->isPreviewEnabled())
        occView->setCadPreviewShape(previewShape);
    else
        occView->clearCadPreview();
}

void CadTaskPanelController::refreshPanelState()
{
    if (!m_panel || !m_cad)
        return;
    m_panel->setSelectionContext(m_cad->selectionContext(m_cad->workpieceDocumentId()));
    if (m_panel->isTransformPageActive())
        updateTransformPreview();
}

void CadTaskPanelController::refreshSketchElements()
{
    if (!m_panel || !m_cad)
        return;
    QVector<lcnc::cad::ui::WidgetCadTaskPanel::SketchElementEntry> entries;
    if (m_cad->isSketchEditing()) {
        for (const auto& snap : m_cad->sketchElementSnapshots())
            entries.append({snap.id, snap.kind, snap.label});
    }
    m_panel->setSketchElements(entries);
    m_panel->setActiveSketchTool(m_cad->sketchTool());
}

void CadTaskPanelController::refreshFinishedSketches()
{
    if (!m_panel || !m_cad)
        return;
    QVector<lcnc::cad::ui::WidgetCadTaskPanel::FinishedSketchEntry> entries;
    for (const auto& snap : m_cad->finishedSketchSnapshots())
        entries.append({snap.sketchId, snap.name, snap.visible, snap.usedByFeature});
    m_panel->setFinishedSketches(entries, m_cad->selectedSketchId());
}

void CadTaskPanelController::updateSketchOverlay()
{
    WidgetOccView* occView = view();
    if (!occView || !m_cad)
        return;
    if (!isCadContextActive()) {
        occView->clearSketchOverlay();
        return;
    }
    QVector<lcnc::view::SketchOverlayItem> items;
    for (const auto& snap : m_cad->sketchOverlaySnapshots()) {
        lcnc::view::SketchOverlayItem item;
        item.key = snap.key;
        item.kind = snap.kind;
        item.plane = snap.plane;
        item.params = snap.params;
        item.visible = snap.visible;
        item.draggable = snap.draggable;
        item.color = snap.selected ? QColor(255, 196, 40)
                   : snap.activeSession ? QColor(60, 220, 255)
                   : snap.usedByFeature ? QColor(120, 135, 145)
                                        : QColor(85, 210, 150);
        items.append(std::move(item));
    }
    occView->setSketchOverlayItems(items);
}

void CadTaskPanelController::handleSketchOverlayPicked(const QString& key)
{
    if (key.isEmpty() || !m_cad || !isCadContextActive())
        return;
    m_cad->setSelectionContext(lcnc::cad::selection::CadSelectionResolver::fromOverlayKey(
        m_cad->workpieceDocumentId(), key, true, m_cad->isSketchEditing()));
    updateSketchOverlay();
    refreshPanelState();
}

void CadTaskPanelController::handleSketchOverlayDrag(const QString& key, double deltaX, double deltaY)
{
    if (!m_cad || !isCadContextActive() || !isActiveSketchOverlayKey(key))
        return;
    const auto context = lcnc::cad::selection::CadSelectionResolver::fromOverlayKey(
        m_cad->workpieceDocumentId(), key, true, true);
    if (context.items.isEmpty())
        return;
    QString error;
    const auto item = context.items.first();
    const bool moved = item.sketchHandleIndex >= 0
        ? m_cad->moveSketchElementHandle(item.sketchElementId, item.sketchHandleIndex,
                                         deltaX, deltaY, &error)
        : m_cad->moveSketchElement(item.sketchElementId, deltaX, deltaY, &error);
    if (!moved)
        LCNC_WARN(lcnc::LogCode::Generic, "CAD sketch overlay move failed: {}", error.toStdString());
}

} // namespace lcnc::app
