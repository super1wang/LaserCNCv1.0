#include "view/toolpath_renderer.h"

#include "view/gui_document.h"
#include "view/graphics_scene.h"
#include "core/algorithms/cam/laser_toolpath.h"

#include <AIS_InteractiveContext.hxx>
#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <Quantity_Color.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <V3d_View.hxx>

namespace lcnc::view {

ToolpathRenderer::ToolpathRenderer() = default;
ToolpathRenderer::~ToolpathRenderer() = default;

bool ToolpathRenderer::setNormalSampleStep(double mm)
{
    if (mm <= 0.0)
        return false;
    m_normalSampleStep = mm;
    return true;
}

void ToolpathRenderer::refresh(GuiDocument* gd,
                               const LaserToolpath& toolpath,
                               const LeadInPreview& preview)
{
    erase(gd);
    if (!gd) return;
    if (m_visible) {
        displayContours(gd, toolpath);
        displayLeadIns(gd, toolpath, preview);
        if (m_showNormals)
            displayNormals(gd, toolpath);
    }
    if (gd->hasView())
        gd->view()->Redraw();
}

void ToolpathRenderer::erase(GuiDocument* gd)
{
    if (!gd) {
        m_contourAis.clear();
        m_leadInAis.clear();
        m_normalAis.clear();
        return;
    }
    GraphicsScene* scene = gd->scene();
    if (!scene) {
        m_contourAis.clear();
        m_leadInAis.clear();
        m_normalAis.clear();
        return;
    }
    for (auto& ais : m_contourAis)
        if (!ais.IsNull()) scene->eraseShape(ais);
    for (auto& ais : m_leadInAis)
        if (!ais.IsNull()) scene->eraseShape(ais);
    for (auto& ais : m_normalAis)
        if (!ais.IsNull()) scene->eraseShape(ais);
    m_contourAis.clear();
    m_leadInAis.clear();
    m_normalAis.clear();
}

void ToolpathRenderer::setVisible(GuiDocument* gd, bool visible)
{
    if (m_visible == visible) return;
    m_visible = visible;
    if (!gd) return;
    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull()) return;

    auto toggle = [&](QList<Handle(AIS_Shape)>& list) {
        for (auto& ais : list) {
            if (ais.IsNull()) continue;
            if (visible)
                ctx->Display(ais, Standard_False);
            else
                ctx->Erase(ais, Standard_False);
        }
    };
    toggle(m_contourAis);
    toggle(m_leadInAis);
    toggle(m_normalAis);
    if (gd->hasView())
        gd->view()->Redraw();
}

void ToolpathRenderer::displayContours(GuiDocument* gd, const LaserToolpath& tp)
{
    GraphicsScene* scene = gd->scene();
    if (!scene) return;
    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    const Quantity_Color green(0.1, 0.8, 0.2, Quantity_TOC_RGB);

    for (int i = 0; i < tp.contourCount(); ++i) {
        const LaserContour& c = tp.contour(i);
        if (!c.enabled || c.wire.IsNull()) continue;
        Handle(AIS_Shape) ais = scene->displayShape(c.wire, false, true);
        scene->setShapeColor(ais, green);
        if (!ctx.IsNull()) ctx->Deactivate(ais);
        m_contourAis.append(ais);
    }
}

void ToolpathRenderer::displayLeadIns(GuiDocument* gd,
                                     const LaserToolpath& tp,
                                     const LeadInPreview& preview)
{
    GraphicsScene* scene = gd->scene();
    if (!scene) return;
    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    const Quantity_Color red(0.9, 0.15, 0.15, Quantity_TOC_RGB);
    const Quantity_Color yellow(0.95, 0.8, 0.1, Quantity_TOC_RGB);

    const double length = tp.globalLeadInLength();
    const double angle  = tp.globalNormalAngle();

    for (int i = 0; i < tp.contourCount(); ++i) {
        const LaserContour& c = tp.contour(i);
        if (!c.enabled || !c.leadIn.valid) continue;
        TopoDS_Edge leadEdge = LaserToolpathBuilder::computeLeadInEdge(c, length, angle);
        if (leadEdge.IsNull()) continue;
        Handle(AIS_Shape) ais = scene->displayShape(leadEdge, false, false);
        scene->setShapeColor(ais, red);
        ais->SetWidth(2.0);
        if (!ctx.IsNull()) ctx->Deactivate(ais);
        m_leadInAis.append(ais);
    }

    if (preview.valid
        && preview.contourIndex >= 0
        && preview.contourIndex < tp.contourCount()) {
        const LaserContour& source = tp.contour(preview.contourIndex);
        if (source.enabled) {
            LaserContour previewContour = source;
            previewContour.leadIn.entryPoint = preview.entryPoint;
            previewContour.leadIn.entryParam = preview.entryParam;
            previewContour.leadIn.valid = true;
            TopoDS_Edge previewEdge =
                LaserToolpathBuilder::computeLeadInEdge(previewContour, length, angle);
            if (!previewEdge.IsNull()) {
                Handle(AIS_Shape) ais = scene->displayShape(previewEdge, false, false);
                scene->setShapeColor(ais, yellow);
                ais->SetWidth(2.5);
                if (!ctx.IsNull()) ctx->Deactivate(ais);
                m_leadInAis.append(ais);
            }
        }
    }
}

void ToolpathRenderer::displayNormals(GuiDocument* gd, const LaserToolpath& tp)
{
    GraphicsScene* scene = gd->scene();
    if (!scene) return;
    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    const Quantity_Color amber(0.95, 0.55, 0.10, Quantity_TOC_RGB);
    constexpr double kNormalLength = 5.0;

    for (int i = 0; i < tp.contourCount(); ++i) {
        const LaserContour& contour = tp.contour(i);
        if (!contour.enabled || contour.points.empty()) continue;

        BRep_Builder builder;
        TopoDS_Compound compound;
        builder.MakeCompound(compound);
        bool hasSegments = false;
        double accum = 0.0;
        gp_Pnt prev;
        bool hasPrev = false;

        for (const ToolpathPoint& point : contour.points) {
            if (hasPrev) accum += prev.Distance(point.position);
            const bool emitNow = !hasPrev || accum >= m_normalSampleStep;
            if (emitNow) {
                const gp_Pnt endPt(
                    point.position.X() + point.normal.X() * kNormalLength,
                    point.position.Y() + point.normal.Y() * kNormalLength,
                    point.position.Z() + point.normal.Z() * kNormalLength);
                BRepBuilderAPI_MakeEdge edgeMaker(point.position, endPt);
                if (edgeMaker.IsDone()) {
                    builder.Add(compound, edgeMaker.Edge());
                    hasSegments = true;
                }
                accum = 0.0;
            }
            prev = point.position;
            hasPrev = true;
        }
        if (!hasSegments) continue;
        Handle(AIS_Shape) ais = scene->displayShape(compound, false, false);
        scene->setShapeColor(ais, amber);
        ais->SetWidth(1.5);
        if (!ctx.IsNull()) ctx->Deactivate(ais);
        m_normalAis.append(ais);
    }
}

} // namespace lcnc::view
