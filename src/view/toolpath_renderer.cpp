#include "view/toolpath_renderer.h"

#include "view/gui_document.h"
#include "core/algorithms/cam/laser_toolpath.h"
#include "core/kinematics/machine_kinematics.h"

#include <AIS_InteractiveContext.hxx>
#include <AIS_InteractiveObject.hxx>
#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <Quantity_Color.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <V3d_View.hxx>

namespace lcnc::view {

namespace {

constexpr double kNormalLength = 5.0;

void applyLocalTransform(const Handle(AIS_InteractiveContext)& ctx,
                         const Handle(AIS_Shape)& ais,
                         const gp_Trsf& transform)
{
    if (ais.IsNull())
        return;

    ais->SetLocalTransformation(transform);
    if (!ctx.IsNull())
        ctx->RecomputePrsOnly(ais, Standard_False);
}

} // namespace

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
                               MachineKinematics* kin,
                               const LeadInPreview& preview)
{
    clearAis(gd, false);
    if (!gd) return;

    ensureBundleCount(gd, toolpath.contourCount());
    if (m_visible) {
        for (int i = 0; i < toolpath.contourCount(); ++i)
            rebuildLeadInAis(gd, toolpath, i);
        if (m_showNormals) {
            for (int i = 0; i < toolpath.contourCount(); ++i)
                rebuildNormalAis(gd, toolpath, i);
        }
        rebuildPreviewAis(gd, toolpath, preview);
    }
    updateTransforms(gd, toolpath, kin);
    rebuildContourMirror();
    redraw(gd);
}

void ToolpathRenderer::refreshLeadIns(GuiDocument* gd,
                                      const LaserToolpath& toolpath,
                                      MachineKinematics* kin,
                                      const LeadInPreview& preview)
{
    if (!gd) return;
    ensureBundleCount(gd, toolpath.contourCount());
    for (ContourAisBundle& bundle : m_bundles)
        eraseAis(gd, bundle.leadIn);
    eraseAis(gd, m_previewLeadInAis);
    m_previewContourIndex = -1;

    if (m_visible) {
        for (int i = 0; i < toolpath.contourCount(); ++i)
            rebuildLeadInAis(gd, toolpath, i);
        rebuildPreviewAis(gd, toolpath, preview);
    }
    updateTransforms(gd, toolpath, kin);
    redraw(gd);
}

void ToolpathRenderer::refreshNormals(GuiDocument* gd,
                                      const LaserToolpath& toolpath,
                                      MachineKinematics* kin)
{
    if (!gd) return;
    ensureBundleCount(gd, toolpath.contourCount());
    for (ContourAisBundle& bundle : m_bundles)
        eraseAis(gd, bundle.normal);

    if (m_visible && m_showNormals) {
        for (int i = 0; i < toolpath.contourCount(); ++i)
            rebuildNormalAis(gd, toolpath, i);
    }
    updateTransforms(gd, toolpath, kin);
    redraw(gd);
}

void ToolpathRenderer::refreshContour(GuiDocument* gd,
                                      const LaserToolpath& toolpath,
                                      MachineKinematics* kin,
                                      int contourIndex,
                                      const LeadInPreview& preview)
{
    if (!gd || contourIndex < 0 || contourIndex >= toolpath.contourCount())
        return;

    ensureBundleCount(gd, toolpath.contourCount());
    ContourAisBundle& bundle = m_bundles[contourIndex];
    eraseBundle(gd, bundle);

    if (m_visible) {
        rebuildLeadInAis(gd, toolpath, contourIndex);
        if (m_showNormals)
            rebuildNormalAis(gd, toolpath, contourIndex);
    }

    if (preview.valid && preview.contourIndex == contourIndex) {
        eraseAis(gd, m_previewLeadInAis);
        m_previewContourIndex = -1;
        if (m_visible)
            rebuildPreviewAis(gd, toolpath, preview);
    }

    updateTransforms(gd, toolpath, kin);
    rebuildContourMirror();
    redraw(gd);
}

void ToolpathRenderer::updateTransforms(GuiDocument* gd,
                                        const LaserToolpath& toolpath,
                                        MachineKinematics* kin)
{
    if (!gd)
        return;

    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull())
        return;

    const int count = qMin(m_bundles.size(), toolpath.contourCount());
    for (int i = 0; i < count; ++i) {
        gp_Trsf transform;
        if (kin && !toolpath.contour(i).workpieceEntry.isEmpty())
            transform = kin->computeWpcTransform(toolpath.contour(i).workpieceEntry);

        ContourAisBundle& bundle = m_bundles[i];
        applyLocalTransform(ctx, bundle.leadIn, transform);
        applyLocalTransform(ctx, bundle.normal, transform);
    }

    if (!m_previewLeadInAis.IsNull()
        && m_previewContourIndex >= 0
        && m_previewContourIndex < toolpath.contourCount()) {
        gp_Trsf transform;
        if (kin && !toolpath.contour(m_previewContourIndex).workpieceEntry.isEmpty())
            transform = kin->computeWpcTransform(toolpath.contour(m_previewContourIndex).workpieceEntry);
        applyLocalTransform(ctx, m_previewLeadInAis, transform);
    }
}

int ToolpathRenderer::contourIndexForAis(const Handle(AIS_InteractiveObject)& object) const
{
    if (object.IsNull())
        return -1;
    Q_UNUSED(object);
    return -1;
}

QList<int> ToolpathRenderer::selectedContourIndexes(GuiDocument* gd) const
{
    QList<int> indexes;
    if (!gd)
        return indexes;

    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull())
        return indexes;

    for (ctx->InitSelected(); ctx->MoreSelected(); ctx->NextSelected()) {
        const int index = contourIndexForAis(ctx->SelectedInteractive());
        if (index >= 0 && !indexes.contains(index))
            indexes.append(index);
    }

    return indexes;
}

int ToolpathRenderer::selectedContourIndex(GuiDocument* gd) const
{
    const QList<int> indexes = selectedContourIndexes(gd);
    return indexes.isEmpty() ? -1 : indexes.last();
}

void ToolpathRenderer::erase(GuiDocument* gd)
{
    clearAis(gd, true);
}

void ToolpathRenderer::setVisible(GuiDocument* gd, bool visible)
{
    if (m_visible == visible) return;
    m_visible = visible;
    if (!gd) return;
    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull()) return;

    auto toggle = [&](const Handle(AIS_Shape)& ais) {
        if (ais.IsNull()) return;
        if (visible)
            ctx->Display(ais, Standard_False);
        else
            ctx->Erase(ais, Standard_False);
    };

    for (const ContourAisBundle& bundle : m_bundles) {
        toggle(bundle.leadIn);
        toggle(bundle.normal);
    }
    toggle(m_previewLeadInAis);
    redraw(gd);
}

void ToolpathRenderer::ensureBundleCount(GuiDocument* gd, int count)
{
    while (m_bundles.size() > count) {
        ContourAisBundle bundle = m_bundles.takeLast();
        eraseBundle(gd, bundle);
    }
    while (m_bundles.size() < count)
        m_bundles.append(ContourAisBundle{});
    rebuildContourMirror();
}

void ToolpathRenderer::clearAis(GuiDocument* gd, bool updateView)
{
    for (ContourAisBundle& bundle : m_bundles)
        eraseBundle(gd, bundle);
    eraseAis(gd, m_previewLeadInAis);
    m_previewContourIndex = -1;
    m_bundles.clear();
    m_contourAis.clear();
    if (updateView)
        redraw(gd);
}

void ToolpathRenderer::eraseAis(GuiDocument* gd, Handle(AIS_Shape)& ais)
{
    if (ais.IsNull())
        return;

    if (gd) {
        const Handle(AIS_InteractiveContext)& ctx = gd->context();
        if (!ctx.IsNull())
            ctx->Erase(ais, Standard_False);
    }
    ais.Nullify();
}

void ToolpathRenderer::eraseBundle(GuiDocument* gd, ContourAisBundle& bundle)
{
    eraseAis(gd, bundle.leadIn);
    eraseAis(gd, bundle.normal);
}

void ToolpathRenderer::rebuildLeadInAis(GuiDocument* gd,
                                        const LaserToolpath& tp,
                                        int contourIndex)
{
    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull()) return;
    if (contourIndex < 0 || contourIndex >= tp.contourCount()) return;
    if (contourIndex >= m_bundles.size()) return;

    ContourAisBundle& bundle = m_bundles[contourIndex];
    eraseAis(gd, bundle.leadIn);

    const LaserContour& contour = tp.contour(contourIndex);
    if (!contour.enabled || !contour.leadIn.valid)
        return;

    TopoDS_Edge leadEdge = LaserToolpathBuilder::computeLeadInEdge(contour);
    if (leadEdge.IsNull())
        return;

    const Quantity_Color red(0.9, 0.15, 0.15, Quantity_TOC_RGB);
    Handle(AIS_Shape) ais = new AIS_Shape(leadEdge);
    ctx->Display(ais, AIS_WireFrame, 0, Standard_False);
    ctx->SetColor(ais, red, Standard_False);
    ctx->SetWidth(ais, 2.0, Standard_False);
    ctx->Deactivate(ais);
    bundle.leadIn = ais;
}

void ToolpathRenderer::rebuildNormalAis(GuiDocument* gd,
                                        const LaserToolpath& tp,
                                        int contourIndex)
{
    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull()) return;
    if (contourIndex < 0 || contourIndex >= tp.contourCount()) return;
    if (contourIndex >= m_bundles.size()) return;

    ContourAisBundle& bundle = m_bundles[contourIndex];
    eraseAis(gd, bundle.normal);

    const LaserContour& contour = tp.contour(contourIndex);
    if (!contour.enabled || contour.points.empty())
        return;

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
    if (!hasSegments)
        return;

    const Quantity_Color amber(0.95, 0.55, 0.10, Quantity_TOC_RGB);
    Handle(AIS_Shape) ais = new AIS_Shape(compound);
    ctx->Display(ais, AIS_WireFrame, 0, Standard_False);
    ctx->SetColor(ais, amber, Standard_False);
    ctx->SetWidth(ais, 1.8, Standard_False);
    ctx->Deactivate(ais);
    bundle.normal = ais;
}

void ToolpathRenderer::rebuildPreviewAis(GuiDocument* gd,
                                         const LaserToolpath& tp,
                                         const LeadInPreview& preview)
{
    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull()) return;
    eraseAis(gd, m_previewLeadInAis);
    m_previewContourIndex = -1;

    if (!preview.valid
        || preview.contourIndex < 0
        || preview.contourIndex >= tp.contourCount()) {
        return;
    }

    const LaserContour& source = tp.contour(preview.contourIndex);
    if (!source.enabled)
        return;

    LaserContour previewContour = source;
    if (!LaserToolpathBuilder::setContourStart(previewContour, preview.pointIndex))
        return;
    TopoDS_Edge previewEdge = LaserToolpathBuilder::computeLeadInEdge(previewContour);
    if (previewEdge.IsNull())
        return;

    const Quantity_Color yellow(0.95, 0.8, 0.1, Quantity_TOC_RGB);
    Handle(AIS_Shape) ais = new AIS_Shape(previewEdge);
    ctx->Display(ais, AIS_WireFrame, 0, Standard_False);
    ctx->SetColor(ais, yellow, Standard_False);
    ctx->SetWidth(ais, 2.5, Standard_False);
    ctx->Deactivate(ais);
    m_previewLeadInAis = ais;
    m_previewContourIndex = preview.contourIndex;
}

void ToolpathRenderer::rebuildContourMirror()
{
    m_contourAis.clear();
}

void ToolpathRenderer::redraw(GuiDocument* gd)
{
    if (!gd)
        return;
    if (gd->hasView()) {
        gd->view()->Redraw();
        return;
    }

    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (!ctx.IsNull())
        ctx->UpdateCurrentViewer();
}

} // namespace lcnc::view
