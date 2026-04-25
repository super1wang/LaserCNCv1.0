#include "view/rendering_manager.h"

#include "core/document/lcnc_document.h"
#include "core/document/xcaf_utils.h"
#include "core/kernel/kernel.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/logging/logger.h"
#include "view/graphics_scene.h"
#include "view/gui_document.h"

#include <AIS_InteractiveContext.hxx>
#include <AIS_ListIteratorOfListOfInteractive.hxx>
#include <AIS_ListOfInteractive.hxx>
#include <Aspect_TypeOfLine.hxx>
#include <Graphic3d_MaterialAspect.hxx>
#include <Graphic3d_NameOfMaterial.hxx>
#include <Graphic3d_RenderingMode.hxx>
#include <Graphic3d_RenderingParams.hxx>
#include <Graphic3d_TypeOfBackfacingModel.hxx>
#include <Graphic3d_AspectFillArea3d.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Prs3d_ShadingAspect.hxx>
#include <Quantity_Color.hxx>
#include <TDF_LabelSequence.hxx>
#include <V3d_AmbientLight.hxx>
#include <V3d_DirectionalLight.hxx>
#include <V3d_Light.hxx>
#include <V3d_ListOfLight.hxx>
#include <V3d_Viewer.hxx>
#include <V3d_View.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <QTimer>
#include <QList>
#include <QSet>
#include <QtGlobal>

namespace lcnc::view {

namespace {
Quantity_Color toQuantity(const QColor& color)
{
    return Quantity_Color(color.redF(), color.greenF(), color.blueF(), Quantity_TOC_RGB);
}

QColor darkerGradientColor(const QColor& color)
{
    QColor bottom = color.darker(155);
    bottom.setAlpha(255);
    return bottom;
}

Graphic3d_NameOfMaterial materialName(const QString& id)
{
    const QString key = id.toLower();
    if (key == QStringLiteral("steel")) return Graphic3d_NameOfMaterial_Steel;
    if (key == QStringLiteral("aluminum")) return Graphic3d_NameOfMaterial_Aluminum;
    if (key == QStringLiteral("chrome")) return Graphic3d_NameOfMaterial_Chrome;
    if (key == QStringLiteral("metal")) return Graphic3d_NameOfMaterial_Metalized;
    if (key == QStringLiteral("satin")) return Graphic3d_NameOfMaterial_Satin;
    if (key == QStringLiteral("plastic")) return Graphic3d_NameOfMaterial_Plastified;
    if (key == QStringLiteral("shiny_plastic")) return Graphic3d_NameOfMaterial_ShinyPlastified;
    return Graphic3d_NameOfMaterial_Plastified;
}

int displayModeFromStartup(StartupDisplayMode mode)
{
    return mode == StartupDisplayMode::Wireframe ? 0 : 1;
}

void applyPresetDefaults(RenderProfileSettings& profile, bool cam)
{
    switch (profile.qualityPreset) {
    case RenderQualityPreset::Low:
        profile.renderMethod = RenderMethod::Rasterization;
        profile.antiAliasing = false;
        profile.msaaSamples = 0;
        profile.shadows = false;
        profile.reflections = false;
        profile.adaptiveSampling = false;
        profile.frustumCulling = true;
        profile.backFaceCulling = cam;
        profile.deviationCoefficient = cam ? 0.14 : 0.10;
        profile.deviationAngle = cam ? 0.75 : 0.65;
        profile.edgeWidth = 0.6;
        break;
    case RenderQualityPreset::Medium:
        profile.renderMethod = RenderMethod::Rasterization;
        profile.antiAliasing = true;
        profile.msaaSamples = 2;
        profile.shadows = false;
        profile.reflections = false;
        profile.adaptiveSampling = false;
        profile.frustumCulling = true;
        profile.backFaceCulling = cam;
        profile.deviationCoefficient = cam ? 0.06 : 0.05;
        profile.deviationAngle = cam ? 0.40 : 0.35;
        profile.edgeWidth = 0.8;
        break;
    case RenderQualityPreset::High:
        profile.antiAliasing = true;
        profile.msaaSamples = cam ? 4 : 8;
        profile.shadows = !cam;
        profile.reflections = !cam;
        profile.frustumCulling = true;
        profile.backFaceCulling = false;
        profile.deviationCoefficient = cam ? 0.025 : 0.01;
        profile.deviationAngle = cam ? 0.20 : 0.12;
        profile.edgeWidth = 1.1;
        break;
    case RenderQualityPreset::Custom:
        break;
    }
}
} // namespace

RenderingManager::RenderingManager(GuiDocument* document, QObject* parent)
    : QObject(parent)
    , m_document(document)
    , m_applyTimer(new QTimer(this))
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "RenderingManager ctor");
    m_colors.machineAxisColors = defaultMachineAxisColors();
    m_applyTimer->setSingleShot(true);
    m_applyTimer->setInterval(0);
    connect(m_applyTimer, &QTimer::timeout, this, &RenderingManager::flushPendingApply);
}

void RenderingManager::setMachineView(bool machineView)
{
    m_machineView = machineView;
}

void RenderingManager::configure(const RenderProfileSettings& profile, const ColorSettings& colors)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "RenderingManager::configure machineView={} quality={} method={}",
               m_machineView,
               static_cast<int>(profile.qualityPreset),
               static_cast<int>(profile.renderMethod));
    m_profile = profile;
    m_colors = colors;
    if (m_colors.machineAxisColors.isEmpty())
        m_colors.machineAxisColors = defaultMachineAxisColors();
}

void RenderingManager::requestApply(RenderDirtyFlags flags)
{
    if (flags == RenderDirtyFlags(RenderDirtyFlag::None))
        return;
    m_pendingFlags |= flags;
    if (!m_applyTimer->isActive())
        m_applyTimer->start();
}

void RenderingManager::applyNow(RenderDirtyFlags flags)
{
    if (!m_document || flags == RenderDirtyFlags(RenderDirtyFlag::None))
        return;

    LCNC_DEBUG(lcnc::LogCode::Generic,
               "RenderingManager::applyNow flags={} machineView={}",
               static_cast<int>(flags.toInt()), m_machineView);

    if (flags.testFlag(RenderDirtyFlag::DefaultDisplay)) {
        setRuntimeDisplayMode(displayModeFromStartup(m_profile.defaultDisplayMode), false);
    }
    if (flags.testFlag(RenderDirtyFlag::Profile)) {
        applyViewRenderingParams();
        applyLighting();
    }
    if (flags.testFlag(RenderDirtyFlag::Background)) {
        applyBackground();
    }
    if (flags.testFlag(RenderDirtyFlag::Highlight)) {
        applyHighlight();
    }
}

void RenderingManager::setRuntimeDisplayMode(int displayMode, bool faceBoundary)
{
    if (!m_document || !m_document->scene())
        return;

    const Handle(AIS_InteractiveContext)& ctx = m_document->scene()->context();
    if (ctx.IsNull())
        return;

    LCNC_DEBUG(lcnc::LogCode::Generic,
               "RenderingManager::setRuntimeDisplayMode mode={} edges={}",
               displayMode, faceBoundary);

    m_runtimeDisplayMode = displayMode;
    m_runtimeFaceBoundary = faceBoundary;
    ctx->DefaultDrawer()->SetFaceBoundaryDraw(faceBoundary);
    ctx->SetDisplayMode(displayMode, Standard_False);

    AIS_ListOfInteractive objects;
    ctx->DisplayedObjects(objects);
    for (AIS_ListIteratorOfListOfInteractive it(objects); it.More(); it.Next()) {
        const Handle(AIS_Shape) shape = Handle(AIS_Shape)::DownCast(it.Value());
        if (shape.IsNull())
            continue;
        ctx->SetDisplayMode(shape, displayMode, Standard_False);
        if (!shape->Attributes().IsNull())
            shape->Attributes()->SetFaceBoundaryDraw(faceBoundary);
        ctx->Redisplay(shape, Standard_False);
    }
    ctx->UpdateCurrentViewer();
}

void RenderingManager::applyDocumentStyles(const QMap<QString, Handle(AIS_Shape)>& aisMap)
{
    if (!m_document || !m_document->scene())
        return;

    LcncDocument* doc = m_document->document();
    if (!doc)
        return;

    QSet<QString> machineEntries;
    const TDF_LabelSequence machineLabels = doc->entityLabels(LcncDocument::EntityKind::Machine);
    for (int i = 1; i <= machineLabels.Length(); ++i)
        machineEntries.insert(XcafUtils::entry(machineLabels.Value(i)));

    QSet<QString> workpieceEntries;
    const TDF_LabelSequence workpieceLabels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
    for (int i = 1; i <= workpieceLabels.Length(); ++i)
        workpieceEntries.insert(XcafUtils::entry(workpieceLabels.Value(i)));

    MachineKinematics* kin = doc->machineKinematics();
    for (auto it = aisMap.cbegin(); it != aisMap.cend(); ++it) {
        const QString& entry = it.key();
        const Handle(AIS_Shape)& ais = it.value();
        if (ais.IsNull())
            continue;

        const bool isMachineShape = machineEntries.contains(entry);
        const bool isWorkpieceShape = workpieceEntries.contains(entry) || !isMachineShape;
        const QString axisName = (isMachineShape && kin) ? kin->axisForShape(entry) : QString();
        applyShapeStyle(entry, ais, isMachineShape, isWorkpieceShape, axisName);
    }

    const Handle(AIS_InteractiveContext)& ctx = m_document->scene()->context();
    if (!ctx.IsNull())
        ctx->UpdateCurrentViewer();
}

RenderProfileSettings RenderingManager::effectiveProfile() const
{
    RenderProfileSettings effective = m_profile;
    applyPresetDefaults(effective, m_machineView);
    return effective;
}

void RenderingManager::applyViewRenderingParams()
{
    if (!m_document || !m_document->hasView())
        return;

    const RenderProfileSettings p = effectiveProfile();
    Handle(V3d_View) view = m_document->view();
    Graphic3d_RenderingParams& params = view->ChangeRenderingParams();
    params.Method = p.renderMethod == RenderMethod::RayTracing
        ? Graphic3d_RM_RAYTRACING
        : Graphic3d_RM_RASTERIZATION;
    params.NbMsaaSamples = p.antiAliasing ? qBound(0, p.msaaSamples, 8) : 0;
    params.RenderResolutionScale = p.antiAliasing ? 1.0f : static_cast<Standard_ShortReal>(qBound(0.25, p.renderResolutionScale, 2.0));
    params.IsShadowEnabled = p.shadows ? Standard_True : Standard_False;
    params.IsReflectionEnabled = p.reflections ? Standard_True : Standard_False;
    params.IsAntialiasingEnabled = p.adaptiveSampling ? Standard_True : Standard_False;
    params.AdaptiveScreenSampling = p.adaptiveSampling ? Standard_True : Standard_False;
    params.RaytracingDepth = qBound(1, p.raytracingDepth, 8);
    params.RayTracingTileSize = qBound(8, p.rayTracingTileSize, 128);
    params.NbRayTracingTiles = p.adaptiveSampling ? qBound(1, p.rayTracingTileCount, 1024) : -1;
    params.FrustumCullingState = p.frustumCulling
        ? Graphic3d_RenderingParams::FrustumCulling_On
        : Graphic3d_RenderingParams::FrustumCulling_Off;

    LCNC_INFO(lcnc::LogCode::Generic,
              "View rendering params applied: cam={} method={} msaa={} shadows={} cull={}",
              m_machineView,
              static_cast<int>(p.renderMethod),
              params.NbMsaaSamples,
              p.shadows,
              p.frustumCulling);
}

void RenderingManager::applyBackground()
{
    if (!m_document || !m_document->hasView())
        return;

    const QColor top = m_machineView ? m_colors.camBackgroundColor : m_colors.cadBackgroundColor;
    const QColor bottom = darkerGradientColor(top);
    m_document->view()->SetBgGradientColors(
        toQuantity(top), toQuantity(bottom), Aspect_GFM_VER, Standard_False);
    m_document->view()->Redraw();
}

void RenderingManager::applyHighlight()
{
    if (!m_document || !m_document->scene())
        return;

    const Handle(AIS_InteractiveContext)& ctx = m_document->scene()->context();
    if (ctx.IsNull())
        return;

    const Quantity_Color selection = toQuantity(m_colors.selectionColor);
    const Quantity_Color hover = toQuantity(m_colors.hoverColor);
    if (Handle(Prs3d_Drawer) selected = ctx->HighlightStyle(Prs3d_TypeOfHighlight_Selected); !selected.IsNull()) {
        selected->SetColor(selection);
        selected->SetDisplayMode(m_colors.highlightDisplayMode);
        selected->SetLineAspect(new Prs3d_LineAspect(selection, Aspect_TOL_SOLID, m_colors.highlightLineWidth));
        Handle(Prs3d_ShadingAspect) shading = new Prs3d_ShadingAspect();
        shading->SetColor(selection);
        selected->SetShadingAspect(shading);
        selected->SetFaceBoundaryDraw(true);
        selected->SetFaceBoundaryAspect(new Prs3d_LineAspect(selection, Aspect_TOL_SOLID, m_colors.highlightLineWidth));
    }
    if (Handle(Prs3d_Drawer) dynamic = ctx->HighlightStyle(Prs3d_TypeOfHighlight_Dynamic); !dynamic.IsNull()) {
        dynamic->SetColor(hover);
        dynamic->SetDisplayMode(m_colors.highlightDisplayMode);
    }
    ctx->UpdateCurrentViewer();
}

void RenderingManager::applyLighting()
{
    if (!m_document || !m_document->scene())
        return;

    const RenderProfileSettings p = effectiveProfile();
    const Handle(V3d_Viewer)& viewer = m_document->scene()->viewer();
    if (viewer.IsNull())
        return;

    QList<Handle(V3d_Light)> existingLights;
    for (V3d_ListOfLightIterator it(viewer->DefinedLights()); it.More(); it.Next())
        existingLights.append(it.Value());
    for (const Handle(V3d_Light)& light : existingLights)
        viewer->DelLight(light);

    const Standard_Real ambientLevel = qBound(0.45, p.ambientLight, 0.85);
    const Quantity_Color keyColor(0.72, 0.72, 0.72, Quantity_TOC_RGB);
    const Quantity_Color fillColor(0.38, 0.38, 0.38, Quantity_TOC_RGB);
    const auto addDirectionalLight = [&viewer](V3d_TypeOfOrientation orientation,
                                               const Quantity_Color& color) {
        Handle(V3d_DirectionalLight) light = new V3d_DirectionalLight(
            orientation, color, Standard_True);
        viewer->AddLight(light);
    };

    addDirectionalLight(V3d_XposYnegZpos, keyColor);
    addDirectionalLight(V3d_XnegYposZpos, fillColor);
    addDirectionalLight(V3d_XposYposZneg, fillColor);
    addDirectionalLight(V3d_XnegYnegZneg, fillColor);
    Handle(V3d_AmbientLight) ambient = new V3d_AmbientLight(
        Quantity_Color(ambientLevel, ambientLevel, ambientLevel, Quantity_TOC_RGB));
    viewer->AddLight(ambient);
    viewer->SetLightOn();
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "RenderingManager::applyLighting machineView={} ambient={}",
               m_machineView, ambientLevel);
}

void RenderingManager::applyShapeStyle(const QString& entry,
                                        const Handle(AIS_Shape)& ais,
                                        bool isMachineShape,
                                        bool isWorkpieceShape,
                                        const QString& axisName)
{
    Q_UNUSED(entry);
    const RenderProfileSettings p = effectiveProfile();
    if (ais.IsNull() || !m_document || !m_document->scene())
        return;

    const Handle(AIS_InteractiveContext)& ctx = m_document->scene()->context();
    if (ctx.IsNull())
        return;

    QColor color = m_colors.workpieceColor;
    if (isMachineShape) {
        const QString key = axisName.isEmpty() ? QStringLiteral("BASE") : axisName;
        color = m_colors.machineAxisColors.value(key,
            m_colors.machineAxisColors.value(QStringLiteral("BASE"), QColor(189, 189, 194)));
    } else if (!isWorkpieceShape) {
        color = QColor(200, 200, 210);
    }

    ais->SetOwnDeviationCoefficient(p.deviationCoefficient);
    ais->SetOwnDeviationAngle(p.deviationAngle);
    if (!ais->Attributes().IsNull()) {
        ais->Attributes()->SetFaceBoundaryDraw(m_runtimeFaceBoundary);
        ais->Attributes()->FaceBoundaryAspect()->SetColor(Quantity_NOC_GRAY40);
        ais->Attributes()->FaceBoundaryAspect()->SetWidth(p.edgeWidth);
        if (!ais->Attributes()->ShadingAspect().IsNull()
            && !ais->Attributes()->ShadingAspect()->Aspect().IsNull()) {
            ais->Attributes()->ShadingAspect()->Aspect()->SetFaceCulling(
                p.backFaceCulling
                    ? Graphic3d_TypeOfBackfacingModel_Auto
                    : Graphic3d_TypeOfBackfacingModel_DoubleSided);
        }
    }

    Graphic3d_MaterialAspect material(materialName(p.material));
    ctx->SetMaterial(ais, material, Standard_False);
    ctx->SetColor(ais, toQuantity(color), Standard_False);
    ctx->Redisplay(ais, Standard_False);
}

void RenderingManager::flushPendingApply()
{
    const RenderDirtyFlags flags = m_pendingFlags;
    m_pendingFlags = RenderDirtyFlags(RenderDirtyFlag::None);
    applyNow(flags);
    if (flags.testFlag(RenderDirtyFlag::Profile) || flags.testFlag(RenderDirtyFlag::Colors)) {
        // GuiDocument owns the AIS map, so re-use its style entry point.
        if (m_document)
            m_document->applyMachineDisplayStyle();
    }
}

} // namespace lcnc::view
