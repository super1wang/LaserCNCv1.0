#include "view/machine_guide_renderer.h"

#include "view/gui_document.h"
#include "view/graphics_scene.h"
#include "core/logging/logger.h"
#include "core/kinematics/machine_kinematics.h"

#include <AIS_InteractiveContext.hxx>
#include <AIS_DisplayMode.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRep_Builder.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <Graphic3d_MaterialAspect.hxx>
#include <Graphic3d_NameOfMaterial.hxx>
#include <Graphic3d_ZLayerId.hxx>
#include <Quantity_Color.hxx>
#include <TopoDS_Compound.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <cmath>

namespace lcnc::view {

MachineGuideRenderer::MachineGuideRenderer() = default;
MachineGuideRenderer::~MachineGuideRenderer() = default;

QMap<QString, Handle(AIS_Shape)>& MachineGuideRenderer::guideMap(GuiDocument* gd)
{
    return m_axisGuideAisByDocument[gd];
}

const QMap<QString, Handle(AIS_Shape)>* MachineGuideRenderer::guideMap(GuiDocument* gd) const
{
    const auto it = m_axisGuideAisByDocument.constFind(gd);
    return it == m_axisGuideAisByDocument.cend() ? nullptr : &it.value();
}

void MachineGuideRenderer::erase(GuiDocument* gd)
{
    if (!gd) { m_axisGuideAisByDocument.clear(); return; }
    GraphicsScene* scene = gd->scene();
    auto it = m_axisGuideAisByDocument.find(gd);
    if (it == m_axisGuideAisByDocument.end())
        return;
    if (!scene) {
        m_axisGuideAisByDocument.erase(it);
        return;
    }
    for (auto& ais : it.value()) {
        if (!ais.IsNull()) scene->eraseObject(ais, false);
    }
    m_axisGuideAisByDocument.erase(it);
}

void MachineGuideRenderer::refresh(GuiDocument* gd,
                                   MachineKinematics* kin,
                                   const gp_Pnt& cutterHeadWorldTip)
{
    erase(gd);
    if (!gd || !kin) return;
    GraphicsScene* scene = gd->scene();
    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (!scene || ctx.IsNull()) return;
    auto& axisGuideAis = guideMap(gd);

    LCNC_DEBUG(lcnc::LogCode::Generic,
               "MachineGuideRenderer::refresh cfg='{}' axes={} headTip=({:.3f},{:.3f},{:.3f}) rotaryVisible={} headVisible={}",
               kin->configType().toStdString(),
               kin->axes().size(),
               cutterHeadWorldTip.X(),
               cutterHeadWorldTip.Y(),
               cutterHeadWorldTip.Z(),
               m_rotaryAxisVisible,
               m_cutterHeadVisible);

    auto axisColor = [](const QString& axisName) {
        if (axisName == QStringLiteral("A"))
            return Quantity_Color(0.95, 0.55, 0.10, Quantity_TOC_RGB);
        if (axisName == QStringLiteral("B"))
            return Quantity_Color(0.10, 0.70, 0.95, Quantity_TOC_RGB);
        if (axisName == QStringLiteral("C"))
            return Quantity_Color(0.90, 0.20, 0.90, Quantity_TOC_RGB);
        return Quantity_Color(0.20, 0.45, 0.95, Quantity_TOC_RGB);
    };

    bool hasRotaryAxis = false;
    for (const MachineAxisDef& axis : kin->axes()) {
        if (axis.motionType != MachineAxisDef::Rotary)
            continue;
        hasRotaryAxis = true;
        const QString axisName = axis.name.trimmed().toUpper();
        const gp_Vec axisVector(axis.direction);
        constexpr double kRotaryGuideHalfLength = 50.0;
        const gp_Pnt p0 = axis.origin.Translated(axisVector * -kRotaryGuideHalfLength);
        const gp_Pnt p1 = axis.origin.Translated(axisVector *  kRotaryGuideHalfLength);
        BRepBuilderAPI_MakeEdge edgeMaker(p0, p1);
        if (!edgeMaker.IsDone()) continue;
        Handle(AIS_Shape) ais = scene->displayShape(edgeMaker.Edge(), false, false, false);
        scene->setShapeColor(ais, axisColor(axisName), false);
        ais->SetWidth(3.0);
        ctx->SetZLayer(ais, Graphic3d_ZLayerId_Topmost);
        ctx->Deactivate(ais);
        axisGuideAis.insert(QStringLiteral("axis:%1").arg(axisName), ais);
    }

    if (hasRotaryAxis) {
        constexpr double kCenterSphereRadius = 3.5;
        const gp_Pnt center = rotationCenter(kin);
        BRepPrimAPI_MakeSphere sphereMaker(center, kCenterSphereRadius);
        if (sphereMaker.IsDone()) {
            Handle(AIS_Shape) sphereAis = scene->displayShape(sphereMaker.Shape(), false, false, false);
            scene->setShapeColor(sphereAis, Quantity_Color(0.95, 0.95, 0.15, Quantity_TOC_RGB), false);
            ctx->SetZLayer(sphereAis, Graphic3d_ZLayerId_Topmost);
            ctx->Deactivate(sphereAis);
            axisGuideAis.insert(QStringLiteral("center:sphere"), sphereAis);
        }
    }

    constexpr double kHeadConeHeight  = 30.0;
    constexpr double kHeadConeRadius  = 8.0;
    constexpr double kHeadConeTipRadius = 0.35;
    const gp_Pnt coneTip(0.0, 0.0, 0.0);
    const gp_Pnt coneBaseCenter(0.0, 0.0, kHeadConeHeight);
    gp_Ax2 coneAxis(coneBaseCenter, gp_Dir(0.0, 0.0, -1.0));
    BRepPrimAPI_MakeCone coneMaker(coneAxis, kHeadConeRadius, kHeadConeTipRadius, kHeadConeHeight);
    if (coneMaker.IsDone()) {
        TopoDS_Shape coneShape = coneMaker.Shape();
        BRepMesh_IncrementalMesh(coneShape, 0.5);
        Handle(AIS_Shape) coneAis = new AIS_Shape(coneShape);
        coneAis->SetColor(Quantity_Color(1.0, 0.0, 0.0, Quantity_TOC_RGB));
        coneAis->SetMaterial(Graphic3d_MaterialAspect(Graphic3d_NameOfMaterial_ShinyPlastified));
        // 模拟刀头是置顶辅助对象。实体锥若留在默认 Z 层，会被机床实体遮住，
        // 而同组的置顶线框仍然可见，最终看起来就像刀头只能以线框显示。
        // 同时设置对象自己的显示模式，保证隐藏后再次显示以及全局线框切换时
        // 仍使用实体着色 presentation。
        coneAis->SetDisplayMode(AIS_Shaded);
        ctx->Display(coneAis, AIS_Shaded, 0, Standard_False);
        ctx->SetZLayer(coneAis, Graphic3d_ZLayerId_Topmost);
        ctx->Deactivate(coneAis);
        axisGuideAis.insert(QStringLiteral("head:cone"), coneAis);
    } else {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "MachineGuideRenderer::refresh failed to create cutter head cone");
    }

    TopoDS_Compound coneWire;
    BRep_Builder builder;
    builder.MakeCompound(coneWire);
    BRepBuilderAPI_MakeEdge baseMaker(
        gp_Circ(gp_Ax2(coneBaseCenter, gp_Dir(0.0, 0.0, 1.0)), kHeadConeRadius));
    if (baseMaker.IsDone())
        builder.Add(coneWire, baseMaker.Edge());
    for (int i = 0; i < 8; ++i) {
        const double angle = (2.0 * M_PI * i) / 8.0;
        const gp_Pnt basePoint(kHeadConeRadius * std::cos(angle),
                               kHeadConeRadius * std::sin(angle),
                               kHeadConeHeight);
        BRepBuilderAPI_MakeEdge sideMaker(coneTip, basePoint);
        if (sideMaker.IsDone())
            builder.Add(coneWire, sideMaker.Edge());
    }
    Handle(AIS_Shape) wireAis = scene->displayShape(coneWire, false, false, false);
    scene->setShapeColor(wireAis, Quantity_Color(1.0, 0.0, 0.0, Quantity_TOC_RGB), false);
    wireAis->SetWidth(3.0);
    ctx->SetZLayer(wireAis, Graphic3d_ZLayerId_Topmost);
    ctx->Deactivate(wireAis);
    axisGuideAis.insert(QStringLiteral("head:wire"), wireAis);

    applyVisibility(gd);
    updateTransforms(gd, kin, cutterHeadWorldTip);
}

void MachineGuideRenderer::setRotaryAxisVisible(GuiDocument* gd, bool visible)
{
    m_rotaryAxisVisible = visible;
    applyVisibility(gd);
}

void MachineGuideRenderer::setCutterHeadVisible(GuiDocument* gd, bool visible)
{
    m_cutterHeadVisible = visible;
    applyVisibility(gd);
}

void MachineGuideRenderer::applyVisibility(GuiDocument* gd)
{
    if (!gd)
        return;
    GraphicsScene* scene = gd->scene();
    if (!scene)
        return;
    const auto guideIt = m_axisGuideAisByDocument.constFind(gd);
    if (guideIt == m_axisGuideAisByDocument.cend())
        return;

    const auto& axisGuideAis = guideIt.value();
    for (auto it = axisGuideAis.cbegin(); it != axisGuideAis.cend(); ++it) {
        const bool isRotaryAxis = it.key().startsWith(QStringLiteral("axis:"));
        const bool isCenter = it.key().startsWith(QStringLiteral("center:"));
        const bool isHead = it.key().startsWith(QStringLiteral("head:"));
        const bool visible = ((isRotaryAxis || isCenter) && m_rotaryAxisVisible)
            || (isHead && m_cutterHeadVisible);
        if (visible)
            scene->displayObject(it.value(), false);
        else
            scene->eraseObject(it.value(), false);
    }
    if (gd->hasView())
        gd->view()->Redraw();
}

void MachineGuideRenderer::updateTransforms(GuiDocument* gd,
                                            MachineKinematics* kin,
                                            const gp_Pnt& cutterHeadWorldTip)
{
    if (!gd || !kin) return;
    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull()) return;
    const auto guideIt = m_axisGuideAisByDocument.constFind(gd);
    if (guideIt == m_axisGuideAisByDocument.cend())
        return;
    const auto& axisGuideAis = guideIt.value();

    auto applyTransform = [&](const QString& key, const gp_Trsf& trsf) {
        const auto it = axisGuideAis.constFind(key);
        if (it == axisGuideAis.cend() || it.value().IsNull()) return;
        it.value()->SetLocalTransformation(trsf);
        ctx->RecomputePrsOnly(it.value(), Standard_False);
    };

    for (const MachineAxisDef& axis : kin->axes()) {
        if (axis.motionType != MachineAxisDef::Rotary)
            continue;
        const QString axisName = axis.name.trimmed().toUpper();
        applyTransform(QStringLiteral("axis:%1").arg(axisName), kin->computeAxisTransform(axisName));
    }

    gp_Trsf headTransform;
    headTransform.SetTranslation(gp_Vec(gp_Pnt(0.0, 0.0, 0.0), cutterHeadWorldTip));
    applyTransform(QStringLiteral("head:cone"), headTransform);
    applyTransform(QStringLiteral("head:wire"), headTransform);
}

gp_Pnt MachineGuideRenderer::rotationCenter(MachineKinematics* kin) const
{
    if (!kin)
        return gp_Pnt(0.0, 0.0, 0.0);

    auto originOf = [kin](const QString& axisName, gp_Pnt* out) {
        const MachineAxisDef* axis = kin->findAxis(axisName);
        if (!axis || axis->motionType != MachineAxisDef::Rotary)
            return false;
        if (out)
            *out = axis->origin;
        return true;
    };

    gp_Pnt a, b, c;
    if (kin->configType() == QStringLiteral("VERTICAL_AC_TABLE")
        && originOf(QStringLiteral("A"), &a)
        && originOf(QStringLiteral("C"), &c)) {
        return gp_Pnt(c.X(), a.Y(), a.Z());
    }
    if (kin->configType() == QStringLiteral("VERTICAL_BC_TABLE")
        && originOf(QStringLiteral("B"), &b)
        && originOf(QStringLiteral("C"), &c)) {
        return gp_Pnt(b.X(), c.Y(), b.Z());
    }

    for (const MachineAxisDef& axis : kin->axes()) {
        if (axis.motionType == MachineAxisDef::Rotary)
            return axis.origin;
    }
    return gp_Pnt(0.0, 0.0, 0.0);
}

} // namespace lcnc::view
