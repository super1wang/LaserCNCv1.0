#include "view/machine_guide_renderer.h"

#include "view/gui_document.h"
#include "view/graphics_scene.h"
#include "core/kinematics/machine_kinematics.h"

#include <AIS_InteractiveContext.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <Quantity_Color.hxx>
#include <gp_Ax2.hxx>
#include <gp_Vec.hxx>

namespace lcnc::view {

MachineGuideRenderer::MachineGuideRenderer() = default;
MachineGuideRenderer::~MachineGuideRenderer() = default;

void MachineGuideRenderer::erase(GuiDocument* gd)
{
    if (!gd) { m_axisGuideAis.clear(); return; }
    GraphicsScene* scene = gd->scene();
    if (!scene) { m_axisGuideAis.clear(); return; }
    for (auto& ais : m_axisGuideAis) {
        if (!ais.IsNull()) scene->eraseObject(ais, false);
    }
    m_axisGuideAis.clear();
}

void MachineGuideRenderer::refresh(GuiDocument* gd,
                                   MachineKinematics* kin,
                                   const gp_Pnt& cutterHeadModelPos)
{
    erase(gd);
    if (!gd || !kin) return;
    GraphicsScene* scene = gd->scene();
    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (!scene || ctx.IsNull()) return;

    auto axisColor = [](const QString& axisName) {
        if (axisName == QStringLiteral("A"))
            return Quantity_Color(0.95, 0.55, 0.10, Quantity_TOC_RGB);
        if (axisName == QStringLiteral("C"))
            return Quantity_Color(0.90, 0.20, 0.90, Quantity_TOC_RGB);
        return Quantity_Color(0.20, 0.45, 0.95, Quantity_TOC_RGB);
    };

    const QStringList visibleAxes = {QStringLiteral("A"), QStringLiteral("C")};
    for (const QString& axisName : visibleAxes) {
        const MachineAxisDef* axisDef = kin->findAxis(axisName);
        if (!axisDef) continue;
        const gp_Vec axisVector(axisDef->direction);
        constexpr double kRotaryGuideHalfLength = 50.0;
        const gp_Pnt p0 = axisDef->origin.Translated(axisVector * -kRotaryGuideHalfLength);
        const gp_Pnt p1 = axisDef->origin.Translated(axisVector *  kRotaryGuideHalfLength);
        BRepBuilderAPI_MakeEdge edgeMaker(p0, p1);
        if (!edgeMaker.IsDone()) continue;
        Handle(AIS_Shape) ais = scene->displayShape(edgeMaker.Edge(), false, false, false);
        scene->setShapeColor(ais, axisColor(axisName), false);
        ais->SetWidth(3.0);
        ctx->Deactivate(ais);
        m_axisGuideAis.insert(QStringLiteral("axis:%1").arg(axisName), ais);
    }

    constexpr double kHeadGuideLength = 80.0;
    constexpr double kHeadConeHeight  = 18.0;
    constexpr double kHeadConeRadius  = 5.0;
    const gp_Pnt headTip = cutterHeadModelPos;
    const gp_Pnt headTop = headTip.Translated(gp_Vec(0.0, 0.0, kHeadGuideLength));

    BRepBuilderAPI_MakeEdge headAxisMaker(headTop, headTip);
    if (headAxisMaker.IsDone()) {
        Handle(AIS_Shape) axisAis = scene->displayShape(headAxisMaker.Edge(), false, false, false);
        scene->setShapeColor(axisAis, axisColor(QString()), false);
        axisAis->SetWidth(2.5);
        ctx->Deactivate(axisAis);
        m_axisGuideAis.insert(QStringLiteral("head:axis"), axisAis);
    }

    gp_Ax2 coneAxis(headTip.Translated(gp_Vec(0.0, 0.0, kHeadConeHeight)), gp_Dir(0.0, 0.0, -1.0));
    BRepPrimAPI_MakeCone coneMaker(coneAxis, kHeadConeRadius, 0.0, kHeadConeHeight);
    if (coneMaker.IsDone()) {
        Handle(AIS_Shape) coneAis = scene->displayShape(coneMaker.Shape(), false, false, false);
        scene->setShapeColor(coneAis, Quantity_Color(0.15, 0.55, 0.95, Quantity_TOC_RGB), false);
        ctx->Deactivate(coneAis);
        m_axisGuideAis.insert(QStringLiteral("head:cone"), coneAis);
    }

    updateTransforms(gd, kin);
}

void MachineGuideRenderer::updateTransforms(GuiDocument* gd, MachineKinematics* kin)
{
    if (!gd || !kin) return;
    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull()) return;

    auto applyTransform = [&](const QString& key, const gp_Trsf& trsf) {
        const auto it = m_axisGuideAis.constFind(key);
        if (it == m_axisGuideAis.cend() || it.value().IsNull()) return;
        it.value()->SetLocalTransformation(trsf);
        ctx->RecomputePrsOnly(it.value(), Standard_False);
    };

    applyTransform(QStringLiteral("axis:A"), kin->computeAxisTransform(QStringLiteral("A")));
    applyTransform(QStringLiteral("axis:C"), kin->computeAxisTransform(QStringLiteral("C")));
    const gp_Trsf headTransform = kin->computeAxisTransform(QStringLiteral("Z"));
    applyTransform(QStringLiteral("head:axis"), headTransform);
    applyTransform(QStringLiteral("head:cone"), headTransform);
}

} // namespace lcnc::view
