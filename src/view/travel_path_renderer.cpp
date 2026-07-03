#include "view/travel_path_renderer.h"

#include "view/gui_document.h"
#include "core/kinematics/machine_kinematics.h"

#include <AIS_InteractiveContext.hxx>
#include <Aspect_TypeOfLine.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRep_Builder.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Quantity_Color.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>

namespace lcnc::view {

TravelPathRenderer::TravelPathRenderer() = default;
TravelPathRenderer::~TravelPathRenderer() = default;

void TravelPathRenderer::setVisible(bool on)
{
    m_visible = on;
}

void TravelPathRenderer::erase(GuiDocument* gd)
{
    if (m_ais.IsNull()) return;
    if (gd) {
        const Handle(AIS_InteractiveContext)& ctx = gd->context();
        if (!ctx.IsNull())
            ctx->Erase(m_ais, Standard_False);
    }
    m_ais.Nullify();
    m_workpieceEntry.clear();
}

void TravelPathRenderer::refresh(GuiDocument* gd, const QVector<Segment>& segments)
{
    // 不可见 / 数据不足 → 仅擦除
    if (!gd || !m_visible || segments.size() < 2) {
        erase(gd);
        return;
    }
    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull()) return;

    m_workpieceEntry.clear();
    for (const Segment& segment : segments) {
        if (!segment.workpieceEntry.isEmpty()) {
            m_workpieceEntry = segment.workpieceEntry;
            break;
        }
    }

    // 先擦旧
    if (!m_ais.IsNull()) {
        ctx->Erase(m_ais, Standard_False);
        m_ais.Nullify();
    }

    // 构造 compound：相邻轮廓之间一段空程 Edge + 一个朝向终点的小箭头
    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);

    int addedEdges = 0;
    for (int i = 1; i < segments.size(); ++i) {
        const Segment& prev = segments[i - 1];
        const Segment& next = segments[i];
        const gp_Pnt p1(prev.ex, prev.ey, prev.ez);
        const gp_Pnt p2(next.sx, next.sy, next.sz);
        const double segLen = p1.Distance(p2);
        if (segLen < 1e-6) continue; // 同一点，跳过
        BRepBuilderAPI_MakeEdge mk(p1, p2);
        if (!mk.IsDone()) continue;
        builder.Add(compound, mk.Edge());

        // 箭头：从段中点出发，两根回指 (-dir) ± 法线 的短线组成 V 形。
        // 翼线长度固定 0.1mm，避免随段长缩放在不同尺度下视觉过大/过小。
        constexpr double kArrowLen = 0.1;
        const gp_Pnt mid(0.5 * (p1.X() + p2.X()),
                         0.5 * (p1.Y() + p2.Y()),
                         0.5 * (p1.Z() + p2.Z()));
        gp_Vec dir(p1, p2);
        if (dir.Magnitude() < 1e-6) continue;
        dir.Normalize();
        // 选一个与 dir 不平行的轴作为参考，构造法线
        gp_Vec refAxis(0.0, 0.0, 1.0);
        if (std::abs(dir.Z()) > 0.9) refAxis = gp_Vec(1.0, 0.0, 0.0);
        gp_Vec side = dir.Crossed(refAxis);
        if (side.Magnitude() < 1e-6) continue;
        side.Normalize();
        const double halfWidth = kArrowLen * 0.55;
        // V 形开口指向 dir 反方向 (-dir)：从段中点出发
        const gp_Pnt tip = mid;
        const gp_Pnt wingA(tip.X() - dir.X() * kArrowLen + side.X() * halfWidth,
                           tip.Y() - dir.Y() * kArrowLen + side.Y() * halfWidth,
                           tip.Z() - dir.Z() * kArrowLen + side.Z() * halfWidth);
        const gp_Pnt wingB(tip.X() - dir.X() * kArrowLen - side.X() * halfWidth,
                           tip.Y() - dir.Y() * kArrowLen - side.Y() * halfWidth,
                           tip.Z() - dir.Z() * kArrowLen - side.Z() * halfWidth);
        BRepBuilderAPI_MakeEdge mkA(tip, wingA);
        BRepBuilderAPI_MakeEdge mkB(tip, wingB);
        if (mkA.IsDone()) builder.Add(compound, mkA.Edge());
        if (mkB.IsDone()) builder.Add(compound, mkB.Edge());

        ++addedEdges;
    }
    if (addedEdges == 0) return;

    m_ais = new AIS_Shape(compound);

    // 蓝色虚线：覆盖 wire / line / 默认绘制属性
    Handle(Prs3d_LineAspect) dash = new Prs3d_LineAspect(
        Quantity_Color(0.4, 0.6, 1.0, Quantity_TOC_RGB),
        Aspect_TOL_DASH,
        1.5);
    const Handle(Prs3d_Drawer)& drawer = m_ais->Attributes();
    drawer->SetWireAspect(dash);
    drawer->SetLineAspect(dash);
    drawer->SetUnFreeBoundaryAspect(dash);
    drawer->SetFreeBoundaryAspect(dash);
    drawer->SetSeenLineAspect(dash);

    ctx->Display(m_ais, AIS_WireFrame, -1, Standard_False);
    ctx->Deactivate(m_ais); // 禁拾取，避免干扰用户多选
    updateTransforms(gd, nullptr);
}

void TravelPathRenderer::updateTransforms(GuiDocument* gd, MachineKinematics* kin)
{
    if (!gd || m_ais.IsNull())
        return;

    gp_Trsf transform;
    if (kin && !m_workpieceEntry.isEmpty())
        transform = kin->computeWpcTransform(m_workpieceEntry);

    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    m_ais->SetLocalTransformation(transform);
    if (!ctx.IsNull())
        ctx->RecomputePrsOnly(m_ais, Standard_False);
}

} // namespace lcnc::view
