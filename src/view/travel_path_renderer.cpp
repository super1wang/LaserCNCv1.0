#include "view/travel_path_renderer.h"

#include "view/gui_document.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/logging/logger.h"

#include <AIS_InteractiveContext.hxx>
#include <Aspect_TypeOfLine.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRep_Builder.hxx>
#include <Graphic3d_ZLayerId.hxx>
#include <GeomAPI_Interpolate.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Quantity_Color.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TColgp_HArray1OfPnt.hxx>
#include <Standard_Failure.hxx>
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
    if (!gd || !m_visible || segments.isEmpty()) {
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
    bool allVerified = true;
    const auto addEdge = [&](const gp_Pnt& p1, const gp_Pnt& p2) {
        const double segLen = p1.Distance(p2);
        if (segLen < 1e-6) return; // 同一点，跳过
        BRepBuilderAPI_MakeEdge mk(p1, p2);
        if (!mk.IsDone()) return;
        builder.Add(compound, mk.Edge());
        ++addedEdges;
    };

    const auto addCurveEdge = [&](const QVector<Segment::Waypoint>& waypoints) {
        if (waypoints.size() < 3)
            return false;
        try {
            Handle(TColgp_HArray1OfPnt) poles =
                new TColgp_HArray1OfPnt(1, waypoints.size());
            for (int index = 0; index < waypoints.size(); ++index) {
                const auto& waypoint = waypoints.at(index);
                poles->SetValue(index + 1, gp_Pnt(waypoint.x, waypoint.y, waypoint.z));
            }
            GeomAPI_Interpolate interpolation(poles, Standard_False, 1e-5);
            interpolation.Perform();
            if (!interpolation.IsDone())
                return false;
            BRepBuilderAPI_MakeEdge edge(interpolation.Curve());
            if (!edge.IsDone())
                return false;
            builder.Add(compound, edge.Edge());
            ++addedEdges;
            return true;
        } catch (const Standard_Failure& failure) {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "cam.travel: failed to interpolate preview edge: {}",
                     failure.GetMessageString() ? failure.GetMessageString() : "OpenCASCADE failure");
            return false;
        } catch (...) {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "cam.travel: unknown exception while interpolating preview edge");
            return false;
        }
    };

    const auto addArrow = [&](const gp_Pnt& p1, const gp_Pnt& p2) {
        // 每条完整过渡只画一个箭头。不能在每个离散弦段上重复绘制，
        // 否则平滑球面圆弧会被几十个 V 形标记覆盖。
        // 翼线长度固定 0.1mm，避免随段长缩放在不同尺度下视觉过大/过小。
        constexpr double kArrowLen = 0.1;
        const gp_Pnt mid(0.5 * (p1.X() + p2.X()),
                         0.5 * (p1.Y() + p2.Y()),
                         0.5 * (p1.Z() + p2.Z()));
        gp_Vec dir(p1, p2);
        if (dir.Magnitude() < 1e-6) return;
        dir.Normalize();
        // 选一个与 dir 不平行的轴作为参考，构造法线
        gp_Vec refAxis(0.0, 0.0, 1.0);
        if (std::abs(dir.Z()) > 0.9) refAxis = gp_Vec(1.0, 0.0, 0.0);
        gp_Vec side = dir.Crossed(refAxis);
        if (side.Magnitude() < 1e-6) return;
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
    };

    for (const Segment& segment : segments) {
        allVerified = allVerified && segment.verified;
        if (segment.waypoints.size() >= 2) {
            // Prefer one interpolated OCC Edge for the original nominal curve.
            // Segment lines remain a robust fallback for degenerate samples.
            // 中文翻译：原始空程曲线优先构造成一条 OCC Edge，并用虚线样式显示。
            if (!addCurveEdge(segment.waypoints)) {
                for (int i = 1; i < segment.waypoints.size(); ++i) {
                    const Segment::Waypoint& a = segment.waypoints.at(i - 1);
                    const Segment::Waypoint& b = segment.waypoints.at(i);
                    addEdge(gp_Pnt(a.x, a.y, a.z), gp_Pnt(b.x, b.y, b.z));
                }
            }
            const int middle = segment.waypoints.size() / 2;
            const Segment::Waypoint& before = segment.waypoints.at(middle - 1);
            const Segment::Waypoint& after = segment.waypoints.at(middle);
            addArrow(gp_Pnt(before.x, before.y, before.z),
                     gp_Pnt(after.x, after.y, after.z));
        }
    }
    if (addedEdges == 0) {
        for (int i = 1; i < segments.size(); ++i) {
            const Segment& prev = segments[i - 1];
            const Segment& next = segments[i];
            const gp_Pnt from(prev.ex, prev.ey, prev.ez);
            const gp_Pnt to(next.sx, next.sy, next.sz);
            addEdge(from, to);
            addArrow(from, to);
        }
    }
    if (addedEdges == 0) return;

    m_ais = new AIS_Shape(compound);

    // 蓝色虚线：覆盖 wire / line / 默认绘制属性
    const Quantity_Color color = allVerified
        ? Quantity_Color(0.20, 0.85, 1.0, Quantity_TOC_RGB)
        : Quantity_Color(1.0, 0.25, 0.2, Quantity_TOC_RGB);
    Handle(Prs3d_LineAspect) dash = new Prs3d_LineAspect(
        color,
        Aspect_TOL_DASH,
        2.5);
    const Handle(Prs3d_Drawer)& drawer = m_ais->Attributes();
    drawer->SetWireAspect(dash);
    drawer->SetLineAspect(dash);
    drawer->SetUnFreeBoundaryAspect(dash);
    drawer->SetFreeBoundaryAspect(dash);
    drawer->SetSeenLineAspect(dash);

    // Match the proven ToolpathRenderer display path, then keep the travel
    // overlay in an independent top layer so surface-coincident samples do not
    // disappear into the shaded workpiece because of depth fighting.
    m_ais->SetDisplayMode(AIS_WireFrame);
    ctx->Display(m_ais, AIS_WireFrame, 0, Standard_False);
    ctx->SetColor(m_ais, color, Standard_False);
    ctx->SetWidth(m_ais, 2.5, Standard_False);
    ctx->SetZLayer(m_ais, Graphic3d_ZLayerId_Topmost);
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
