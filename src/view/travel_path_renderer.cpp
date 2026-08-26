#include "view/travel_path_renderer.h"

#include "core/kinematics/machine_kinematics.h"
#include "core/logging/logger.h"
#include "view/gui_document.h"

#include <AIS_InteractiveContext.hxx>
#include <Aspect_TypeOfLine.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRep_Builder.hxx>
#include <GeomAPI_Interpolate.hxx>
#include <Graphic3d_ZLayerId.hxx>
#include <NCollection_HArray1.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Quantity_Color.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <algorithm>
#include <cmath>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

namespace lcnc::view {
namespace {

int collisionStateBucket(lcnc::cam::CollisionValidationState state)
{
    switch (state) {
    case lcnc::cam::CollisionValidationState::Safe:
    case lcnc::cam::CollisionValidationState::Disabled:
        return 0;
    case lcnc::cam::CollisionValidationState::Pending:
        return 1;
    case lcnc::cam::CollisionValidationState::Collision:
        return 3;
    case lcnc::cam::CollisionValidationState::Warning:
    case lcnc::cam::CollisionValidationState::Indeterminate:
    default:
        return 2;
    }
}

Quantity_Color collisionStateColor(int bucket)
{
    switch (bucket) {
    case 0: return Quantity_Color(0.20, 0.85, 1.0, Quantity_TOC_RGB);
    case 1: return Quantity_Color(0.55, 0.58, 0.62, Quantity_TOC_RGB);
    case 2: return Quantity_Color(1.0, 0.68, 0.12, Quantity_TOC_RGB);
    case 3: return Quantity_Color(1.0, 0.20, 0.16, Quantity_TOC_RGB);
    default: return Quantity_Color(0.55, 0.58, 0.62, Quantity_TOC_RGB);
    }
}

} // namespace

TravelPathRenderer::TravelPathRenderer() = default;
TravelPathRenderer::~TravelPathRenderer() = default;

void TravelPathRenderer::setVisible(bool on)
{
    m_visible = on;
}

void TravelPathRenderer::erase(GuiDocument* gd)
{
    if (gd) {
        const Handle(AIS_InteractiveContext)& ctx = gd->context();
        if (!ctx.IsNull()) {
            for (auto& ais : m_aisByState) {
                if (!ais.IsNull())
                    ctx->Erase(ais, false);
            }
        }
    }
    for (auto& ais : m_aisByState)
        ais.Nullify();
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
    for (auto& ais : m_aisByState) {
        if (!ais.IsNull()) {
            ctx->Erase(ais, false);
            ais.Nullify();
        }
    }

    // Build one compound per safety state so a single unknown cutting edge no
    // longer paints every unrelated rapid transition red.
    std::array<TopoDS_Compound, 4> compounds;
    std::array<BRep_Builder, 4> builders;
    for (int bucket = 0; bucket < 4; ++bucket)
        builders[bucket].MakeCompound(compounds[bucket]);

    std::array<int, 4> addedEdges{};
    const auto addEdge = [&](const gp_Pnt& p1, const gp_Pnt& p2,
                             lcnc::cam::CollisionValidationState state) {
        const double segLen = p1.Distance(p2);
        if (segLen < 1e-6) return; // 同一点，跳过
        BRepBuilderAPI_MakeEdge mk(p1, p2);
        if (!mk.IsDone()) return;
        const int bucket = collisionStateBucket(state);
        builders[bucket].Add(compounds[bucket], mk.Edge());
        ++addedEdges[bucket];
    };

    const auto addCurveEdge = [&](const QVector<Segment::Waypoint>& waypoints,
                                  lcnc::cam::CollisionValidationState state) {
        if (waypoints.size() < 3)
            return false;
        try {
            occ::handle<NCollection_HArray1<gp_Pnt>> poles =
                new NCollection_HArray1<gp_Pnt>(1, waypoints.size());
            for (int index = 0; index < waypoints.size(); ++index) {
                const auto& waypoint = waypoints.at(index);
                poles->SetValue(index + 1, gp_Pnt(waypoint.x, waypoint.y, waypoint.z));
            }
            GeomAPI_Interpolate interpolation(poles, false, 1e-5);
            interpolation.Perform();
            if (!interpolation.IsDone())
                return false;
            BRepBuilderAPI_MakeEdge edge(interpolation.Curve());
            if (!edge.IsDone())
                return false;
            const int bucket = collisionStateBucket(state);
            builders[bucket].Add(compounds[bucket], edge.Edge());
            ++addedEdges[bucket];
            return true;
        } catch (const Standard_Failure& failure) {
            LCNC_ERR(lcnc::LogCode::Generic, "cam.travel: failed to interpolate preview edge: {}",
                     failure.what() ? failure.what() : "OpenCASCADE failure");
            return false;
        } catch (...) {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "cam.travel: unknown exception while interpolating preview edge");
            return false;
        }
    };

    const auto addArrow = [&](const gp_Pnt& p1, const gp_Pnt& p2,
                              lcnc::cam::CollisionValidationState state) {
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
        const int bucket = collisionStateBucket(state);
        if (mkA.IsDone()) builders[bucket].Add(compounds[bucket], mkA.Edge());
        if (mkB.IsDone()) builders[bucket].Add(compounds[bucket], mkB.Edge());
    };

    for (const Segment& segment : segments) {
        if (segment.waypoints.size() >= 2) {
            QVector<Segment::Waypoint> traverseWaypoints;
            QVector<Segment::Waypoint> arrowWaypoints;
            auto traverseState = lcnc::cam::CollisionValidationState::Pending;
            auto arrowState = lcnc::cam::CollisionValidationState::Pending;
            const auto flushTraverse = [&]() {
                if (traverseWaypoints.size() < 2) {
                    traverseWaypoints.clear();
                    return;
                }
                // Only the traverse phase is interpolated. Retract and
                // approach are executable normal-only moves and must remain
                // straight in the preview.
                // 中文翻译：仅空程段做曲线插值；上升与下降是沿局部法线的执行直线。
                if (!addCurveEdge(traverseWaypoints, traverseState)) {
                    for (int index = 1; index < traverseWaypoints.size(); ++index) {
                        const auto& a = traverseWaypoints.at(index - 1);
                        const auto& b = traverseWaypoints.at(index);
                        addEdge(gp_Pnt(a.x, a.y, a.z),
                                gp_Pnt(b.x, b.y, b.z), traverseState);
                    }
                }
                if (traverseWaypoints.size() > arrowWaypoints.size()) {
                    arrowWaypoints = traverseWaypoints;
                    arrowState = traverseState;
                }
                traverseWaypoints.clear();
            };
            for (int index = 1; index < segment.waypoints.size(); ++index) {
                const auto& previous = segment.waypoints.at(index - 1);
                const auto& current = segment.waypoints.at(index);
                if (current.incomingPhase == lcnc::cam::RapidSegmentPhase::Traverse) {
                    if (!traverseWaypoints.isEmpty()
                        && current.collisionState != traverseState) {
                        flushTraverse();
                    }
                    if (traverseWaypoints.isEmpty()) {
                        traverseWaypoints.append(previous);
                        traverseState = current.collisionState;
                    }
                    traverseWaypoints.append(current);
                    continue;
                }
                flushTraverse();
                addEdge(gp_Pnt(previous.x, previous.y, previous.z),
                        gp_Pnt(current.x, current.y, current.z),
                        current.collisionState);
            }
            flushTraverse();
            if (arrowWaypoints.size() >= 2) {
                const int middle = arrowWaypoints.size() / 2;
                const auto& before = arrowWaypoints.at(middle - 1);
                const auto& after = arrowWaypoints.at(middle);
                addArrow(gp_Pnt(before.x, before.y, before.z),
                         gp_Pnt(after.x, after.y, after.z), arrowState);
            }
        }
    }
    const auto totalAddedEdges = [&] {
        return addedEdges[0] + addedEdges[1] + addedEdges[2] + addedEdges[3];
    };
    if (totalAddedEdges() == 0) {
        for (int i = 1; i < segments.size(); ++i) {
            const Segment& prev = segments[i - 1];
            const Segment& next = segments[i];
            const gp_Pnt from(prev.ex, prev.ey, prev.ez);
            const gp_Pnt to(next.sx, next.sy, next.sz);
            addEdge(from, to, next.collisionState);
            addArrow(from, to, next.collisionState);
        }
    }
    if (totalAddedEdges() == 0) return;

    for (int bucket = 0; bucket < 4; ++bucket) {
        if (addedEdges[bucket] == 0)
            continue;
        const Quantity_Color color = collisionStateColor(bucket);
        Handle(Prs3d_LineAspect) dash = new Prs3d_LineAspect(
            color, Aspect_TOL_DASH, 2.5);
        Handle(AIS_Shape) ais = new AIS_Shape(compounds[bucket]);
        const Handle(Prs3d_Drawer)& drawer = ais->Attributes();
        drawer->SetWireAspect(dash);
        drawer->SetLineAspect(dash);
        drawer->SetUnFreeBoundaryAspect(dash);
        drawer->SetFreeBoundaryAspect(dash);
        drawer->SetSeenLineAspect(dash);
        ais->SetDisplayMode(AIS_WireFrame);
        ctx->Display(ais, AIS_WireFrame, 0, false);
        ctx->SetColor(ais, color, false);
        ctx->SetWidth(ais, 2.5, false);
        ctx->SetZLayer(ais, Graphic3d_ZLayerId_Topmost);
        ctx->Deactivate(ais);
        m_aisByState[bucket] = ais;
    }
    updateTransforms(gd, nullptr);
}

void TravelPathRenderer::updateTransforms(GuiDocument* gd, MachineKinematics* kin)
{
    if (!gd)
        return;

    gp_Trsf transform;
    if (kin && !m_workpieceEntry.isEmpty())
        transform = kin->computeWpcTransform(m_workpieceEntry);

    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    for (auto& ais : m_aisByState) {
        if (ais.IsNull())
            continue;
        ais->SetLocalTransformation(transform);
        if (!ctx.IsNull())
            ctx->RecomputePrsOnly(ais, false);
    }
}

} // namespace lcnc::view
