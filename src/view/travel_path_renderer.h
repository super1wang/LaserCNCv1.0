#pragma once

#include "core/project/cam/collision_validation_contracts.h"

/**
 * @file travel_path_renderer.h
 * @brief 在 OCC 视图里用虚线绘制相邻轮廓间空程（travel）路径的渲染器。
 *
 * 由 CAM 模块持有，每段虚线连接「上一轮廓真实末点 → 下一轮廓真实起点（lead-in 起点）」。
 * 数据契约：调用方传入一份"按加工顺序排好"的端点序列（OCC-free 结构体），
 * 渲染器内部构造 Edge → AIS_Shape，应用 dash 线型并 Display 到 GuiDocument。
 */

#include <QVector>
#include <QString>
#include <AIS_Shape.hxx>

#include <cstdint>

class GuiDocument;
class MachineKinematics;

namespace lcnc::view {

class TravelPathRenderer
{
public:
    /// 按加工顺序排列的一段工件局部坐标端点。
    /// startXYZ 已是"轮廓真实起点"（lead-in 起点优先），endXYZ 是轮廓末点。
    struct Segment
    {
        struct Waypoint
        {
            double x{0.0}, y{0.0}, z{0.0};
            /// Phase of the executable segment entering this waypoint.  The
            /// first waypoint has no incoming segment and keeps the default.
            lcnc::cam::RapidSegmentPhase incomingPhase{
                lcnc::cam::RapidSegmentPhase::Traverse};
        };
        std::uint64_t contourId{0};
        QString workpieceEntry;
        double sx{0.0}, sy{0.0}, sz{0.0};
        double ex{0.0}, ey{0.0}, ez{0.0};
        bool verified{true};
        /// When provided, render the collision-verified rapid polyline rather
        /// than the historical straight endpoint connection. Coordinates are
        /// workpiece-local and follow workpieceEntry during transform refresh.
        QVector<Waypoint> waypoints;
    };

    TravelPathRenderer();
    ~TravelPathRenderer();

    /// 设置可见性。OFF 时会立即擦除 AIS，ON 时仅置标志（实际构建在 refresh）。
    void setVisible(bool on);
    bool isVisible() const { return m_visible; }

    /// 用传入的 segments 重建虚线 AIS 并 Display 到 gd。
    /// segments 为空时只 erase；一对轮廓也必须显示其唯一的空程路径。
    /// visible == false 时只 erase。
    void refresh(GuiDocument* gd, const QVector<Segment>& segments);

    /// 坐标变化时只更新整体局部变换，不重建 AIS。
    void updateTransforms(GuiDocument* gd, MachineKinematics* kin);

    /// 强制擦除当前 AIS（gd 仍持有 ctx 时调用）。
    void erase(GuiDocument* gd);

private:
    bool                m_visible{false};
    Handle(AIS_Shape)   m_ais;
    QString             m_workpieceEntry;
};

} // namespace lcnc::view
