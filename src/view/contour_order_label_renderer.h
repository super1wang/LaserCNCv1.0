#pragma once

/**
 * @file contour_order_label_renderer.h
 * @brief 在 OCC 视图里按加工顺序在每条轮廓起点附近绘制序号文字标注的渲染器。
 *
 * 由 CAM 模块持有，与 TravelPathRenderer 同源：调用方传入一份"按加工顺序排好"
 * 的起点序列（OCC-free 结构体），渲染器为每条轮廓构造一个 AIS_TextLabel 并
 * Display 到 GuiDocument。坐标变化时通过 computeWpcTransform 重新计算每个标签
 * 的世界坐标并 SetPosition，使序号随工件轴系姿态一同移动（AIS_TextLabel 不响应
 * SetLocalTransformation，故不能用虚线渲染器的变换矩阵方案）。
 */

#include <QVector>
#include <QString>
#include <AIS_TextLabel.hxx>
#include <gp_Pnt.hxx>

#include <cstdint>

class GuiDocument;
class MachineKinematics;

namespace lcnc::view {

class ContourOrderLabelRenderer
{
public:
    /// 按加工顺序排列的一条轮廓起点标注。
    /// sx/sy/sz 为轮廓真实起点（lead-in 起点优先），处于工件局部 WPC 坐标系。
    struct Label
    {
        std::uint64_t contourId{0};
        QString workpieceEntry;
        double sx{0.0}, sy{0.0}, sz{0.0};
        int order{0}; ///< 1 起的加工序号
    };

    ContourOrderLabelRenderer();
    ~ContourOrderLabelRenderer();

    /// 设置可见性。OFF 时立即擦除 AIS，ON 时仅置标志（实际构建在 refresh）。
    void setVisible(bool on);
    bool isVisible() const { return m_visible; }

    /// 用传入的 labels 重建文字 AIS 并 Display 到 gd。
    /// labels 为空时只 erase 不重建。visible == false 时只 erase。
    /// kin 用于把每条轮廓的局部起点变换到世界坐标（AIS_TextLabel 不响应
    /// SetLocalTransformation，故直接用 SetPosition 写入世界坐标）。
    void refresh(GuiDocument* gd, MachineKinematics* kin, const QVector<Label>& labels);

    /// 坐标变化时按各自 workpieceEntry 重新计算世界坐标并 SetPosition，不重建 AIS。
    void updateTransforms(GuiDocument* gd, MachineKinematics* kin);

    /// 强制擦除当前所有 AIS（gd 仍持有 ctx 时调用）。
    void erase(GuiDocument* gd);

private:
    struct Entry
    {
        Handle(AIS_TextLabel) ais;
        QString workpieceEntry;
        gp_Pnt localPoint;
    };

    bool           m_visible{false};
    QVector<Entry> m_entries;
};

} // namespace lcnc::view
