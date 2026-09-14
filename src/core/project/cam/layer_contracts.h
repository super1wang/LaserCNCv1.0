#pragma once

#include <cstdint>

namespace lcnc::cam {

/**
 * @brief 切割链表的排序策略（CAM 侧权威定义，OCC-free 头）。
 *
 * Phase A 引入；Phase B 起作为 lcnc::process::CuttingPlanSortStrategy 的别名源，
 * 避免在 process 与 cam 两侧维护双份定义。
 */
enum class CuttingPlanSortStrategy
{
    CamOrder,         ///< 维持 CAM 中轮廓的原始顺序
    LayerThenContour, ///< 按图层 id 再按轮廓 id（默认）
    ToolThenLayer,    ///< 按工具名再按图层
    Manual,           ///< 按 manualContourOrder 中 contourId 的位置（轮廓级手动顺序）
};

/**
 * @brief 自动排序时的主方向（与 Ribbon 下拉一一对应）。
 */
enum class AutoSortAxis { XPos, XNeg, YPos, YNeg, ZPos, ZNeg };

/**
 * @brief LayerManager 发出 layerPropertyChanged 时附带的属性枚举。
 */
enum class LayerProperty : std::uint8_t
{
    Name,
    Color,
    Enabled,
    ToolName,
    CompensationIndex,
    IncludedContours,
    ContourMembership, ///< 轮廓在图层之间的归属变化（assignContourToLayer）
    ManualOrder,
};

} // namespace lcnc::cam
