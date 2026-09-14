#pragma once

/**
 * @file contour_order_planner.h
 * @brief 按"主方向 + 桶内最近邻贪心"规划轮廓加工顺序。
 *
 * 输入：每条轮廓的 id 与世界坐标系下的起点/末点（OCC-free）。
 * 输出：一份有序 contourId 列表。
 *
 * 算法语义：
 *   1. 按主方向（X+/X-/Y+/...）的标量函数 primary(start) 升序粗排；
 *   2. 按 `primaryBucketTolMm` 将连续主坐标相近的轮廓分到同一桶；
 *   3. 桶内：以与"上一桶末点"距离最近的轮廓为种子，之后每次贪心
 *      选取起点距当前末点最近的剩余轮廓。
 */

#include <QVector>
#include <cstdint>

namespace lcnc::cam {

enum class PrimaryAxis { XPos, XNeg, YPos, YNeg, ZPos, ZNeg };

struct ContourEndpoints
{
    std::uint64_t id{0};
    double sx{0.0}, sy{0.0}, sz{0.0};   ///< 起点（lead-in 起点优先）
    double ex{0.0}, ey{0.0}, ez{0.0};   ///< 末点
};

struct ContourOrderParams
{
    PrimaryAxis axis{PrimaryAxis::XPos};
    double      primaryBucketTolMm{0.5};
};

QVector<std::uint64_t> planContourOrder(const QVector<ContourEndpoints>& inputs,
                                        const ContourOrderParams& params);

} // namespace lcnc::cam
