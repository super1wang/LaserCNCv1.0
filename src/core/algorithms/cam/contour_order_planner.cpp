#include "core/algorithms/cam/contour_order_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lcnc::cam {

namespace {

inline double primaryOf(const ContourEndpoints& c, PrimaryAxis axis)
{
    switch (axis) {
    case PrimaryAxis::XPos: return  c.sx;
    case PrimaryAxis::XNeg: return -c.sx;
    case PrimaryAxis::YPos: return  c.sy;
    case PrimaryAxis::YNeg: return -c.sy;
    case PrimaryAxis::ZPos: return  c.sz;
    case PrimaryAxis::ZNeg: return -c.sz;
    }
    return 0.0;
}

inline double distSq(double x1, double y1, double z1, double x2, double y2, double z2)
{
    const double dx = x1 - x2, dy = y1 - y2, dz = z1 - z2;
    return dx * dx + dy * dy + dz * dz;
}

} // namespace

QVector<std::uint64_t> planContourOrder(const QVector<ContourEndpoints>& inputs,
                                        const ContourOrderParams& params)
{
    QVector<std::uint64_t> result;
    if (inputs.isEmpty()) return result;
    result.reserve(inputs.size());

    if (inputs.size() == 1) {
        result.append(inputs.first().id);
        return result;
    }

    // 1. 按主坐标升序粗排
    QVector<ContourEndpoints> sorted = inputs;
    std::stable_sort(sorted.begin(), sorted.end(),
                     [axis = params.axis](const ContourEndpoints& a, const ContourEndpoints& b) {
                         const double aPrimary = primaryOf(a, axis);
                         const double bPrimary = primaryOf(b, axis);
                         if (aPrimary != bPrimary)
                             return aPrimary < bPrimary;
                         return a.id < b.id;
                     });

    const double tol = std::max(params.primaryBucketTolMm, 0.0);

    // 2. 分桶：相邻主坐标差 > tol 即开新桶
    QVector<QVector<ContourEndpoints>> buckets;
    {
        QVector<ContourEndpoints> cur;
        double lastPrimary = 0.0;
        for (int i = 0; i < sorted.size(); ++i) {
            const double p = primaryOf(sorted[i], params.axis);
            if (cur.isEmpty() || std::abs(p - lastPrimary) <= tol) {
                cur.append(sorted[i]);
            } else {
                buckets.append(cur);
                cur.clear();
                cur.append(sorted[i]);
            }
            lastPrimary = p;
        }
        if (!cur.isEmpty()) buckets.append(cur);
    }

    // 3. 桶内最近邻；桶种子优先选择距上一桶末点最近的
    bool haveCurrent = false;
    double curX = 0.0, curY = 0.0, curZ = 0.0;
    for (auto& bucket : buckets) {
        while (!bucket.isEmpty()) {
            int pick = 0;
            if (haveCurrent) {
                double bestD = std::numeric_limits<double>::infinity();
                for (int i = 0; i < bucket.size(); ++i) {
                    const double d = distSq(curX, curY, curZ,
                                            bucket[i].sx, bucket[i].sy, bucket[i].sz);
                    if (d < bestD || (d == bestD && bucket[i].id < bucket[pick].id)) {
                        bestD = d;
                        pick = i;
                    }
                }
            }
            // 否则用 bucket[0]（已按主坐标排序，主方向最前）
            const ContourEndpoints chosen = bucket[pick];
            bucket.removeAt(pick);
            result.append(chosen.id);
            curX = chosen.ex;
            curY = chosen.ey;
            curZ = chosen.ez;
            haveCurrent = true;
        }
    }

    return result;
}

} // namespace lcnc::cam
