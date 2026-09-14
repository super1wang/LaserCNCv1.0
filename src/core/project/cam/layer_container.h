#pragma once

#include "core/algorithms/cam/laser_toolpath.h"
#include "core/project/cam/cam_data_contracts.h"
#include "core/project/cam/layer_contracts.h"

#include <QColor>
#include <QList>
#include <QSet>
#include <QString>
#include <QVector>

#include <cstdint>

namespace lcnc::cam {

// 枚举搬迁至 core/project/cam/layer_contracts.h（OCC-free），供 process 端复用。
// 历史名字在本头里仍可见（来自 contracts include）。

/**
 * @brief 图层 + 轮廓 + 排序的"单一权威"存储。
 *
 * 在 Phase A 阶段，LayerContainer 是 LaserToolpath 上层的一层薄包装：
 *   - 图层级字段（name/color/enabled/toolName/compensationIndex/includedContours）
 *     直接操纵 ToolpathLayer 字段。
 *   - 容器级字段（manualContourOrder/sortStrategy）存在 LayerContainer。
 *
 * 这层包装的存在意义：把所有 mutator 集中到一处，配合 LayerManager 发出细粒度
 * 信号，以便：
 *   1. ProjectExplorer / 加工链表面板 / TravelPath 渲染器 等订阅方按属性精细刷新。
 *   2. Phase B 把 Process 端的 m_jobs/m_manualContourOrder/m_sortStrategy 下沉过来。
 *   3. Phase C 之后 AIS 直接以 ContourId 为键，UI 层不再读 XCAF。
 *
 * 线程：与 CamDataManager 同生命周期（CamModule 拥有），所有 mutator 假定主线程调用。
 */
class LayerContainer
{
public:
    LayerContainer() = default;

    /// 由 CamDataManager 在构造时把 m_toolpath 注入；不持有所有权。
    void attach(LaserToolpath* toolpath) { m_toolpath = toolpath; }

    LaserToolpath*       toolpath()       { return m_toolpath; }
    const LaserToolpath* toolpath() const { return m_toolpath; }

    // ── 查询 ──────────────────────────────────────────────────────────────
    /// 图层 id 按存储顺序遍历（即创建顺序，等同于 LaserToolpath::layers()）。
    QVector<std::uint64_t> orderedLayerIds() const;
    /// 找不到返回 nullptr。
    ToolpathLayer*         layer(std::uint64_t layerId);
    const ToolpathLayer*   layer(std::uint64_t layerId) const;
    /// 图层下当前的轮廓 id 列表（按 m_toolpath.contours() 顺序）。
    QList<ContourId>       contoursInLayer(std::uint64_t layerId) const;

    /// 容器级人工顺序（轮廓级，Manual 策略生效）。
    const QVector<ContourId>& manualContourOrder() const { return m_manualContourOrder; }
    bool                      isInManualOrder(ContourId id) const { return m_manualOrderSet.contains(id); }

    CuttingPlanSortStrategy sortStrategy() const { return m_sortStrategy; }

    // ── 可变（true=有变化，false=无效或无变化；调用方据此决定是否通知）─
    bool setLayerName(std::uint64_t layerId, const QString& name);
    bool setLayerColor(std::uint64_t layerId, const QColor& color);
    bool setLayerEnabled(std::uint64_t layerId, bool enabled);
    bool setLayerToolName(std::uint64_t layerId, const QString& toolName);
    bool setLayerCompensationIndex(std::uint64_t layerId, const QString& compensationIndex);
    bool setLayerIncludedContours(std::uint64_t layerId, const QSet<ContourId>& included);
    /// 把单个轮廓挪到指定图层；调用方需自己重算 syncLayerContourIds。
    bool assignContourToLayer(ContourId contourId, std::uint64_t layerId);

    /// 完全覆盖手动顺序（自动去 0 与去重，保留先后）；返回是否真的变化。
    bool setManualContourOrder(const QVector<ContourId>& ids);
    /// 追加：跳过 0 与已存在 id；返回真实新增的条数。
    int  appendToManualOrder(const QVector<ContourId>& ids);
    /// 从手动顺序中移除指定 id（若存在）；返回是否真的变化。
    bool removeFromManualOrder(const QVector<ContourId>& ids);
    bool clearManualOrder();

    bool setSortStrategy(CuttingPlanSortStrategy s);

private:
    LaserToolpath*           m_toolpath{nullptr};
    QVector<ContourId>       m_manualContourOrder;
    QSet<ContourId>          m_manualOrderSet;
    CuttingPlanSortStrategy  m_sortStrategy{CuttingPlanSortStrategy::LayerThenContour};
};

} // namespace lcnc::cam
