#pragma once

#include "core/project/cam/layer_container.h"

#include <QObject>

#include <cstdint>

namespace lcnc::cam {

/**
 * @brief LayerContainer 的 Qt 信号外壳。
 *
 * 把 LayerContainer 的可变操作以"通知"接口暴露给 UI / Process / GuiDocument，
 * 取代原来分散的 CamModule::toolpathLayersChanged 单一信号。
 *
 * 调用方式（CAM 内部 mutator）：
 *     bool changed = container.setLayerName(layerId, "outer");
 *     if (changed) layerManager.emitLayerPropertyChanged(layerId, LayerProperty::Name);
 *
 * Phase A 仅把 CamDataManager 现有的 updateToolpathLayer/setToolpathLayerEnabled/
 * reorderContoursById/reorderContours 改为走容器+信号；外部调用方暂时仍订阅
 * CamModule::toolpathLayersChanged 以保持兼容，Phase B 起再逐步迁移到细粒度信号。
 */
class LayerManager : public QObject
{
    Q_OBJECT
public:
    explicit LayerManager(LayerContainer* container, QObject* parent = nullptr)
        : QObject(parent)
        , m_container(container)
    {}

    LayerContainer*       container()       { return m_container; }
    const LayerContainer* container() const { return m_container; }

    // ── 透传 mutator + 自动发信号（推荐统一从这里走）─────────────────────
    bool setLayerName(std::uint64_t layerId, const QString& name);
    bool setLayerColor(std::uint64_t layerId, const QColor& color);
    bool setLayerEnabled(std::uint64_t layerId, bool enabled);
    bool setLayerToolName(std::uint64_t layerId, const QString& toolName);
    bool setLayerCompensationIndex(std::uint64_t layerId, const QString& compensationIndex);
    bool setLayerIncludedContours(std::uint64_t layerId, const QSet<ContourId>& included);
    bool assignContourToLayer(ContourId contourId, std::uint64_t layerId);

    bool setManualContourOrder(const QVector<ContourId>& ids);
    int  appendToManualOrder(const QVector<ContourId>& ids);
    bool removeFromManualOrder(const QVector<ContourId>& ids);
    bool clearManualOrder();

    bool setSortStrategy(CuttingPlanSortStrategy s);

    // ── 给 CamDataManager 内部用 —— 复杂操作（重排/同步图层）发完信号 ───
    /// 图层列表整体重建（新建/清空 toolpath 等），通知订阅方做整树刷新。
    void emitLayersReset();
    /// 图层顺序变了。
    void emitLayersReordered();
    /// 单个属性变了。
    void emitLayerPropertyChanged(std::uint64_t layerId, LayerProperty p);
    /// 单个图层被加入/删除（用于增量节点更新）。
    void emitLayerAdded(std::uint64_t layerId);
    void emitLayerRemoved(std::uint64_t layerId);
    /// 轮廓在图层之间的归属或图层内顺序变了。
    void emitContourMembershipChanged();

signals:
    /// 图层池整体重建（生成 toolpath、清空 toolpath、loadFromDir 后）。
    void layersReset();
    void layerAdded(std::uint64_t layerId);
    void layerRemoved(std::uint64_t layerId);
    void layersReordered();
    void layerPropertyChanged(std::uint64_t layerId, lcnc::cam::LayerProperty property);
    void contourMembershipChanged();
    void manualContourOrderChanged();
    void sortStrategyChanged(lcnc::cam::CuttingPlanSortStrategy strategy);

private:
    LayerContainer* m_container{nullptr};
};

} // namespace lcnc::cam
