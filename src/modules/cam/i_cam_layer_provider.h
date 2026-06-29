#pragma once

#include "core/kernel/i_service.h"
#include "core/project/cam/cam_data_contracts.h"
#include "core/project/cam/layer_contracts.h"

#include <QObject>
#include <QString>
#include <QSet>
#include <QVector>

#include <cstdint>

namespace lcnc::cam {

/**
 * @brief OCC-free 图层视图与可变接口，跨模块共享。
 *
 * 用途：Process 模块的切割链表服务不允许包含 OCC 类型，但需要访问 CAM 的
 * 图层级状态（toolName / enabled / 人工顺序 / 排序策略 / 包含轮廓子集）。
 * 自 Phase B 起，这些状态由 LayerContainer 统一持有；ICamLayerProvider 在
 * CamModule::init() 中以 IService 形式注册，Process 端通过 ServiceRegistry
 * 取到后做"读写两面"。
 *
 * 信号源是 LayerManager（Qt 信号），但本接口本身保持 IService 纯虚——
 * 实现端可选择把 LayerManager 暴露出来供高级订阅，或仅靠 revision() 拉式刷新。
 *
 * 不允许将本接口下沉到 process/runtime —— 它只能在 process 与 cam 的"边界层"
 * （例如 ProcessCuttingPlanService）使用。
 */

/// 图层的只读快照（OCC-free），供 Process 端做链表构建。
struct LayerSnapshot
{
    std::uint64_t       layerId{0};
    QString             name;
    QString             toolName;
    QString             compensationIndex;
    bool                enabled{true};
    QSet<ContourId>     includedContours; ///< 空 = 全选；非空 = 显式子集。
    QVector<ContourId>  contourIds;       ///< 该图层下的轮廓 id（按 CAM 顺序）。
};

class ICamLayerProvider : public lcnc::IService
{
public:
    ~ICamLayerProvider() override = default;

    // ── 只读 ──────────────────────────────────────────────────────────────
    virtual QVector<LayerSnapshot>      layers() const = 0;
    virtual QVector<ContourId>          manualContourOrder() const = 0;
    virtual CuttingPlanSortStrategy     sortStrategy() const = 0;
    virtual AutoSortAxis                lastAutoSortAxis() const = 0;
    /// 每次任何 mutator 成功执行 revision 自增；UI/订阅方据此判断"快照失效"。
    virtual std::uint64_t               revision() const = 0;

    // ── 可变 ──────────────────────────────────────────────────────────────
    virtual void setLayerToolName(std::uint64_t layerId, const QString& toolName) = 0;
    virtual void setLayerEnabled(std::uint64_t layerId, bool enabled) = 0;
    virtual void setLayerCompensationIndex(std::uint64_t layerId, const QString& index) = 0;
    virtual void setLayerIncludedContours(std::uint64_t layerId, const QSet<ContourId>& included) = 0;

    virtual void setManualContourOrder(const QVector<ContourId>& ids) = 0;
    virtual int  appendToManualOrder(const QVector<ContourId>& ids) = 0;
    virtual void removeFromManualOrder(const QVector<ContourId>& ids) = 0;
    virtual void clearManualOrder() = 0;

    virtual void setSortStrategy(CuttingPlanSortStrategy s) = 0;
    virtual void setLastAutoSortAxis(AutoSortAxis a) = 0;

    /// Qt 信号源；订阅方可挂细粒度信号。可能返回 nullptr（实现可选）。
    virtual QObject* notifier() const { return nullptr; }
};

} // namespace lcnc::cam
