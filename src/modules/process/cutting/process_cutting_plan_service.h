#pragma once

#include "core/kernel/i_service.h"
#include "core/project/cam/cam_data_contracts.h"
#include "core/project/cam/layer_contracts.h" // OCC-free: enums only
#include "modules/process/cutting/i_process_cutting_plan_provider.h"

#include <QColor>
#include <QHash>
#include <QObject>
#include <QSet>
#include <QString>
#include <QStringList>
#include <QVector>

#include <cstdint>
#include <memory>

namespace lcnc::cam {
class ICamToolpathProvider;
class ICamLayerProvider;
}

namespace lcnc::process {

/**
 * @brief 单条图层的工艺配置 —— 把图层和切割工具/顺序绑定在一起。
 *
 * Phase B：所有字段的"权威存储"已下沉到 CAM 端 LayerContainer/ToolpathLayer；
 * ProcessLayerJob 仍作为对外 DTO 由 ProcessCuttingPlanService 派发给 UI/命令使用，
 * 但 service 内部不再缓存 ProcessLayerJob 数组——每次取值都向 ICamLayerProvider
 * 拉一份最新快照。
 */
struct ProcessLayerJob
{
    std::uint64_t layerId{0};       ///< 跟 ToolpathLayer::layerId 对齐
    QString       layerName;        ///< 从 CAM 拷过来的镜像名，作为 fallback / 显示
    QString       toolName;         ///< 绑定的工具名（ToolFactory::GetTool 可识别）
    bool          enabled{true};    ///< 是否参与本次切割（独立于 CAM 的图层 enabled）
    int           order{0};         ///< @deprecated Phase B：保留字段供旧调用方读，不参与排序。
    QString       compensationIndex;///< 该图层默认的补偿索引（可被节点参数覆盖）
    /// 该图层中要参与切割的轮廓子集。
    /// 空 = 全选（兼容老配置 + 默认行为）；非空 = 仅包含集合内的 contourId。
    QSet<lcnc::cam::ContourId> includedContours;
};

/// Phase B：CAM 端定义为权威；Process 侧保留同名别名以最小化扩散修改。
using CuttingPlanSortStrategy = lcnc::cam::CuttingPlanSortStrategy;
using AutoSortAxis            = lcnc::cam::AutoSortAxis;

QString autoSortAxisToString(AutoSortAxis a);
AutoSortAxis autoSortAxisFromString(const QString& s, AutoSortAxis def = AutoSortAxis::XPos);

QString sortStrategyToString(CuttingPlanSortStrategy s);
CuttingPlanSortStrategy sortStrategyFromString(const QString& s,
                                               CuttingPlanSortStrategy def = CuttingPlanSortStrategy::LayerThenContour);

/**
 * @brief Process 模块的项目级"加工链表管理服务"。
 *
 * Phase B 起：本服务**不再持有任何图层级状态**，所有 toolName/enabled/manual
 * order/sort strategy/included contours 都从 ICamLayerProvider 拉取或写回，
 * 切割链表的构建逻辑（buildCuttingList）仍留在 Process 侧，但只是 CAM 视图。
 *
 * 持久化策略：本服务不写任何项目文件。CAM 工艺/排序状态由 core 的 cam_toolpath_io
 * 随 .lcnc 统一读写；v1 旧档 `process_cutting_plan.toml` 的一次性迁移也已移入 core。
 */
class ProcessCuttingPlanService : public QObject,
                                  public lcnc::process::IProcessCuttingPlanProvider
{
    Q_OBJECT
public:
    explicit ProcessCuttingPlanService(QObject* parent = nullptr);
    ~ProcessCuttingPlanService() override;

    /// 注入 CAM 刀路源（由 ProcessModule 在 init 时设置）。
    void setToolpathProvider(std::shared_ptr<lcnc::cam::ICamToolpathProvider> provider);
    /// 注入 CAM 图层视图（Phase B 起作为权威存储入口）。
    void setLayerProvider(std::shared_ptr<lcnc::cam::ICamLayerProvider> provider);

    /// 当前所有图层工艺配置（按 layerId 顺序遍历的稳定快照）。
    QVector<ProcessLayerJob> layerJobs() const;
    /// 取单条；不存在时返回带 valid=false 形态？这里直接返回是否找到。
    bool layerJob(std::uint64_t layerId, ProcessLayerJob* out) const;

    /// 设置/覆盖单条工艺配置；写入后立即 emit planChanged()。
    void setLayerJob(const ProcessLayerJob& job);
    /// 批量设置（如对话框 Apply 一次性写入）。
    void setLayerJobs(const QVector<ProcessLayerJob>& jobs);
    /// 清空所有配置（新建项目时调用）。Phase B：实际重置在 CAM 容器内进行。
    void clearAll();

    // ── 排序策略 ───────────────────────────────────────────────────────────
    CuttingPlanSortStrategy sortStrategy() const;
    void setSortStrategy(CuttingPlanSortStrategy s);

    // ── 与 CAM 同步 ────────────────────────────────────────────────────────
    /// Phase B：CAM 是唯一存储，本方法只触发一次 revision 自增 + planChanged。
    void syncFromCam();

    // ── 工具列表（供 UI 下拉用）────────────────────────────────────────────
    QStringList availableToolNames() const;

    // ── 切割链表生成（供 NormalCuttingManager 用）──────────────────────────
    struct CuttingListEntry
    {
        lcnc::cam::ContourId contourId{0};
        std::uint64_t        layerId{0};
        QString              layerName;
        QString              contourName;          ///< 来自 ToolpathExportContour::contourName
        QString              toolName;             ///< 已根据 layer job 解析
        QString              compensationIndex;    ///< 已根据 layer job 解析
        int                  sequence{0};          ///< 1-based 序号，按当前策略 + 启用过滤后的位置
    };
    struct CuttingListFilter
    {
        int startSequence{1};   ///< 1-based，<=0 视为 1
        int endSequence{0};     ///< 0 = 不限
    };
    /// 按当前策略 + filter 生成一条已排序、已过滤、已绑工具的切割链表。
    QVector<CuttingListEntry> buildCuttingList(const CuttingListFilter& filter = {}) const;

    // ── 手动轮廓顺序（Manual 策略生效）─────────────────────────────────────
    QVector<lcnc::cam::ContourId> manualContourOrder() const;
    void setManualContourOrder(const QVector<lcnc::cam::ContourId>& ids);
    int  appendToManualOrder(const QVector<lcnc::cam::ContourId>& ids);
    void removeFromManualOrder(const QVector<lcnc::cam::ContourId>& ids);
    void clearManualOrder();

    // ── 自动排序 ──────────────────────────────────────────────────────────
    bool applyAutoSort(AutoSortAxis axis, QString* errorMessage = nullptr);

    AutoSortAxis lastAutoSortAxis() const;
    void         setLastAutoSortAxis(AutoSortAxis a);

    // ── IProcessCuttingPlanProvider 实现 ──────────────────────────────────
    QVector<lcnc::cam::ContourId> orderedContourIds() const override;
    std::uint64_t                 planRevision() const override { return m_planRevision; }

    /// 列出某图层下的全部轮廓（来自 CAM snapshot），供 UI 渲染轮廓复选行。
    struct ContourBrief
    {
        lcnc::cam::ContourId contourId{0};
        QString name;
        bool enabledInCam{true};   ///< 来自 ToolpathExportContour::enabled
    };
    QVector<ContourBrief> contoursInLayer(std::uint64_t layerId) const;

    // 项目持久化已全部下沉到 core（lcnc::cam::saveCamToolpath / loadCamToolpath /
    // migrateLegacyProcessCuttingPlan，由 LcncProjectManager 统一调度）。本服务不再做项目文件 IO。

    /// 供 ProcessModule 把 CAM LayerManager 的 Qt 信号桥接到本服务的
    /// planChanged/manualOrderChanged。仅由 ProcessModule 调用。
    void notifyExternalPlanChanged();
    void notifyExternalManualOrderChanged();

signals:
    /// 配置发生变化（工具映射 / 顺序 / 策略 / 同步），UI 与 manager 据此刷新。
    void planChanged();
    /// 仅手动顺序变化时单独发；UI 想细粒度只刷右表的话可订这个。
    void manualOrderChanged();

private:
    void bumpRevisionAndNotify(bool manualOnly = false);
    void wireLayerProviderSignals();

    /// 单调递增；任何变化（plan/manual/strategy/sync/load）都自增。
    std::uint64_t m_planRevision{0};

    std::shared_ptr<lcnc::cam::ICamToolpathProvider> m_provider;
    std::shared_ptr<lcnc::cam::ICamLayerProvider>    m_layerProvider;
};

} // namespace lcnc::process
