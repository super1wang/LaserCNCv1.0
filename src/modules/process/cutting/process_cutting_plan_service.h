#pragma once

#include "core/kernel/i_service.h"
#include "core/project/cam/cam_data_contracts.h"
#include "core/project/cam/layer_contracts.h" // OCC-free: enums only
#include "modules/cam/i_cam_tool_offset_provider.h"

#include <QColor>
#include <QHash>
#include <QObject>
#include <QSet>
#include <QString>
#include <QStringList>
#include <QVector>

#include <atomic>
#include <cstdint>
#include <memory>

namespace lcnc::cam {
class ICamToolpathProvider;
class ICamLayerProvider;
class ICamContourSequenceProvider;
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
    QString       compensationIndex;///< 该图层默认的补偿索引（可被节点参数覆盖）
    /// 该图层中要参与切割的轮廓子集。
    /// 空 = 全选（兼容老配置 + 默认行为）；非空 = 仅包含集合内的 contourId。
    QSet<lcnc::cam::ContourId> includedContours;
};

/**
 * @brief Process 模块的项目级工艺绑定和执行链表服务。
 *
 * Phase B 起：本服务**不再持有任何图层级状态**，所有 toolName/enabled/manual
 * included contours 都从 ICamLayerProvider 拉取或写回。CAM 是唯一的排序
 * 与空程规划边界；Process 只按 CAM 已确认的 contourId 序列附加工艺工具、
 * 补偿和执行范围。
 *
 * 持久化策略：本服务不写任何项目文件。CAM 工艺/排序状态由 core 的 cam_toolpath_io
 * 随 .lcnc 统一读写；v1 旧档 `process_cutting_plan.toml` 的一次性迁移也已移入 core。
 */
class ProcessCuttingPlanService : public QObject,
                                  public lcnc::cam::ICamToolOffsetProvider
{
    Q_OBJECT
public:
    explicit ProcessCuttingPlanService(QObject* parent = nullptr);
    ~ProcessCuttingPlanService() override;

    /// 注入 CAM 刀路源（由 ProcessModule 在 init 时设置）。
    void setToolpathProvider(std::shared_ptr<lcnc::cam::ICamToolpathProvider> provider);
    /// 注入 CAM 图层视图（Phase B 起作为权威存储入口）。
    void setLayerProvider(std::shared_ptr<lcnc::cam::ICamLayerProvider> provider);
    /// CAM owns the authoritative contour ordering; Process only enriches the
    /// sequence with execution-specific tool and compensation data.
    void setContourSequenceProvider(std::shared_ptr<lcnc::cam::ICamContourSequenceProvider> provider);

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

    // ── 与 CAM 同步 ────────────────────────────────────────────────────────
    /// CAM 变更后仅刷新 Process 的派生执行链表，不改变顺序或排序策略。
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
    QVector<CuttingListEntry> buildCuttingList(const CuttingListFilter& filter) const;
    QVector<CuttingListEntry> buildCuttingList() const
    {
        return buildCuttingList(CuttingListFilter{});
    }

    /// Process-only cache revision for tool/compensation bindings and the
    /// latest CAM sequence notification.  It is not a sorting revision and
    /// does not grant Process authority to alter CAM order.
    std::uint64_t planRevision() const noexcept { return m_planRevision.load(); }

    // ── CAM 工具显示桥接 ─────────────────────────────────────────────────
    bool rapidDisplayOffsetMm(const QString& toolName, double* offsetMm) const override;

    /// 列出某图层下的全部轮廓（来自 CAM snapshot），供 UI 渲染轮廓复选行。
    struct ContourBrief
    {
        lcnc::cam::ContourId contourId{0};
        QString name;
        bool enabledInCam{true};   ///< 来自 ToolpathExportContour::enabled
    };
    QVector<ContourBrief> contoursInLayer(std::uint64_t layerId) const;

    // Project persistence is owned by the core package services; this service performs no project file I/O.

    /// 供 ProcessModule 把 CAM LayerManager 的 Qt 信号桥接到本服务的
    /// planChanged。仅由 ProcessModule 调用。
    void notifyExternalPlanChanged();

signals:
    /// 配置发生变化（工具映射 / CAM 图层同步），UI 与 manager 据此刷新。
    void planChanged();

private:
    void bumpRevisionAndNotify();
    void wireLayerProviderSignals();

    /// 单调递增；工艺绑定或 CAM 序列通知变化时自增，用于 Process 执行缓存。
    std::atomic<std::uint64_t> m_planRevision{0};

    std::shared_ptr<lcnc::cam::ICamToolpathProvider> m_provider;
    std::shared_ptr<lcnc::cam::ICamLayerProvider>    m_layerProvider;
    std::shared_ptr<lcnc::cam::ICamContourSequenceProvider> m_sequenceProvider;
};

} // namespace lcnc::process
