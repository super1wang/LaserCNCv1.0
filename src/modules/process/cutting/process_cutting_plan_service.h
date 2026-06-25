#pragma once

#include "core/kernel/i_service.h"
#include "modules/cam/contracts/cam_data_contracts.h"
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

namespace lcnc::cam { class ICamToolpathProvider; }

namespace lcnc::process {

/**
 * @brief 单条图层的工艺配置 —— 把图层和切割工具/顺序绑定在一起。
 *
 * 由 ProcessCuttingPlanService 维护，跟随 .lcnc 项目持久化。
 * 图层本身的"名字 + 颜色 + 启用"仍由 CAM 模块拥有；这里仅记录工艺层面的扩展。
 */
struct ProcessLayerJob
{
    std::uint64_t layerId{0};       ///< 跟 ToolpathLayer::layerId 对齐
    QString       layerName;        ///< 从 CAM 拷过来的镜像名，作为 fallback / 显示
    QString       toolName;         ///< 绑定的工具名（ToolFactory::GetTool 可识别）
    bool          enabled{true};    ///< 是否参与本次切割（独立于 CAM 的图层 enabled）
    int           order{0};         ///< 用户自定义顺序（仅 Manual 策略时生效）
    QString       compensationIndex;///< 该图层默认的补偿索引（可被节点参数覆盖）
    /// 该图层中要参与切割的轮廓子集。
    /// 空 = 全选（兼容老配置 + 默认行为）；非空 = 仅包含集合内的 contourId。
    QSet<lcnc::cam::ContourId> includedContours;
};

/**
 * @brief 切割链表的排序策略（与 ProcessToolpathSorter 平行，但允许"用户自定义"）。
 */
enum class CuttingPlanSortStrategy
{
    CamOrder,            ///< 维持 CAM 中轮廓的原始顺序
    LayerThenContour,    ///< 按图层 id 再按轮廓 id（默认）
    ToolThenLayer,       ///< 按工具名再按图层
    Manual,              ///< 按 m_manualContourOrder 中 contourId 的位置（轮廓级手动顺序）
};

/**
 * @brief 自动排序时的主方向（与 Ribbon 下拉一一对应）。
 */
enum class AutoSortAxis { XPos, XNeg, YPos, YNeg, ZPos, ZNeg };

QString autoSortAxisToString(AutoSortAxis a);
AutoSortAxis autoSortAxisFromString(const QString& s, AutoSortAxis def = AutoSortAxis::XPos);

QString sortStrategyToString(CuttingPlanSortStrategy s);
CuttingPlanSortStrategy sortStrategyFromString(const QString& s,
                                               CuttingPlanSortStrategy def = CuttingPlanSortStrategy::LayerThenContour);

/**
 * @brief Process 模块的项目级"加工链表管理服务"。
 *
 * 职责
 *   1. 维护图层 → 工具 / 启用 / 顺序 / 补偿索引 的映射（独立于 CAM）。
 *   2. 监听 CAM 图层变更，把新增图层入表、删除消失的、同步图层名。
 *   3. 暴露给 NormalCuttingManager 的工艺数据访问 API。
 *   4. 提供工具名候选列表（来自 ToolFactory）。
 *   5. 持久化到 .lcnc 包内的 process_cutting_plan.toml。
 *
 * 设计要点
 *   - 不依赖 OCC 类型，不引用 ToolpathLayer 自身的 toolName 字段。
 *   - syncFromCam() 调用安全：保留已有映射，对新出现 layerId 创建默认 ProcessLayerJob。
 *   - planChanged() 信号用于 UI 与 NormalCuttingManager 间的弱耦合。
 *
 * 生命周期
 *   ProcessModule::init() 时实例化、注册到 Kernel ServiceRegistry，
 *   订阅 CamModule::toolpathLayersChanged 信号自动同步。
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

    /// 当前所有图层工艺配置（按 layerId 顺序遍历的稳定快照）。
    QVector<ProcessLayerJob> layerJobs() const;
    /// 取单条；不存在时返回带 valid=false 形态？这里直接返回是否找到。
    bool layerJob(std::uint64_t layerId, ProcessLayerJob* out) const;

    /// 设置/覆盖单条工艺配置；写入后立即 emit planChanged()。
    void setLayerJob(const ProcessLayerJob& job);
    /// 批量设置（如对话框 Apply 一次性写入）。
    void setLayerJobs(const QVector<ProcessLayerJob>& jobs);
    /// 清空所有配置（新建项目时调用）。
    void clearAll();

    // ── 排序策略 ───────────────────────────────────────────────────────────
    CuttingPlanSortStrategy sortStrategy() const { return m_sortStrategy; }
    void setSortStrategy(CuttingPlanSortStrategy s);

    // ── 与 CAM 同步 ────────────────────────────────────────────────────────
    /// 拉取最新 CAM 刀路快照，对每个图层创建/同步 ProcessLayerJob；
    /// 已删除的图层会被清理。空 jobs 表示从未配置；同步后会保留已有的工具/顺序/启用。
    void syncFromCam();

    // ── 工具列表（供 UI 下拉用）────────────────────────────────────────────
    /// 返回当前 ToolFactory 中已注册的工具名列表（按 index 顺序）。
    /// 始终至少包含一个空字符串 "" 表示"未指定"。
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
    /// 直接读取当前手动顺序列表。
    const QVector<lcnc::cam::ContourId>& manualContourOrder() const { return m_manualContourOrder; }
    /// 完全覆盖手动顺序。
    void setManualContourOrder(const QVector<lcnc::cam::ContourId>& ids);
    /// 追加：跳过 0 与已存在 id，保留追加先后。返回真实新增的条数。
    int  appendToManualOrder(const QVector<lcnc::cam::ContourId>& ids);
    /// 从手动顺序中移除指定 id（若存在）。
    void removeFromManualOrder(const QVector<lcnc::cam::ContourId>& ids);
    /// 清空手动顺序。
    void clearManualOrder();

    // ── 自动排序 ──────────────────────────────────────────────────────────
    /// 按 axis 的主方向 + 最近邻贪心，从当前 snapshot 生成手动顺序，
    /// 同时把 sortStrategy 切到 Manual。成功返回 true。
    bool applyAutoSort(AutoSortAxis axis, QString* errorMessage = nullptr);

    /// 记忆最后一次 Ribbon 上选择的轴模式（仅用于 UI 状态恢复）。
    AutoSortAxis lastAutoSortAxis() const { return m_lastAutoSortAxis; }
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

    // ── 持久化 ─────────────────────────────────────────────────────────────
    /// 文件名（位于项目包根目录），LcncProjectPackage 自动 zip 进 .lcnc。
    static QString kPlanFileName();
    /// 将当前状态写入指定目录下的 process_cutting_plan.toml。
    bool saveToProjectDir(const QString& packageDir, QString* errorMessage = nullptr) const;
    /// 从指定目录读取 process_cutting_plan.toml；文件不存在视为"项目无配置"（返回 true）。
    bool loadFromProjectDir(const QString& packageDir, QString* errorMessage = nullptr);

signals:
    /// 配置发生变化（工具映射 / 顺序 / 策略 / 同步），UI 与 manager 据此刷新。
    void planChanged();
    /// 仅手动顺序变化时单独发；UI 想细粒度只刷右表的话可订这个。
    void manualOrderChanged();

private:
    void bumpRevisionAndNotify(bool manualOnly = false);

    QHash<std::uint64_t, ProcessLayerJob> m_jobs;
    QVector<std::uint64_t>                m_layerOrder;   ///< 图层稳定顺序（CAM 中第一次出现的次序）
    CuttingPlanSortStrategy               m_sortStrategy{CuttingPlanSortStrategy::LayerThenContour};

    /// Manual 策略下的轮廓顺序；m_manualOrderSet 是 O(1) 查重镜像。
    QVector<lcnc::cam::ContourId>         m_manualContourOrder;
    QSet<lcnc::cam::ContourId>            m_manualOrderSet;

    /// Ribbon 自动排序下拉的最后一次选择（仅 UI 恢复用）。
    AutoSortAxis                          m_lastAutoSortAxis{AutoSortAxis::XPos};

    /// 单调递增；任何变化（plan/manual/strategy/sync/load）都自增。
    std::uint64_t                         m_planRevision{0};

    std::shared_ptr<lcnc::cam::ICamToolpathProvider> m_provider;
};

} // namespace lcnc::process
