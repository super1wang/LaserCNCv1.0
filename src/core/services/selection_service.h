#pragma once

/**
 * @file selection_service.h
 * @brief 跨模块共享的"轮廓选择顺序"记录服务。
 *
 * 应用中两类 UI 都会选择 CAM 轮廓：
 *   - 主窗口左侧 Project Explorer 树（QTreeWidget，ExtendedSelection）；
 *   - CAM 模块的 OCC 视图拾取。
 *
 * 两边的"已选集合"在用户交互过程中此起彼伏，但本服务只关心一件事：
 *   - 把每次"新被选中"的 contourId 按时间顺序追加到内部序列，已选过的不重排；
 *   - 取消选中时从序列中移除。
 *
 * 这样 CAM 的手动加工顺序命令就能拿到一份带顺序的
 * contourId 列表，按"先点的先加工"语义写入 CAM 图层顺序容器。
 *
 * 本服务位于 core 层：
 *   - app/main_window、modules/cam、modules/process 三方都允许读写；
 *   - 仅依赖 std::uint64_t / QString / Qt 信号，不含 OCC，符合 core 层约束。
 */

#include <QObject>
#include <QString>
#include <QVector>
#include <QHash>

#include <cstdint>

#include "core/kernel/i_service.h"

namespace lcnc::core {

struct SelectionEntry
{
    std::uint64_t contourId{0};   ///< 0 表示非轮廓选中（图层、工件 entry 等），本服务忽略
    QString       entry;          ///< OCC entry 或 Project Explorer NodeKey（可选，调试/反查用）
    qint64        sequence{0};    ///< 单调递增的选中编号，由服务内部分配

    enum Source { ProjectExplorer, OccView, External };
    Source        source{External};
};

class SelectionService : public QObject, public lcnc::IService
{
    Q_OBJECT
public:
    explicit SelectionService(QObject* parent = nullptr);
    ~SelectionService() override = default;

    // ── 写入 ────────────────────────────────────────────────────────────────
    /// 记录一条新选中。已存在则保留原有顺序（不重排），并发出 selectionChanged。
    /// contourId == 0 的条目被静默忽略。
    void recordSelected(const SelectionEntry& entry);
    /// 批量记录，按数组顺序逐条追加，最后只 emit 一次 selectionChanged。
    void recordSelectedBatch(const QVector<SelectionEntry>& entries);

    /// 移除某个 contourId（若不存在则忽略）。
    void removeContour(std::uint64_t contourId);

    /// 清空所有选择。默认清全部；传入 source 时仅清来自该来源的条目。
    void clear(SelectionEntry::Source source = SelectionEntry::External);

    // ── 读 ──────────────────────────────────────────────────────────────────
    QVector<std::uint64_t>      contoursInSelectionOrder() const;
    QVector<SelectionEntry>     allEntriesInOrder() const { return m_entries; }
    int                         contourSelectionCount() const { return m_entries.size(); }
    bool                        isEmpty() const { return m_entries.isEmpty(); }

signals:
    void selectionChanged();

private:
    QVector<SelectionEntry>                 m_entries;       ///< 保序
    QHash<std::uint64_t, int>               m_indexById;     ///< contourId → m_entries 下标
    qint64                                  m_nextSequence{1};
};

} // namespace lcnc::core
