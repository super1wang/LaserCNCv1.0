#include "core/services/selection_service.h"

namespace lcnc::core {

SelectionService::SelectionService(QObject* parent)
    : QObject(parent)
{
}

void SelectionService::recordSelected(const SelectionEntry& entry)
{
    if (entry.contourId == 0) return;
    if (m_indexById.contains(entry.contourId)) return; // 已选过，保持原顺序

    SelectionEntry stored = entry;
    stored.sequence = m_nextSequence++;
    m_indexById.insert(stored.contourId, m_entries.size());
    m_entries.append(stored);
    emit selectionChanged();
}

void SelectionService::recordSelectedBatch(const QVector<SelectionEntry>& entries)
{
    bool changed = false;
    for (const SelectionEntry& e : entries) {
        if (e.contourId == 0) continue;
        if (m_indexById.contains(e.contourId)) continue;
        SelectionEntry stored = e;
        stored.sequence = m_nextSequence++;
        m_indexById.insert(stored.contourId, m_entries.size());
        m_entries.append(stored);
        changed = true;
    }
    if (changed) emit selectionChanged();
}

void SelectionService::removeContour(std::uint64_t contourId)
{
    auto it = m_indexById.find(contourId);
    if (it == m_indexById.end()) return;
    const int idx = it.value();
    m_indexById.erase(it);
    m_entries.removeAt(idx);
    // 重建索引（O(n)，规模小，可接受）
    for (int i = idx; i < m_entries.size(); ++i)
        m_indexById[m_entries[i].contourId] = i;
    emit selectionChanged();
}

void SelectionService::clear(SelectionEntry::Source source)
{
    if (m_entries.isEmpty()) return;
    bool changed = false;
    if (source == SelectionEntry::External) {
        // 全清
        m_entries.clear();
        m_indexById.clear();
        changed = true;
    } else {
        QVector<SelectionEntry> kept;
        kept.reserve(m_entries.size());
        for (const auto& e : m_entries) {
            if (e.source == source) {
                changed = true;
                continue;
            }
            kept.append(e);
        }
        if (changed) {
            m_entries = kept;
            m_indexById.clear();
            for (int i = 0; i < m_entries.size(); ++i)
                m_indexById[m_entries[i].contourId] = i;
        }
    }
    if (changed) emit selectionChanged();
}

QVector<std::uint64_t> SelectionService::contoursInSelectionOrder() const
{
    QVector<std::uint64_t> out;
    out.reserve(m_entries.size());
    for (const auto& e : m_entries) out.append(e.contourId);
    return out;
}

} // namespace lcnc::core
