#include "app/controllers/project_explorer_controller.h"

#include "app/project_explorer_tree_utils.h"

#include <QScrollBar>
#include <QSet>
#include <QSignalBlocker>
#include <QTreeWidget>
#include <QTreeWidgetItemIterator>

#include <algorithm>

namespace lcnc::app {
namespace {

QString nodeKey(const QTreeWidgetItem* item)
{
    return item ? item->data(0, ProjectExplorerRoles::NodeKey).toString() : QString{};
}

QSet<QString> expandedKeys(QTreeWidget* tree)
{
    QSet<QString> keys;
    if (!tree)
        return keys;
    QTreeWidgetItemIterator iterator(tree);
    while (*iterator) {
        if ((*iterator)->isExpanded()) {
            const QString key = nodeKey(*iterator);
            if (!key.isEmpty())
                keys.insert(key);
        }
        ++iterator;
    }
    return keys;
}

QTreeWidgetItem* findByKey(QTreeWidget* tree, const QString& key)
{
    if (!tree || key.isEmpty())
        return nullptr;
    QTreeWidgetItemIterator iterator(tree);
    while (*iterator) {
        if (nodeKey(*iterator) == key)
            return *iterator;
        ++iterator;
    }
    return nullptr;
}

} // namespace

ProjectExplorerController::ProjectExplorerController(QTreeWidget* tree)
    : m_tree(tree)
{
}

void ProjectExplorerController::rebuild(const ProjectExplorerSnapshot& snapshot)
{
    if (!m_tree)
        return;

    const QSet<QString> keys = expandedKeys(m_tree);
    const QString currentKey = nodeKey(m_tree->currentItem());
    const int scroll = m_tree->verticalScrollBar()
        ? m_tree->verticalScrollBar()->value()
        : 0;

    const QSignalBlocker blocker(m_tree);
    populateProjectExplorerTree(m_tree, snapshot);

    QTreeWidgetItemIterator iterator(m_tree);
    while (*iterator) {
        const QString key = nodeKey(*iterator);
        if (!key.isEmpty())
            (*iterator)->setExpanded(keys.contains(key));
        ++iterator;
    }
    if (QTreeWidgetItem* current = findByKey(m_tree, currentKey))
        m_tree->setCurrentItem(current);
    if (m_tree->verticalScrollBar())
        m_tree->verticalScrollBar()->setValue(scroll);
}

std::optional<ProjectExplorerController::ContourSelection>
ProjectExplorerController::selectContour(std::uint64_t contourId, int fallbackIndex)
{
    if (!m_tree || (contourId == 0 && fallbackIndex < 0))
        return std::nullopt;

    QTreeWidgetItem* target = nullptr;
    QTreeWidgetItemIterator iterator(m_tree);
    while (*iterator) {
        const auto id = (*iterator)->data(0, ProjectExplorerRoles::ContourId).toULongLong();
        if (projectNodeKind(*iterator) == ProjectExplorerNodeKind::ToolpathContour
            && ((contourId != 0 && id == contourId)
                || (contourId == 0
                    && (*iterator)->data(0, ProjectExplorerRoles::ContourIndex).toInt() == fallbackIndex))) {
            target = *iterator;
            break;
        }
        ++iterator;
    }
    if (!target)
        return std::nullopt;

    const QSignalBlocker blocker(m_tree);
    m_tree->clearSelection();
    m_tree->setCurrentItem(target);
    target->setSelected(true);
    m_tree->scrollToItem(target);
    return ContourSelection{target->data(0, ProjectExplorerRoles::ContourId).toULongLong(),
                            target->data(0, ProjectExplorerRoles::ContourIndex).toInt()};
}

std::optional<ProjectExplorerController::ContourSelection>
ProjectExplorerController::selectContours(const QList<std::uint64_t>& contourIds,
                                          const QList<int>& fallbackIndexes)
{
    if (!m_tree || (contourIds.isEmpty() && fallbackIndexes.isEmpty()))
        return std::nullopt;

    QTreeWidgetItem* first = nullptr;
    QTreeWidgetItem* last = nullptr;
    const QSignalBlocker blocker(m_tree);
    m_tree->clearSelection();
    QTreeWidgetItemIterator iterator(m_tree);
    while (*iterator) {
        if (projectNodeKind(*iterator) == ProjectExplorerNodeKind::ToolpathContour) {
            const auto id = (*iterator)->data(0, ProjectExplorerRoles::ContourId).toULongLong();
            const int index = (*iterator)->data(0, ProjectExplorerRoles::ContourIndex).toInt();
            if ((id != 0 && contourIds.contains(id)) || fallbackIndexes.contains(index)) {
                (*iterator)->setSelected(true);
                if (!first)
                    first = *iterator;
                last = *iterator;
            }
        }
        ++iterator;
    }
    if (!last)
        return std::nullopt;
    m_tree->setCurrentItem(last);
    m_tree->scrollToItem(first ? first : last);
    return ContourSelection{last->data(0, ProjectExplorerRoles::ContourId).toULongLong(),
                            last->data(0, ProjectExplorerRoles::ContourIndex).toInt()};
}

void ProjectExplorerController::selectEntries(DocumentId documentId,
                                              const QStringList& entries)
{
    if (!m_tree)
        return;
    const QSignalBlocker blocker(m_tree);
    m_tree->clearSelection();
    if (entries.isEmpty())
        return;
    QTreeWidgetItemIterator iterator(m_tree);
    while (*iterator) {
        const auto kind = projectNodeKind(*iterator);
        const bool documentMatches = documentId == kInvalidDocumentId
            || (*iterator)->data(0, ProjectExplorerRoles::DocId).toInt() == documentId;
        const QString entry = (*iterator)->data(0, ProjectExplorerRoles::Entry).toString();
        if (isCadProjectNode(kind) && documentMatches && !entry.isEmpty() && entries.contains(entry))
            (*iterator)->setSelected(true);
        ++iterator;
    }
}

std::optional<ProjectExplorerController::ContourOrder>
ProjectExplorerController::contourOrder() const
{
    if (!m_tree)
        return std::nullopt;
    ContourOrder result;
    bool hasContourRoot = false;
    QTreeWidgetItemIterator iterator(m_tree);
    while (*iterator) {
        if (projectNodeKind(*iterator) == ProjectExplorerNodeKind::ToolpathContour) {
            hasContourRoot = true;
            const int index = (*iterator)->data(0, ProjectExplorerRoles::ContourIndex).toInt();
            const auto id = (*iterator)->data(0, ProjectExplorerRoles::ContourId).toULongLong();
            result.indexes.append(index);
            result.ids.append(id);
            result.hasStableIds = result.hasStableIds && id != 0;
            if (*iterator == m_tree->currentItem()) {
                result.selectedRow = result.indexes.size() - 1;
                result.selectedId = id;
            }
        }
        ++iterator;
    }
    return hasContourRoot ? std::optional<ContourOrder>(result) : std::nullopt;
}

} // namespace lcnc::app
