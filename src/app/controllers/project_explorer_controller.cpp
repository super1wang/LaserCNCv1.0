#include "app/controllers/project_explorer_controller.h"

#include "app/project_explorer_tree_utils.h"
#include "modules/cad/selection/cad_selection_resolver.h"

#include <QItemSelectionModel>
#include <QScrollBar>
#include <QSet>
#include <QSignalBlocker>
#include <QTreeWidget>
#include <QTreeWidgetItemIterator>

#include <algorithm>
#include <functional>

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

// 把节点滚动到可见区，但不展开任何折叠的祖先节点（拾取同步时不自动展开工程树）。
void scrollWithoutExpand(QTreeWidget* tree, QTreeWidgetItem* item)
{
    if (!tree || !item)
        return;
    for (QTreeWidgetItem* parent = item->parent(); parent; parent = parent->parent()) {
        if (!parent->isExpanded())
            return;
    }
    tree->scrollToItem(item);
}

// 程序化设置当前项时，QAbstractItemView::currentChanged 会在 autoScroll 开启时调用
// scrollTo -> QTreeView::scrollTo 展开折叠的祖先节点。本守卫在构造时关闭 autoScroll，
// 析构时恢复，避免拾取同步把工程树自动展开（QSignalBlocker 不阻断 selectionModel 的
// currentChanged 信号，故必须从 autoScroll 入口拦截）。
class AutoScrollGuard
{
public:
    explicit AutoScrollGuard(QTreeWidget* tree)
        : m_tree(tree)
        , m_wasAutoScroll(tree ? tree->hasAutoScroll() : false)
    {
        if (m_tree)
            m_tree->setAutoScroll(false);
    }
    ~AutoScrollGuard()
    {
        if (m_tree)
            m_tree->setAutoScroll(m_wasAutoScroll);
    }
    Q_DISABLE_COPY_MOVE(AutoScrollGuard)
private:
    QTreeWidget* m_tree{nullptr};
    bool m_wasAutoScroll{false};
};

QSet<QString> selectedNodeKeys(QTreeWidget* tree)
{
    QSet<QString> keys;
    if (!tree)
        return keys;
    const QList<QTreeWidgetItem*> items = tree->selectedItems();
    for (QTreeWidgetItem* item : items) {
        const QString key = nodeKey(item);
        if (!key.isEmpty())
            keys.insert(key);
    }
    return keys;
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
    const QSet<QString> selectedKeys = selectedNodeKeys(m_tree);
    const QString currentKey = nodeKey(m_tree->currentItem());
    const int scroll = m_tree->verticalScrollBar()
        ? m_tree->verticalScrollBar()->value()
        : 0;

    const QSignalBlocker blocker(m_tree);
    const AutoScrollGuard autoScrollGuard(m_tree);
    populateProjectExplorerTree(m_tree, snapshot);

    QTreeWidgetItemIterator iterator(m_tree);
    while (*iterator) {
        const QString key = nodeKey(*iterator);
        if (!key.isEmpty()) {
            (*iterator)->setExpanded(keys.contains(key));
            if (selectedKeys.contains(key))
                (*iterator)->setSelected(true);
        }
        ++iterator;
    }
    if (QTreeWidgetItem* current = findByKey(m_tree, currentKey))
        m_tree->setCurrentItem(current, 0, QItemSelectionModel::NoUpdate);
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
    const AutoScrollGuard autoScrollGuard(m_tree);
    m_tree->clearSelection();
    m_tree->setCurrentItem(target, 0, QItemSelectionModel::NoUpdate);
    target->setSelected(true);
    scrollWithoutExpand(m_tree, target);
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
    const AutoScrollGuard autoScrollGuard(m_tree);
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
    m_tree->setCurrentItem(last, 0, QItemSelectionModel::NoUpdate);
    scrollWithoutExpand(m_tree, first ? first : last);
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

std::optional<ProjectExplorerController::VisibilityChange>
ProjectExplorerController::visibilityChange(QTreeWidgetItem* item)
{
    if (!m_tree || !item)
        return std::nullopt;

    const auto kind = projectNodeKind(item);
    VisibilityChange result;
    result.visible = item->checkState(0) == Qt::Checked;

    const auto cascade = [this, &result](QTreeWidgetItem* root,
                                          const std::function<bool(QTreeWidgetItem*)>& shouldChange) {
        const QSignalBlocker blocker(m_tree);
        const bool updatesEnabled = m_tree->updatesEnabled();
        m_tree->setUpdatesEnabled(false);
        std::function<void(QTreeWidgetItem*)> visit = [&](QTreeWidgetItem* node) {
            if ((node->flags() & Qt::ItemIsUserCheckable) && shouldChange(node))
                node->setCheckState(0, result.visible ? Qt::Checked : Qt::Unchecked);
            for (int index = 0; index < node->childCount(); ++index)
                visit(node->child(index));
        };
        for (int index = 0; index < root->childCount(); ++index)
            visit(root->child(index));
        m_tree->setUpdatesEnabled(updatesEnabled);
    };

    const auto collectCad = [&result](QTreeWidgetItem* root) {
        QMap<DocumentId, QSet<QString>> entriesByDocument;
        std::function<void(QTreeWidgetItem*)> visit = [&](QTreeWidgetItem* node) {
            if (!node || !isCadProjectNode(projectNodeKind(node)))
                return;
            const DocumentId documentId = node->data(0, ProjectExplorerRoles::DocId).toInt();
            const QString key = node->data(0, ProjectExplorerRoles::NodeKey).toString();
            if (documentId != kInvalidDocumentId
                && lcnc::cad::selection::CadSelectionResolver::isFinishedSketchNode(key)) {
                result.sketches.append({documentId,
                    lcnc::cad::selection::CadSelectionResolver::sketchIdFromNodeKey(key)});
            }
            if (documentId != kInvalidDocumentId) {
                for (const QString& entry : node->data(0, ProjectExplorerRoles::LeafEntries).toStringList()) {
                    if (!entry.isEmpty())
                        entriesByDocument[documentId].insert(entry);
                }
            }
            for (int index = 0; index < node->childCount(); ++index)
                visit(node->child(index));
        };
        visit(root);
        for (auto it = entriesByDocument.cbegin(); it != entriesByDocument.cend(); ++it)
            result.cadEntries.append({it.key(), it.value().values()});
    };

    if (isCadProjectNode(kind)) {
        result.target = VisibilityChange::Target::Cad;
        const DocumentId documentId = item->data(0, ProjectExplorerRoles::DocId).toInt();
        const QString key = item->data(0, ProjectExplorerRoles::NodeKey).toString();
        const QString entry = item->data(0, ProjectExplorerRoles::Entry).toString();
        if (documentId != kInvalidDocumentId
            && lcnc::cad::selection::CadSelectionResolver::isFinishedSketchNode(key)) {
            result.sketches.append({documentId,
                lcnc::cad::selection::CadSelectionResolver::sketchIdFromNodeKey(key)});
            return result;
        }
        if (entry.isEmpty()) {
            cascade(item, [](QTreeWidgetItem* child) { return isCadProjectNode(projectNodeKind(child)); });
            collectCad(item);
        } else if (documentId != kInvalidDocumentId) {
            result.cadEntries.append({documentId,
                item->data(0, ProjectExplorerRoles::LeafEntries).toStringList()});
        }
        return result;
    }
    if (kind == ProjectExplorerNodeKind::MachiningFaceRoot) {
        result.target = VisibilityChange::Target::MachiningFaces;
        return result;
    }
    if (kind == ProjectExplorerNodeKind::ToolpathRoot) {
        result.target = VisibilityChange::Target::AllContours;
        cascade(item, [](QTreeWidgetItem* child) { return isToolpathProjectNode(projectNodeKind(child)); });
        return result;
    }
    if (kind == ProjectExplorerNodeKind::ToolpathLayer) {
        result.target = VisibilityChange::Target::Layer;
        result.layerId = item->data(0, ProjectExplorerRoles::LayerId).toULongLong();
        cascade(item, [](QTreeWidgetItem* child) {
            return projectNodeKind(child) == ProjectExplorerNodeKind::ToolpathContour;
        });
        return result;
    }
    if (kind == ProjectExplorerNodeKind::ToolpathContour) {
        result.target = VisibilityChange::Target::Contour;
        result.contourId = item->data(0, ProjectExplorerRoles::ContourId).toULongLong();
        result.contourIndex = item->data(0, ProjectExplorerRoles::ContourIndex).toInt();
        return result;
    }
    return std::nullopt;
}

} // namespace lcnc::app
