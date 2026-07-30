#include "app/controllers/project_explorer_controller.h"

#include "app/project_explorer_tree_utils.h"

#include <QScrollBar>
#include <QSet>
#include <QSignalBlocker>
#include <QTreeWidget>
#include <QTreeWidgetItemIterator>

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

} // namespace lcnc::app
