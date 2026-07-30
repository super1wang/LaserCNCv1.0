#include "app/project_explorer_tree_utils.h"

#include <QColor>
#include <QFont>
#include <QIcon>
#include <QTreeWidget>

namespace lcnc::app {

namespace {

QIcon iconForProjectNode(ProjectExplorerNodeKind kind)
{
    switch (kind) {
    case ProjectExplorerNodeKind::WorkpieceRoot:
    case ProjectExplorerNodeKind::CadDocument:
        return QIcon(":/icons/new_doc.svg");
    case ProjectExplorerNodeKind::CadSketch:
    case ProjectExplorerNodeKind::CadTemporarySketch:
        return QIcon(":/icons/sketch.svg");
    case ProjectExplorerNodeKind::CadShape:
    case ProjectExplorerNodeKind::CadSketchElement:
        return QIcon(":/icons/shape.svg");
    case ProjectExplorerNodeKind::ToolpathRoot:
    case ProjectExplorerNodeKind::ToolpathLayer:
    case ProjectExplorerNodeKind::ToolpathContour:
        return QIcon(":/icons/toolpath.svg");
    case ProjectExplorerNodeKind::MachiningFaceRoot:
    case ProjectExplorerNodeKind::MachiningFace:
        return QIcon(":/icons/shape.svg");
    case ProjectExplorerNodeKind::CadGroup:
    default:
        return QIcon(":/icons/machine.svg");
    }
}

void configureProjectTreeItem(QTreeWidgetItem* item,
                              QTreeWidgetItem* parent,
                              const ProjectExplorerNode& node)
{
    item->setText(0, node.displayName);
    item->setText(1, node.infoText);
    item->setIcon(0, iconForProjectNode(node.kind));
    item->setData(0, ProjectExplorerRoles::NodeKind, static_cast<int>(node.kind));
    item->setData(0, ProjectExplorerRoles::DocId, node.documentId);
    item->setData(0, ProjectExplorerRoles::NodeKey, node.nodeKey);
    item->setData(0, ProjectExplorerRoles::Entry, node.entry);
    item->setData(0, ProjectExplorerRoles::LeafEntries, node.leafEntries);
    item->setData(0, ProjectExplorerRoles::ContourIndex, node.contourIndex);
    item->setData(0, ProjectExplorerRoles::ContourId, static_cast<qulonglong>(node.contourId));
    item->setData(0, ProjectExplorerRoles::LayerId, static_cast<qulonglong>(node.layerId));
    item->setData(0, ProjectExplorerRoles::MachiningFaceId, static_cast<qulonglong>(node.machiningFaceId));
    if (!node.toolTip.isEmpty())
        item->setToolTip(0, node.toolTip);

    Qt::ItemFlags flags = item->flags();
    flags = node.checkable ? (flags | Qt::ItemIsUserCheckable)
                           : (flags & ~Qt::ItemIsUserCheckable);
    flags = node.selectable ? (flags | Qt::ItemIsSelectable)
                            : (flags & ~Qt::ItemIsSelectable);
    flags = node.draggable ? (flags | Qt::ItemIsDragEnabled)
                           : (flags & ~Qt::ItemIsDragEnabled);
    flags = node.droppable ? (flags | Qt::ItemIsDropEnabled)
                           : (flags & ~Qt::ItemIsDropEnabled);
    item->setFlags(flags);
    if (node.checkable)
        item->setCheckState(0, node.checked ? Qt::Checked : Qt::Unchecked);
    if (node.muted)
        item->setForeground(0, Qt::gray);
    if (node.kind == ProjectExplorerNodeKind::ToolpathLayer && node.layerColor.isValid()) {
        item->setForeground(0, node.layerColor.darker(130));
        item->setBackground(1, node.layerColor.lighter(175));
    }
    if (!parent) {
        QFont font = item->font(0);
        font.setBold(true);
        item->setFont(0, font);
    }
}

void addProjectNode(QTreeWidget* tree,
                    QTreeWidgetItem* parent,
                    const ProjectExplorerNode& node)
{
    QTreeWidgetItem* item = parent
        ? new QTreeWidgetItem(parent)
        : new QTreeWidgetItem(tree);

    configureProjectTreeItem(item, parent, node);

    for (const auto& child : node.children)
        addProjectNode(tree, item, child);
}

} // namespace

ProjectExplorerNodeKind projectNodeKind(const QTreeWidgetItem* item)
{
    if (!item)
        return ProjectExplorerNodeKind::WorkpieceRoot;
    return static_cast<ProjectExplorerNodeKind>(
        item->data(0, ProjectExplorerRoles::NodeKind).toInt());
}

void populateProjectExplorerTree(QTreeWidget* tree, const ProjectExplorerSnapshot& snapshot)
{
    if (!tree)
        return;

    tree->clear();
    for (const auto& root : snapshot.roots)
        addProjectNode(tree, nullptr, root);
}

} // namespace lcnc::app
