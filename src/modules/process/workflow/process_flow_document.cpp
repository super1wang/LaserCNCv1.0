#include "modules/process/workflow/process_flow_document.h"

#include "modules/process/workflow/process_node_registry.h"

namespace lcnc::process {

struct ProcessFlowDocument::NodeLocation
{
    QVector<ProcessNode>* siblings{nullptr};
    int row{-1};
};

void ProcessFlowDocument::clear()
{
    m_rootNodes.clear();
    markDirty();
}

void ProcessFlowDocument::setRootNodes(QVector<ProcessNode> nodes)
{
    m_rootNodes = std::move(nodes);
    markDirty();
}

ProcessNode& ProcessFlowDocument::appendRootNode(ProcessNode node)
{
    m_rootNodes.append(std::move(node));
    markDirty();
    return m_rootNodes.last();
}

ProcessNode* ProcessFlowDocument::appendNode(const QString& parentId, ProcessNode node)
{
    const ProcessNode* parent = parentId.isEmpty() ? nullptr : nodeById(parentId);
    if (!parentId.isEmpty() && !parent)
        return nullptr;
    const ProcessNodeType* parentType = parent ? &parent->type : nullptr;
    if (!ProcessNodeRegistry::instance().canPlaceNode(node.type, parentType))
        return nullptr;

    QVector<ProcessNode>* children = childrenForParent(parentId);
    if (!children)
        return nullptr;

    children->append(std::move(node));
    markDirty();
    return &children->last();
}

bool ProcessFlowDocument::removeNode(const QString& id)
{
    NodeLocation location = locateNode(id);
    if (!location.siblings || location.row < 0)
        return false;

    location.siblings->removeAt(location.row);
    markDirty();
    return true;
}

bool ProcessFlowDocument::moveNode(const QString& id, const QString& targetParentId, int targetRow)
{
    if (id.isEmpty() || id == targetParentId)
        return false;

    NodeLocation source = locateNode(id);
    if (!source.siblings || source.row < 0)
        return false;

    ProcessNode movingNode = source.siblings->at(source.row);
    if (containsNodeRecursive(movingNode, targetParentId))
        return false;

    const ProcessNode* targetParent = targetParentId.isEmpty() ? nullptr : nodeById(targetParentId);
    if (!targetParentId.isEmpty() && !targetParent)
        return false;
    const ProcessNodeType* targetParentType = targetParent ? &targetParent->type : nullptr;
    if (!ProcessNodeRegistry::instance().canPlaceNode(movingNode.type, targetParentType))
        return false;

    source.siblings->removeAt(source.row);

    QVector<ProcessNode>* targetSiblings = childrenForParent(targetParentId);
    if (!targetSiblings) {
        source.siblings->insert(source.row, std::move(movingNode));
        return false;
    }

    if (targetSiblings == source.siblings && targetRow > source.row)
        --targetRow;
    targetRow = qBound(0, targetRow, targetSiblings->size());
    targetSiblings->insert(targetRow, std::move(movingNode));
    markDirty();
    return true;
}

ProcessNode* ProcessFlowDocument::nodeById(const QString& id)
{
    return nodeByIdRecursive(m_rootNodes, id);
}

const ProcessNode* ProcessFlowDocument::nodeById(const QString& id) const
{
    return nodeByIdRecursive(m_rootNodes, id);
}

ProcessNode* ProcessFlowDocument::nodeByIdRecursive(QVector<ProcessNode>& nodes, const QString& id)
{
    if (id.isEmpty())
        return nullptr;

    for (ProcessNode& node : nodes) {
        if (node.id == id)
            return &node;
        if (ProcessNode* child = nodeByIdRecursive(node.children, id))
            return child;
    }
    return nullptr;
}

const ProcessNode* ProcessFlowDocument::nodeByIdRecursive(const QVector<ProcessNode>& nodes, const QString& id) const
{
    if (id.isEmpty())
        return nullptr;

    for (const ProcessNode& node : nodes) {
        if (node.id == id)
            return &node;
        if (const ProcessNode* child = nodeByIdRecursive(node.children, id))
            return child;
    }
    return nullptr;
}

ProcessFlowDocument::NodeLocation ProcessFlowDocument::locateNode(const QString& id)
{
    return locateNodeRecursive(m_rootNodes, id);
}

ProcessFlowDocument::NodeLocation ProcessFlowDocument::locateNodeRecursive(QVector<ProcessNode>& nodes, const QString& id)
{
    for (int row = 0; row < nodes.size(); ++row) {
        if (nodes[row].id == id)
            return { &nodes, row };
        NodeLocation childLocation = locateNodeRecursive(nodes[row].children, id);
        if (childLocation.siblings)
            return childLocation;
    }
    return {};
}

bool ProcessFlowDocument::containsNodeRecursive(const ProcessNode& node, const QString& id) const
{
    if (id.isEmpty())
        return false;
    if (node.id == id)
        return true;

    for (const ProcessNode& child : node.children) {
        if (containsNodeRecursive(child, id))
            return true;
    }
    return false;
}

QVector<ProcessNode>* ProcessFlowDocument::childrenForParent(const QString& parentId)
{
    if (parentId.isEmpty())
        return &m_rootNodes;

    ProcessNode* parent = nodeById(parentId);
    if (!parent || !ProcessNodeRegistry::instance().canHaveChildren(parent->type))
        return nullptr;
    return &parent->children;
}

} // namespace lcnc::process
