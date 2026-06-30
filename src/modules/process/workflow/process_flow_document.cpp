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
    resetToDefault();
}

void ProcessFlowDocument::resetToDefault()
{
    m_rootNodes.clear();
    m_rootNodes.append(ProcessNodeRegistry::instance().createDefaultNode(ProcessNodeType::Start));
    m_rootNodes.append(ProcessNodeRegistry::instance().createDefaultNode(ProcessNodeType::NormalCutting));
    m_rootNodes.append(ProcessNodeRegistry::instance().createDefaultNode(ProcessNodeType::Stop));
    markDirty();
}

void ProcessFlowDocument::setRootNodes(QVector<ProcessNode> nodes)
{
    m_rootNodes = std::move(nodes);
    ensureRequiredNodes();
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

    if (parentId.isEmpty()) {
        int insertRow = children->size();
        for (int i = 0; i < children->size(); ++i) {
            if (children->at(i).type == ProcessNodeType::Stop) {
                insertRow = i;
                break;
            }
        }
        children->insert(insertRow, std::move(node));
        markDirty();
        return &(*children)[insertRow];
    }

    children->append(std::move(node));
    markDirty();
    return &children->last();
}

bool ProcessFlowDocument::removeNode(const QString& id)
{
    NodeLocation location = locateNode(id);
    if (!location.siblings || location.row < 0)
        return false;
    if (ProcessNodeRegistry::instance().isRequired(location.siblings->at(location.row).type))
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
    if (ProcessNodeRegistry::instance().isRequired(movingNode.type))
        return false;
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

void ProcessFlowDocument::ensureRequiredNodes()
{
    bool hasStart = false;
    bool hasStop = false;
    QVector<ProcessNode> normalized;
    normalized.reserve(m_rootNodes.size() + 2);

    for (ProcessNode& node : m_rootNodes) {
        if (node.type == ProcessNodeType::Start) {
            if (hasStart)
                continue;
            hasStart = true;
            node.enabled = true;
            node.state = ProcessNodeState::Enabled;
            node.children.clear();
            normalized.prepend(std::move(node));
            continue;
        }
        if (node.type == ProcessNodeType::Stop) {
            if (hasStop)
                continue;
            hasStop = true;
            node.enabled = true;
            node.state = ProcessNodeState::Enabled;
            node.children.clear();
            // Stop 稍后统一放到末尾。
            normalized.append(std::move(node));
            continue;
        }
        normalized.append(std::move(node));
    }

    if (!hasStart)
        normalized.prepend(ProcessNodeRegistry::instance().createDefaultNode(ProcessNodeType::Start));

    ProcessNode stopNode;
    bool haveStopInNormalized = false;
    for (int i = 0; i < normalized.size(); ++i) {
        if (normalized.at(i).type == ProcessNodeType::Stop) {
            stopNode = std::move(normalized[i]);
            normalized.removeAt(i);
            haveStopInNormalized = true;
            break;
        }
    }
    if (!haveStopInNormalized)
        stopNode = ProcessNodeRegistry::instance().createDefaultNode(ProcessNodeType::Stop);
    normalized.append(std::move(stopNode));
    m_rootNodes = std::move(normalized);
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
