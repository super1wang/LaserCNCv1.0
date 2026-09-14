#pragma once

#include "modules/process/workflow/process_node.h"

namespace lcnc::process {

/**
 * @brief Owns the editable process workflow tree independent of Qt views.
 */
class ProcessFlowDocument
{
public:
    QVector<ProcessNode>& rootNodes() { return m_rootNodes; }
    const QVector<ProcessNode>& rootNodes() const { return m_rootNodes; }

    void clear();
    void resetToDefault();
    void setRootNodes(QVector<ProcessNode> nodes);
    ProcessNode& appendRootNode(ProcessNode node);
    ProcessNode* appendNode(const QString& parentId, ProcessNode node);
    bool removeNode(const QString& id);
    bool moveNode(const QString& id, const QString& targetParentId, int targetRow);
    void ensureRequiredNodes();

    bool isEmpty() const { return m_rootNodes.isEmpty(); }
    bool isDirty() const { return m_dirty; }
    void markClean() { m_dirty = false; }
    void markDirty() { m_dirty = true; }

    ProcessNode* nodeById(const QString& id);
    const ProcessNode* nodeById(const QString& id) const;

private:
    struct NodeLocation;

    ProcessNode* nodeByIdRecursive(QVector<ProcessNode>& nodes, const QString& id);
    const ProcessNode* nodeByIdRecursive(const QVector<ProcessNode>& nodes, const QString& id) const;
    NodeLocation locateNode(const QString& id);
    NodeLocation locateNodeRecursive(QVector<ProcessNode>& nodes, const QString& id);
    bool containsNodeRecursive(const ProcessNode& node, const QString& id) const;
    QVector<ProcessNode>* childrenForParent(const QString& parentId);

    QVector<ProcessNode> m_rootNodes;
    bool m_dirty{false};
};

} // namespace lcnc::process
