#include "modules/process/ui/process_flow_model.h"

#include "modules/process/workflow/process_node_registry.h"

#include <QColor>

namespace lcnc::process {

namespace {

constexpr const char* kNodeMimeType = "application/x-lcnc-process-node-id";

QColor stateColor(const ProcessNode& node)
{
    if (!node.enabled || node.state == ProcessNodeState::Disabled)
        return QColor(QStringLiteral("#F3F4F6"));

    switch (node.state) {
    case ProcessNodeState::Running: return QColor(QStringLiteral("#DCFCE7"));
    case ProcessNodeState::Paused: return QColor(QStringLiteral("#FEF3C7"));
    case ProcessNodeState::Stopped: return QColor(QStringLiteral("#FEE2E2"));
    default: return QColor(QStringLiteral("#E0F2FE"));
    }
}

} // namespace

ProcessFlowModel::ProcessFlowModel(QObject* parent)
    : QAbstractItemModel(parent)
{
}

void ProcessFlowModel::setDocument(ProcessFlowDocument* document)
{
    beginResetModel();
    m_document = document;
    endResetModel();
}

QModelIndex ProcessFlowModel::index(int row, int column, const QModelIndex& parentIndex) const
{
    if (!m_document || row < 0 || column < 0 || column >= columnCount(parentIndex))
        return QModelIndex();

    const QVector<ProcessNode>* children = childrenOf(parentIndex);
    if (!children || row >= children->size())
        return QModelIndex();

    return createIndex(row, column, const_cast<ProcessNode*>(&children->at(row)));
}

QModelIndex ProcessFlowModel::parent(const QModelIndex& index) const
{
    if (!index.isValid())
        return QModelIndex();

    const ProcessNode* node = nodeFromIndex(index);
    const ProcessNode* parentNode = parentNodeOf(node);
    if (!parentNode)
        return QModelIndex();

    return indexForNode(parentNode, 0);
}

int ProcessFlowModel::rowCount(const QModelIndex& parentIndex) const
{
    const QVector<ProcessNode>* children = childrenOf(parentIndex);
    return children ? children->size() : 0;
}

int ProcessFlowModel::columnCount(const QModelIndex& parent) const
{
    Q_UNUSED(parent);
    return 2;
}

QVariant ProcessFlowModel::data(const QModelIndex& index, int role) const
{
    const ProcessNode* node = nodeFromIndex(index);
    if (!node)
        return QVariant();

    switch (role) {
    case Qt::DisplayRole:
    case Qt::EditRole:
        return index.column() == 0 ? node->name : ProcessNodeRegistry::instance().summary(*node);
    case Qt::BackgroundRole:
        return stateColor(*node);
    case NodeIdRole:
        return node->id;
    case NodeTypeRole:
        return processNodeTypeToString(node->type);
    case NodeStateRole:
        return processNodeStateToString(node->state);
    case NodeEnabledRole:
        return node->enabled;
    default:
        return QVariant();
    }
}

QVariant ProcessFlowModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation != Qt::Horizontal || role != Qt::DisplayRole)
        return QVariant();

    return section == 0 ? tr("Process") : tr("Info");
}

Qt::ItemFlags ProcessFlowModel::flags(const QModelIndex& index) const
{
    Qt::ItemFlags itemFlags = QAbstractItemModel::flags(index) | Qt::ItemIsDropEnabled;
    if (index.isValid()) {
        itemFlags |= Qt::ItemIsSelectable | Qt::ItemIsEnabled;
        if (const ProcessNode* node = nodeFromIndex(index)) {
            if (ProcessNodeRegistry::instance().isMovable(node->type))
                itemFlags |= Qt::ItemIsDragEnabled;
        }
    }
    return itemFlags;
}

QStringList ProcessFlowModel::mimeTypes() const
{
    return { QString::fromLatin1(kNodeMimeType) };
}

QMimeData* ProcessFlowModel::mimeData(const QModelIndexList& indexes) const
{
    auto* mime = new QMimeData();
    for (const QModelIndex& index : indexes) {
        if (index.column() != 0)
            continue;
        const QString id = nodeIdFromIndex(index);
        if (!id.isEmpty()) {
            mime->setData(kNodeMimeType, id.toUtf8());
            break;
        }
    }
    return mime;
}

bool ProcessFlowModel::dropMimeData(const QMimeData* data,
                                    Qt::DropAction action,
                                    int row,
                                    int column,
                                    const QModelIndex& parentIndex)
{
    if (!m_document || !data || action != Qt::MoveAction || column > 0)
        return false;
    if (!data->hasFormat(kNodeMimeType))
        return false;

    const QString nodeId = QString::fromUtf8(data->data(kNodeMimeType)).trimmed();
    const QString parentId = nodeIdFromIndex(parentIndex);
    const int targetRow = row < 0 ? rowCount(parentIndex) : row;

    beginResetModel();
    const bool moved = m_document->moveNode(nodeId, parentId, targetRow);
    endResetModel();
    return moved;
}

Qt::DropActions ProcessFlowModel::supportedDropActions() const
{
    return Qt::MoveAction;
}

ProcessNode* ProcessFlowModel::nodeFromIndex(const QModelIndex& index) const
{
    if (!index.isValid())
        return nullptr;
    return static_cast<ProcessNode*>(index.internalPointer());
}

QString ProcessFlowModel::nodeIdFromIndex(const QModelIndex& index) const
{
    const ProcessNode* node = nodeFromIndex(index);
    return node ? node->id : QString();
}

void ProcessFlowModel::resetFromDocument()
{
    beginResetModel();
    endResetModel();
}

bool ProcessFlowModel::appendNode(const QString& parentId, ProcessNodeType type)
{
    if (!m_document)
        return false;

    beginResetModel();
    const bool ok = m_document->appendNode(parentId, ProcessNodeRegistry::instance().createDefaultNode(type)) != nullptr;
    endResetModel();
    return ok;
}

bool ProcessFlowModel::removeNode(const QString& id)
{
    if (!m_document || id.isEmpty())
        return false;

    beginResetModel();
    const bool ok = m_document->removeNode(id);
    endResetModel();
    return ok;
}

bool ProcessFlowModel::setNodeEnabled(const QString& id, bool enabled)
{
    if (!m_document)
        return false;

    ProcessNode* node = m_document->nodeById(id);
    if (!node)
        return false;
    if (!ProcessNodeRegistry::instance().isDisableable(node->type))
        return false;

    node->enabled = enabled;
    node->state = enabled ? ProcessNodeState::Enabled : ProcessNodeState::Disabled;
    m_document->markDirty();
    resetFromDocument();
    return true;
}

bool ProcessFlowModel::updateNode(const ProcessNode& node)
{
    if (!m_document)
        return false;

    ProcessNode* target = m_document->nodeById(node.id);
    if (!target)
        return false;

    *target = node;
    m_document->markDirty();
    resetFromDocument();
    return true;
}

bool ProcessFlowModel::clear()
{
    if (!m_document)
        return false;

    beginResetModel();
    m_document->resetToDefault();
    endResetModel();
    return true;
}

QModelIndex ProcessFlowModel::indexForNode(const ProcessNode* node, int column) const
{
    if (!node)
        return QModelIndex();

    const int row = rowOfNode(node);
    if (row < 0)
        return QModelIndex();
    return createIndex(row, column, const_cast<ProcessNode*>(node));
}

const ProcessNode* ProcessFlowModel::parentNodeOf(const ProcessNode* target) const
{
    if (!m_document || !target)
        return nullptr;
    return parentNodeOfRecursive(m_document->rootNodes(), target);
}

const ProcessNode* ProcessFlowModel::parentNodeOfRecursive(const QVector<ProcessNode>& nodes,
                                                           const ProcessNode* target) const
{
    for (const ProcessNode& node : nodes) {
        for (const ProcessNode& child : node.children) {
            if (&child == target)
                return &node;
        }
        if (const ProcessNode* parentNode = parentNodeOfRecursive(node.children, target))
            return parentNode;
    }
    return nullptr;
}

int ProcessFlowModel::rowOfNode(const ProcessNode* node) const
{
    if (!m_document || !node)
        return -1;
    return rowOfNodeRecursive(m_document->rootNodes(), node);
}

int ProcessFlowModel::rowOfNodeRecursive(const QVector<ProcessNode>& nodes, const ProcessNode* node) const
{
    for (int row = 0; row < nodes.size(); ++row) {
        if (&nodes.at(row) == node)
            return row;
        const int childRow = rowOfNodeRecursive(nodes.at(row).children, node);
        if (childRow >= 0)
            return childRow;
    }
    return -1;
}

const QVector<ProcessNode>* ProcessFlowModel::childrenOf(const QModelIndex& parentIndex) const
{
    if (!m_document)
        return nullptr;
    if (!parentIndex.isValid())
        return &m_document->rootNodes();

    const ProcessNode* parentNode = nodeFromIndex(parentIndex);
    return parentNode ? &parentNode->children : nullptr;
}

} // namespace lcnc::process
