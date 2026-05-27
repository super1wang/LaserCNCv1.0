#pragma once

#include "modules/process/workflow/process_flow_document.h"

#include <QAbstractItemModel>
#include <QMimeData>

namespace lcnc::process {

/**
 * @brief Qt item-model adapter for ProcessFlowDocument.
 *
 * The model does not own workflow data; it exposes the document to views and
 * keeps edits routed through stable node ids.
 */
class ProcessFlowModel : public QAbstractItemModel
{
    Q_OBJECT
public:
    enum Role {
        NodeIdRole = Qt::UserRole + 1,
        NodeTypeRole,
        NodeStateRole,
        NodeEnabledRole,
    };

    explicit ProcessFlowModel(QObject* parent = nullptr);

    void setDocument(ProcessFlowDocument* document);
    ProcessFlowDocument* document() const { return m_document; }

    QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
    QModelIndex parent(const QModelIndex& index) const override;
    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
    Qt::ItemFlags flags(const QModelIndex& index) const override;

    QStringList mimeTypes() const override;
    QMimeData* mimeData(const QModelIndexList& indexes) const override;
    bool dropMimeData(const QMimeData* data,
                      Qt::DropAction action,
                      int row,
                      int column,
                      const QModelIndex& parent) override;
    Qt::DropActions supportedDropActions() const override;

    ProcessNode* nodeFromIndex(const QModelIndex& index) const;
    QString nodeIdFromIndex(const QModelIndex& index) const;

    void resetFromDocument();
    bool appendNode(const QString& parentId, ProcessNodeType type);
    bool removeNode(const QString& id);
    bool setNodeEnabled(const QString& id, bool enabled);
    bool updateNode(const ProcessNode& node);
    bool clear();

private:
    QModelIndex indexForNode(const ProcessNode* node, int column = 0) const;
    const ProcessNode* parentNodeOf(const ProcessNode* target) const;
    const ProcessNode* parentNodeOfRecursive(const QVector<ProcessNode>& nodes,
                                             const ProcessNode* target) const;
    int rowOfNode(const ProcessNode* node) const;
    int rowOfNodeRecursive(const QVector<ProcessNode>& nodes, const ProcessNode* node) const;
    const QVector<ProcessNode>* childrenOf(const QModelIndex& parent) const;

    ProcessFlowDocument* m_document{nullptr};
};

} // namespace lcnc::process
