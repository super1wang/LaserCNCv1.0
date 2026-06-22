#pragma once

#include "modules/process/workflow/process_node_type.h"

#include <QTreeView>

class QAction;
class QMenu;

namespace lcnc::process {

class ProcessFlowModel;

/**
 * @brief Workflow tree view with process-node actions and edit dispatch.
 */
class ProcessFlowTreeView : public QTreeView
{
    Q_OBJECT
public:
    explicit ProcessFlowTreeView(QWidget* parent = nullptr);

    void setFlowModel(ProcessFlowModel* model);
    ProcessFlowModel* flowModel() const { return m_model; }

signals:
    void editNodeRequested(const QString& nodeId);

private slots:
    void showContextMenu(const QPoint& pos);
    void deleteCurrentNode();
    void enableCurrentNode();
    void disableCurrentNode();
    void clearNodes();
    void loadFromFile();
    void saveToFile();
    void editCurrentNode(const QModelIndex& index);

private:
    QString currentNodeId() const;
    QString insertionParentId() const;
    void addNode(ProcessNodeType type);
    void editNode(const QString& nodeId);

    ProcessFlowModel* m_model{nullptr};
    QMenu* m_contextMenu{nullptr};
    QMenu* m_addMenu{nullptr};
    QAction* m_deleteAction{nullptr};
    QAction* m_enableAction{nullptr};
    QAction* m_disableAction{nullptr};
};

} // namespace lcnc::process
