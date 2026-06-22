#include "modules/process/ui/process_flow_tree_view.h"

#include "modules/process/ui/process_flow_model.h"
#include "modules/process/ui/process_node_edit_dialog.h"
#include "modules/process/steps/process_step_registry.h"
#include "modules/process/workflow/process_flow_store.h"
#include "modules/process/workflow/process_node_registry.h"

#include "core/logging/logger.h"

#include <QAction>
#include <QFileDialog>
#include <QHeaderView>
#include <QMenu>
#include <QMessageBox>
#include <QMap>

namespace lcnc::process {

ProcessFlowTreeView::ProcessFlowTreeView(QWidget* parent)
    : QTreeView(parent)
{
    setSelectionBehavior(QAbstractItemView::SelectRows);
    setSelectionMode(QAbstractItemView::SingleSelection);
    setContextMenuPolicy(Qt::CustomContextMenu);
    setDragEnabled(true);
    setAcceptDrops(true);
    setDropIndicatorShown(true);
    setDragDropMode(QAbstractItemView::InternalMove);
    setDefaultDropAction(Qt::MoveAction);
    setEditTriggers(QAbstractItemView::NoEditTriggers);

    m_contextMenu = new QMenu(this);
    m_addMenu = new QMenu(tr("Add"), this);
    m_contextMenu->addMenu(m_addMenu);
    m_deleteAction = m_contextMenu->addAction(tr("Delete"), this, &ProcessFlowTreeView::deleteCurrentNode);
    m_enableAction = m_contextMenu->addAction(tr("Enable"), this, &ProcessFlowTreeView::enableCurrentNode);
    m_disableAction = m_contextMenu->addAction(tr("Disable"), this, &ProcessFlowTreeView::disableCurrentNode);
    m_contextMenu->addSeparator();
    m_contextMenu->addAction(tr("Load..."), this, &ProcessFlowTreeView::loadFromFile);
    m_contextMenu->addAction(tr("Save..."), this, &ProcessFlowTreeView::saveToFile);
    m_contextMenu->addSeparator();
    m_contextMenu->addAction(tr("Clear"), this, &ProcessFlowTreeView::clearNodes);

    QMap<QString, QMenu*> categoryMenus;
    Q_UNUSED(categoryMenus);

    connect(this, &QTreeView::customContextMenuRequested,
            this, &ProcessFlowTreeView::showContextMenu);
    connect(this, &QTreeView::doubleClicked,
            this, &ProcessFlowTreeView::editCurrentNode);
}

void ProcessFlowTreeView::setFlowModel(ProcessFlowModel* model)
{
    m_model = model;
    setModel(model);
    if (header())
        header()->setStretchLastSection(true);
    expandAll();
}

void ProcessFlowTreeView::showContextMenu(const QPoint& pos)
{
    // 每次弹出菜单时按当前 step registry 状态重建 Add 子菜单，确保
    // 启用/禁用插件后立即生效。
    m_addMenu->clear();
    QMap<QString, QMenu*> categoryMenus;
    for (const auto& descriptor : ProcessStepRegistry::instance().descriptors()) {
        if (descriptor.type == ProcessNodeType::Base || !descriptor.addable)
            continue;
        QMenu* categoryMenu = categoryMenus.value(descriptor.category, nullptr);
        if (!categoryMenu) {
            categoryMenu = m_addMenu->addMenu(descriptor.category);
            categoryMenus.insert(descriptor.category, categoryMenu);
        }
        categoryMenu->addAction(descriptor.displayName, this,
                                [this, type = descriptor.type] { addNode(type); });
    }
    if (m_addMenu->isEmpty())
        m_addMenu->addAction(tr("无可用步骤"))->setEnabled(false);

    const QModelIndex idx = indexAt(pos);
    const bool hasNode = idx.isValid();
    bool canDelete = hasNode;
    bool canDisable = hasNode;
    if (hasNode && m_model) {
        if (ProcessNode* node = m_model->nodeFromIndex(idx)) {
            canDelete = ProcessNodeRegistry::instance().isDeletable(node->type);
            canDisable = ProcessNodeRegistry::instance().isDisableable(node->type);
        }
    }
    m_deleteAction->setEnabled(canDelete);
    m_enableAction->setEnabled(canDisable);
    m_disableAction->setEnabled(canDisable);
    m_contextMenu->exec(viewport()->mapToGlobal(pos));
}

void ProcessFlowTreeView::deleteCurrentNode()
{
    if (m_model)
        m_model->removeNode(currentNodeId());
}

void ProcessFlowTreeView::enableCurrentNode()
{
    if (m_model)
        m_model->setNodeEnabled(currentNodeId(), true);
}

void ProcessFlowTreeView::disableCurrentNode()
{
    if (m_model)
        m_model->setNodeEnabled(currentNodeId(), false);
}

void ProcessFlowTreeView::clearNodes()
{
    if (m_model)
        m_model->clear();
}

void ProcessFlowTreeView::loadFromFile()
{
    if (!m_model || !m_model->document())
        return;

    const QString filePath = QFileDialog::getOpenFileName(
        this,
        tr("加载流程"),
        QString(),
        tr("Process Flow (*.toml);;All Files (*.*)"));
    if (filePath.isEmpty())
        return;

    QString errorMessage;
    if (!ProcessFlowStore::loadFromFile(filePath, *m_model->document(), &errorMessage)) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "process.flow.view: load '{}' failed: {}",
                  filePath.toStdString(),
                  errorMessage.toStdString());
        QMessageBox::warning(this, tr("加载失败"), errorMessage);
        return;
    }

    m_model->resetFromDocument();
    expandAll();
}

void ProcessFlowTreeView::saveToFile()
{
    if (!m_model || !m_model->document())
        return;

    const QString filePath = QFileDialog::getSaveFileName(
        this,
        tr("保存流程"),
        QString(),
        tr("Process Flow (*.toml);;All Files (*.*)"));
    if (filePath.isEmpty())
        return;

    QString errorMessage;
    if (!ProcessFlowStore::saveToFile(filePath, *m_model->document(), &errorMessage)) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "process.flow.view: save '{}' failed: {}",
                  filePath.toStdString(),
                  errorMessage.toStdString());
        QMessageBox::warning(this, tr("保存失败"), errorMessage);
        return;
    }

    m_model->document()->markClean();
}

void ProcessFlowTreeView::editCurrentNode(const QModelIndex& index)
{
    if (!m_model || !index.isValid())
        return;

    editNode(m_model->nodeIdFromIndex(index));
}

QString ProcessFlowTreeView::currentNodeId() const
{
    return m_model ? m_model->nodeIdFromIndex(currentIndex()) : QString();
}

QString ProcessFlowTreeView::insertionParentId() const
{
    if (!m_model)
        return QString();

    const QModelIndex index = currentIndex();
    ProcessNode* node = m_model->nodeFromIndex(index);
    if (node && ProcessNodeRegistry::instance().canHaveChildren(node->type))
        return node->id;
    return QString();
}

void ProcessFlowTreeView::addNode(ProcessNodeType type)
{
    if (!m_model)
        return;

    m_model->appendNode(insertionParentId(), type);
    expandAll();
}

void ProcessFlowTreeView::editNode(const QString& nodeId)
{
    if (!m_model || nodeId.isEmpty())
        return;

    ProcessNode* node = m_model->document() ? m_model->document()->nodeById(nodeId) : nullptr;
    if (!node)
        return;

    ProcessNodeEditDialog dialog(*node, this);
    if (dialog.exec() != QDialog::Accepted)
        return;

    m_model->updateNode(dialog.node());
    emit editNodeRequested(nodeId);
}

} // namespace lcnc::process
