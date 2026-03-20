#include "app/widget_model_tree.h"
#include "base/lcnc_document.h"
#include "base/xcaf_utils.h"

#include <QVBoxLayout>
#include <QTreeWidgetItem>
#include <QHeaderView>

WidgetModelTree::WidgetModelTree(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    m_tree = new QTreeWidget(this);
    m_tree->setHeaderLabel(tr("模型树"));
    m_tree->setColumnCount(1);
    m_tree->setAnimated(true);
    m_tree->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_tree->header()->setVisible(false);

    layout->addWidget(m_tree);

    connect(m_tree, &QTreeWidget::itemSelectionChanged,
            this, &WidgetModelTree::onItemSelectionChanged);
    connect(m_tree, &QTreeWidget::itemDoubleClicked,
            this, &WidgetModelTree::onItemDoubleClicked);
}

void WidgetModelTree::clear()
{
    m_tree->clear();
}

void WidgetModelTree::rebuildForDocument(LcncDocument* doc)
{
    m_tree->clear();
    if (!doc) return;

    // ── 机台模型 group ────────────────────────────────────────────────────
    auto* machineItem = new QTreeWidgetItem(m_tree);
    machineItem->setText(0, tr("机台模型"));
    machineItem->setIcon(0, QIcon(":/icons/machine.svg"));
    machineItem->setData(0, Qt::UserRole, "group:machine");
    machineItem->setFlags(machineItem->flags() & ~Qt::ItemIsSelectable);
    populateGroup(machineItem, doc, LcncDocument::EntityKind::Machine);

    // ── 工件模型 group ────────────────────────────────────────────────────
    auto* workpieceItem = new QTreeWidgetItem(m_tree);
    workpieceItem->setText(0, tr("工件模型"));
    workpieceItem->setIcon(0, QIcon(":/icons/workpiece.svg"));
    workpieceItem->setData(0, Qt::UserRole, "group:workpiece");
    workpieceItem->setFlags(workpieceItem->flags() & ~Qt::ItemIsSelectable);
    populateGroup(workpieceItem, doc, LcncDocument::EntityKind::Workpiece);

    m_tree->expandAll();
}

void WidgetModelTree::populateGroup(QTreeWidgetItem*         groupItem,
                                    LcncDocument*            doc,
                                    LcncDocument::EntityKind kind)
{
    TDF_LabelSequence labels = doc->entityLabels(kind);
    for (int i = 1; i <= labels.Length(); ++i) {
        TDF_Label lbl = labels.Value(i);
        auto* item = new QTreeWidgetItem(groupItem);
        item->setText(0, XcafUtils::name(lbl));
        item->setIcon(0, QIcon(":/icons/shape.svg"));
        item->setData(0, Qt::UserRole, XcafUtils::entry(lbl));
    }
}

void WidgetModelTree::onItemSelectionChanged()
{
    QList<QTreeWidgetItem*> sel = m_tree->selectedItems();
    if (!sel.isEmpty()) {
        QString entry = sel.first()->data(0, Qt::UserRole).toString();
        if (!entry.startsWith("group:"))
            emit entitySelected(entry);
    }
}

void WidgetModelTree::onItemDoubleClicked(QTreeWidgetItem* item, int)
{
    QString entry = item->data(0, Qt::UserRole).toString();
    if (!entry.isEmpty() && !entry.startsWith("group:"))
        emit entitySelected(entry);
}
