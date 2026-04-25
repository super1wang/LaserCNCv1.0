#include "modules/cad/ui/widget_model_tree.h"
#include "core/document/lcnc_document.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/document/xcaf_utils.h"

#include <QVBoxLayout>
#include <QTreeWidgetItem>
#include <QTreeWidgetItemIterator>
#include <QHeaderView>
#include <QMenu>
#include <QAction>
#include <QMessageBox>
#include <functional>

#include <TDF_LabelSequence.hxx>

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
    // 选中节点高亮：蓝底白字（仅作用于本树，避免污染全局 QSS）。
    m_tree->setStyleSheet(QStringLiteral(
        "QTreeWidget::item:selected { background-color: #2A6FDB; color: white; }"
        "QTreeWidget::item:selected:!active { background-color: #2A6FDB; color: white; }"));

    layout->addWidget(m_tree);

    connect(m_tree, &QTreeWidget::itemSelectionChanged,
            this, &WidgetModelTree::onItemSelectionChanged);
    connect(m_tree, &QTreeWidget::itemDoubleClicked,
            this, &WidgetModelTree::onItemDoubleClicked);
    m_tree->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(m_tree, &QTreeWidget::customContextMenuRequested,
            this, &WidgetModelTree::onContextMenuRequested);
    connect(m_tree, &QTreeWidget::itemChanged,
            this, &WidgetModelTree::onItemChanged);
}

void WidgetModelTree::clear()
{
    m_doc = nullptr;
    m_tree->clear();
}

void WidgetModelTree::rebuildForDocument(LcncDocument* doc)
{
    m_doc = doc;
    m_tree->clear();
    if (!doc) return;

    MachineKinematics* kin = doc->machineKinematics();

    // ── 机台模型 group ─── axis-grouped view (always shown when axes exist) ─
    if (!kin->axes().isEmpty()) {
        auto* axisItem = new QTreeWidgetItem(m_tree);
        axisItem->setText(0, tr("机台模型"));
        axisItem->setIcon(0, QIcon(":/icons/machine.svg"));
        axisItem->setData(0, Qt::UserRole, "group:axisnodes");
        axisItem->setFlags((axisItem->flags() & ~Qt::ItemIsSelectable) | Qt::ItemIsUserCheckable);
        axisItem->setCheckState(0, Qt::Checked);
        populateMachineGroup(axisItem, doc, kin);
    }

    // ── 工件模型 group ────────────────────────────────────────────────────
    auto* workpieceItem = new QTreeWidgetItem(m_tree);
    workpieceItem->setText(0, tr("工件模型"));
    workpieceItem->setIcon(0, QIcon(":/icons/workpiece.svg"));
    workpieceItem->setData(0, Qt::UserRole, "group:workpiece");
    workpieceItem->setFlags((workpieceItem->flags() & ~Qt::ItemIsSelectable) | Qt::ItemIsUserCheckable);
    workpieceItem->setCheckState(0, Qt::Checked);
    populateGroup(workpieceItem, doc, LcncDocument::EntityKind::Workpiece, kin);

    // Only expand the workpiece group by default; keep machine/axis groups collapsed
    for (int i = 0; i < m_tree->topLevelItemCount(); ++i) {
        QTreeWidgetItem* item = m_tree->topLevelItem(i);
        const QString data = item->data(0, Qt::UserRole).toString();
        if (data == "group:workpiece")
            m_tree->expandItem(item);
    }
}

void WidgetModelTree::populateGroup(QTreeWidgetItem*         groupItem,
                                    LcncDocument*            doc,
                                    LcncDocument::EntityKind kind,
                                    MachineKinematics*       kin)
{
    // Prefer the import hierarchy tree when available (shows assembly structure)
    const QList<LcncDocument::ShapeTreeNode>& tree = doc->entityTree(kind);
    if (!tree.isEmpty()) {
        addTreeNodes(groupItem, tree, kind, kin);
        return;
    }

    // Fallback: flat list for shapes created via CAD commands
    TDF_LabelSequence labels = doc->entityLabels(kind);
    for (int i = 1; i <= labels.Length(); ++i) {
        TDF_Label lbl = labels.Value(i);
        const QString entry = XcafUtils::entry(lbl);
        const QString name  = XcafUtils::name(lbl);
        auto* item = new QTreeWidgetItem(groupItem);
        item->setIcon(0, QIcon(":/icons/shape.svg"));
        item->setData(0, Qt::UserRole, entry);

        if (kin && kind == LcncDocument::EntityKind::Workpiece) {
            const QString ax = kin->mountedAxis(entry);
            item->setText(0, ax.isEmpty() ? name : tr("%1  [→ %2]").arg(name, ax));
        } else {
            item->setText(0, name);
        }
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(0, Qt::Checked);
    }
}

void WidgetModelTree::addTreeNodes(
    QTreeWidgetItem*                          parent,
    const QList<LcncDocument::ShapeTreeNode>& nodes,
    LcncDocument::EntityKind                  kind,
    MachineKinematics*                        kin)
{
    for (const auto& node : nodes) {
        auto* item = new QTreeWidgetItem(parent);

        if (node.entry.isEmpty()) {
            // Virtual assembly/group node — not selectable, shown as folder
            item->setText(0, node.displayName);
            item->setIcon(0, QIcon(":/icons/machine.svg"));
            item->setData(0, Qt::UserRole, QStringLiteral("group:asm"));
            item->setFlags((item->flags() & ~Qt::ItemIsSelectable) | Qt::ItemIsUserCheckable);
            item->setCheckState(0, Qt::Checked);
            if (!node.children.isEmpty())
                addTreeNodes(item, node.children, kind, kin);
        } else {
            // Real shape node — selectable
            item->setIcon(0, QIcon(":/icons/shape.svg"));
            item->setData(0, Qt::UserRole, node.entry);

            if (kin && kind == LcncDocument::EntityKind::Workpiece) {
                const QString ax = kin->mountedAxis(node.entry);
                item->setText(0, ax.isEmpty() ? node.displayName
                              : tr("%1  [→ %2]").arg(node.displayName, ax));
            } else if (kin && kind == LcncDocument::EntityKind::Machine) {
                const QString ax = kin->axisForShape(node.entry);
                item->setText(0, ax.isEmpty() ? node.displayName
                              : tr("%1  [%2]").arg(node.displayName, ax));
            } else {
                item->setText(0, node.displayName);
            }
            item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
            item->setCheckState(0, Qt::Checked);
        }
    }
}

void WidgetModelTree::populateMachineGroup(QTreeWidgetItem*   groupItem,
                                           LcncDocument*      doc,
                                           MachineKinematics* kin)
{
    TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Machine);

    // Build entry→name map for quick lookup
    QMap<QString,QString> entryName;
    for (int i = 1; i <= labels.Length(); ++i) {
        TDF_Label lbl = labels.Value(i);
        entryName.insert(XcafUtils::entry(lbl), XcafUtils::name(lbl));
    }

    // Track which shapes have been placed under an axis sub-item
    QSet<QString> placed;

    for (const auto& axis : kin->axes()) {
        // Build axis header text
        QString axisText;
        if (axis.name == "BASE") {
            axisText = tr("BASE（固定基座）");
        } else if (axis.motionType == MachineAxisDef::Linear) {
            axisText = tr("%1 轴  (线性  ±%2 mm)").arg(axis.name).arg(axis.maxVal, 0, 'f', 0);
        } else {
            const QString range = (axis.maxVal >= 9000.0)
                ? tr("连续旋转")
                : tr("±%1°").arg(axis.maxVal, 0, 'f', 0);
            axisText = tr("%1 轴  (旋转  %2)").arg(axis.name, range);
        }

        auto* axisItem = new QTreeWidgetItem(groupItem);
        axisItem->setText(0, axisText);
        axisItem->setIcon(0, QIcon(":/icons/coordinate.svg"));
        axisItem->setData(0, Qt::UserRole, QStringLiteral("axis:%1").arg(axis.name));
        axisItem->setFlags((axisItem->flags() & ~Qt::ItemIsSelectable) | Qt::ItemIsUserCheckable);
        axisItem->setCheckState(0, Qt::Checked);
        const QFont boldFont = [&]{ QFont f = axisItem->font(0); f.setBold(true); return f; }();
        axisItem->setFont(0, boldFont);

        // Machine shapes assigned to this axis
        const QStringList entries = kin->shapesForAxis(axis.name);
        for (const QString& entry : entries) {
            const QString name = entryName.value(entry, entry);
            auto* item = new QTreeWidgetItem(axisItem);
            item->setText(0, name);
            item->setIcon(0, QIcon(":/icons/shape.svg"));
            item->setData(0, Qt::UserRole, entry);
            item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
            item->setCheckState(0, Qt::Checked);
            placed.insert(entry);
        }

        // Workpieces mounted to this axis
        const QStringList wpcEntries = kin->workpiecesOnAxis(axis.name);
        TDF_LabelSequence wpcLabels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
        QMap<QString,QString> wpcEntryName;
        for (int i = 1; i <= wpcLabels.Length(); ++i) {
            TDF_Label lbl = wpcLabels.Value(i);
            wpcEntryName.insert(XcafUtils::entry(lbl), XcafUtils::name(lbl));
        }
        for (const QString& wEntry : wpcEntries) {
            const QString wName = wpcEntryName.value(wEntry, wEntry);
            auto* wItem = new QTreeWidgetItem(axisItem);
            wItem->setText(0, tr("⚙ %1").arg(wName));
            wItem->setIcon(0, QIcon(":/icons/workpiece.svg"));
            wItem->setData(0, Qt::UserRole, wEntry);
            wItem->setForeground(0, QColor(80, 160, 80));
            wItem->setFlags(wItem->flags() | Qt::ItemIsUserCheckable);
            wItem->setCheckState(0, Qt::Checked);
        }
    }

    // Unassigned shapes
    bool hasUnassigned = false;
    QTreeWidgetItem* unassignedItem = nullptr;

    for (auto it = entryName.cbegin(); it != entryName.cend(); ++it) {
        if (!placed.contains(it.key())) {
            if (!hasUnassigned) {
                unassignedItem = new QTreeWidgetItem(groupItem);
                unassignedItem->setText(0, tr("（未分配）"));
                unassignedItem->setData(0, Qt::UserRole, "group:unassigned");
                unassignedItem->setFlags((unassignedItem->flags() & ~Qt::ItemIsSelectable) | Qt::ItemIsUserCheckable);
                unassignedItem->setCheckState(0, Qt::Checked);
                unassignedItem->setForeground(0, QColor(160, 100, 60));
                hasUnassigned = true;
            }
            auto* item = new QTreeWidgetItem(unassignedItem);
            item->setText(0, it.value());
            item->setIcon(0, QIcon(":/icons/shape.svg"));
            item->setData(0, Qt::UserRole, it.key());
            item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
            item->setCheckState(0, Qt::Checked);
        }
    }
}

void WidgetModelTree::onItemSelectionChanged()
{
    QStringList all;
    for (auto* item : m_tree->selectedItems()) {
        const QString data = item->data(0, Qt::UserRole).toString();
        if (!data.startsWith("group:") && !data.startsWith("axis:"))
            all << data;
    }
    if (!all.isEmpty())
        emit entitySelected(all.first());
    emit selectionChanged(all);
}

void WidgetModelTree::onItemDoubleClicked(QTreeWidgetItem* item, int)
{
    QString entry = item->data(0, Qt::UserRole).toString();
    if (!entry.isEmpty() && !entry.startsWith("group:"))
        emit entitySelected(entry);
}

void WidgetModelTree::highlightEntries(const QStringList& entries)
{
    QSignalBlocker blocker(m_tree);
    m_tree->clearSelection();
    if (entries.isEmpty()) return;

    QTreeWidgetItemIterator it(m_tree);
    while (*it) {
        const QString data = (*it)->data(0, Qt::UserRole).toString();
        if (entries.contains(data)) {
            (*it)->setSelected(true);
            // Keep both machine-model and axis-node trees collapsed unless the user expands them.
            QTreeWidgetItem* topLevel = *it;
            while (topLevel->parent()) topLevel = topLevel->parent();
            const QString topLevelKey = topLevel->data(0, Qt::UserRole).toString();
            const bool keepCollapsed = topLevelKey == QStringLiteral("group:machine")
                || topLevelKey == QStringLiteral("group:axisnodes");
            if (!keepCollapsed) {
                for (QTreeWidgetItem* p = (*it)->parent(); p; p = p->parent())
                    p->setExpanded(true);
            }
        }
        ++it;
    }
}

void WidgetModelTree::onContextMenuRequested(const QPoint& pos)
{
    if (!m_doc) return;
    QTreeWidgetItem* item = m_tree->itemAt(pos);
    if (!item) return;

    // Only show context menus inside the "轴系模型" group
    QTreeWidgetItem* topLevel = item;
    while (topLevel->parent()) topLevel = topLevel->parent();
    if (topLevel->data(0, Qt::UserRole).toString() != "group:axisnodes")
        return;

    // Determine axis name and (optional) leaf shape entry
    QString axisName;
    QString shapeEntry;
    {
        const QString d = item->data(0, Qt::UserRole).toString();
        if (d.startsWith("axis:")) {
            axisName = d.mid(5);
        } else if (!d.startsWith("group:")) {
            shapeEntry = d;
            for (QTreeWidgetItem* p = item->parent(); p; p = p->parent()) {
                const QString pd = p->data(0, Qt::UserRole).toString();
                if (pd.startsWith("axis:")) { axisName = pd.mid(5); break; }
            }
        }
    }
    if (axisName.isEmpty()) return;  // "（未分配）" or similar — ignore

    QMenu menu(this);
    QAction* actRemove = nullptr;
    if (!shapeEntry.isEmpty())
        actRemove = menu.addAction(tr("删除所选节点"));
    QAction* actClear = menu.addAction(tr("清除该轴系所有标记节点"));

    QAction* chosen = menu.exec(m_tree->viewport()->mapToGlobal(pos));
    if (!chosen) return;

    if (chosen == actRemove) {
        emit axisShapeUnassignRequested(shapeEntry);
    } else if (chosen == actClear) {
        emit axisAssignmentsClearRequested(axisName);
    }
}

void WidgetModelTree::onItemChanged(QTreeWidgetItem* item, int /*column*/)
{
    if (m_blockItemChanged || !item) return;

    const Qt::CheckState state = item->checkState(0);
    const QString entry = item->data(0, Qt::UserRole).toString();

    const bool isGroup = entry.isEmpty()
                      || entry.startsWith("group:")
                      || entry.startsWith("axis:");

    if (isGroup) {
        // 收集当前文档所有工件 entry，cascade 时跳过它们 —— 隐藏机台不应连带隐藏工件。
        QSet<QString> workpieceEntries;
        if (m_doc) {
            const TDF_LabelSequence wpcLabels =
                m_doc->entityLabels(LcncDocument::EntityKind::Workpiece);
            for (int i = 1; i <= wpcLabels.Length(); ++i)
                workpieceEntries.insert(XcafUtils::entry(wpcLabels.Value(i)));
        }
        const bool skipWorkpieces = entry == QStringLiteral("group:machine")
                                 || entry == QStringLiteral("group:axisnodes")
                                 || entry.startsWith("axis:");

        // Cascade check state to all descendants, then emit for each leaf.
        m_blockItemChanged = true;
        std::function<void(QTreeWidgetItem*)> cascade = [&](QTreeWidgetItem* node) {
            for (int i = 0; i < node->childCount(); ++i) {
                QTreeWidgetItem* child = node->child(i);
                const QString e = child->data(0, Qt::UserRole).toString();
                if (skipWorkpieces && workpieceEntries.contains(e)) {
                    cascade(child);
                    continue;
                }
                if (child->flags() & Qt::ItemIsUserCheckable)
                    child->setCheckState(0, state);
                cascade(child);
            }
        };
        cascade(item);
        m_blockItemChanged = false;

        // Emit visibilityChanged for every real leaf (skip workpieces when隐藏机台分组).
        std::function<void(QTreeWidgetItem*)> emitLeaves = [&](QTreeWidgetItem* node) {
            for (int i = 0; i < node->childCount(); ++i) {
                QTreeWidgetItem* child = node->child(i);
                const QString e = child->data(0, Qt::UserRole).toString();
                if (skipWorkpieces && workpieceEntries.contains(e)) {
                    emitLeaves(child);
                    continue;
                }
                if (!e.isEmpty() && !e.startsWith("group:") && !e.startsWith("axis:"))
                    emit visibilityChanged(e, state == Qt::Checked);
                emitLeaves(child);
            }
        };
        emitLeaves(item);
        return;
    }

    emit visibilityChanged(entry, state == Qt::Checked);
}
