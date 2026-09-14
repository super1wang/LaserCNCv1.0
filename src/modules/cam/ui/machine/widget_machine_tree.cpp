#include "modules/cam/ui/machine/widget_machine_tree.h"

#include "core/document/lcnc_document.h"
#include "core/document/xcaf_utils.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/machine/machine_workspace.h"

#include <NCollection_Sequence.hxx>
#include <QHeaderView>
#include <QMap>
#include <QSet>
#include <QStringList>
#include <QTreeWidgetItem>
#include <TDF_Label.hxx>

namespace {
constexpr int kRoleEntry = Qt::UserRole;      // shape 节点存 XCAF entry 串
constexpr int kRoleIsAxis = Qt::UserRole + 1; // 标记节点是否为轴分组（用于级联 toggle）
} // namespace

WidgetMachineTree::WidgetMachineTree(QWidget* parent)
    : QWidget(parent)
{
    setupUi();
}

void WidgetMachineTree::setupUi()
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(2);

    m_tree = new QTreeWidget(this);
    m_tree->setHeaderHidden(true);
    m_tree->setRootIsDecorated(true);
    m_tree->setSelectionMode(QAbstractItemView::SingleSelection);
    m_tree->setAnimated(true);
    layout->addWidget(m_tree);

    connect(m_tree, &QTreeWidget::itemChanged, this, [this](QTreeWidgetItem* item, int /*col*/) {
        if (m_rebuilding)
            return;

        const bool isAxis = item->data(0, kRoleIsAxis).toBool();
        const bool checked = (item->checkState(0) == Qt::Checked);
        const QString entry = item->data(0, kRoleEntry).toString();

        if (!m_machineModelVisible && checked) {
            m_rebuilding = true;
            item->setCheckState(0, Qt::Unchecked);
            for (int i = 0; i < item->childCount(); ++i)
                item->child(i)->setCheckState(0, Qt::Unchecked);
            m_rebuilding = false;
            return;
        }

        QList<QPair<QString, bool>> changes;
        if (isAxis) {
            // 轴节点：级联所有子 shape
            m_rebuilding = true;
            for (int i = 0; i < item->childCount(); ++i) {
                QTreeWidgetItem* child = item->child(i);
                child->setCheckState(0, checked ? Qt::Checked : Qt::Unchecked);
                const QString childEntry = child->data(0, kRoleEntry).toString();
                if (!childEntry.isEmpty())
                    changes.append({childEntry, checked});
            }
            m_rebuilding = false;
        } else if (!entry.isEmpty()) {
            // 单个 shape 节点
            changes.append({entry, checked});
        }

        for (const auto& change : std::as_const(changes)) {
            emit shapeVisibilityChanged(change.first, change.second);
        }
    });
}

void WidgetMachineTree::setWorkspace(lcnc::cam::MachineWorkspace* workspace)
{
    m_workspace = workspace;
    rebuild();
}

void WidgetMachineTree::setVisibilityState(bool machineModelVisible, const QStringList& visibleEntries)
{
    m_machineModelVisible = machineModelVisible;
    m_visibleEntries = QSet<QString>(visibleEntries.cbegin(), visibleEntries.cend());
    rebuild();
}

void WidgetMachineTree::rebuild()
{
    if (!m_tree)
        return;

    m_rebuilding = true;
    m_tree->clear();

    if (!m_workspace) {
        m_rebuilding = false;
        return;
    }

    LcncDocument* doc = m_workspace->document();
    MachineKinematics* kin = m_workspace->kinematics();
    if (!doc || !kin) {
        m_rebuilding = false;
        return;
    }

    // 收集所有机台实体
    QMap<QString, QString> nameByEntry;  // entry → displayName
    {
        const NCollection_Sequence<TDF_Label> labels =
            doc->entityLabels(LcncDocument::EntityKind::Machine);
        for (int i = 1; i <= labels.Length(); ++i) {
            const TDF_Label lbl = labels.Value(i);
            const QString entry = XcafUtils::entry(lbl);
            const QString nm = XcafUtils::name(lbl);
            nameByEntry.insert(entry, nm.isEmpty() ? entry : nm);
        }
    }

    if (nameByEntry.isEmpty()) {
        auto* item = new QTreeWidgetItem(m_tree);
        // 中文翻译：（未加载机台模型）
        item->setText(0, tr("(Machine model not loaded)"));
        item->setFlags(item->flags() & ~Qt::ItemIsSelectable);
        m_rebuilding = false;
        return;
    }

    auto* root = new QTreeWidgetItem(m_tree);
    // 中文翻译：机台模型
    root->setText(0, tr("Machine model"));
    root->setExpanded(true);
    root->setFlags(root->flags() & ~Qt::ItemIsUserCheckable);

    const auto axes = kin->axes();
    QSet<QString> placed;

    if (axes.isEmpty()) {
        for (auto it = nameByEntry.cbegin(); it != nameByEntry.cend(); ++it) {
            auto* shapeNode = new QTreeWidgetItem(root);
            shapeNode->setText(0, it.value());
            shapeNode->setData(0, kRoleEntry, it.key());
            shapeNode->setData(0, kRoleIsAxis, false);
            shapeNode->setFlags(shapeNode->flags() | Qt::ItemIsUserCheckable);
            shapeNode->setCheckState(0, m_machineModelVisible && m_visibleEntries.contains(it.key())
                                        ? Qt::Checked
                                        : Qt::Unchecked);
        }
        m_rebuilding = false;
        return;
    }

    // 按轴分组
    for (const MachineAxisDef& axis : axes) {
        const QStringList entries = kin->shapesForAxis(axis.name);
        QTreeWidgetItem* axisNode = buildAxisNode(axis.name, entries, nameByEntry);
        if (axisNode)
            root->addChild(axisNode);
        for (const QString& e : entries)
            placed.insert(e);
    }

    // 未分配
    QTreeWidgetItem* unassigned = buildUnassignedNode(nameByEntry, placed);
    if (unassigned)
        root->addChild(unassigned);

    m_rebuilding = false;
}

QTreeWidgetItem* WidgetMachineTree::buildAxisNode(const QString& axisName,
                                                    const QStringList& entries,
                                                    const QMap<QString, QString>& nameMap)
{
    if (entries.isEmpty())
        return nullptr;

    auto* node = new QTreeWidgetItem();
    // 中文翻译：%1 轴
    node->setText(0, tr("%1 axis").arg(axisName));
    node->setExpanded(true);
    node->setFlags(node->flags() | Qt::ItemIsUserCheckable);
    node->setData(0, kRoleIsAxis, true);
    bool allChildrenChecked = !entries.isEmpty();

    for (const QString& entry : entries) {
        auto* child = new QTreeWidgetItem(node);
        child->setText(0, nameMap.value(entry, entry));
        child->setData(0, kRoleEntry, entry);
        child->setData(0, kRoleIsAxis, false);
        child->setFlags(child->flags() | Qt::ItemIsUserCheckable);
        const bool checked = m_machineModelVisible && m_visibleEntries.contains(entry);
        child->setCheckState(0, checked ? Qt::Checked : Qt::Unchecked);
        allChildrenChecked = allChildrenChecked && checked;
    }
    node->setCheckState(0, allChildrenChecked ? Qt::Checked : Qt::Unchecked);
    return node;
}

QTreeWidgetItem* WidgetMachineTree::buildUnassignedNode(const QMap<QString, QString>& nameMap,
                                                         const QSet<QString>& placed)
{
    QList<QString> unplaced;
    for (auto it = nameMap.cbegin(); it != nameMap.cend(); ++it) {
        if (!placed.contains(it.key()))
            unplaced.append(it.key());
    }
    if (unplaced.isEmpty())
        return nullptr;

    auto* node = new QTreeWidgetItem();
    // 中文翻译：未分配
    node->setText(0, tr("Not allocated"));
    node->setExpanded(true);
    node->setFlags(node->flags() | Qt::ItemIsUserCheckable);
    node->setData(0, kRoleIsAxis, true);
    bool allChildrenChecked = !unplaced.isEmpty();

    for (const QString& entry : unplaced) {
        auto* child = new QTreeWidgetItem(node);
        child->setText(0, nameMap.value(entry, entry));
        child->setData(0, kRoleEntry, entry);
        child->setData(0, kRoleIsAxis, false);
        child->setFlags(child->flags() | Qt::ItemIsUserCheckable);
        const bool checked = m_machineModelVisible && m_visibleEntries.contains(entry);
        child->setCheckState(0, checked ? Qt::Checked : Qt::Unchecked);
        allChildrenChecked = allChildrenChecked && checked;
    }
    node->setCheckState(0, allChildrenChecked ? Qt::Checked : Qt::Unchecked);
    return node;
}
