#include "Process_TreeView.h"

#include <QAction>
#include <QFileDialog>
#include <QMenu>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QStandardPaths>

#include <fstream>

namespace {

QString itemTypeText(ItemType type)
{
    switch (type) {
    case ItemType::Start: return QStringLiteral("Start");
    case ItemType::Stop: return QStringLiteral("Stop");
    case ItemType::Wait: return QStringLiteral("Wait");
    case ItemType::Axis: return QStringLiteral("Axis");
    case ItemType::Group: return QStringLiteral("Group");
    case ItemType::If: return QStringLiteral("If");
    case ItemType::Loop: return QStringLiteral("Loop");
    default: return QStringLiteral("Base");
    }
}

ItemType itemTypeFromText(const QString& text)
{
    if (text == QStringLiteral("Start")) return ItemType::Start;
    if (text == QStringLiteral("Stop")) return ItemType::Stop;
    if (text == QStringLiteral("Wait")) return ItemType::Wait;
    if (text == QStringLiteral("Axis")) return ItemType::Axis;
    if (text == QStringLiteral("Group")) return ItemType::Group;
    if (text == QStringLiteral("If")) return ItemType::If;
    if (text == QStringLiteral("Loop")) return ItemType::Loop;
    return ItemType::Base;
}

QString itemStateText(ItemState state)
{
    switch (state) {
    case ItemState::Disable: return QStringLiteral("Disable");
    case ItemState::Editing: return QStringLiteral("Editing");
    case ItemState::Unavailable: return QStringLiteral("Unavailable");
    case ItemState::Run: return QStringLiteral("Run");
    case ItemState::Stop: return QStringLiteral("Stop");
    case ItemState::Pause: return QStringLiteral("Pause");
    case ItemState::Unuse: return QStringLiteral("Unuse");
    case ItemState::Unrun: return QStringLiteral("Unrun");
    case ItemState::Enable:
    default: return QStringLiteral("Enable");
    }
}

ItemState itemStateFromText(const QString& text)
{
    if (text == QStringLiteral("Disable")) return ItemState::Disable;
    if (text == QStringLiteral("Editing")) return ItemState::Editing;
    if (text == QStringLiteral("Unavailable")) return ItemState::Unavailable;
    if (text == QStringLiteral("Run")) return ItemState::Run;
    if (text == QStringLiteral("Stop")) return ItemState::Stop;
    if (text == QStringLiteral("Pause")) return ItemState::Pause;
    if (text == QStringLiteral("Unuse")) return ItemState::Unuse;
    if (text == QStringLiteral("Unrun")) return ItemState::Unrun;
    return ItemState::Enable;
}

toml::value serializeItem(TreeItem* item)
{
    toml::value value;
    if (!item)
        return value;

    value["type"] = itemTypeText(item->type()).toStdString();
    value["state"] = itemStateText(item->state()).toStdString();
    value["label"] = item->data(0).toString().toStdString();
    value["info"] = item->data(1).toString().toStdString();

    toml::array children;
    for (int row = 0; row < item->childCount(); ++row)
        children.push_back(serializeItem(item->child(row)));
    value["children"] = children;
    return value;
}

TreeItem* deserializeItem(const toml::value& value)
{
    if (!value.is_table())
        return nullptr;

    const auto& table = value.as_table();
    const QString typeText = table.count("type") && table.at("type").is_string()
        ? QString::fromStdString(table.at("type").as_string())
        : QStringLiteral("Base");
    const QString label = table.count("label") && table.at("label").is_string()
        ? QString::fromStdString(table.at("label").as_string())
        : typeText;
    const QString info = table.count("info") && table.at("info").is_string()
        ? QString::fromStdString(table.at("info").as_string())
        : QString();
    const QString stateText = table.count("state") && table.at("state").is_string()
        ? QString::fromStdString(table.at("state").as_string())
        : QStringLiteral("Enable");

    auto* item = new TreeItem(label);
    item->m_type = itemTypeFromText(typeText);
    item->m_state = itemStateFromText(stateText);
    item->m_stateSave = item->m_state;
    item->setData(1, info);

    if (table.count("children") && table.at("children").is_array()) {
        for (const toml::value& childValue : table.at("children").as_array()) {
            if (TreeItem* child = deserializeItem(childValue))
                item->appendChild(child);
        }
    }

    return item;
}

void collectItems(TreeItem* parent, std::vector<Item>& output, int parentIndex)
{
    if (!parent)
        return;

    for (int row = 0; row < parent->childCount(); ++row) {
        TreeItem* child = parent->child(row);
        Item item;
        item.iParentIndex = parentIndex;
        item.iChildrenIndex = row;
        item.Type = child->type();
        item.maps = child->maps();
        output.push_back(item);
        collectItems(child, output, row);
    }
}

} // namespace

ProcessTreeView::ProcessTreeView(QWidget* parent)
    : QTreeView(parent)
{
    m_model = new TreeModel(QStringList() << tr("Process") << tr("Info"), this);
    setModel(m_model);
    setSelectionBehavior(QAbstractItemView::SelectRows);
    setSelectionMode(QAbstractItemView::SingleSelection);
    setContextMenuPolicy(Qt::CustomContextMenu);
    setDragEnabled(true);
    setAcceptDrops(true);
    setDropIndicatorShown(true);
    setDragDropMode(QAbstractItemView::InternalMove);

    m_menuRight = new QMenu(this);
    m_menuAdd = new QMenu(tr("Add"), this);

    m_actionAdd = m_menuRight->addMenu(m_menuAdd);
    m_actionDelete = m_menuRight->addAction(tr("Delete"), this, &ProcessTreeView::OnTriggeredActionDelete);
    m_actionEnable = m_menuRight->addAction(tr("Enable"), this, &ProcessTreeView::OnTriggeredActionEnable);
    m_actionDisable = m_menuRight->addAction(tr("Disable"), this, &ProcessTreeView::OnTriggeredActionDisable);
    m_menuRight->addSeparator();
    m_actionClear = m_menuRight->addAction(tr("Clear"), this, &ProcessTreeView::OnTriggeredActionClear);
    m_actionSave = m_menuRight->addAction(tr("Save"), this, [this] { Save(); });
    m_actionLoad = m_menuRight->addAction(tr("Load"), this, [this] { Load(); });

    m_actionStart = m_menuAdd->addAction(tr("Start"), this, &ProcessTreeView::OnTriggeredActionStart);
    m_actionStop = m_menuAdd->addAction(tr("Stop"), this, &ProcessTreeView::OnTriggeredActionStop);
    m_menuAdd->addSeparator();
    m_actionWait = m_menuAdd->addAction(tr("Wait"), this, &ProcessTreeView::OnTriggeredActionWait);
    m_actionAxis = m_menuAdd->addAction(tr("Axis"), this, &ProcessTreeView::OnTriggeredActionAxis);
    m_actionGroup = m_menuAdd->addAction(tr("Group"), this, &ProcessTreeView::OnTriggeredActionGroup);
    m_actionIf = m_menuAdd->addAction(tr("If"), this, &ProcessTreeView::OnTriggeredActionIf);
    m_actionLoop = m_menuAdd->addAction(tr("Loop"), this, &ProcessTreeView::OnTriggeredActionLoop);

    connect(this, &QTreeView::customContextMenuRequested,
            this, &ProcessTreeView::OnCustomContextMenuRequested);
    connect(this, &QTreeView::clicked,
            this, &ProcessTreeView::OnClickTreeView);
}

void ProcessTreeView::setParent(QMainWindow* parentWindow)
{
    m_parentWindow = parentWindow;
}

void ProcessTreeView::CreateNewFileData()
{
    if (m_model->rowCount() > 0)
        m_model->removeRows(0, m_model->rowCount());
}

QString ProcessTreeView::GetIndexRelation(const QModelIndex& index)
{
    if (!index.isValid())
        return QString();

    QStringList parts;
    QModelIndex current = index;
    while (current.isValid()) {
        parts.prepend(QString::number(current.row()));
        current = current.parent();
    }
    return parts.join(QLatin1Char('.'));
}

void ProcessTreeView::Save(QString fileName)
{
    if (fileName.isEmpty()) {
        fileName = QFileDialog::getSaveFileName(
            this,
            tr("Save process"),
            QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation),
            tr("Process TOML (*.toml)"));
    }
    if (fileName.isEmpty())
        return;

    toml::value root;
    if (!SaveValue(root))
        return;

    std::ofstream out(fileName.toStdString(), std::ios::binary);
    if (!out.is_open())
        return;
    out << toml::format(root);
}

bool ProcessTreeView::SaveValue(toml::value& valueProcess)
{
    toml::array items;
    TreeItem* root = m_model->root();
    for (int row = 0; root && row < root->childCount(); ++row)
        items.push_back(serializeItem(root->child(row)));
    valueProcess["Process"]["items"] = items;
    return true;
}

void ProcessTreeView::Load(QString fileName)
{
    if (fileName.isEmpty()) {
        fileName = QFileDialog::getOpenFileName(
            this,
            tr("Load process"),
            QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation),
            tr("Process TOML (*.toml)"));
    }
    if (fileName.isEmpty())
        return;

    try {
        toml::value root = toml::parse(fileName.toStdString());
        LoadValue(root);
    } catch (...) {
        return;
    }
}

bool ProcessTreeView::LoadValue(const toml::value& valueProcess)
{
    if (!valueProcess.is_table())
        return false;
    const auto& root = valueProcess.as_table();
    if (!root.count("Process") || !root.at("Process").is_table())
        return false;
    const auto& process = root.at("Process").as_table();
    if (!process.count("items") || !process.at("items").is_array())
        return false;

    CreateNewFileData();
    for (const toml::value& itemValue : process.at("items").as_array()) {
        if (TreeItem* item = deserializeItem(itemValue))
            m_model->appendRow(item);
    }
    expandAll();
    viewport()->update();
    return true;
}

void ProcessTreeView::ViewportUpdate()
{
    viewport()->update();
}

QModelIndex ProcessTreeView::ModelIndex(int parentIndex, int childIndex)
{
    if (parentIndex < 0)
        return QModelIndex();

    QModelIndex parent = m_model->index(parentIndex, 0, QModelIndex());
    if (childIndex < 0)
        return parent;
    return m_model->index(childIndex, 0, parent);
}

TreeItem* ProcessTreeView::GetItem(int parentIndex, int childIndex)
{
    return m_model->itemFromIndex(ModelIndex(parentIndex, childIndex));
}

std::vector<Item> ProcessTreeView::GetTreeItemVector()
{
    std::vector<Item> items;
    collectItems(m_model->root(), items, -1);
    return items;
}

int ProcessTreeView::Count(int parentIndex)
{
    if (parentIndex < 0)
        return m_model->rowCount();
    return m_model->rowCount(ModelIndex(parentIndex));
}

void ProcessTreeView::OnCustomContextMenuRequested(QPoint pos)
{
    m_currentModelIndex = indexAt(pos);
    const bool hasItem = m_currentModelIndex.isValid();
    m_actionDelete->setEnabled(hasItem);
    m_actionEnable->setEnabled(hasItem);
    m_actionDisable->setEnabled(hasItem);
    m_menuRight->exec(viewport()->mapToGlobal(pos));
}

void ProcessTreeView::OnTriggeredActionDelete()
{
    if (m_currentModelIndex.isValid())
        m_model->removeRow(m_currentModelIndex);
}

void ProcessTreeView::OnTriggeredActionEnable()
{
    if (TreeItem* item = m_model->itemFromIndex(m_currentModelIndex)) {
        item->SetState(ItemState::Enable);
        item->SwitchState(ItemState::Enable);
        viewport()->update();
    }
}

void ProcessTreeView::OnTriggeredActionDisable()
{
    if (TreeItem* item = m_model->itemFromIndex(m_currentModelIndex)) {
        item->SetState(ItemState::Disable);
        item->SwitchState(ItemState::Disable);
        viewport()->update();
    }
}

void ProcessTreeView::OnTriggeredActionClear()
{
    CreateNewFileData();
}

void ProcessTreeView::OnTriggeredActionStart()
{
    addItem(ItemType::Start, QStringLiteral("Start"));
}

void ProcessTreeView::OnTriggeredActionStop()
{
    addItem(ItemType::Stop, QStringLiteral("Stop"));
}

void ProcessTreeView::OnTriggeredActionWait()
{
    addItem(ItemType::Wait, QStringLiteral("Wait"));
}

void ProcessTreeView::OnTriggeredActionAxis()
{
    addItem(ItemType::Axis, QStringLiteral("Axis"));
}

void ProcessTreeView::OnTriggeredActionGroup()
{
    addItem(ItemType::Group, QStringLiteral("Group"));
}

void ProcessTreeView::OnTriggeredActionIf()
{
    addItem(ItemType::If, QStringLiteral("If"));
}

void ProcessTreeView::OnTriggeredActionLoop()
{
    addItem(ItemType::Loop, QStringLiteral("Loop"));
}

void ProcessTreeView::OnClickTreeView(const QModelIndex& index)
{
    m_currentModelIndex = index;
}

void ProcessTreeView::drawBranches(QPainter* painter, const QRect& rect, const QModelIndex& index) const
{
    painter->save();

    auto* item = static_cast<TreeItem*>(index.internalPointer());
    QColor color(QStringLiteral("#FFFFFF"));
    if (item) {
        switch (item->state()) {
        case ItemState::Disable: color = QColor(QStringLiteral("#F3F4F6")); break;
        case ItemState::Run: color = QColor(QStringLiteral("#DCFCE7")); break;
        case ItemState::Pause: color = QColor(QStringLiteral("#FEF3C7")); break;
        case ItemState::Stop: color = QColor(QStringLiteral("#FEE2E2")); break;
        default: color = QColor(QStringLiteral("#E0F2FE")); break;
        }
    }

    painter->setPen(QPen(QColor(QStringLiteral("#D1D5DB")), 1));
    painter->setBrush(color);
    painter->drawRect(rect.adjusted(1, 1, -1, -1));
    painter->restore();

    QTreeView::drawBranches(painter, rect, index);
}

void ProcessTreeView::mouseDoubleClickEvent(QMouseEvent* event)
{
    m_currentModelIndex = indexAt(event->pos());
    QTreeView::mouseDoubleClickEvent(event);
}

TreeItem* ProcessTreeView::createItem(ItemType type, const QString& label) const
{
    auto* item = new TreeItem(label);
    item->m_type = type;
    item->m_state = ItemState::Enable;
    item->m_stateSave = ItemState::Enable;
    item->setData(1, tr("Ready"));
    item->m_maps[QStringLiteral("type")] = itemTypeText(type);
    return item;
}

void ProcessTreeView::insertDataItem(const QModelIndex& insertIndex, TreeItem* item)
{
    if (!item)
        return;

    if (insertIndex.isValid()) {
        TreeItem* parentItem = m_model->itemFromIndex(insertIndex);
        if (parentItem && (parentItem->type() == ItemType::Group || parentItem->type() == ItemType::If || parentItem->type() == ItemType::Loop)) {
            m_model->appendRow(item, insertIndex);
        } else {
            m_model->insertRow(insertIndex.row() + 1, item, insertIndex.parent());
        }
    } else {
        m_model->appendRow(item);
    }

    expandAll();
}

void ProcessTreeView::addItem(ItemType type, const QString& label)
{
    insertDataItem(m_currentModelIndex, createItem(type, label));
    viewport()->update();
}