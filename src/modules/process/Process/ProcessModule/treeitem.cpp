#include "treeitem.h"
#include <QStringList>

// 初始化静态变量
bool TreeItem::m_bChinese = true;

TreeItem::TreeItem(TreeItem* parent)
{
    m_parentItem = parent;
    m_type = ItemType::Base;
	m_state = ItemState::Enable;
	m_stateSave = ItemState::Enable;
	m_itemData.resize(2);
}

TreeItem::TreeItem(const QString& text, TreeItem* parent)
{
	m_parentItem = parent;
	m_type = ItemType::Base;
	m_state = ItemState::Enable;
	m_stateSave = ItemState::Enable;
	m_itemData.resize(2);
	setData(0, text);
}

TreeItem::TreeItem(const QVector<QVariant>& data, TreeItem* parent)
{
	m_parentItem = parent;
	m_type = ItemType::Base;
	m_state = ItemState::Enable;
	m_stateSave = ItemState::Enable;
	m_itemData.resize(2);
	m_itemData = data;
}

TreeItem::~TreeItem()
{
	qDeleteAll(m_childItems);
	m_childItems.clear();
	qDeleteAll(m_childItems);
}

TreeItem* TreeItem::parentItem()
{
	return m_parentItem;
}

TreeItem* TreeItem::parent() const
{
	return m_parentItem;
}

TreeItem* TreeItem::child(int row)
{
	if (row < 0 || row >= m_childItems.size())
		return nullptr;
	return m_childItems.at(row);
}

bool TreeItem::appendChild(TreeItem* child)
{
	if (!child)
		return false;

	if (child->m_parentItem)
	{
		if (!child->m_parentItem->removeChild(child))
			return false;
	}

	try 
	{
		child->m_parentItem = this;
		m_childItems.append(child);
		return true;
	}
	catch (const std::exception& e)
	{
		child->m_parentItem = nullptr; // 回滚状态
		return false;
	}
}

bool TreeItem::insertChild(int row, TreeItem* child)
{
  	if (!child)
		return false;

	if (row < 0 || row > m_childItems.size())
		row = qBound(0, row, m_childItems.size()); // 自动修正到安全范围

	if (child->m_parentItem)
	{
		child->m_parentItem->removeChild(child);
	}

	try 
	{
		child->m_parentItem = this;
		m_childItems.insert(row, child);
		return true;
	}
	catch (const std::exception& e)
	{
		child->m_parentItem = nullptr; // 回滚状态
		return false;
	}
}

bool TreeItem::removeChild(TreeItem* child)
{
	if (!child)
		return false;

	if (child->m_parentItem != this)
		return false;

	if (!m_childItems.removeOne(child))
		return false;

	child->m_parentItem = nullptr;	// 清理父项指针（但不删除对象）
	return true;
}

int TreeItem::childCount() const
{
	return m_childItems.count();
}

int TreeItem::columnCount() const
{
	return m_itemData.count();
}

bool TreeItem::insertChildren(int position, int count, int columns)
{
	if (position < 0 || position > m_childItems.size())
		return false;

	for (int row = 0; row < count; ++row) {
		QVector<QVariant> data(columns);
		TreeItem* item = new TreeItem(data, this);
		m_childItems.insert(position, item);
	}

	return true;
}

bool TreeItem::insertColumns(int position, int columns)
{
	if (position < 0 || position > m_itemData.size())
		return false;

	for (int column = 0; column < columns; ++column)
		m_itemData.insert(position, QVariant());

	foreach(TreeItem * child, m_childItems)
		child->insertColumns(position, columns);

	return true;
}

bool TreeItem::removeChildren(int position, int count)
{
	if (position < 0 || position + count > m_childItems.size())
		return false;

	for (int row = 0; row < count; ++row)
		delete m_childItems.takeAt(position);

	return true;
}

bool TreeItem::removeColumns(int position, int columns)
{
	if (position < 0 || position + columns > m_itemData.size())
		return false;

	for (int column = 0; column < columns; ++column)
		m_itemData.remove(position);

	foreach(TreeItem * child, m_childItems)
		child->removeColumns(position, columns);

	return true;
}

QVariant TreeItem::data(int column) const
{
	if (column < 0 || column >= m_itemData.size())
		return QVariant();
	return m_itemData.at(column);
}

bool TreeItem::setData(int column, const QVariant& value)
{
	if (column < 0 || column >= m_itemData.size())
		return false;

	m_itemData[column] = value;
	return true;
}

int TreeItem::row() const
{
	if (m_parentItem)
		return m_parentItem->m_childItems.indexOf(const_cast<TreeItem*>(this));

	return 0;
}

void TreeItem::setRow(int row)
{
	Q_ASSERT(m_parentItem);
	if (!m_parentItem)
		return;

	m_parentItem->m_childItems.move(this->row(), row);
}

TreeItem* TreeItem::clone() const
{
	TreeItem* newItem = nullptr;

	if (!m_itemData.isEmpty()) {
		newItem = new TreeItem(m_itemData, nullptr);
	}
	else {
		newItem = new TreeItem(static_cast<TreeItem*>(nullptr));
	}

	newItem->m_type = m_type;
	newItem->m_state = m_state;
	newItem->m_stateSave = m_stateSave;
	newItem->m_maps = m_maps;

	for (TreeItem* child : m_childItems)
	{
		TreeItem* clonedChild = child->clone();
		newItem->appendChild(clonedChild);
	}

	return newItem;
}

void TreeItem::Edit()
{
	if (m_type == ItemType::Base)
	{
		foreach(TreeItem * p, m_childItems)
		{
			p->Edit();
		}
	}
}

void TreeItem::UpdateInfo()
{
	if (m_type == ItemType::Base)
	{
		foreach(TreeItem * p, m_childItems)
		{
			p->UpdateInfo();
		}
	}
}

void TreeItem::SwitchState(ItemState state)
{
	if (m_type != ItemType::Base)
	{
		m_state = state == ItemState::StateSave ? m_stateSave : state;
		return;
	}

	if (m_type == ItemType::Base)
	{
		foreach(TreeItem * p, m_childItems)
		{
			p->SwitchState(state);
		}
	}
}

void TreeItem::SetState(ItemState state)
{
	if (m_type != ItemType::Base)
	{
		if (state != ItemState::StateSave)
		{
			m_state = state;
			m_stateSave = state;
		}
		return;
	}

	if (m_type == ItemType::Base)
	{
		foreach(TreeItem * p, m_childItems)
		{
			p->SetState(state);
		}
	}
}

void TreeItem::SetMaps(map<QString, QString> maps)
{
	if (m_type != ItemType::Base)
	{
		m_maps = maps;
		return;
	}

	if (m_type == ItemType::Base)
	{
		foreach(TreeItem * p, m_childItems)
		{
			p->SetMaps(maps);
		}
	}
}

ItemType TreeItem::GetType()
{
	if (m_type != ItemType::Base)
		return m_type;

	ItemType state = ItemType::Base;
	if (m_type == ItemType::Base)
	{
		foreach(TreeItem * p, m_childItems)
		{
			state = p->GetType();
		}
	}
	return state;
}

ItemState TreeItem::GetState()
{
	if (m_type != ItemType::Base)
		return m_state;

	ItemState state = ItemState::Unavailable;
	if (m_type == ItemType::Base)
	{
		foreach(TreeItem * p, m_childItems)
		{
			state = p->GetState();
		}
	}
	return state;
}

map<QString, QString> TreeItem::GetMaps()
{
	if (m_type != ItemType::Base)
		return m_maps;

	map<QString, QString> maps;
	if (m_type == ItemType::Base)
	{
		foreach(TreeItem * p, m_childItems)
		{
			maps = p->GetMaps();
		}
	}
	return maps;
}
