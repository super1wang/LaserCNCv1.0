#include "Process_Start.h"
#include <qDebug>


ProcessStart::ProcessStart(TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Start;
	m_state			= ItemState::Enable;
	m_stateSave		= m_state;
	m_itemData.resize(2);
}

ProcessStart::ProcessStart(const QString& text, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Start;
	m_state			= ItemState::Enable;
	m_stateSave		= m_state;
	m_itemData.resize(2);
	UpdateInfo();
}

ProcessStart::ProcessStart(const QVector<QVariant>& data, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Start;
	m_state			= ItemState::Enable;
	m_stateSave		= m_state;
	m_itemData.resize(2);
	m_itemData		= data;
}

ProcessStart::~ProcessStart(void)
{
	qDeleteAll(m_childItems);
	m_childItems.clear();
	qDeleteAll(m_childItems);
}

TreeItem* ProcessStart::clone() const
{
	TreeItem* newItem = nullptr;

	if (!m_itemData.isEmpty()) {
		newItem = new ProcessStart(QStringLiteral("Start"));
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

void ProcessStart::Edit()
{

}

void ProcessStart::UpdateInfo()
{
	if (TreeItem::IsChinese())
	{
		setData(0, "\xE8\xB5\xB7\xE5\xA7\x8B"); // 起始
		setData(1, "\xE6\xB5\x81\xE7\xA8\x8B\xE8\xB5\xB7\xE7\x82\xB9"); // 流程起点
	}
	else
	{
		setData(0, "Start");
		setData(1, "Process Start");
	}
}

void ProcessStart::SwitchState(ItemState state)
{
	if (state == ItemState::StateSave)
	{
		if (m_state == ItemState::Unrun || m_state == ItemState::Run ||
			m_state == ItemState::Pause || m_state == ItemState::Stop)
			SetState(ItemState::Enable);
		else if (m_state == ItemState::Unuse)
			SetState(ItemState::Disable);
		else
			SetState(m_stateSave);
	}
	else
	{
		m_stateSave = m_state;
		SetState(state);
	}
}
