#include "Process_Stop.h"
#include <qDebug>


ProcessStop::ProcessStop(TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Stop;
	m_state			= ItemState::Enable;
	m_stateSave		= m_state;
	m_itemData.resize(2);
}

ProcessStop::ProcessStop(const QString& text, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Stop;
	m_state			= ItemState::Enable;
	m_stateSave		= m_state;
	m_itemData.resize(2);
	UpdateInfo();
}

ProcessStop::ProcessStop(const QVector<QVariant>& data, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Stop;
	m_state			= ItemState::Enable;
	m_stateSave		= m_state;
	m_itemData.resize(2);
	m_itemData		= data;
}

ProcessStop::~ProcessStop(void)
{
	qDeleteAll(m_childItems);
	m_childItems.clear();
	qDeleteAll(m_childItems);
}

TreeItem* ProcessStop::clone() const
{
	TreeItem* newItem = nullptr;

	if (!m_itemData.isEmpty()) {
		newItem = new ProcessStop(QStringLiteral("Stop"));
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

void ProcessStop::Edit()
{

}

void ProcessStop::UpdateInfo()
{
	if (TreeItem::IsChinese())
	{
		setData(0, "\xE5\x81\x9C\xE6\xAD\xA2"); // 停止
		setData(1, "\xE5\x81\x9C\xE6\xAD\xA2\xE6\xB5\x81\xE7\xA8\x8B"); // 停止流程
	}
	else
	{
		setData(0, "Stop");
		setData(1, "Stop Process");
	}
}

void ProcessStop::SwitchState(ItemState state)
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
