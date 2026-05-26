#pragma once
#include "treeitem.h"

class ProcessStart :
	public TreeItem
{
public:
	explicit							ProcessStart(TreeItem* parent = 0);
	explicit							ProcessStart(const QString& text, TreeItem* parent = 0);
	explicit							ProcessStart(const QVector<QVariant>& data, TreeItem* parent = 0);
	~ProcessStart(void);

public:
	virtual TreeItem*					clone() const;
	virtual void						Edit();
	virtual void						UpdateInfo();
	virtual void						SwitchState(ItemState state = ItemState::StateSave);
	virtual void						SetState(ItemState state)				{ m_state	= state; };
	virtual void                        SetMaps(map<QString, QString> maps)		{ m_maps	= maps;	 };
	virtual ItemType					GetType()								{ return	m_type;  };
	virtual ItemState					GetState()								{ return	m_state; };
	virtual map<QString, QString>       GetMaps()								{ return	m_maps;  };

private:

};
