#pragma once
#include "treeitem.h"

class ProcessStop :
	public TreeItem
{
public:
	explicit							ProcessStop(TreeItem* parent = 0);
	explicit							ProcessStop(const QString& text, TreeItem* parent = 0);
	explicit							ProcessStop(const QVector<QVariant>& data, TreeItem* parent = 0);
	~ProcessStop(void);

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
