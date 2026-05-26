#pragma once
#include "treeitem.h"
#include "ui_Process_Group.h"

class ProcessGroup :
	public TreeItem
{
public:
	explicit							ProcessGroup(TreeItem *parent = 0);
	explicit							ProcessGroup(const QString &text, TreeItem *parent = 0);
	explicit							ProcessGroup(const QVector<QVariant> &data, TreeItem *parent = 0);
	~ProcessGroup(void);

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

public:
	void								SetGroupName(QString& QstrValue)		{ m_maps["Name"] = QstrValue; };

	QString								GetGroupName()							{ return	m_maps["Name"]; };

};

class Dialog_ProcessSetting_Group :
	public QDialog
{
	Q_OBJECT

public:
	explicit				Dialog_ProcessSetting_Group(QWidget* parent = 0);
	~Dialog_ProcessSetting_Group();

private:
	Ui::Dialog_ProcessSetting_Group*	Dialog_Group;

public slots:
	void					ViewSetting();
	void					ButtonOK();
	void					ButtonCancel();
	void					reject();

public:
	ProcessGroup*			m_pProcessGroup;
};
