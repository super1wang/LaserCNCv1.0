#pragma once
#include "treeitem.h"
#include "ui_Process_RunGroup.h"

class ProcessRunGroup :
	public TreeItem
{
public:
	explicit							ProcessRunGroup(TreeItem *parent = 0);
	explicit							ProcessRunGroup(const QString &text, TreeItem *parent = 0);
	explicit							ProcessRunGroup(const QVector<QVariant> &data, TreeItem *parent = 0);
	~ProcessRunGroup(void);

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
	void								SetRunGroupName(QString& QstrValue)		{ m_maps["Name"]	= QstrValue; };
	void								SetRunGroupThread(QString& QstrValue)	{ m_maps["Thread"]	= QstrValue; };

	QString								GetRunGroupName()						{ return	m_maps["Name"];		};
	QString								GetRunGroupThread()						{ return	m_maps["Thread"];	};

};

class Dialog_ProcessSetting_RunGroup :
	public QDialog
{
	Q_OBJECT

public:
	explicit				Dialog_ProcessSetting_RunGroup(QWidget* parent = 0);
	~Dialog_ProcessSetting_RunGroup();

private:
	Ui::Dialog_ProcessSetting_RunGroup*	Dialog_RunGroup;

public slots:
	void					ViewSetting();
	void					ButtonOK();
	void					ButtonCancel();
	void					reject();

public:
	ProcessRunGroup*		m_pProcessRunGroup;
};
