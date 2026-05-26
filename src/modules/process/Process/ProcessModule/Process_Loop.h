#pragma once
#include "treeitem.h"
#include "ui_Process_Loop.h"

class ProcessLoop :
	public TreeItem
{
public:
	explicit							ProcessLoop(TreeItem* parent = 0);
	explicit							ProcessLoop(const QString& text, TreeItem* parent = 0);
	explicit							ProcessLoop(const QVector<QVariant>& data, TreeItem* parent = 0);
	~ProcessLoop(void);

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
	void								SetLoopNumber(int& iValue)				{ m_maps["Number"] = QString::number(iValue); };

	int									GetLoopNumber()							{ return	m_maps["Number"].toInt(); };

private:
	int						m_iNumber = 1;

};

class Dialog_ProcessSetting_Loop :
	public QDialog
{
	Q_OBJECT

public:
	explicit				Dialog_ProcessSetting_Loop(QWidget* parent = 0);
	~Dialog_ProcessSetting_Loop();

private:
	Ui::Dialog_ProcessSetting_Loop* Dialog_Loop;

public slots:
	void					ViewSetting();
	void					ButtonOK();
	void					ButtonCancel();
	void					reject();

public:
	ProcessLoop*			m_pProcessLoop;
};
