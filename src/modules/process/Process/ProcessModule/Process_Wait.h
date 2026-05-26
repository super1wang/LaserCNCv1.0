#pragma once
#include "treeitem.h"
#include "ui_Process_Wait.h"

class ProcessWait :
	public TreeItem
{
public:
	explicit							ProcessWait(TreeItem *parent = 0);
	explicit							ProcessWait(const QString &text, TreeItem *parent = 0);
	explicit							ProcessWait(const QVector<QVariant> &data, TreeItem *parent = 0);
	~ProcessWait(void);

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
	void								SetWaitValue(double& dValue)			{ m_maps["Value"] = QString::number(dValue); };
	void								SetWaitUnit(int& iValue)				{ m_maps["Unit"]  = QString::number(iValue); };

	double								GetWaitValue()							{ return	m_maps["Value"].toDouble(); };
	int									GetWaitUnit()							{ return	m_maps["Unit"].toInt();     };

};

class Dialog_ProcessSetting_Wait :
	public QDialog
{
	Q_OBJECT

public:
	explicit				Dialog_ProcessSetting_Wait(QWidget* parent = 0);
	~Dialog_ProcessSetting_Wait();

private:
	Ui::Dialog_ProcessSetting_Wait*		Dialog_Wait;

public slots:
	void					ViewSetting();
	void					ButtonOK();
	void					ButtonCancel();
	void					reject();

public:
	ProcessWait*			m_pProcessWait;
};
