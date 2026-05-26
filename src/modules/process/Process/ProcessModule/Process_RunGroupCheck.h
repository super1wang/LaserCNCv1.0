#pragma once
#include "treeitem.h"
#include "ui_Process_RunGroupCheck.h"

class ProcessRunGroupCheck :
	public TreeItem
{
public:
	explicit							ProcessRunGroupCheck(TreeItem *parent = 0);
	explicit							ProcessRunGroupCheck(const QString &text, TreeItem *parent = 0);
	explicit							ProcessRunGroupCheck(const QVector<QVariant> &data, TreeItem *parent = 0);
	~ProcessRunGroupCheck(void);

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
	void								SetRunGroupCheckValue(double& dValue)			{ m_maps["Value"] = QString::number(dValue); };
	void								SetRunGroupCheckUnit(int& iValue)				{ m_maps["Unit"]  = QString::number(iValue); };

	double								GetRunGroupCheckValue()							{ return	m_maps["Value"].toDouble(); };
	int									GetRunGroupCheckUnit()							{ return	m_maps["Unit"].toInt();     };

};

class Dialog_ProcessSetting_RunGroupCheck :
	public QDialog
{
	Q_OBJECT

public:
	explicit				Dialog_ProcessSetting_RunGroupCheck(QWidget* parent = 0);
	~Dialog_ProcessSetting_RunGroupCheck();

private:
	Ui::Dialog_ProcessSetting_RunGroupCheck*		Dialog_RunGroupCheck;

public slots:
	void					ViewSetting();
	void					ButtonOK();
	void					ButtonCancel();
	void					reject();

public:
	ProcessRunGroupCheck*			m_pProcessRunGroupCheck;
};
