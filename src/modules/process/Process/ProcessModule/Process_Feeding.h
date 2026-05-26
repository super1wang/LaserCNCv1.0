#pragma once
#include "treeitem.h"
#include "ui_Process_Feeding.h"

class ProcessFeeding :
	public TreeItem
{
public:
	explicit							ProcessFeeding(TreeItem *parent = 0);
	explicit							ProcessFeeding(const QString &text, TreeItem *parent = 0);
	explicit							ProcessFeeding(const QVector<QVariant> &data, TreeItem *parent = 0);
	~ProcessFeeding(void);

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
	void								SetFeedingOpenChuck(QString& QstrValue)		{ m_maps["OpenChuck"]	 = QstrValue; };
	void								SetFeedingOpenPliers(QString& QstrValue)	{ m_maps["OpenPliers"]	 = QstrValue; };
	void								SetFeedingCloseChuck(QString& QstrValue)	{ m_maps["CloseChuck"]	 = QstrValue; };
	void								SetFeedingClosePliers(QString& QstrValue)	{ m_maps["ClosePliers"]	 = QstrValue; };
	void								SetFeedingCompensation(QString& QstrValue)	{ m_maps["Compensation"] = QstrValue; };
	void								SetFeedingAxisSpeed(int& iValue)			{ m_maps["AxisSpeed"]	 = QString::number(iValue); };


	QString								GetFeedingOpenChuck()		{ return	m_maps["OpenChuck"];		 };
	QString								GetFeedingOpenPliers()		{ return	m_maps["OpenPliers"];		 };
	QString								GetFeedingCloseChuck()		{ return	m_maps["CloseChuck"];		 };
	QString								GetFeedingClosePliers()		{ return	m_maps["ClosePliers"];		 };
	QString								GetFeedingCompensation()	{ return	m_maps["Compensation"];		 };
	int									GetFeedingAxisSpeed()		{ return	m_maps["AxisSpeed"].toInt(); };

};

class Dialog_ProcessSetting_Feeding :
	public QDialog
{
	Q_OBJECT

public:
	explicit				Dialog_ProcessSetting_Feeding(QWidget* parent = 0);
	~Dialog_ProcessSetting_Feeding();

private:
	Ui::Dialog_ProcessSetting_Feeding*	Dialog_Feeding;

public slots:
	void					ViewSetting();
	void					ButtonOK();
	void					ButtonCancel();
	void					reject();

public:
	ProcessFeeding*			m_pProcessFeeding;
};
