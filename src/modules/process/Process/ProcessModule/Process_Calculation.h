#pragma once
#include "treeitem.h"
#include "ui_Process_Calculation.h"

class ProcessCalculation :
	public TreeItem
{
public:
	explicit							ProcessCalculation(TreeItem *parent = 0);
	explicit							ProcessCalculation(const QString &text, TreeItem *parent = 0);
	explicit							ProcessCalculation(const QVector<QVariant> &data, TreeItem *parent = 0);
	~ProcessCalculation(void);

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
	void								SetCalculationIndex(QString& QstrValue)	{ m_maps["Index"] 	= QstrValue; };
	void								SetCalculationX(QString& QstrValue)		{ m_maps["X"] 		= QstrValue; };
	void								SetCalculationY(QString& QstrValue)		{ m_maps["Y"] 		= QstrValue; };

	QString								GetCalculationIndex()					{ return	m_maps["Index"]; };
	QString								GetCalculationX()						{ return	m_maps["X"]; 	 };
	QString								GetCalculationY()						{ return	m_maps["Y"]; 	 };

};

class Dialog_ProcessSetting_Calculation :
	public QDialog
{
	Q_OBJECT

public:
	explicit				Dialog_ProcessSetting_Calculation(QWidget* parent = 0);
	~Dialog_ProcessSetting_Calculation();

private:
	Ui::Dialog_ProcessSetting_Calculation*	Dialog_Calculation;

public slots:
	void					ViewSetting();
	void					ButtonOK();
	void					ButtonCancel();
	void					reject();

public:
	ProcessCalculation*		m_pProcessCalculation;
};
