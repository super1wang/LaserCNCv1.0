#pragma once
#include "treeitem.h"
#include "ui_Process_AutoFocus.h"

class ProcessAutoFocus :
	public TreeItem
{
public:
	explicit							ProcessAutoFocus(TreeItem* parent = 0);
	explicit							ProcessAutoFocus(const QString& text, TreeItem* parent = 0);
	explicit							ProcessAutoFocus(const QVector<QVariant> &data, TreeItem *parent = 0);
	~ProcessAutoFocus(void);

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
	void								SetAutoFocusNote			(QString& qstrValue)	{ m_maps["Note"]			= qstrValue; };
	void								SetAutoFocusStepInterval	(QString& qstrValue)	{ m_maps["StepInterval"]	= qstrValue; };
	void								SetAutoFocusEnergyStep		(QString& qstrValue)	{ m_maps["EnergyStep"]		= qstrValue; };
	void								SetAutoFocusFrequencyStep	(QString& qstrValue)	{ m_maps["FrequencyStep"]	= qstrValue; };
	void								SetAutoFocusPulseWidthStep	(QString& qstrValue)	{ m_maps["PulseWidthStep"]	= qstrValue; };
	void								SetAutoFocusCuttingHighStep	(QString& qstrValue)	{ m_maps["CuttingHighStep"] = qstrValue; };
	
	QString								GetAutoFocusNote()				{ return m_maps["Note"];			};
	QString								GetAutoFocusStepInterval()		{ return m_maps["StepInterval"];	};
	QString								GetAutoFocusEnergyStep()		{ return m_maps["EnergyStep"];		};
	QString								GetAutoFocusFrequencyStep()		{ return m_maps["FrequencyStep"];	};
	QString								GetAutoFocusPulseWidthStep()	{ return m_maps["PulseWidthStep"];	};
	QString								GetAutoFocusCuttingHighStep()	{ return m_maps["CuttingHighStep"]; };
};

class Dialog_ProcessSetting_AutoFocus :
	public QDialog
{
	Q_OBJECT

public:
	explicit				Dialog_ProcessSetting_AutoFocus(QWidget* parent = 0);
	~Dialog_ProcessSetting_AutoFocus();

private:
	Ui::Dialog_ProcessSetting_AutoFocus*		Dialog_AutoFocus;

public slots:
	void					ViewSetting();
	void					ButtonOK();
	void					ButtonCancel();
	void					reject();

public:
	ProcessAutoFocus*			m_pProcessAutoFocus;
};
