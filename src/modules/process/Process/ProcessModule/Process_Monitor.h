#pragma once
#include "treeitem.h"
#include "ui_Process_Monitor.h"

class ProcessMonitor :
	public TreeItem
{
public:
	explicit							ProcessMonitor(TreeItem* parent = 0);
	explicit							ProcessMonitor(const QString& text, TreeItem* parent = 0);
	explicit							ProcessMonitor(const QVector<QVariant>& data, TreeItem* parent = 0);
	~ProcessMonitor(void);

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
	void								SetMonitorInterLock(bool& bValue)				{ m_maps["InterLock"]			= QString::number(bValue); };
	void								SetMonitorPressure(bool& bValue)				{ m_maps["Pressure"]			= QString::number(bValue); };
	void								SetMonitorWaterLeakage(bool& bValue)			{ m_maps["WaterLeakage"]		= QString::number(bValue); };
	void								SetMonitorWaterPressure(bool& bValue)			{ m_maps["WaterPressure"]		= QString::number(bValue); };
	void								SetMonitorWaterPressureLimit(double& dValue)	{ m_maps["WaterPressureLimit"]	= QString::number(dValue); };
	void								SetMonitorWaterTankError(bool& bValue)			{ m_maps["WaterTankError"]		= QString::number(bValue); };
	void								SetMonitorWaterTankLevel(bool& bValue)			{ m_maps["WaterTankLevel"]		= QString::number(bValue); };
	void								SetMonitorWaterTankLevelLimit(double& dValue)	{ m_maps["WaterTankLevelLimit"] = QString::number(dValue); };

	bool								GetMonitorInterLock()							{ return	m_maps["InterLock"].toInt();			  };
	bool								GetMonitorPressure()							{ return	m_maps["Pressure"].toInt();				  };
	bool								GetMonitorWaterLeakage()						{ return	m_maps["WaterLeakage"].toInt();			  };
	bool								GetMonitorWaterPressure()						{ return	m_maps["WaterPressure"].toInt();		  };
	double								GetMonitorWaterPressureLimit()					{ return	m_maps["WaterPressureLimit"].toDouble();  };
	bool								GetMonitorWaterTankError()						{ return	m_maps["WaterTankError"].toInt();		  };
	bool								GetMonitorWaterTankLevel()						{ return	m_maps["WaterTankLevel"].toInt();		  };
	double								GetMonitorWaterTankLevelLimit()					{ return	m_maps["WaterTankLevelLimit"].toDouble(); };

};

class Dialog_ProcessSetting_Monitor :
	public QDialog
{
	Q_OBJECT

public:
	explicit				Dialog_ProcessSetting_Monitor(QWidget* parent = 0);
	~Dialog_ProcessSetting_Monitor();

private:
	Ui::Dialog_ProcessSetting_Monitor*	Dialog_Monitor;

public slots:
	void					ViewSetting();
	void					ButtonOK();
	void					ButtonCancel();
	void					reject();

public:
	ProcessMonitor*			m_pProcessMonitor;
};
