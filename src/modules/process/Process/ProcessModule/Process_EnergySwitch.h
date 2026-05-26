#pragma once
#include "treeitem.h"
#include "ui_Process_EnergySwitch.h"

class ProcessEnergySwitch :
	public TreeItem
{
public:
	explicit							ProcessEnergySwitch(TreeItem* parent = 0);
	explicit							ProcessEnergySwitch(const QString& text, TreeItem* parent = 0);
	explicit							ProcessEnergySwitch(const QVector<QVariant>& data, TreeItem* parent = 0);
	~ProcessEnergySwitch(void);

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
	void								SetEnergySwitchPpDivider1(QString& qstrValue)	{ m_maps["PpDivider1"]	= qstrValue; };
	void								SetEnergySwitchPpDivider2(QString& qstrValue)	{ m_maps["PpDivider2"]	= qstrValue; };
	void								SetEnergySwitchPpDivider3(QString& qstrValue)	{ m_maps["PpDivider3"]	= qstrValue; };
	void								SetEnergySwitchPpDivider4(QString& qstrValue)	{ m_maps["PpDivider4"]	= qstrValue; };
	void								SetEnergySwitchPpDivider5(QString& qstrValue)	{ m_maps["PpDivider5"]	= qstrValue; };
	void								SetEnergySwitchPpDivider6(QString& qstrValue)	{ m_maps["PpDivider6"]	= qstrValue; };
	void								SetEnergySwitchPpDivider7(QString& qstrValue)	{ m_maps["PpDivider7"]	= qstrValue; };
	void								SetEnergySwitchPpDivider8(QString& qstrValue)	{ m_maps["PpDivider8"]	= qstrValue; };
	void								SetEnergySwitchPpDivider9(QString& qstrValue)	{ m_maps["PpDivider9"]	= qstrValue; };
	void								SetEnergySwitchDuration1(QString& qstrValue)	{ m_maps["Duration1"]	= qstrValue; };
	void								SetEnergySwitchDuration2(QString& qstrValue)	{ m_maps["Duration2"]	= qstrValue; };
	void								SetEnergySwitchDuration3(QString& qstrValue)	{ m_maps["Duration3"]	= qstrValue; };
	void								SetEnergySwitchDuration4(QString& qstrValue)	{ m_maps["Duration4"]	= qstrValue; };
	void								SetEnergySwitchDuration5(QString& qstrValue)	{ m_maps["Duration5"]	= qstrValue; };
	void								SetEnergySwitchDuration6(QString& qstrValue)	{ m_maps["Duration6"]	= qstrValue; };
	void								SetEnergySwitchDuration7(QString& qstrValue)	{ m_maps["Duration7"]	= qstrValue; };
	void								SetEnergySwitchDuration8(QString& qstrValue)	{ m_maps["Duration8"]	= qstrValue; };
	void								SetEnergySwitchDuration9(QString& qstrValue)	{ m_maps["Duration9"]	= qstrValue; };

	QString								GetEnergySwitchPpDivider1()				{ return	m_maps["PpDivider1"];		 };
	QString								GetEnergySwitchPpDivider2()				{ return	m_maps["PpDivider2"];		 };
	QString								GetEnergySwitchPpDivider3()				{ return	m_maps["PpDivider3"];		 };
	QString								GetEnergySwitchPpDivider4()				{ return	m_maps["PpDivider4"];		 };
	QString								GetEnergySwitchPpDivider5()				{ return	m_maps["PpDivider5"];		 };
	QString								GetEnergySwitchPpDivider6()				{ return	m_maps["PpDivider6"];		 };
	QString								GetEnergySwitchPpDivider7()				{ return	m_maps["PpDivider7"];		 };
	QString								GetEnergySwitchPpDivider8()				{ return	m_maps["PpDivider8"];		 };
	QString								GetEnergySwitchPpDivider9()				{ return	m_maps["PpDivider9"];		 };
	QString								GetEnergySwitchDuration1()				{ return	m_maps["Duration1"];		 };
	QString								GetEnergySwitchDuration2()				{ return	m_maps["Duration2"];		 };
	QString								GetEnergySwitchDuration3()				{ return	m_maps["Duration3"];		 };
	QString								GetEnergySwitchDuration4()				{ return	m_maps["Duration4"];		 };
	QString								GetEnergySwitchDuration5()				{ return	m_maps["Duration5"];		 };
	QString								GetEnergySwitchDuration6()				{ return	m_maps["Duration6"];		 };
	QString								GetEnergySwitchDuration7()				{ return	m_maps["Duration7"];		 };
	QString								GetEnergySwitchDuration8()				{ return	m_maps["Duration8"];		 };
	QString								GetEnergySwitchDuration9()				{ return	m_maps["Duration9"];		 };
};

class Dialog_ProcessSetting_EnergySwitch :
	public QDialog
{
	Q_OBJECT

public:
	explicit				Dialog_ProcessSetting_EnergySwitch(QWidget* parent = 0);
	~Dialog_ProcessSetting_EnergySwitch();

private:
	Ui::Dialog_ProcessSetting_EnergySwitch*		Dialog_EnergySwitch;

public slots:
	void					ViewSetting();
	void					ButtonOK();
	void					ButtonCancel();
	void					reject();

public:
	ProcessEnergySwitch*	m_pProcessEnergySwitch;
};
