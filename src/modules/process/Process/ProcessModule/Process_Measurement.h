#pragma once
#include "treeitem.h"
#include "ui_Process_Measurement.h"

class ProcessMeasurement :
	public TreeItem
{
public:
	explicit							ProcessMeasurement(TreeItem *parent = 0);
	explicit							ProcessMeasurement(const QString &text, TreeItem *parent = 0);
	explicit							ProcessMeasurement(const QVector<QVariant> &data, TreeItem *parent = 0);
	~ProcessMeasurement(void);

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
	void								SetMeasurementSource(QString& QstrValue)	{ m_maps["Source"] 	= QstrValue; };
	void								SetMeasurementIndex(QString& QstrValue)		{ m_maps["Index"] 	= QstrValue; };
	void								SetMeasurementX(QString& QstrValue)			{ m_maps["X"] 		= QstrValue; };
	void								SetMeasurementY(QString& QstrValue)			{ m_maps["Y"] 		= QstrValue; };

	QString								GetMeasurementSource()					{ return	m_maps["Source"]; };
	QString								GetMeasurementIndex()					{ return	m_maps["Index"]; };
	QString								GetMeasurementX()						{ return	m_maps["X"]; 	 };
	QString								GetMeasurementY()						{ return	m_maps["Y"]; 	 };

};

class Dialog_ProcessSetting_Measurement :
	public QDialog
{
	Q_OBJECT

public:
	explicit				Dialog_ProcessSetting_Measurement(QWidget* parent = 0);
	~Dialog_ProcessSetting_Measurement();

private:
	Ui::Dialog_ProcessSetting_Measurement*	Dialog_Measurement;

public slots:
	void					ViewSetting();
	void					ButtonOK();
	void					ButtonCancel();
	void					reject();

public:
	ProcessMeasurement*		m_pProcessMeasurement;
};
