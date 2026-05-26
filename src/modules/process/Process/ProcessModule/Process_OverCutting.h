#pragma once
#include "treeitem.h"
#include "ui_Process_OverCutting.h"

class ProcessOverCutting :
	public TreeItem
{
public:
	explicit							ProcessOverCutting(TreeItem *parent = 0);
	explicit							ProcessOverCutting(const QString &text, TreeItem *parent = 0);
	explicit							ProcessOverCutting(const QVector<QVariant> &data, TreeItem *parent = 0);
	~ProcessOverCutting(void);

public:
	virtual TreeItem*					clone() const;
	virtual void						Edit();
	virtual void						UpdateInfo();
	virtual void						SwitchState(ItemState state = ItemState::StateSave);
	virtual void						SetState(ItemState state)				{ m_state	= state; };
	virtual void                        SetMaps(map<QString, QString> maps)		{
		m_maps = maps;
		m_maps.erase("AutoDivision");
		m_maps.erase("SafetyMargin");
		m_maps.erase("MinSegLen");
		if (m_maps.find("SuggestedLen") == m_maps.end())
			m_maps["SuggestedLen"] = "0";
		if (m_maps.find("EqualLength") == m_maps.end())
			m_maps["EqualLength"] = "1";
	};
	virtual ItemType					GetType()								{ return	m_type;  };
	virtual ItemState					GetState()								{ return	m_state; };
	virtual map<QString, QString>       GetMaps()								{ return	m_maps;  };

public:
	void 								SetOverCuttingNote			(QString& QstrValue)	{ m_maps["Note"]			= QstrValue; };
	void								SetOverCuttingCount			(bool bValue)			{ m_maps["Count"]			= QString::number(bValue); }
	void								SetOverCuttingLayer			(QString& QstrValue)	{ m_maps["Layer"]			= QstrValue; };
	void								SetOverCuttingXCompensation	(QString& QstrValue)	{ m_maps["XCompensation"]	= QstrValue; };
	void								SetOverCuttingXSpeed		(int& iValue)			{ m_maps["XSpeed"]			= QString::number(iValue); };
	void								SetOverCuttingYCompensation	(QString& QstrValue)	{ m_maps["YCompensation"]	= QstrValue; };
	void								SetOverCuttingYSpeed		(int& iValue)			{ m_maps["YSpeed"]			= QString::number(iValue); };
	void								SetOverCuttingOpenChuck		(QString& QstrValue)	{ m_maps["OpenChuck"]		= QstrValue; };
	void								SetOverCuttingOpenPliers	(QString& QstrValue)	{ m_maps["OpenPliers"]		= QstrValue; };
	void								SetOverCuttingCloseChuck	(QString& QstrValue)	{ m_maps["CloseChuck"]		= QstrValue; };
	void								SetOverCuttingClosePliers	(QString& QstrValue)	{ m_maps["ClosePliers"]		= QstrValue; };
	void								SetOverCuttingEqualLength	(bool bValue)			{ m_maps["EqualLength"]	= QString::number(bValue); };
	void								SetOverCuttingSuggestedLen	(QString& QstrValue)	{
		m_maps.erase("AutoDivision");
		m_maps.erase("SafetyMargin");
		m_maps.erase("MinSegLen");
		m_maps["SuggestedLen"] = QstrValue;
	};

	QString								GetOverCuttingNote()			{ return	m_maps["Note"];						};
	bool								GetOverCuttingCount()			{ return	QVariant(m_maps["Count"]).toBool(); };
	QString								GetOverCuttingLayer()			{ return	m_maps["Layer"];					};
	QString								GetOverCuttingXCompensation()	{ return	m_maps["XCompensation"];			};
	int									GetOverCuttingXSpeed()			{ return	m_maps["XSpeed"].toInt();			};
	QString								GetOverCuttingYCompensation()	{ return	m_maps["YCompensation"];			};
	int									GetOverCuttingYSpeed()			{ return	m_maps["YSpeed"].toInt();			};
	QString								GetOverCuttingOpenChuck()		{ return	m_maps["OpenChuck"];				};
	QString								GetOverCuttingOpenPliers()		{ return	m_maps["OpenPliers"];				};
	QString								GetOverCuttingCloseChuck()		{ return	m_maps["CloseChuck"];				};
	QString								GetOverCuttingClosePliers()		{ return	m_maps["ClosePliers"];				};
	bool								GetOverCuttingEqualLength()		{ return	QVariant(m_maps["EqualLength"]).toBool(); };
	QString								GetOverCuttingSuggestedLen()	{ return	m_maps["SuggestedLen"];			};


};

class Dialog_ProcessSetting_OverCutting :
	public QDialog
{
	Q_OBJECT

public:
	explicit				Dialog_ProcessSetting_OverCutting(QWidget* parent = 0);
	~Dialog_ProcessSetting_OverCutting();

private:
	Ui::Dialog_ProcessSetting_OverCutting*	Dialog_OverCutting;

public slots:
	void					ViewSetting();
	void					ButtonOK();
	void					ButtonCancel();
	void					ButtonAutoDivision();
	void					reject();

public:
	ProcessOverCutting*			m_pProcessOverCutting;
};
