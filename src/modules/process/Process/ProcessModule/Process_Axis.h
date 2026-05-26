#pragma once
#include "treeitem.h"
#include "ui_Process_Axis.h"

class ProcessAxis :
	public TreeItem
{
public:
	explicit							ProcessAxis(TreeItem* parent = 0);
	explicit							ProcessAxis(const QString& text, TreeItem* parent = 0);
	explicit							ProcessAxis(const QVector<QVariant> &data, TreeItem *parent = 0);
	~ProcessAxis(void);

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
	void								SetAxisNote	(QString&	qstrValue)		{ m_maps["Note"]	= qstrValue;				};
	void								SetAxisAxis	(QString&	qstrValue)		{ m_maps["Axis"]	= qstrValue;				};
	void								SetAxisSpeed(int&		iValue)			{ m_maps["Speed"]	= QString::number(iValue);	};
	void								SetAxisModel(int&		iValue)			{ m_maps["Mode"]	= QString::number(iValue);	};
	void								SetAxisPos	(QString&	qstrValue)		{ m_maps["Pos"]		= qstrValue;				};
	
	QString								GetAxisNote()							{ return	m_maps["Note"];				};
	QString								GetAxisAxis()							{ return	m_maps["Axis"];				};
	int									GetAxisSpeed()							{ return	m_maps["Speed"].toInt();	};
	int									GetAxisModel()							{ return	m_maps["Mode"].toInt();     };
	QString								GetAxisPos()							{ return	m_maps["Pos"];				};

};

class Dialog_ProcessSetting_Axis :
	public QDialog
{
	Q_OBJECT

public:
	explicit				Dialog_ProcessSetting_Axis(QWidget* parent = 0);
	~Dialog_ProcessSetting_Axis();

private:
	Ui::Dialog_ProcessSetting_Axis*		Dialog_Axis;

public slots:
	void					ViewSetting();
	void					ButtonOK();
	void					ButtonCancel();
	void					reject();

public:
	ProcessAxis*			m_pProcessAxis;
};
