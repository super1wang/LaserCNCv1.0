#pragma once
#include "treeitem.h"
#include "ui_Process_If.h"

class ProcessIf :
	public TreeItem
{
public:
	explicit							ProcessIf(TreeItem* parent = 0);
	explicit							ProcessIf(const QString& text, TreeItem* parent = 0);
	explicit							ProcessIf(const QVector<QVariant>& data, TreeItem* parent = 0);
	~ProcessIf(void);

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
	void								SetIfNote	(QString& QstrValue)		{ m_maps["Note"] = QstrValue; };
	void								SetIfType	(int& iValue)				{ m_maps["Type"] = QString::number(iValue); };
	void								SetIfIO		(QString& QstrValue)		{ m_maps["IO"]	 = QstrValue; };
	void								SetIfOn		(QString& QstrValue)		{ m_maps["On"]   = QstrValue; };
	void								SetIfOff	(QString& QstrValue)		{ m_maps["Off"]  = QstrValue; };

	QString								GetIfNote()								{ return	m_maps["Note"]; };
	int									GetIfType()								{ return	m_maps["Type"].toInt(); };
	QString								GetIfIO()								{ return	m_maps["IO"];   };
	QString								GetIfOn()								{ return	m_maps["On"];   };
	QString								GetIfOff()								{ return	m_maps["Off"];  };

};

class Dialog_ProcessSetting_If :
	public QDialog
{
	Q_OBJECT

public:
	explicit				Dialog_ProcessSetting_If(QWidget* parent = 0);
	~Dialog_ProcessSetting_If();

private:
	Ui::Dialog_ProcessSetting_If*		Dialog_If;

public slots:
	void					ViewSetting();
	void					ButtonOK();
	void					ButtonCancel();
	void					UpdatePage();
	void					reject();

public:
	ProcessIf*				m_pProcessIf;
	QStringList				m_DigitalINList;
	QStringList				m_DigitalOUTList;
	int						m_iSaveIndex;
};
