#pragma once
#include "treeitem.h"
#include "ui_Process_IO.h"

class ProcessIO :
	public TreeItem
{
public:
	explicit							ProcessIO(TreeItem* parent = 0);
	explicit							ProcessIO(const QString& text, TreeItem* parent = 0);
	explicit							ProcessIO(const QVector<QVariant>& data, TreeItem* parent = 0);
	~ProcessIO(void);

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
	void								SetIONote	(QString& QstrValue)		{ m_maps["Note"]	= QstrValue;	};
	void								SetIOType	(int& iValue)				{ m_maps["Type"]	= QString::number(iValue); };
	void								SetIOIO		(QString& QstrValue)		{ m_maps["IO"] = QstrValue; };
	void								SetIOValue	(QString& QstrValue)		{ m_maps["Value"]	= QstrValue;	};

	QString								GetIONote()								{ return	m_maps["Note"];			};
	int									GetIOType()								{ return	m_maps["Type"].toInt(); };
	QString								GetIOIO()								{ return	m_maps["IO"]; };
	QString								GetIOValue()							{ return	m_maps["Value"];		};

};

class Dialog_ProcessSetting_IO :
	public QDialog
{
	Q_OBJECT

public:
	explicit				Dialog_ProcessSetting_IO(QWidget* parent = 0);
	~Dialog_ProcessSetting_IO();

private:
	Ui::Dialog_ProcessSetting_IO*		Dialog_IO;

public slots:
	void					ViewSetting();
	void					ButtonOK();
	void					ButtonCancel();
	void					UpdatePage();
	void					UpdateLineEdit();
	void					reject();

public:
	ProcessIO*				m_pProcessIO;
	QStringList				m_DigitalList;
	QStringList				m_AnalogList;
	int						m_iSaveIndex;
	bool					m_bIONone;
};
