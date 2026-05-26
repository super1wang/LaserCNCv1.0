#pragma once
#include "treeitem.h"
#include "ui_Process_Compare.h"

class ProcessCompare :
	public TreeItem
{
public:
	explicit							ProcessCompare(TreeItem* parent = 0);
	explicit							ProcessCompare(const QString& text, TreeItem* parent = 0);
	explicit							ProcessCompare(const QVector<QVariant>& data, TreeItem* parent = 0);
	~ProcessCompare(void);

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
	void								SetCompareNote		(QString& QstrValue)		{ m_maps["Note"] 	= QstrValue; };
	void								SetCompareFormula	(QString& QstrValue)		{ m_maps["Formula"] = QstrValue; };
	void								SetCompareTrue		(QString& QstrValue)		{ m_maps["True"]	= QstrValue; };
	void								SetCompareFalse		(QString& QstrValue)		{ m_maps["False"]   = QstrValue; };

	QString								GetCompareNote()								{ return	m_maps["Note"]; 	};
	QString								GetCompareFormula()								{ return	m_maps["Formula"]; 	};
	QString								GetCompareTrue()								{ return	m_maps["True"];   	};
	QString								GetCompareFalse()								{ return	m_maps["False"];   	};

};

class Dialog_ProcessSetting_Compare :
	public QDialog
{
	Q_OBJECT

public:
	explicit				Dialog_ProcessSetting_Compare(QWidget* parent = 0);
	~Dialog_ProcessSetting_Compare();

private:
	Ui::Dialog_ProcessSetting_Compare*		Dialog_Compare;

public slots:
	void					ViewSetting();
	void					ButtonOK();
	void					ButtonCancel();
	void					reject();

public:
	ProcessCompare*			m_pProcessCompare;
};
