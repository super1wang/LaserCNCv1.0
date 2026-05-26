#pragma once
#include "treeitem.h"
#include "ui_Process_Cutting.h"

class ProcessCutting :
	public TreeItem
{
public:
	explicit							ProcessCutting(TreeItem *parent = 0);
	explicit							ProcessCutting(const QString &text, TreeItem *parent = 0);
	explicit							ProcessCutting(const QVector<QVariant> &data, TreeItem *parent = 0);
	~ProcessCutting(void);

public:
	virtual TreeItem*					clone() const;
	virtual void						Edit();
	virtual void						UpdateInfo();
	virtual void						SwitchState(ItemState state = ItemState::StateSave);
	virtual void						SetState(ItemState state)					{ m_state	= state; };
	virtual void                        SetMaps(map<QString, QString> maps)			{ m_maps	= maps;	 };
	virtual ItemType					GetType()									{ return	m_type;  };
	virtual ItemState					GetState()									{ return	m_state; };
	virtual map<QString, QString>       GetMaps()									{ return	m_maps;  };

public:
	void								SetCuttingNote				(QString& QstrValue)	{ m_maps["Note"]				= QstrValue; };
	void								SetCuttingCount				(bool bValue)			{ m_maps["Count"]				= QString::number(bValue); };
	void								SetCuttingStartNumber		(QString& QstrValue)	{ m_maps["StartNumber"]			= QstrValue; };
	void								SetCuttingEndNumber			(QString& QstrValue)	{ m_maps["EndNumber"]			= QstrValue; };
	void								SetCuttingCompensationIndex	(QString& QstrValue)	{ m_maps["CompensationIndex"]	= QstrValue; };


	QString								GetCuttingNote()							{ return	m_maps["Note"];				};
	bool								GetCuttingCount()							{ return	QVariant(m_maps["Count"]).toBool(); };
	QString								GetCuttingStartNumber()						{ return	m_maps["StartNumber"];		};
	QString								GetCuttingEndNumber()						{ return	m_maps["EndNumber"];		};
	QString								GetCuttingCompensationIndex()				{ return	m_maps["CompensationIndex"];};
};

class Dialog_ProcessSetting_Cutting :
	public QDialog
{
	Q_OBJECT

public:
	explicit				Dialog_ProcessSetting_Cutting(QWidget* parent = 0);
	~Dialog_ProcessSetting_Cutting();

private:
	Ui::Dialog_ProcessSetting_Cutting*	Dialog_Cutting;

public slots:
	void					ViewSetting();
	void					ButtonOK();
	void					ButtonCancel();
	void					reject();

public:
	ProcessCutting*			m_pProcessCutting;
};

