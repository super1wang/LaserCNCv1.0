#pragma once
#include "treeitem.h"
#include "ui_Process_Commands.h"

class ProcessCommands :
	public TreeItem
{
public:
	explicit							ProcessCommands(TreeItem* parent = 0);
	explicit							ProcessCommands(const QString& text, TreeItem* parent = 0);
	explicit							ProcessCommands(const QVector<QVariant> &data, TreeItem *parent = 0);
	~ProcessCommands(void);

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
	void								SetCommandsNote	(QString&	qstrValue)		{ m_maps["Note"] = qstrValue; };
	void								SetCommandsFile	(QString&	qstrValue)		{ m_maps["File"] = qstrValue; };
	
	QString								GetCommandsNote()							{ return	m_maps["Note"];	};
	QString								GetCommandsFile()							{ return	m_maps["File"];	};

};

#include <QFileDialog>

class Dialog_ProcessSetting_Commands :
	public QDialog
{
	Q_OBJECT

public:
	explicit				Dialog_ProcessSetting_Commands(QWidget* parent = 0);
	~Dialog_ProcessSetting_Commands();

private:
	Ui::Dialog_ProcessSetting_Commands*		Dialog_Commands;

public slots:
	void					ViewSetting();
	void					ButtonOK();
	void					ButtonCancel();
	void					ButtonFile();
	void					reject();

public:
	ProcessCommands*			m_pProcessCommands;
};
