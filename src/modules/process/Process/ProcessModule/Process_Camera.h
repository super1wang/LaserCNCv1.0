#pragma once
#include "treeitem.h"
#include "ui_Process_Camera.h"

class ProcessCamera :
	public TreeItem
{
public:
	explicit							ProcessCamera(TreeItem* parent = 0);
	explicit							ProcessCamera(const QString& text, TreeItem* parent = 0);
	explicit							ProcessCamera(const QVector<QVariant>& data, TreeItem* parent = 0);
	~ProcessCamera(void);

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
	void								SetWorkflowID(const QString& value)		{ m_maps["WorkflowID"] = value; m_maps.erase("Tool"); };

	QString							GetWorkflowID() const					{
		auto it = m_maps.find("WorkflowID");
		if (it != m_maps.end()) return it->second;
		auto legacyIt = m_maps.find("Tool");
		return legacyIt != m_maps.end() ? legacyIt->second : QString();
	};

};

class Dialog_ProcessSetting_Camera :
	public QDialog
{
	Q_OBJECT

public:
	explicit				Dialog_ProcessSetting_Camera(QWidget* parent = 0);
	~Dialog_ProcessSetting_Camera();

private:
	Ui::Dialog_ProcessSetting_Camera*	Dialog_Camera;

public slots:
	void					ViewSetting();
	void					ButtonOK();
	void					ButtonCancel();
	void					reject();

public:
	ProcessCamera*			m_pProcessCamera;
};