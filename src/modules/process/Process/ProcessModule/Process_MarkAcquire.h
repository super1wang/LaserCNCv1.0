#pragma once

#include "treeitem.h"
#include <QDialog>

class QComboBox;
class QHeaderView;
class QPushButton;
class QSpinBox;
class QTableWidget;

class ProcessMarkAcquire :
	public TreeItem
{
public:
	explicit							ProcessMarkAcquire(TreeItem* parent = 0);
	explicit							ProcessMarkAcquire(const QString& text, TreeItem* parent = 0);
	explicit							ProcessMarkAcquire(const QVector<QVariant>& data, TreeItem* parent = 0);
	~ProcessMarkAcquire(void);

public:
	virtual TreeItem*				clone() const;
	virtual void					Edit();
	virtual void					UpdateInfo();
	virtual void					SwitchState(ItemState state = ItemState::StateSave);
	virtual void					SetState(ItemState state)				{ m_state = state; };
	virtual void					SetMaps(map<QString, QString> maps)	{ m_maps = maps; };
	virtual ItemType				GetType()								{ return m_type; };
	virtual ItemState				GetState()							{ return m_state; };
	virtual map<QString, QString>	GetMaps()							{ return m_maps; };

public:
	int								GetCount() const;
	void							SetCount(int count);
	QString							rowValue(int row, const QString& suffix) const;
	void							setRowValue(int row, const QString& suffix, const QString& value);
};

class Dialog_ProcessSetting_MarkAcquire :
	public QDialog
{
public:
	explicit							Dialog_ProcessSetting_MarkAcquire(QWidget* parent = 0);
	~Dialog_ProcessSetting_MarkAcquire() override;

	void							ViewSetting();
	void							ButtonOK();
	void							ButtonCancel();
	void							reject() override;

	ProcessMarkAcquire* m_pProcessMarkAcquire = nullptr;

private:
	void							rebuildTable();
	void							populateCombos();
	QStringList					availableMarkIds() const;
	QStringList					availableWorkflowIds() const;

private:
	QSpinBox*     m_countSpin = nullptr;
	QTableWidget* m_table = nullptr;
	QPushButton*  m_btnOk = nullptr;
	QPushButton*  m_btnCancel = nullptr;
	ItemState     m_stateBeforeEdit = ItemState::Unavailable;
};
