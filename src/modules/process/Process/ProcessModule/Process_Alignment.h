#pragma once

#include "treeitem.h"
#include <QDialog>

class QCheckBox;
class QComboBox;
class QPushButton;
class QSpinBox;
class QTableWidget;

class ProcessAlignment :
	public TreeItem
{
public:
	explicit							ProcessAlignment(TreeItem* parent = 0);
	explicit							ProcessAlignment(const QString& text, TreeItem* parent = 0);
	explicit							ProcessAlignment(const QVector<QVariant>& data, TreeItem* parent = 0);
	~ProcessAlignment(void);

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
	int								GetMode() const;
	void							SetMode(int mode);
	int								GetGroupCount() const;
	void							SetGroupCount(int count);
	QString							rowValue(int row, const QString& suffix) const;
	void							setRowValue(int row, const QString& suffix, const QString& value);
};

class Dialog_ProcessSetting_Alignment :
	public QDialog
{
public:
	explicit							Dialog_ProcessSetting_Alignment(QWidget* parent = 0);
	~Dialog_ProcessSetting_Alignment() override;

	void							ViewSetting();
	void							ButtonOK();
	void							ButtonCancel();
	void							reject() override;

	ProcessAlignment* m_pProcessAlignment = nullptr;

private:
	void							rebuildTable();
	void							populateCombos();
	QStringList					availableMarkIds() const;

private:
	QComboBox*    m_modeCombo = nullptr;
	QSpinBox*     m_groupCountSpin = nullptr;
	QTableWidget* m_table = nullptr;
	QPushButton*  m_btnOk = nullptr;
	QPushButton*  m_btnCancel = nullptr;
	ItemState     m_stateBeforeEdit = ItemState::Unavailable;
};
