#pragma once

#include <QDialog>
#include "ui_Setting_MotionControl.h"
#include "Service.h"
#include <QMessageBox>
#include <QInputDialog>

class Dialog_Setting_MotionControl : public QDialog
{
	Q_OBJECT

public:
	Dialog_Setting_MotionControl(QWidget* parent = nullptr);
	~Dialog_Setting_MotionControl();

public:
	void setUI();
	void ClearChange();
	void InitSetting();
	void SetPage(table table_Set = {});
	void GetPage(table& table_Page);
	bool GetChanged(table table_Page, table& table_Changed);

private:
	void populateAxisTable(const table& table_Set);
	void setupTableValidators(int row, int col, const QString& key);
	void rebuildAxisNames();
	bool isMachineAxis(const QString& name) const;

private slots:
	void onTableCellChanged(int row, int column);
	void onAddAxis();
	void onDeleteAxis();
	void TypeChanged();

public:
	Ui::Dialog_Setting_MotionControl	ui;

private:
	// Column index constants
	enum Column
	{
		Col_AxisName = 0,
		Col_Index,
		Col_HomeIndex,
		Col_Resolution,
		Col_LowSpeed,
		Col_MediumSpeed,
		Col_HighSpeed,
		Col_Acceleration,
		Col_Jerk,
		Col_LeftLimit,
		Col_RightLimit,
		Col_Count
	};

	set<pair<string, string>>			set_Changed;
	table								table_Temp;
	QStringList							m_axisNames;		// merged: machine + extension
	QStringList							m_machineAxisNames;	// axes from DT::IsAxisUse (machine config)
	QStringList							m_extensionAxisNames;// user-added extension axes
	bool								m_bGTN{false};
};
