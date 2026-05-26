#pragma once

#include <QDialog>
#include "ui_Setting_MotionControl.h"
#include "Service.h"
#include <qmessagebox.h>

class Dialog_Setting_MotionControl : public QDialog
{
	Q_OBJECT

public:
	Dialog_Setting_MotionControl(QWidget* parent = nullptr);
	~Dialog_Setting_MotionControl();

public: 
	void setUI();
	void ClearChange() { set_Changed.clear(); table_Temp.clear(); };
	void InitSetting();
	void SetPage(table table_Set = {});
	void GetPage(table& table_Page);
	bool GetChanged(table table_Page, table& table_Changed);

private:
	void CreatAxis(string strAxis, int iIndex, table& table);
	void setupLineEditValidators(QWidget* dialog);

private slots:
	void UpdatePage();
	void lineEditChanged();
	void comboBoxChanged();
	void TypeChanged();

public:
	Ui::Dialog_Setting_MotionControl	ui;

private:
	set<pair<string, string>>			set_Changed;		// 记录修改值的Tabale及Key
	string								str_Axis;			// 记录变化前的轴系选择
	table								table_Temp;			// 临时记录修改内容
};
