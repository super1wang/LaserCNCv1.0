#pragma once

#include <QDialog>
#include "ui_Setting_Water.h"
#include "Service.h"

class Dialog_Setting_Water : public QDialog
{
	Q_OBJECT

public:
	Dialog_Setting_Water(QWidget* parent = nullptr);
	~Dialog_Setting_Water();

public:
	void ClearChange() { set_Changed.clear(); };
	void InitSetting();
	void SetPage(table table_Set = {});
	void GetPage(table& table_Page);
	bool GetChanged(table table_Page, table& table_Changed);

private:
	void setupLineEditValidators(QWidget* dialog);

private slots:
	void lineEditChanged();
	void checkBoxChanged();

private:
	Ui::Dialog_Setting_Water		ui;
	set<pair<string, string>>		set_Changed;		// 记录修改值的Tabale及Key
};
