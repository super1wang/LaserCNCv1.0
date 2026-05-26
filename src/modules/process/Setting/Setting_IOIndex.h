#pragma once

#include <QDialog>
#include "ui_Setting_IOIndex.h"
#include "Service.h"

class Dialog_Setting_IOIndex : public QDialog
{
	Q_OBJECT

public:
	Dialog_Setting_IOIndex(QWidget* parent = nullptr);
	~Dialog_Setting_IOIndex();

public:
	void ClearChange() { set_Changed.clear(); };
	void InitSetting();
	void SetPage(table table_SetD = {}, table table_SetA = {});
	void GetPage(table& table_PageD, table& table_PageA);
	bool GetChanged(table table_PageD, table table_PageA, table& table_ChangedD, table& table_ChangedA);
	void SetIndexEnabled(bool bEnabled);

private:
	void setupLineEditValidators(QWidget* dialog);

private slots:
	void lineEditChanged();

private:
	Ui::Dialog_Setting_IOIndex	ui;
	set<pair<string, string>>	set_Changed;		// 记录修改值的Tabale及Key
};
