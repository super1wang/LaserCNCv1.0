#pragma once

#include <QDialog>
#include "ui_Setting_Digital.h"
#include "Service.h"

class Dialog_Setting_Digital : public QDialog
{
	Q_OBJECT

public:
	Dialog_Setting_Digital(QWidget* parent = nullptr);
	~Dialog_Setting_Digital();

public:
	void ClearChange() { set_Changed.clear(); };
	void InitSetting();
	void SetPage(table table_Set = {});
	void GetPage(table& table_Page);
	bool GetChanged(table table_Page, table& table_Changed);
	void SetIDEnabled(bool bEnabled);
	void SetIndexEnabled(bool bEnabled);

private:
	void addRow(bool bIN);
	void setRows(bool bIN, int totalRows);

public slots:
	void lineEditChanged();

private:
	Ui::Dialog_Setting_Digital	ui;
	set<pair<string, string>>	set_Changed;		// 记录修改值的Tabale及Key

	int		m_iINRowCount;
	int		m_iOUTRowCount;
};
