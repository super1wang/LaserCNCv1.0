#pragma once

#include <QDialog>
#include "ui_Setting_Axis.h"
#include "Service.h"

class Dialog_Setting_Axis : public QDialog
{
	Q_OBJECT

public:
	Dialog_Setting_Axis(QWidget* parent = nullptr);
	~Dialog_Setting_Axis();

public:
	void ClearChange() { set_Changed.clear();  table_Temp.clear(); };
	void InitSetting();
	void SetPage(table table_Set = {});
	void GetPage(table& table_Page);
	bool GetChanged(table table_Page, table& table_Changed);

private:
	void CreatAxis(string strAxis, table& table);
	void rebuildAxisChoices(const table& table_Set);
	void setupLineEditValidators(QWidget* dialog);

private slots:
	void UpdatePage();
	void lineEditChanged();
	void comboBoxChanged();

public:
	Ui::Dialog_Setting_Axis		ui;

private:
	set<pair<string, string>>	set_Changed;		// 记录修改值的Tabale及Key
	string						str_Axis;			// 记录变化前的轴系选择
	table						table_Temp;			// 临时记录修改内容

	QList<QLineEdit*>			m_qlLineEditF;		// 存float类型的LineEdit控件指针
};
