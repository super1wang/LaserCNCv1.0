#pragma once

#include <QDialog>
#include "ui_Setting_Gas.h"
#include "Service.h"

class Dialog_Setting_Gas : public QDialog
{
	Q_OBJECT

public:
	Dialog_Setting_Gas(QWidget* parent = nullptr);
	~Dialog_Setting_Gas();

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

public:
	Ui::Dialog_Setting_Gas		ui;

private:
	set<pair<string, string>>	set_Changed;		// 记录修改值的Tabale及Key

	QList<QLineEdit*>			m_qlLineEditI;		// 存int	类型的LineEdit控件指针
	QList<QLineEdit*>			m_qlLineEditF;		// 存float类型的LineEdit控件指针
};
