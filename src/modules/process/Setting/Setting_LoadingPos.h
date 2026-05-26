#pragma once

#include <QDialog>
#include "ui_Setting_LoadingPos.h"
#include "Service.h"

class Dialog_Setting_LoadingPos : public QDialog
{
	Q_OBJECT

public:
	Dialog_Setting_LoadingPos(QWidget *parent = nullptr);
	~Dialog_Setting_LoadingPos();

public:
	void ClearChange() { set_Changed.clear(); };
	void InitSetting();
	void SetPage(table table_Set = {});
	void GetPage(table& table_Page);
	bool GetChanged(table table_Page, table& table_Changed);

private slots:
	void lineEditChanged();
	void checkBoxChanged();

private:
	void setupLineEditValidators(QWidget* dialog);
	void setupCheckBoxValidators(QWidget* dialog);

public:
	Ui::Dialog_Setting_LoadingPos	ui;

private:
	set<pair<string, string>>		set_Changed;		// 记录修改值的Tabale及Key
	QList<QLineEdit*>				m_qlLineEditF;		// 存float类型的LineEdit控件指针
	QList<QCheckBox*>				m_qlCheckBoxB;		// 存bool类型的CheckBox控件指针

};

