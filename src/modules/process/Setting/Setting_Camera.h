#pragma once

#include <QDialog>
#include "ui_Setting_Camera.h"
#include "Service.h"

class Dialog_Setting_Camera : public QDialog
{
	Q_OBJECT

public:
	Dialog_Setting_Camera(QWidget *parent = nullptr);
	~Dialog_Setting_Camera();

public:
	void ClearChange() { set_Changed.clear(); };
	void InitSetting();
	void SetPage(table table_Set = {});
	void GetPage(table& table_Page);
	bool GetChanged(table table_Page, table& table_Changed);
	void SetCommandNameEnabled(bool bEnabled);

private slots:
	void lineEditChanged();

private:
	void setupLineEditValidators(QWidget* dialog);

public:
	Ui::Dialog_Setting_Camera	ui;
	set<pair<string, string>>	set_Changed;		// 记录修改值的Tabale及Key

private:
	QList<QLineEdit*>			m_qlLineEditS;		// 存string类型的LineEdit控件指针
	QList<QLineEdit*>			m_qlLineEditI;		// 存int	类型的LineEdit控件指针
	QList<QLineEdit*>			m_qlLineEditF;		// 存float类型的LineEdit控件指针

};

