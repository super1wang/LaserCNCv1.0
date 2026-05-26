#pragma once

#include <QDialog>
#include "ui_Setting_Laser.h"
#include "Service.h"

class Dialog_Setting_Laser : public QDialog
{
	Q_OBJECT

public:
	Dialog_Setting_Laser(QWidget* parent = nullptr);
	~Dialog_Setting_Laser();

public:
	void ClearChange() { set_Changed.clear(); };
	void InitSetting();
	void SetPage(table table_Set = {});
	void GetPage(table& table_Page);
	bool GetChanged(table table_Page, table& table_Changed);

private:
	void setupLineEditValidators(QWidget* dialog);
	void setupComboBoxValidators(QWidget* dialog);

public slots:
	void UpdatePage();

private slots:
	void lineEditChanged();
	void comboBoxChanged();
	void checkBoxChanged();

public:
	Ui::Dialog_Setting_Laser	ui;

private:
	set<pair<string, string>>	set_Changed;
	QList<QLineEdit*>			m_qlLineEditS;
	QList<QLineEdit*>			m_qlLineEditI;
	QList<QLineEdit*>			m_qlLineEditF;
	QList<QComboBox*>			m_qlComboBoxS;
};
