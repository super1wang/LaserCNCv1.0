#pragma once

#include <QDialog>
#include "ui_qg_dlgautomationsetting.h"

class QG_dlgAutomationSetting : public QDialog
{
	Q_OBJECT

public:
	QG_dlgAutomationSetting(QWidget *parent = nullptr);
	~QG_dlgAutomationSetting();

private:
	Ui::QG_dlgAutomationSettingClass ui;
};
