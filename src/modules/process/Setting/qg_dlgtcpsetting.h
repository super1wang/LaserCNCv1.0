#pragma once

#include <QDialog>
#include "ui_qg_dlgtcpsetting.h"

class QG_dlgTCPSetting : public QDialog
{
	Q_OBJECT

public:
	QG_dlgTCPSetting(QWidget *parent = nullptr);
	~QG_dlgTCPSetting();

private:
	Ui::QG_dlgTCPSettingClass ui;
};
