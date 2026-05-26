#pragma once

#include <QDialog>
#include "ui_qg_dlgsensorsetting.h"

class QG_dlgSensorSetting : public QDialog
{
	Q_OBJECT

public:
	QG_dlgSensorSetting(QWidget *parent = nullptr);
	~QG_dlgSensorSetting();

private:
	Ui::QG_dlgSensorSettingClass ui;
};
