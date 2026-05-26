#pragma once

#include <QDialog>
#include "ui_QG_dlgCuttingProcessSetting.h"

class QG_dlgCuttingProcessSetting : public QDialog
{
	Q_OBJECT

public:
	QG_dlgCuttingProcessSetting(QWidget *parent = nullptr);
	~QG_dlgCuttingProcessSetting();

private:
	Ui::QG_dlgCuttingProcessSettingClass ui;
};
