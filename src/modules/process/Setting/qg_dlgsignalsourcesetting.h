#pragma once

#include <QDialog>
#include "ui_qg_dlgsignalsourcesetting.h"

class QG_dlgSignalSourceSetting : public QDialog
{
	Q_OBJECT

public:
	QG_dlgSignalSourceSetting(QWidget *parent = nullptr);
	~QG_dlgSignalSourceSetting();

private:
	Ui::QG_dlgSignalSourceSettingClass ui;
};
