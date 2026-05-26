#pragma once

#include <QDialog>
#include "ui_qg_dlgsmcsetting.h"

class QG_dlgSMCSetting : public QDialog
{
	Q_OBJECT

public:
	QG_dlgSMCSetting(QWidget *parent = nullptr);
	~QG_dlgSMCSetting();

private:
	Ui::QG_dlgSMCSettingClass ui;
};
