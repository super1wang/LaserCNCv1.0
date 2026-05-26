#pragma once

#include <QDialog>
#include "ui_qg_dlgjsonsetting.h"

class QG_dlgJsonSetting : public QDialog
{
	Q_OBJECT

public:
	QG_dlgJsonSetting(QWidget *parent = nullptr);
	~QG_dlgJsonSetting();

private:
	Ui::QG_dlgJsonSettingClass ui;
};
