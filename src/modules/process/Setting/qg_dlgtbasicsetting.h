#pragma once

#include <QDialog>
#include "ui_qg_dlgtbasicsetting.h"

class QG_dlgTBasicSetting : public QDialog
{
	Q_OBJECT

public:
	QG_dlgTBasicSetting(QWidget *parent = nullptr);
	~QG_dlgTBasicSetting();

public slots:
	void	SetToolName();
	void	SetCuttingHeight();
	void	SetIdleZHeight();
	void	SetCuttingHeightCompensate();
	void	SetCheckBox_StopBlow(int);
	void	SetCheckBox_TroughFlag(int);
	void	SetCheckBox_PunchMode(int);
	void	SetTroughBuffer();
	void	SetExtend();
	void	SetExtend_End();
	void	SetWaitFirst();
	void	SetCheckBox_Y_Zero(int);
	void	SetYPosition();


private:
	Ui::QG_dlgOtherSettingClass ui;
};
