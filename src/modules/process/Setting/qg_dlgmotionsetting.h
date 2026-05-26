#pragma once

#include <QDialog>
#include "ui_qg_dlgmotionsetting.h"
#include "Service.h"

class QG_dlgMotionSetting : public QDialog
{
	Q_OBJECT

public:
	QG_dlgMotionSetting(QWidget *parent = nullptr);
	~QG_dlgMotionSetting();

public:
	void SetService(Service*);

public slots:
	
	void	SetLineVelocity();
	void	SetArcVelocity();
	void	SetLineAcc();
	void	SetLineJerk();
	void	SetJumpVelocity_X();
	void	SetJumpVelocity_Y();
	void	SetJumpVelocity_Z();
	void	SetJumpAcc();
	void	SetJumpJerk();
	void	SetCornerVelocity();
	void	SetCornerAngle();
	void	SetXsegVelocity();

private:
	Service* m_pService;

private:
	Ui::QG_dlgToolSettingClass ui;
};
