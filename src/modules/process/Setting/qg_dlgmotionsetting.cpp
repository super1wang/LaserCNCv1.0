#include "qg_dlgmotionsetting.h"

QG_dlgMotionSetting::QG_dlgMotionSetting(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::FramelessWindowHint);

	
	connect(ui.lineEdit_line_velocity, SIGNAL(editingFinished()), this, SLOT(SetLineVelocity()));
	connect(ui.lineEdit_line_acc, SIGNAL(editingFinished()), this, SLOT(SetLineAcc()));
	connect(ui.lineEdit_line_jerk, SIGNAL(editingFinished()), this, SLOT(SetLineJerk()));
	connect(ui.lineEdit_arc_velocity, SIGNAL(editingFinished()), this, SLOT(SetArcVelocity()));
	connect(ui.lineEdit_jump_velocity_x, SIGNAL(editingFinished()), this, SLOT(SetJumpVelocity_X()));
	connect(ui.lineEdit_jump_velocity_y, SIGNAL(editingFinished()), this, SLOT(SetJumpVelocity_Y()));
	connect(ui.lineEdit_jump_velocity_z, SIGNAL(editingFinished()), this, SLOT(SetJumpVelocity_Z()));
	connect(ui.lineEdit_jump_acc, SIGNAL(editingFinished()), this, SLOT(SetJumpAcc()));
	connect(ui.lineEdit_jump_jerk, SIGNAL(editingFinished()), this, SLOT(SetJumpJerk()));
	connect(ui.lineEdit_corner_velocity, SIGNAL(editingFinished()), this, SLOT(SetCornerVelocity()));
	connect(ui.lineEdit_corner_angle, SIGNAL(editingFinished()), this, SLOT(SetCornerAngle()));
	connect(ui.lineEdit_XSEG_velocity, SIGNAL(editingFinished()), this, SLOT(SetXsegVelocity()));
}

QG_dlgMotionSetting::~QG_dlgMotionSetting()
{}

void QG_dlgMotionSetting::SetService(Service* pService)
{
	m_pService = pService;
	
}

void QG_dlgMotionSetting::SetLineVelocity()
{
	
}

void QG_dlgMotionSetting::SetArcVelocity()
{
	
}

void QG_dlgMotionSetting::SetLineAcc()
{
	
}

void QG_dlgMotionSetting::SetLineJerk()
{
	
}

void QG_dlgMotionSetting::SetJumpVelocity_X()
{
	
}

void QG_dlgMotionSetting::SetJumpVelocity_Y()
{
	
}

void QG_dlgMotionSetting::SetJumpVelocity_Z()
{
	
}

void QG_dlgMotionSetting::SetJumpAcc()
{
	
}

void QG_dlgMotionSetting::SetJumpJerk()
{
	
}

void QG_dlgMotionSetting::SetCornerVelocity()
{
	
}

void QG_dlgMotionSetting::SetCornerAngle()
{
	
}

void QG_dlgMotionSetting::SetXsegVelocity()
{
	
}