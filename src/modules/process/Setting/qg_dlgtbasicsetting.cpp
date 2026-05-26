#include "qg_dlgtbasicsetting.h"

QG_dlgTBasicSetting::QG_dlgTBasicSetting(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);

	connect(ui.lineEdit_ToolName, SIGNAL(editingFinished()), this, SLOT(SetToolName()));
	connect(ui.lineEdit_CuttingHeight, SIGNAL(editingFinished()), this, SLOT(SetCuttingHeight()));
	connect(ui.lineEdit_CuttingHeightCompensate, SIGNAL(editingFinished()), this, SLOT(SetCuttingHeightCompensate()));
	connect(ui.lineEdit_IdleZHeight, SIGNAL(editingFinished()), this, SLOT(SetIdleZHeight()));
	connect(ui.checkBox_StopBlow, SIGNAL(currentIndexChanged(int)), this, SLOT(SetCheckBox_StopBlow(int)));
	connect(ui.checkBox_Trough, SIGNAL(currentIndexChanged(int)), this, SLOT(SetCheckBox_TroughFlag(int)));
	connect(ui.checkBox_Punch, SIGNAL(currentIndexChanged(int)), this, SLOT(SetCheckBox_PunchMode(int)));

	connect(ui.lineEdit_TroughBuffer, SIGNAL(editingFinished()), this, SLOT(SetTroughBuffer()));
	connect(ui.lineEdit_Extend, SIGNAL(editingFinished()), this, SLOT(SetExtend()));
	connect(ui.lineEdit_Extend_End, SIGNAL(editingFinished()), this, SLOT(SetExtend_End()));
	connect(ui.lineEdit_wait_first, SIGNAL(editingFinished()), this, SLOT(SetWaitFirst()));
	connect(ui.checkBox_Y_Zero, SIGNAL(currentIndexChanged(int)), this, SLOT(SetCheckBox_Y_Zero(int)));
	connect(ui.lineEdit_Y_Pos, SIGNAL(editingFinished()), this, SLOT(SetYPosition()));
	
	
	
}

QG_dlgTBasicSetting::~QG_dlgTBasicSetting()
{}

void QG_dlgTBasicSetting::SetToolName()
{

}

void QG_dlgTBasicSetting::SetCuttingHeight()
{

}

void QG_dlgTBasicSetting::SetIdleZHeight()
{

}

void QG_dlgTBasicSetting::SetCuttingHeightCompensate()
{

}

void QG_dlgTBasicSetting::SetCheckBox_StopBlow(int)
{

}

void QG_dlgTBasicSetting::SetCheckBox_TroughFlag(int)
{

}

void QG_dlgTBasicSetting::SetCheckBox_PunchMode(int)
{

}

void QG_dlgTBasicSetting::SetTroughBuffer()
{

}

void QG_dlgTBasicSetting::SetExtend()
{

}

void QG_dlgTBasicSetting::SetExtend_End()
{

}

void QG_dlgTBasicSetting::SetWaitFirst()
{

}

void QG_dlgTBasicSetting::SetCheckBox_Y_Zero(int)
{

}

void QG_dlgTBasicSetting::SetYPosition()
{

}
