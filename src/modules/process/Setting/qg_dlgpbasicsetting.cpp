#include "qg_dlgpbasicsetting.h"

QG_dlgPBasicSettings::QG_dlgPBasicSettings(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::FramelessWindowHint);

	connect(ui.lineEdit_Z_Position, SIGNAL(editingFinished()), this, SLOT(SetZPosition()));
	connect(ui.lineEdit_X_Position, SIGNAL(editingFinished()), this, SLOT(SetXPosition()));
	connect(ui.lineEdit_Y_Position, SIGNAL(editingFinished()), this, SLOT(SetYPosition()));
	connect(ui.checkBox_MoveAfterZCutting, SIGNAL(toggled(bool)), this, SLOT(SetCheckBox_MoveAfterZCutting(bool)));
	connect(ui.checkBox_MoveAfterXCutting, SIGNAL(toggled(bool)), this, SLOT(SetCheckBox_MoveAfterXCuttingFlag(bool)));
	connect(ui.checkBox_MoveAfterYCutting, SIGNAL(toggled(bool)), this, SLOT(SetCheckBox_MoveAfterYCuttingFlag(bool)));
	connect(ui.lineEdit_MoveAfterWaiting, SIGNAL(editingFinished()), this, SLOT(SetCheckBox_MoveAfterWaiting()));
	connect(ui.checkBox_ChuckClosedAfterCutting, SIGNAL(toggled()), this, SLOT(SetCheckBox_ChuckClosedAfterCuttingFlag(bool)));

	connect(ui.checkBox_Feeding, SIGNAL(toggled()), this, SLOT(SetFeeding(bool)));
	connect(ui.lineEdit_FeedingNum, SIGNAL(editingFinished()), this, SLOT(SetFeedingNum()));
	connect(ui.lineEdit_FeedingDistance, SIGNAL(editingFinished()), this, SLOT(SetFeedingDistance()));
	connect(ui.checkBox_MoveAfterFeedingY, SIGNAL(toggled()), this, SLOT(SetCheckBox_MoveAfterFeedingY(bool)));
	connect(ui.lineEdit_Position_FeedingY, SIGNAL(editingFinished()), this, SLOT(SetPosition_FeedingY()));
	connect(ui.lineEdit_Pliers_ClosedDelayed, SIGNAL(editingFinished()), this, SLOT(SetPliersClosedDelayed()));
	connect(ui.lineEdit_Pliers_OpenedDelayed, SIGNAL(editingFinished()), this, SLOT(SetPliersOpenedDelayed()));
	connect(ui.lineEdit_Chuck_OpenedDelayed, SIGNAL(editingFinished()), this, SLOT(SetChuckOpenedDelayed()));
	connect(ui.lineEdit_Chuck_ClosedDelayed, SIGNAL(editingFinished()), this, SLOT(SetChuckClosedDelayed()));

	connect(ui.comboBox_LaserControlMode, SIGNAL(currentIndexChanged(QString)), this, SLOT(SetLaserControlMode(QString)));
	connect(ui.lineEdit_CuttingBoundary, SIGNAL(editingFinished()), this, SLOT(SetCuttingCountBoundary()));
	connect(ui.checkBox_Blow, SIGNAL(toggled()), this, SLOT(SetCheckBox_BlowFlag(bool)));
	connect(ui.lineEdit_delay, SIGNAL(editingFinished()), this, SLOT(SetDelay()));
	connect(ui.checkBox_water, SIGNAL(toggled()), this, SLOT(SetCheckBox_WaterCutting(bool)));
	connect(ui.lineEdit_water_time, SIGNAL(editingFinished()), this, SLOT(SetWaterTime()));
	connect(ui.checkBox_Pump, SIGNAL(toggled()), this, SLOT(SetCheckBox_WaterPump(bool)));
	connect(ui.lineEdit_WaterOpen, SIGNAL(editingFinished()), this, SLOT(SetWaterPumpOpenTime()));
	connect(ui.lineEdit_WaterClose, SIGNAL(editingFinished()), this, SLOT(SetWaterPumpCloseTime()));

	connect(ui.checkBox_InterLockFlag, SIGNAL(toggled()), this, SLOT(SetCheckBox_InterLockFlag(bool)));
	connect(ui.checkBox_PressureMonitor, SIGNAL(toggled()), this, SLOT(SetCheckBox_PressureFlag(bool)));
	connect(ui.checkBox_WaterLeakageMonitor, SIGNAL(toggled()), this, SLOT(SetCheckBox_WaterLeakageFlag(bool)));
	connect(ui.checkBox_WaterBoxMonitor, SIGNAL(toggled()), this, SLOT(SetCheckBox_WaterBoxFlag(bool)));
	connect(ui.checkBox_RemainMonitor, SIGNAL(toggled()), this, SLOT(SetCheckBox_RemainFlag(bool)));
	connect(ui.checkBox_WaterPressureMonitor, SIGNAL(toggled()), this, SLOT(SetCheckBox_WaterPressureFlag(bool)));
	connect(ui.lineEdit_WaterPressureL, SIGNAL(editingFinished()), this, SLOT(SetWaterPressureL()));
	connect(ui.checkBox_WaterLevelMonitor, SIGNAL(toggled()), this, SLOT(SetCheckBox_WaterLevelFlag(bool)));
	connect(ui.lineEdit_WaterLevelL, SIGNAL(editingFinished()), this, SLOT(SetWaterLevelL()));

}

QG_dlgPBasicSettings::~QG_dlgPBasicSettings()
{}

void QG_dlgPBasicSettings::SetService(Service* pService)
{
	m_pService = pService;
}

void QG_dlgPBasicSettings::SetZPosition()
{
	QString qstr = ui.lineEdit_Z_Position->text();
	
}

void QG_dlgPBasicSettings::SetXPosition()
{

}

void QG_dlgPBasicSettings::SetYPosition()
{

}

void QG_dlgPBasicSettings::SetCheckBox_MoveAfterZCutting(bool)
{

}

void QG_dlgPBasicSettings::SetCheckBox_MoveAfterXCuttingFlag(bool)
{

}

void QG_dlgPBasicSettings::SetCheckBox_MoveAfterYCuttingFlag(bool)
{

}

void QG_dlgPBasicSettings::SetCheckBox_MoveAfterWaiting()
{

}

void QG_dlgPBasicSettings::SetCheckBox_ChuckClosedAfterCuttingFlag(bool)
{

}

void QG_dlgPBasicSettings::SetFeeding(bool)
{

}

void QG_dlgPBasicSettings::SetFeedingNum()
{

}

void QG_dlgPBasicSettings::SetFeedingDistance()
{

}

void QG_dlgPBasicSettings::SetCheckBox_MoveAfterFeedingY(bool)
{

}

void QG_dlgPBasicSettings::SetPosition_FeedingY()
{

}

void QG_dlgPBasicSettings::SetPliersClosedDelayed()
{

}

void QG_dlgPBasicSettings::SetChuckOpenedDelayed()
{

}

void QG_dlgPBasicSettings::SetChuckClosedDelayed()
{

}

void QG_dlgPBasicSettings::SetPliersOpenedDelayed()
{

}

void QG_dlgPBasicSettings::SetLaserControlMode(QString)
{

}

void QG_dlgPBasicSettings::SetCuttingCountBoundary()
{

}

void QG_dlgPBasicSettings::SetCheckBox_BlowFlag(bool)
{

}

void QG_dlgPBasicSettings::SetDelay()
{

}

void QG_dlgPBasicSettings::SetCheckBox_WaterCutting(bool)
{

}

void QG_dlgPBasicSettings::SetWaterTime()
{

}

void QG_dlgPBasicSettings::SetCheckBox_WaterPump(bool)
{

}

void QG_dlgPBasicSettings::SetWaterPumpOpenTime()
{

}

void QG_dlgPBasicSettings::SetWaterPumpCloseTime()
{

}

void QG_dlgPBasicSettings::SetCheckBox_InterLockFlag(bool)
{

}

void QG_dlgPBasicSettings::SetCheckBox_PressureFlag(bool)
{

}

void QG_dlgPBasicSettings::SetCheckBox_WaterLeakageFlag(bool)
{

}

void QG_dlgPBasicSettings::SetCheckBox_WaterBoxFlag(bool)
{

}

void QG_dlgPBasicSettings::SetCheckBox_RemainFlag(bool)
{

}

void QG_dlgPBasicSettings::SetCheckBox_WaterPressureFlag(bool)
{

}

void QG_dlgPBasicSettings::SetWaterPressureL()
{

}

void QG_dlgPBasicSettings::SetCheckBox_WaterLevelFlag(bool)
{

}

void QG_dlgPBasicSettings::SetWaterLevelL()
{

}
