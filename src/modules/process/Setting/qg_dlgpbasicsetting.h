#pragma once

#include <QDialog>
#include "ui_qg_dlgpbasicsetting.h"
#include "Service.h"

class QG_dlgPBasicSettings : public QDialog
{
	Q_OBJECT

public:
	QG_dlgPBasicSettings(QWidget *parent = nullptr);
	~QG_dlgPBasicSettings();

public:
	void SetService(Service*);

public:
	//切割完轴系移动位置
	void	SetZPosition();
	void	SetXPosition();
	void	SetYPosition();
	void	SetCheckBox_MoveAfterZCutting(bool);
	void	SetCheckBox_MoveAfterXCuttingFlag(bool);
	void	SetCheckBox_MoveAfterYCuttingFlag(bool);
	void	SetCheckBox_MoveAfterWaiting();
	void	SetCheckBox_ChuckClosedAfterCuttingFlag(bool);

	//进给
	void	SetFeeding(bool);
	void	SetFeedingNum();
	void	SetFeedingDistance();
	void	SetCheckBox_MoveAfterFeedingY(bool);
	void	SetPosition_FeedingY();
	void	SetPliersClosedDelayed();
	void	SetChuckOpenedDelayed();
	void	SetChuckClosedDelayed();
	void	SetPliersOpenedDelayed();

	//其他
	void	SetLaserControlMode(QString);
	void	SetCuttingCountBoundary();
	void	SetCheckBox_BlowFlag(bool);
	void	SetDelay();
	void	SetCheckBox_WaterCutting(bool);
	void	SetWaterTime();
	void	SetCheckBox_WaterPump(bool);
	void	SetWaterPumpOpenTime();
	void	SetWaterPumpCloseTime();

	//监控
	void	SetCheckBox_InterLockFlag(bool);
	void	SetCheckBox_PressureFlag(bool);
	void	SetCheckBox_WaterLeakageFlag(bool);
	void	SetCheckBox_WaterBoxFlag(bool);
	void	SetCheckBox_RemainFlag(bool);
	void	SetCheckBox_WaterPressureFlag(bool);
	void	SetWaterPressureL();
	void	SetCheckBox_WaterLevelFlag(bool);
	void	SetWaterLevelL();

private:
	Service* m_pService;
private:
	Ui::QG_dlgCuttingSettingClass ui;
};
