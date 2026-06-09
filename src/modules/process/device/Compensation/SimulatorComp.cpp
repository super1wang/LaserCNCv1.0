#include "SimulatorComp.h"
#include "qc_applicationwindow.h"

SimulatorComp::SimulatorComp() : m_strName("Simulator"), m_bInited(false)
{
}

ErrorCode SimulatorComp::SetSpecialTable(bool bReconnect)
{
	ErrorCode eCode = ErrorCode::ERROR_NONE;
	table tSpecial = SETTINGS->GetTable(SettingSection::Special);
	if (tSpecial.count("SCOMP"))
	{
		table tLaser = tSpecial.at("SCOMP").as_table();
		if (tLaser.count("bTest"))
		{
			bool bTest = tLaser["bTest"].as_boolean();
		}
	}
	eCode = InitMeasurement();
	return eCode;
}

ErrorCode SimulatorComp::InitMeasurement()
{
	MotionControl* pMC = QC_ApplicationWindow::getAppWindow()->getService()->GetMotionControl();
	m_bInited = true;
	return ErrorCode::ERROR_NONE;
}

ErrorCode SimulatorComp::MeasurementComp(CompValue& cValue, int iIndex)
{
	cValue.X = QString::number(iIndex);
	cValue.Y = QString::number(iIndex);
	return ErrorCode::ERROR_NONE;
}