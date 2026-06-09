#include "Service.h"
#include "LogModule.h"
#include "qc_applicationwindow.h"
#include "qc_mdiwindow.h"
#include "qg_graphicview.h"

Service::Service(void):
	m_bCuttingHeadShow(false)
	,m_bShowDirection(false)
	,m_bShowCuttingPath(false)
	,m_bShowPathID(false)
{
	m_pMotionControl = nullptr;
	m_pLaserDevice	 = nullptr;
	SetCuttingDevice("NormalCutting");
	SetToolTable();
	LogModule::InitLog();
}

void Service::SetMotionControl(string strName)
{
	string strDevice = strName;
	if (DT::IsSimulatMode())
	{
		strDevice = "Simulator";
		LOG_SYS_INFO(QObject::tr("MotionControl: %1 Mode.").arg(strDevice.c_str()).toUtf8().data());
	}

	if (strDevice.empty())
	{
		SETTINGS->GetKeyValue("sType", strDevice, SettingSection::MotionControl, "MotionControl");
		LOG_SYS_INFO(QObject::tr("MotionControl: %1").arg(strDevice.c_str()).toUtf8().data());
	}
	m_pMotionControl = m_MCFactory.GetMotionController(strDevice);
	m_strMotionControl = strDevice;
}

void Service::SetLaserDevice(string strName)
{
	string strDevice = strName;
	if (DT::IsSimulatMode())
	{
		strDevice = "Simulator";
		LOG_SYS_INFO(QObject::tr("LaserDevice: %1 Mode.").arg(strDevice.c_str()).toUtf8().data());
	}

	if (strDevice.empty())
	{
		SETTINGS->GetKeyValue("sType", strDevice, SettingSection::Laser, "Laser");
		LOG_SYS_INFO(QObject::tr("LaserDevice: %1").arg(strDevice.c_str()).toUtf8().data());
	}
	m_pLaserDevice = m_LDFactory.GetLaserDevice(strDevice);
	m_strLaserDevice = strDevice;
}

void Service::SetCompDevice(string strName)
{
	string strDevice = strName;
	if (DT::IsSimulatMode())
	{
		strDevice = "Simulator";
		LOG_SYS_INFO(QObject::tr("CompDevice: %1 Mode.").arg(strDevice.c_str()).toUtf8().data());
	}

	if (strDevice.empty())
	{
		strDevice = "Simulator";
		LOG_SYS_INFO(QObject::tr("CompDevice: %1").arg(strDevice.c_str()).toUtf8().data());
	}
	m_pCompDevice = m_COMPFactory.GetCOMPDevice(strDevice);
	m_strCompDevice = strDevice;
}

void Service::SetCuttingDevice(string strName)
{
	m_pCuttingDevice = CuttingFactory::GetCuttingModel(strName);
}

void Service::SetToolTable()
{
	//清空Tool数据
	ClearToolDate();

	table  tabToolIndex = SETTINGS->GetTable(SettingSection::Tool, "ToolIndex");
	string strToolIndex, strToolName;
	for (int i = 0; i < tabToolIndex.size() - 1; i++)
	{
		strToolIndex  	= "sTool_" + std::to_string(i);
		strToolName		= tabToolIndex[strToolIndex].as_string();
		table tabTool	= SETTINGS->GetTable(SettingSection::Tool, strToolName);


		Tool* curtool = new Tool();
		
		curtool->m_strName					= strToolName;
		
		curtool->m_dLineVelocity			= tabTool["fLineVel"]			.as_floating();
		curtool->m_dArcVelocity				= tabTool["fArcVel"]			.as_floating();
		curtool->m_dLineAcc					= tabTool["fCutAcc"]			.as_floating();
		curtool->m_dLineJerk				= tabTool["fCutJerk"]			.as_floating();
		curtool->m_dArcAcc					= tabTool["fCutAcc"]			.as_floating();
		curtool->m_dArcJerk					= tabTool["fCutJerk"]			.as_floating();

		curtool->m_dIdleXVelocity			= tabTool["fXVel"]				.as_floating();
		curtool->m_dIdleAVelocity			= tabTool["fAVel"]				.as_floating();
		curtool->m_dIdleA1Velocity			= tabTool["fA1Vel"]				.as_floating();
		curtool->m_dIdleYVelocity			= tabTool["fYVel"]				.as_floating();
		curtool->m_dIdleX1Velocity			= tabTool["fX1Vel"]				.as_floating();
		curtool->m_dIdleY1Velocity			= tabTool["fY1Vel"]				.as_floating();
		curtool->m_dIdleZVelocity			= tabTool["fZVel"]				.as_floating();
		curtool->m_dIdleXYAccDec			= tabTool["fIdelAcc"]			.as_floating();
		curtool->m_dIdleXYJerk				= tabTool["fIdelJerk"]			.as_floating();
	
		curtool->m_dLaserEnergy				= tabTool["fEnergy"]			.as_floating();
		curtool->m_dLaserPulseWidth			= tabTool["fPluse"]				.as_floating();
		curtool->m_dLaserFrequency			= tabTool["fFrequency"]			.as_floating();
		curtool->m_dLaserAttenuatorPercentage = tabTool["fAttenuatorPercentage"].as_floating();
		curtool->m_dLaserPpDivider			= tabTool["fPpDivider"]		.as_floating();
		curtool->m_iLaserDelay				= static_cast<int>(tabTool["iDelay"].as_integer());
	
		curtool->m_dBeforeOn				= tabTool["fBeforeOpenLaser"]	.as_floating();
		curtool->m_dAfterOn					= tabTool["fAfterOpenLaser"]	.as_floating();
		curtool->m_dAfterOff				= tabTool["fAfterCloseLaser"]	.as_floating();
		curtool->m_dBeforeOff				= tabTool["fBeforeCloseLaser"]	.as_floating();
	
		curtool->m_dJunctionVelocity		= tabTool["fCornerVelocity"]	.as_floating();
		curtool->m_dJunctionAngle			= tabTool["fCornerAngle"]		.as_floating();
		curtool->m_dXsegEndVelocity			= tabTool["fXSEGVelocity"]		.as_floating();

		curtool->m_dCutSmoothTime			= tabTool["fCutSmoothTime"].as_floating();
		curtool->m_dCutSmoothK				= tabTool["fCutSmoothK"].as_floating();
		curtool->m_dAxisSmoothTime			= tabTool["fAxisSmoothTime"].as_floating();
		curtool->m_dAxisSmoothK				= tabTool["fAxisSmoothK"].as_floating();
	
		curtool->m_dCuttingHeight			= tabTool["fCuttingHeight"]		.as_floating();
		curtool->m_dIdleZHeight				= tabTool["fIdleHeight"]		.as_floating();
	
		curtool->m_strDirectionX			= tabTool["sDirectionsX"]		.as_string();
		curtool->m_strDirectionY			= tabTool["sDirectionsY"]		.as_string();

		curtool->m_bStopBlow				= tabTool["bStopBlow"]			.as_boolean();
		curtool->m_bPunch					= tabTool["bPunch"]				.as_boolean();

		curtool->m_bAZero					= tabTool["bSetPosA"]			.as_boolean();
		curtool->m_dAPos					= tabTool["fSetPosA"]			.as_floating();
		curtool->m_bA1Zero 					= tabTool["bSetPosA1"]			.as_boolean();
		curtool->m_dA1Pos 					= tabTool["fSetPosA1"]			.as_floating();

		curtool->m_bXIsMove					= tabTool["bMovePosX"]			.as_boolean();
		curtool->m_dXPosition				= tabTool["fMovePosX"]			.as_floating();
		curtool->m_bX1IsMove				= tabTool["bMovePosX1"]			.as_boolean();
		curtool->m_dX1Position				= tabTool["fMovePosX1"]			.as_floating();
		curtool->m_bAIsMove					= tabTool["bMovePosA"]			.as_boolean();
		curtool->m_dAPosition				= tabTool["fMovePosA"]			.as_floating();
		curtool->m_bA1IsMove				= tabTool["bMovePosA1"]			.as_boolean();
		curtool->m_dA1Position				= tabTool["fMovePosA1"]			.as_floating();
		curtool->m_bYIsMove					= tabTool["bMovePosY"]			.as_boolean();
		curtool->m_dYPosition				= tabTool["fMovePosY"]			.as_floating();
		curtool->m_bY1IsMove				= tabTool["bMovePosY1"]			.as_boolean();
		curtool->m_dY1Position				= tabTool["fMovePosY1"]			.as_floating();
		
		curtool->m_bCuttingHead				= tabTool["bCuttingHead"]		.as_boolean();
		curtool->m_bCrossBridge				= tabTool["bCrossBridge"]		.as_boolean();
		curtool->m_dServoCuttingHeight		= tabTool["fServoCuttingHeight"].as_floating();

		curtool->m_bAxisZLinkage			= tabTool["bAxisZLinkage"]		.as_boolean();
		curtool->m_dLinkedDelay				= tabTool["fLinkedDelay"]		.as_floating();
		curtool->m_sLinkedDirection			= tabTool["sLinkedDirection"]	.as_string();
		curtool->m_iLinkedMode				= tabTool["iLinkedMode"]		.as_integer();
		curtool->m_dLinkageParameterA		= tabTool["fLinkageParameterA"]	.as_floating();
		curtool->m_dLinkageParameterB		= tabTool["fLinkageParameterB"]	.as_floating();
		curtool->m_sLinkedFormula			= tabTool["sLinkedFormula"]		.as_string();

		curtool->m_bTroughFlag				= tabTool["bTrough"]			.as_boolean();
		curtool->m_iTroughBuffer			= tabTool["iRunBuffer"]			.as_integer();
		curtool->m_dCuttingHeightCompensate = tabTool["fCHCompensate"]		.as_floating();
		curtool->m_dExtend					= tabTool["fExtendSctart"]		.as_floating();
		curtool->m_dExtend_End				= tabTool["fExtendEnd"]			.as_floating();
		curtool->m_dWaitFirst				= tabTool["fAccTime"]			.as_floating();
		curtool->m_dTroughDelay				= tabTool["fDelay"]				.as_floating();

		curtool->m_bFlightCutting			= tabTool["bFlightCutting"]		.as_boolean();
		curtool->m_dFlightCutting_MotorDelay = tabTool["fMotorDelay"]		.as_floating();
		bool bEnergySwitchUse = DT::getCustomerID() == "MaiTong"
			|| int(DT::getPermission()) > (int)PermissionLevel::Factory;
		curtool->m_bEnergySwitch			= (bEnergySwitchUse && tabTool.count("bEnergySwitch"))
			? tabTool["bEnergySwitch"].as_boolean()
			: false;


		m_ToolFactory.SetTool(i, *curtool);
		delete curtool;
		curtool = nullptr;
	}
}

void Service::ClearToolDate()
{
	m_ToolFactory.ToolClear();
}

void Service::SetMotionControlTable(const table& table_MotionControl)
{
	if (!table_MotionControl.size())
	{
		string strMotionControl;
		SETTINGS->GetKeyValue("sType", strMotionControl, SettingSection::MotionControl, "MotionControl");
		if (m_strMotionControl != strMotionControl)
		{
			if (m_pMotionControl)
				m_pMotionControl->Disconnect();
			SetMotionControl(strMotionControl);
		}
		m_pMotionControl->SetMotionControlTable();
	}
	else
	{
		m_pMotionControl->SetMotionControlTable(table_MotionControl);
	}
}

void Service::SetDigitalTable(const table& table_Digital)
{
	if (!table_Digital.size())
		m_pMotionControl->SetDigitalTable();
	else
		m_pMotionControl->SetDigitalTable(table_Digital);
}

void Service::SetAnalogTable(const table& table_Analog)
{
	if (!table_Analog.size())
		m_pMotionControl->SetAnalogTable();
	else
		m_pMotionControl->SetAnalogTable(table_Analog);
}

void Service::SetLaserTable(const table& table_Laser)
{
	if (!table_Laser.size())
	{
		string strLaserDevice;
		SETTINGS->GetKeyValue("sType", strLaserDevice, SettingSection::Laser, "Laser");
		if (m_strLaserDevice != strLaserDevice)
		{
			if (m_pLaserDevice)
				m_pLaserDevice->Disconnect();
			SetLaserDevice(strLaserDevice);
		}
		
		if (m_pLaserDevice->GetName() == "AnalogControl")
		{
			double dResolution;
			SETTINGS->GetKeyValue("fResolution", dResolution, SettingSection::Laser, "Laser");
			if (dResolution > 0)
			{
				double dEnergy = 0;
				SETTINGS->GetKeyValue("fEnergy", dEnergy, SettingSection::Laser, "Laser");
				double dValue = dEnergy / 100.0 * dResolution;
				m_pMotionControl->AnalogOutputSet(AnalogOUT::Laser, dValue, true);
			}
		}
		else
			m_pLaserDevice->SetLaserTable();
		SetSignalSourceTable();
	}
	else
	{
		double dResolution;
		SETTINGS->GetKeyValue("fResolution", dResolution, SettingSection::Laser, "Laser");
		if (m_pLaserDevice->GetName() == "AnalogControl" && dResolution > 0)
		{
			double dEnergy = 0;
			SETTINGS->GetKeyValue("fEnergy", dEnergy, SettingSection::Laser, "Laser");
			double dValue = dEnergy/100.0*dResolution;
			m_pMotionControl->AnalogOutputSet(AnalogOUT::Laser, dValue, true);
		}
		else
			m_pLaserDevice->SetLaserTable(table_Laser);
		SetSignalSourceTable(table_Laser);// 额外设置信号源相关，需要先读是否启用信号源，再做下发/参数相关考虑
	}
}

void Service::SetSignalSourceTable(const table& table_Laser)
{
	bool bSerialPortFlag = false;
	SETTINGS->GetKeyValue("bSignal", bSerialPortFlag, SettingSection::Laser, "SignalSource");
	if (!bSerialPortFlag && !DT::IsSimulatMode())
		return;

	if (m_pMotionControl->GetName() == "GTN")
	{
		if (!table_Laser.size())
			m_pMotionControl->SetLaserParameterTable();
		else
			m_pMotionControl->SetLaserParameterTable(table_Laser);
	}
	else
	{
		if (!table_Laser.size())
			m_SignalSource.SetSignalSourceTable();
		else
			m_SignalSource.SetSignalSourceTable(table_Laser);
	}
	
}

void Service::SetGasTable(const table& table_Gas)
{
	// 设置气压下发
	table tGas = table_Gas;
	if (!table_Gas.size())
		tGas = SETTINGS->GetTable(SettingSection::Gas);

	if (tGas.count("Gas") || tGas.count("GasSetting"))
	{
		double dPressure;
		int iConversions;
		SETTINGS->GetKeyValue("fPressure", dPressure, SettingSection::Gas, "Gas");
		SETTINGS->GetKeyValue("iConversions", iConversions, SettingSection::Gas, "GasSetting");
		if (m_pMotionControl)
			m_pMotionControl->AnalogOutputSet(AnalogOUT::Pressure, iConversions / 2.0 * dPressure);
	}
}

void Service::SetCompTable(const table& table_Comp)
{
	SetCompDevice(DT::getCustomerID());
	m_pCompDevice->SetSpecialTable();
}

void Service::RedrawDrawing()
{
	QC_MDIWindow* m = QC_ApplicationWindow::getAppWindow()->getMDIWindow();
	if (m)
	{
		QG_GraphicView* g = m->getGraphicView();
		if (g)
			g->RedrawDrawing();
	}
}
