#include "ACSMotionControl.h"
//#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>
//#include "CoreUtils.h"
#include "bdaqctrl.h"
#include "LogModule.h"

using namespace Automation::BDaq;

using std::ofstream;
using std::ios;
#define DEBUG_MODE 

ACSMotionControl::ACSMotionControl(void)
	: m_bConnectFlag(false)
	, m_strName("ACS")
	, m_hHandle(ACSC_INVALID)
	, m_dPreX(0)
	, m_dPreY(0)
	, m_bErrorOccurred(false)
	, m_dLaserOnBWait(0)
	, m_dLaserOnAWait(0)
	, m_dLaserOffBWait(0)
	, m_dLaserOffAWait(0)
	, m_dDiameter(1)
	, m_iProgramBufferIndex(9)
	, m_dBlowDelay(0)
	, m_bStop(false)
{
	m_vecMotors.reserve(8);  // 预分配8个元素空间
}

ACSMotionControl::~ACSMotionControl(void)
{
}

const string& ACSMotionControl::GetName() const
{
	return m_strName;
}

void ACSMotionControl::LogError()
{
	char ErrorStr[256];
	int ErrorCode, Received;
	ErrorCode = acsc_GetLastError();
	if (acsc_GetErrorString(m_hHandle, ErrorCode, ErrorStr, 255, &Received))
	{
		ErrorStr[Received] = '\0';
		// 移除末尾换行符（如果存在）
		for (int i = Received - 1; i >= 0 && (ErrorStr[i] == '\n' || ErrorStr[i] == '\r'); i--)
		{
			ErrorStr[i] = '\0';
		}
		LOG_SYS_ERROR(ErrorStr);
	}
	else
		LOG_SYS_ERROR(boost::lexical_cast<string>(ErrorCode));
}

void ACSMotionControl::CreateMotor(Axis eAxis, const table& tAxis)
{
	SetAxisIndex(eAxis, tAxis.at("iIndex").as_integer());
	string strName = enum_name(eAxis).data();
	m_mapMotorValue[eAxis].HomeName			= strName + "Home";
	m_mapMotorValue[eAxis].Name				= strName;
	m_mapMotorValue[eAxis].HomeBufferIndex	= tAxis.at("iHomeIndex")	.as_integer();
	m_mapMotorValue[eAxis].Resolution		= tAxis.at("fResolution")	.as_floating();
	m_mapMotorValue[eAxis].Rotary			= tAxis.at("bRotation")		.as_boolean();
	m_mapMotorValue[eAxis].Velocity			= tAxis.at("fVel")			.as_floating();
	m_mapMotorValue[eAxis].Acceleration		= tAxis.at("fAcc")			.as_floating();
	m_mapMotorValue[eAxis].Deceleration		= tAxis.at("fAcc")			.as_floating();
	m_mapMotorValue[eAxis].Jerk				= tAxis.at("fJerk")			.as_floating();
	m_mapMotorValue[eAxis].NegLimit			= tAxis.at("fLeftLimit")	.as_floating();
	m_mapMotorValue[eAxis].PosLimit			= tAxis.at("fRightLimit")	.as_floating();
	m_vecMotors.emplace_back(eAxis);
}

bool ACSMotionControl::Connect()
{
	if (m_hHandle != ACSC_INVALID)
	{
		m_bConnectFlag = true;
		return true;
	}

	DeleteOtherConnections();
	m_hHandle = acsc_OpenCommEthernetTCP((char*)"10.0.0.100", ACSC_SOCKET_STREAM_PORT);
	acsc_StopBuffer(m_hHandle, ACSC_NONE, NULL);
	if (m_hHandle == ACSC_INVALID)
	{
		LogError();
		return false;
	}
	if (!AfterOpenComm())
		return false;

	setlocale(LC_ALL, "Chinese-simplified");
	setlocale(LC_ALL, "C");

	LOG_SYS_INFO("ACS motion control is successfully connected.");
	return true;
}

bool ACSMotionControl::Disconnect()
{
	// 关闭激光
	if (!acsc_SetOutput(m_hHandle, 0, 4, 0, NULL))
	{
		LogError();
		return false;
	}

	StopMotion();
	do {
		Sleep(100);
	} while (IsAxisMoving());

	if (!acsc_CloseComm(m_hHandle))
	{
		LogError();
		return false;
	}
	m_bConnectFlag = false;
	m_hHandle = ACSC_INVALID;
	m_mapMotorValue.clear();
	return true;
}

bool ACSMotionControl::IsConnected()
{
	ACSC_CONNECTION_INFO info;
	if (!acsc_GetConnectionInfo(m_hHandle, &info))
	{
		m_bConnectFlag = false;
		return false;
	}

	if (info.Type == 0)
		m_bConnectFlag = false;
	else
		m_bConnectFlag = true;
	return m_bConnectFlag;
} 

bool ACSMotionControl::Reboot()
{
	if(m_hHandle == ACSC_INVALID)
	{
		return false;
	}
	if(!acsc_ControllerReboot(m_hHandle, 30000))
	{
		LogError();
		return false;
	}
	acsc_CloseComm(m_hHandle);
	m_hHandle = ACSC_INVALID;
	if(!Connect())
	{
		return false;
	}
	return true;
}

bool ACSMotionControl::Home()
{
	// 顺序 Z -> Z1 -> Y1 -> X1 -> Y -> X -> A1 -> A
	int arr[] = { 2, 6, 5, 4, 1, 0, 7, 3 };

	for (int i = 0; i < 8; ++i)
	{
		Axis eAxis = static_cast<Axis>(arr[i]);
		if (!DT::IsAxisUse(eAxis))
			continue;

		if (!RunBufferTillEnd(m_mapMotorValue[eAxis].HomeBufferIndex, 300000))
			return false;
		Sleep(10);

		if (m_bStop)
		{
			m_bStop = false;
			return false;
		}
	}
	return true;
}

bool ACSMotionControl::Home(Axis eAxis)
{
	bool bRun = RunBufferTillEnd(m_mapMotorValue[eAxis].HomeBufferIndex, 300000);
	if (bRun && !IsEnabled(eAxis))
		return false;
	return true;
}

bool ACSMotionControl::IsHomed()
{
	for (Axis axis : m_vecMotors)
	{
		if (!IsHomed(axis))
			return false;
	}
	return true;
}

bool ACSMotionControl::IsHomed(Axis eAxis)
{
	int HomeStatus = 0;
	if (acsc_ReadInteger(m_hHandle, ACSC_NONE, m_mapMotorValue[eAxis].HomeName.data(),
		0, 0, ACSC_NONE, ACSC_NONE, &HomeStatus, NULL))
	{
		if (HomeStatus == 1)
			return true;
		else
			return false;
	}
	else
	{
		LogError();
		return false;
	}
}

bool ACSMotionControl::StopHome()
{
	int iOverTime = 50;
	do
	{
		acsc_StopBuffer(m_hHandle, ACSC_NONE, NULL);
		m_bStop = true;
		StopMotion();
		Sleep(200);
		// 超时检测
		iOverTime--;
		if (!iOverTime)
		{
			LOG_SYS_ERROR("Stop home over time.");
			return false;
		}
	} while (IsHomeBufferRunning());
	return true;
}

bool ACSMotionControl::IsHomeBufferRunning()
{
	for (Axis axis : m_vecMotors)
	{
		if (IsBufferRunning(m_mapMotorValue[axis].HomeBufferIndex))
			return true;
	}
	return false;
}

bool ACSMotionControl::Enable()
{
	for (Axis axis : m_vecMotors)
	{
		if (!Enable(axis))
			return false;
	}
	return true;
}

bool ACSMotionControl::Enable(Axis eAxis)
{
	if (IsEnabled(eAxis))
		return true;

	if (!acsc_Enable(m_hHandle, m_mapMotorValue[eAxis].AxisIndex, NULL))
	{
		LogError();
		acsc_CloseComm(m_hHandle);
		return false;
	}
	if (!acsc_WaitMotorEnabled(m_hHandle, m_mapMotorValue[eAxis].AxisIndex, 1, 3000))
	{
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::Disable()
{
	for (Axis axis : m_vecMotors)
	{
		if (!Disable(axis))
			return false;
	}
	return true;
}

bool ACSMotionControl::Disable(Axis eAxis)
{
	if (!IsEnabled(eAxis))
		return true;

	if (!acsc_Disable(m_hHandle, m_mapMotorValue[eAxis].AxisIndex, NULL))
	{
		LogError();
		acsc_CloseComm(m_hHandle);
		return false;
	}
	if (!acsc_WaitMotorEnabled(m_hHandle, m_mapMotorValue[eAxis].AxisIndex, 0, 3000))
	{
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::IsEnabled()
{
	for (Axis axis : m_vecMotors)
	{
		if (!IsEnabled(axis))
			return false;
	}
	return true;
}

bool ACSMotionControl::IsEnabled(Axis eAxis)
{
	int Status;
	if (!acsc_GetMotorState(m_hHandle, m_mapMotorValue[eAxis].AxisIndex, &Status, NULL))
	{
		LogError();
		return false;
	}

	if (Status & ACSC_MST_ENABLE)
		return true;
	else
		return false;
}

bool ACSMotionControl::SetAxisEnable(Axis eAxis, bool bEnable)
{
	if (bEnable)
		return Enable(eAxis);
	else
		return Disable(eAxis);
}

bool ACSMotionControl::Jog(Axis eAxis, bool bDirection, double dVel)
{
	if(!acsc_Jog(m_hHandle, ACSC_AMF_VELOCITY, 
		m_mapMotorValue[eAxis].AxisIndex, bDirection ? dVel : -dVel, NULL))
	{
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::MoveRelative(Axis eAxis, double dPos, double dVel)
{
	if (!acsc_ExtToPoint(m_hHandle,	ACSC_AMF_RELATIVE|ACSC_AMF_VELOCITY,	
		m_mapMotorValue[eAxis].AxisIndex, dPos, dVel, 0, NULL))
	{
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::MoveAbsolute(Axis eAxis, double dPos, double dVel)
{
	if (!acsc_ExtToPoint(m_hHandle, ACSC_AMF_VELOCITY,
		m_mapMotorValue[eAxis].AxisIndex, dPos, dVel, 0, NULL))
	{
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::MoveMRelative(vector<Axis> vAxis, vector<double> dPos, double dVel)
{
	if ((vAxis.size() != dPos.size()) || vAxis.empty())
		return false;

	double* dPosM = new double[dPos.size()];
	int* iAxesM = new int[vAxis.size() + 1];
	for (int i = 0; i < vAxis.size(); i++)
	{
		iAxesM[i] = m_mapMotorValue[vAxis[i]].AxisIndex;
		dPosM[i]  = dPos[i];
	}
	iAxesM[vAxis.size()] = -1;
	
	if (!acsc_ExtToPointM(m_hHandle, ACSC_AMF_RELATIVE | ACSC_AMF_VELOCITY,
		iAxesM, dPosM, dVel, 0, NULL))
	{
		LogError();
		delete[] dPosM;
		delete[] iAxesM;
		return false;
	}
	delete[] dPosM;
	delete[] iAxesM;
	return true;
}

bool ACSMotionControl::MoveMAbsolute(vector<Axis> vAxis, vector<double> dPos, double dVel)
{
	if ((vAxis.size() != dPos.size()) || vAxis.empty())
		return false;
	
	double* dPosM = new double[dPos.size()];
	int* iAxesM = new int[vAxis.size() + 1];
	for (int i = 0; i < vAxis.size(); i++)
	{
		iAxesM[i] = m_mapMotorValue[vAxis[i]].AxisIndex;
		dPosM[i]  = dPos[i];
	}
	iAxesM[vAxis.size()] = -1;

	if (!acsc_ExtToPointM(m_hHandle, ACSC_AMF_VELOCITY, iAxesM, dPosM, dVel, 0, NULL))
	{
		LogError();
		delete[] dPosM;
		delete[] iAxesM;
		return false;
	}
	delete[] dPosM;
	delete[] iAxesM;
	return true;
}

bool ACSMotionControl::StopMotion()
{
	int i = 0;
	int* iAxisM = new int[m_mapMotorValue.size() + 1];
	for (Axis axis : m_vecMotors)
	{
		iAxisM[i] = m_mapMotorValue[axis].AxisIndex;
		i++;
	}
	iAxisM[i] = -1;

	if (!acsc_HaltM(m_hHandle, iAxisM, NULL))
	{
		LogError();
		delete[] iAxisM;
		return false;
	}
	delete[] iAxisM;
	//借用此处位置复位EnergySwitch的标志位
	bool bEnergySwitchUse = DT::getCustomerID() == "MaiTong"
		|| int(DT::getPermission()) > (int)PermissionLevel::Factory;
	if (bEnergySwitchUse)
		AcscWriteInt("EnergySwitch", 0);
	return true;
}

bool ACSMotionControl::StopMotion(Axis eAxis)
{
	if (!acsc_Halt(m_hHandle, m_mapMotorValue[eAxis].AxisIndex, NULL))
	{
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::IsAxisMoving()
{
	for (Axis axis : m_vecMotors)
	{
		if (IsAxisMoving(axis))
			return true;
	}
	return false;
}

bool ACSMotionControl::IsAxisMoving(Axis eAxis)
{
	int State;
	if (!acsc_GetMotorState(m_hHandle, m_mapMotorValue[eAxis].AxisIndex, &State, NULL))
	{
		//LogError();
		return false;
	}
	if (State & ACSC_MST_MOVE)
		return true;
	else
		return false;
}

bool ACSMotionControl::GetActualPos(Axis eAxis, double& dAPos)
{
	string strAPos = "APOS" + std::to_string(m_mapMotorValue[eAxis].AxisIndex);
	if (!acsc_ReadReal(m_hHandle, ACSC_NONE, strAPos.data(), 
		ACSC_NONE, ACSC_NONE, ACSC_NONE, ACSC_NONE, &dAPos, NULL))
	{
		//LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::GetFeedbackPos(Axis eAxis, double& dFPos)
{
	string strAPos = "FPOS" + std::to_string(m_mapMotorValue[eAxis].AxisIndex);
	if (!acsc_ReadReal(m_hHandle, ACSC_NONE, strAPos.data(),
		ACSC_NONE, ACSC_NONE, ACSC_NONE, ACSC_NONE, &dFPos, NULL))
	{
		//LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::SetAxisIndex(Axis eAxis, int iIndex)
{
	string sTemp = std::to_string(iIndex);
	m_mapMotorValue[eAxis].AxisIndex = iIndex;
	m_mapMotorValue[eAxis].SRLName	 = "SRLIMIT" + sTemp;
	m_mapMotorValue[eAxis].SLLName	 = "SLLIMIT" + sTemp;
	m_mapMotorValue[eAxis].Efac		 = "EFAC" + sTemp;
	return true;
}

bool ACSMotionControl::SetAxisHomeBufferIndex(Axis eAxis, int iIndex)
{
	m_mapMotorValue[eAxis].HomeBufferIndex = iIndex;
	return true;
}

bool ACSMotionControl::SetAxisIsRotary(Axis eAxis, bool bRotary)
{
	m_mapMotorValue[eAxis].Rotary = bRotary;
	return true;
}

bool ACSMotionControl::SetAxisResolution(Axis eAxis, int iResolution)
{
	m_mapMotorValue[eAxis].Resolution = iResolution;
	return true;
}

bool ACSMotionControl::SetAxisTubeDiamater(Axis eAxis, double dDiamater)
{
	if (dDiamater <= 0 || !m_mapMotorValue[eAxis].Rotary)
	{
		return false;
	}

	int iResolutionRatio = m_mapMotorValue[eAxis].Resolution;
	int iEfac = iResolutionRatio * 10000 / (PI * dDiamater);
	if (IsConnected())
	{
		if (!WriteEFAC(eAxis, iEfac))
			return false;

		if (eAxis == Axis::A)
			m_dDiameter = dDiamater;

		double dDiamaterTwo = 0;
		bool bXVSuccess;
		if (0 < dDiamater && dDiamater < 0.35)
		{
			dDiamaterTwo = 0.3;
			bXVSuccess = SetDiamaterXVEL(eAxis, 4.712389);
		}
		else if (0.35 <= dDiamater && dDiamater < 0.45)
		{
			dDiamaterTwo = 0.4;
			bXVSuccess = SetDiamaterXVEL(eAxis, 6.283186);
		}
		else if (0.45 <= dDiamater && dDiamater < 0.55)
		{
			dDiamaterTwo = 0.5;
			bXVSuccess = SetDiamaterXVEL(eAxis, 7.853982);
		}
		else if (0.55 <= dDiamater && dDiamater < 0.65)
		{
			dDiamaterTwo = 0.6;
			bXVSuccess = SetDiamaterXVEL(eAxis, 9.424778);
		}
		else if (0.65 <= dDiamater && dDiamater < 0.75)
		{
			dDiamaterTwo = 0.7;
			bXVSuccess = SetDiamaterXVEL(eAxis, 10.995575);
		}
		else if (0.75 <= dDiamater && dDiamater < 0.85)
		{
			dDiamaterTwo = 0.8;
			bXVSuccess = SetDiamaterXVEL(eAxis, 12.566371);
		}
		else if (0.85 <= dDiamater && dDiamater < 0.95)
		{
			dDiamaterTwo = 0.9;
			bXVSuccess = SetDiamaterXVEL(eAxis, 14.137167);
		}
		else if (0.95 <= dDiamater && dDiamater < 1.0)
		{
			dDiamaterTwo = 1.0;
			bXVSuccess = SetDiamaterXVEL(eAxis, 15.707963);
		}
		else if (dDiamater >= 1)
		{
			dDiamaterTwo = int(dDiamater) + 0.5;
			double dXVEL = PI * dDiamaterTwo * 5;		// 5为轴每秒旋转圈数
			bXVSuccess = SetDiamaterXVEL(eAxis, dXVEL);
		}
		return bXVSuccess;
	}
	return false;
}

bool ACSMotionControl::SetAxisVel(Axis eAxis, double dVel)
{
	if (!acsc_SetVelocity(m_hHandle, m_mapMotorValue[eAxis].AxisIndex, dVel, NULL))
	{
		LOG_SYS_ERROR(fmt::format("Set axis {} vel {} failed.", m_mapMotorValue[eAxis].Name, dVel));
		LogError();
		return false;
	}
	m_mapMotorValue[eAxis].Velocity = dVel;
	return true;
}

bool ACSMotionControl::SetAxisAcc(Axis eAxis, double dAcc)
{
	if (!acsc_SetAccelerationImm(m_hHandle, m_mapMotorValue[eAxis].AxisIndex, dAcc, NULL))
	{
		LOG_SYS_ERROR(fmt::format("Set axis {} acc {} failed.", m_mapMotorValue[eAxis].Name, dAcc));
		LogError();
		return false;
	}
	m_mapMotorValue[eAxis].Acceleration = dAcc;
	return true;
}

bool ACSMotionControl::SetAxisDec(Axis eAxis, double dDec)
{
	if (!acsc_SetDecelerationImm(m_hHandle, m_mapMotorValue[eAxis].AxisIndex, dDec, NULL))
	{
		LOG_SYS_ERROR(fmt::format("Set axis {} dec {} failed.", m_mapMotorValue[eAxis].Name, dDec));
		LogError();
		return false;
	}
	m_mapMotorValue[eAxis].Deceleration = dDec;
	return true;
}

bool ACSMotionControl::SetAxisJerk(Axis eAxis, double dJerk)
{
	if (!acsc_SetJerkImm(m_hHandle, m_mapMotorValue[eAxis].AxisIndex, dJerk, NULL))
	{
		LOG_SYS_ERROR(fmt::format("Set axis {} jerk {} failed.", m_mapMotorValue[eAxis].Name, dJerk));
		LogError();
		return false;
	}
	m_mapMotorValue[eAxis].Jerk = dJerk;
	return true;
}

bool ACSMotionControl::SetAxisNegLimit(Axis eAxis, double dNegLimit)
{
	if (!acsc_WriteReal(m_hHandle, ACSC_NONE, m_mapMotorValue[eAxis].SLLName.data(),
		0, 0, ACSC_NONE, ACSC_NONE, &dNegLimit, NULL))
	{
		LOG_SYS_ERROR(fmt::format("Set axis {} neg limit {} failed.", m_mapMotorValue[eAxis].Name, dNegLimit));
		LogError();
		return false;
	}
	m_mapMotorValue[eAxis].NegLimit = dNegLimit;
	return true;
}

bool ACSMotionControl::SetAxisPosLimit(Axis eAxis, double dPosLimit)
{
	if (!acsc_WriteReal(m_hHandle, ACSC_NONE, m_mapMotorValue[eAxis].SRLName.data(),
		0, 0, ACSC_NONE, ACSC_NONE, &dPosLimit, NULL))
	{
		LOG_SYS_ERROR(fmt::format("Set axis {} pos limit {} failed.", m_mapMotorValue[eAxis].Name, dPosLimit));
		LogError();
		return false;
	}
	m_mapMotorValue[eAxis].PosLimit = dPosLimit;
	return true;
}

bool ACSMotionControl::SetAxisVelAccDecJerk(Axis eAxis, double dVel, double dAcc, double dDec, double dJerk)
{
	int iResult = 0;
	if (!SetAxisVel(eAxis, dVel))
		iResult++;
	if (!SetAxisAcc(eAxis, dAcc))
		iResult++;
	if (!SetAxisDec(eAxis, dDec))
		iResult++;
	if (!SetAxisJerk(eAxis, dJerk))
		iResult++;
	return (bool)!iResult;
}

bool ACSMotionControl::SetAxisSoftLimit(Axis eAxis, double dNegLimit, double dPosLimit)
{
	int iResult = 0;
	if (!SetAxisNegLimit(eAxis, dNegLimit))
		iResult++;
	if (!SetAxisPosLimit(eAxis, dPosLimit))
		iResult++;
	return (bool)!iResult;
}

bool ACSMotionControl::GetAxisIndex(Axis eAxis, int& iIndex)
{
	iIndex = m_mapMotorValue[eAxis].AxisIndex;
	return true;
}

bool ACSMotionControl::GetAxisHomeBufferIndex(Axis eAxis, int& iHomeIndex)
{
	iHomeIndex = m_mapMotorValue[eAxis].HomeBufferIndex;
	return true;
}

bool ACSMotionControl::GetAxisIsRotary(Axis eAxis, bool& bRotary)
{
	bRotary = m_mapMotorValue[eAxis].Rotary;
	return true;
}

bool ACSMotionControl::GetAxisResolution(Axis eAxis, int& iResolution)
{
	iResolution = m_mapMotorValue[eAxis].Resolution;
	return true;
}

bool ACSMotionControl::GetAxisTubeDiamater(Axis eAxis, double& dTubeDiamater)
{
	int iEfac = 0;
	if (!ReadEFAC(eAxis, iEfac))
		return false;
	
	int iResolutionRatio = m_mapMotorValue[eAxis].Resolution;
	dTubeDiamater = iResolutionRatio * 10000 / (iEfac * PI);
	m_dDiameter = dTubeDiamater;
	return true;
}

bool ACSMotionControl::GetAxisVel(Axis eAxis, double& dVel)
{
	if (!acsc_GetVelocity(m_hHandle, m_mapMotorValue[eAxis].AxisIndex, &dVel, NULL))
	{
		dVel = m_mapMotorValue[eAxis].Velocity;
		LOG_SYS_ERROR(fmt::format("Get axis {} vel failed.", m_mapMotorValue[eAxis].Name));
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::GetAxisAcc(Axis eAxis, double& dAcc)
{
	if (!acsc_GetAcceleration(m_hHandle, m_mapMotorValue[eAxis].AxisIndex, &dAcc, NULL))
	{
		dAcc = m_mapMotorValue[eAxis].Acceleration;
		LOG_SYS_ERROR(fmt::format("Get axis {} acc failed.", m_mapMotorValue[eAxis].Name));
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::GetAxisDec(Axis eAxis, double& dDec)
{
	if (!acsc_GetDeceleration(m_hHandle, m_mapMotorValue[eAxis].AxisIndex, &dDec, NULL))
	{
		dDec = m_mapMotorValue[eAxis].Deceleration;
		LOG_SYS_ERROR(fmt::format("Get axis {} dec failed.", m_mapMotorValue[eAxis].Name));
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::GetAxisJerk(Axis eAxis, double& dJerk)
{
	if (!acsc_GetJerk(m_hHandle, m_mapMotorValue[eAxis].AxisIndex, &dJerk, NULL))
	{
		dJerk = m_mapMotorValue[eAxis].Jerk;
		LOG_SYS_ERROR(fmt::format("Get axis {} jerk failed.", m_mapMotorValue[eAxis].Name));
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::GetAxisNegLimit(Axis eAxis, double& dNegLimit)
{
	if (!acsc_ReadReal(m_hHandle, ACSC_NONE, m_mapMotorValue[eAxis].SLLName.data(),
		0, 0, ACSC_NONE, ACSC_NONE, &dNegLimit, NULL))
	{
		dNegLimit = m_mapMotorValue[eAxis].NegLimit;
		LOG_SYS_ERROR(fmt::format("Get axis {} neg limit failed.", m_mapMotorValue[eAxis].Name));
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::GetAxisPosLimit(Axis eAxis, double& dPosLimit)
{
	if (!acsc_ReadReal(m_hHandle, ACSC_NONE, m_mapMotorValue[eAxis].SRLName.data(),
		0, 0, ACSC_NONE, ACSC_NONE, &dPosLimit, NULL))
	{
		dPosLimit = m_mapMotorValue[eAxis].PosLimit;
		LOG_SYS_ERROR(fmt::format("Get axis {} pos limit failed.", m_mapMotorValue[eAxis].Name));
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::GetAxisVelAccDecJerk(Axis eAxis, double& dVel, double& dAcc, double& dDec, double& dJerk)
{
	int iResult = 0;
	if (!GetAxisVel(eAxis, dVel))
		iResult++;
	if (!GetAxisAcc(eAxis, dAcc))
		iResult++;
	if (!GetAxisDec(eAxis, dDec))
		iResult++;
	if (!GetAxisJerk(eAxis, dJerk))
		iResult++;
	return (bool)!iResult;
}

bool ACSMotionControl::GetAxisSoftLimit(Axis eAxis, double& dNegLimit, double& dPosLimit)
{
	int iResult = 0;
	if (!GetAxisNegLimit(eAxis, dNegLimit))
		iResult++;
	if (!GetAxisPosLimit(eAxis, dPosLimit))
		iResult++;
	return (bool)!iResult;
}

void ACSMotionControl::ReadAxisSoftLimit(Axis eAxis, double& dNegLimit, double& dPosLimit)
{
	dNegLimit = m_mapMotorValue[eAxis].NegLimit;
	dPosLimit = m_mapMotorValue[eAxis].PosLimit;
}

bool ACSMotionControl::DigitalOutputSet(DigitalIOData& IOData, int iValue, bool bLogError)
{
	int Value = iValue;
	if (IOData.bInversion)
		Value = iValue ? 0 : 1;

	if (!IOData.bExpand)
	{
		if (!acsc_SetOutput(m_hHandle, IOData.iPort, IOData.iIO, Value, NULL))
		{
			if (bLogError)
				LogError();
			return false;
		}
	}
	else
	{
		int	ioutput;
		if (!acsc_ReadInteger(m_hHandle, ACSC_NONE, IOData.strPort.data(),
			ACSC_NONE, ACSC_NONE, ACSC_NONE, ACSC_NONE, &ioutput, NULL))
		{
			if (bLogError)
				LogError();
			return false;
		}

		int bit = 0x00000001 << IOData.iIO;
		if (Value)
			ioutput = ioutput | bit;
		else
			ioutput = ioutput & (~bit);

		if (!acsc_WriteInteger(m_hHandle, ACSC_NONE, IOData.strPort.data(),
			ACSC_NONE, ACSC_NONE, ACSC_NONE, ACSC_NONE, &ioutput, NULL))
		{
			if (bLogError)
				LogError();
			return false;
		}
	}
	return true;
}

bool ACSMotionControl::DigitalOutputGet(DigitalIOData& IOData, int& iValue, bool bLogError)
{
	if (!IOData.bExpand)
	{
		if (!acsc_GetOutput(m_hHandle, IOData.iPort, IOData.iIO, &iValue, NULL))
		{
			if (bLogError)
				LogError();
			return false;
		}
	}
	else
	{
		int	ioutput;
		if (!acsc_ReadInteger(m_hHandle, ACSC_NONE, IOData.strPort.data(),
			ACSC_NONE, ACSC_NONE, ACSC_NONE, ACSC_NONE, &ioutput, NULL))
		{
			if (bLogError)
				LogError();
			return false;
		}

		int bit = 0x00000001 << IOData.iIO;
		iValue = (ioutput & bit) ? 1 : 0;
	}

	if (IOData.bInversion)
		iValue = iValue ? 0 : 1;

	return true;
}

bool ACSMotionControl::DigitalInputGet(DigitalIOData& IOData, int& iValue, bool bLogError)
{
	if (!IOData.bExpand)
	{
		if (!acsc_GetInput(m_hHandle, IOData.iPort, IOData.iIO, &iValue, NULL))
		{
			if (bLogError)
				LogError();
			return false;
		}
	}
	else
	{
		int	iinput;
		if (!acsc_ReadInteger(m_hHandle, ACSC_NONE, IOData.strPort.data(),
			0, 0, ACSC_NONE, ACSC_NONE, &iinput, NULL))
		{
			if (bLogError)
				LogError();
			return false;
		}

		int bit = 0x00000001 << IOData.iIO;
		iValue = (iinput & bit) ? 1 : 0;
	}

	if (IOData.bInversion)
		iValue = iValue ? 0 : 1;

	return true;
}

bool ACSMotionControl::AnalogOutputSet(AnalogIOData& IOData, double dValue, bool bLogError)
{
	if (!IOData.bExpand)
	{
		if (!acsc_SetAnalogOutputNT(m_hHandle, IOData.iPort, dValue, NULL))
		{
			if (bLogError)
				LogError();
			return false;
		}
	}
	else
	{
		if (!acsc_WriteReal(m_hHandle, ACSC_NONE, IOData.strPort.data(),
			0, 0, ACSC_NONE, ACSC_NONE, &dValue, NULL))
		{
			if (bLogError)
				LogError();
			return false;
		}
	}
	return true;
}

bool ACSMotionControl::AnalogOutputGet(AnalogIOData& IOData, double& dValue, bool bLogError)
{
	if (!IOData.bExpand)
	{
		if (!acsc_GetAnalogOutputNT(m_hHandle, IOData.iPort, &dValue, NULL))
		{
			if (bLogError)
				LogError();
			return false;
		}
	}
	else
	{
		if (!acsc_ReadReal(m_hHandle, ACSC_NONE, IOData.strPort.data(),
			0, 0, ACSC_NONE, ACSC_NONE, &dValue, NULL))
		{
			if (bLogError)
				LogError();
			return false;
		}
	}
	return true;
}

bool ACSMotionControl::AnalogInputGet(AnalogIOData& IOData, double& dValue, bool bLogError)
{
	if (!IOData.bExpand)
	{
		if (!acsc_GetAnalogInputNT(m_hHandle, IOData.iPort, &dValue, NULL))
		{
			if (bLogError)
				LogError();
			return false;
		}
	}
	else
	{
		if (!acsc_ReadReal(m_hHandle, ACSC_NONE, IOData.strPort.data(),
			0, 0, ACSC_NONE, ACSC_NONE, &dValue, NULL))
		{
			if (bLogError)
				LogError();
			return false;
		}
	}
	return true;
}


bool ACSMotionControl::IsQueueFull()
{
	int FullStatus;
	if (!acsc_ReadInteger(m_hHandle, ACSC_NONE, (char*)"gbFifoFull",
		ACSC_NONE, ACSC_NONE, ACSC_NONE, ACSC_NONE, &FullStatus, NULL))
	{
		LogError();
	}
	return FullStatus;
}

bool ACSMotionControl::IsQueueEmpty()
{
	int EmptyStatus;
	if (!acsc_ReadInteger(m_hHandle, ACSC_NONE, (char*)"gbFifoEmpty",
		ACSC_NONE, ACSC_NONE, ACSC_NONE, ACSC_NONE, &EmptyStatus, NULL))
	{
		LogError();
		return false;
	}
	return EmptyStatus;
}

bool ACSMotionControl::IsQueueActive()
{
	return true;
}

bool ACSMotionControl::ClearQueue()
{
	char* cmd = (char*)"#6SR\r";
	if (!acsc_Command(m_hHandle, cmd, strlen(cmd), NULL))
	{
		LogError();
		return false;
	}

	cmd = (char*)"#6X\r";
	if (!acsc_Command(m_hHandle, cmd, strlen(cmd), NULL))
	{
		LogError();
		return false;
	}
	cmd = (char*)"#7SR\r";
	if (!acsc_Command(m_hHandle, cmd, strlen(cmd), NULL))
	{
		LogError();
		return false;
	}
	cmd = (char*)"#7X\r";
	if (!acsc_Command(m_hHandle, cmd, strlen(cmd), NULL))
	{
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::SetShutterOnOffWaitTime(double dBeforeOn, double dAfterOn, double dBeforeOff, double dAfterOff, double dBlowDelay)
{
	m_dLaserOnBWait		= dBeforeOn;
	m_dLaserOnAWait		= dAfterOn;
	m_dLaserOffBWait	= dBeforeOff;
	m_dLaserOffAWait	= dAfterOff;
	m_dBlowDelay		= dBlowDelay;
	return true;
}

bool ACSMotionControl::RunBufferTillEnd(int iBufferIndex, int iTimeout)
{
	if (!acsc_RunBuffer(m_hHandle, iBufferIndex, NULL, NULL))
	{
		LogError();
		return false;
	}
	if (!acsc_WaitProgramEnd(m_hHandle, iBufferIndex, iTimeout))
	{
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::StopBuffer(int iBufferIndex)
{
	if (!acsc_StopBuffer(m_hHandle, iBufferIndex, NULL))
	{
		LogError();
		return false;
	}
	while (IsBufferRunning(iBufferIndex))
		Sleep(10);
	return true;
}

bool ACSMotionControl::StopAllBuffer()
{
	if (!acsc_StopBuffer(m_hHandle, ACSC_NONE, NULL))
	{
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::ErrorOccurred() const
{
	return m_bErrorOccurred;
}

bool ACSMotionControl::StopQueue()
{
	return StopBuffer(m_iProgramBufferIndex);
}

bool ACSMotionControl::IsOffsetCutting()
{
	return IsBufferRunning(m_iProgramBufferIndex);
}

bool ACSMotionControl::IsBufferRunning(int iBufferIndex)
{
	int State;
	if (!acsc_GetProgramState(m_hHandle, iBufferIndex, &State, NULL))
	{
		LogError();
		return false;
	}
	return State & ACSC_PST_RUN;
}

bool ACSMotionControl::AfterOpenComm()
{
	if (!acsc_CompileBuffer(m_hHandle, 8, NULL))
	{
		LogError();
		return false;
	}

	for (Axis axis : m_vecMotors)
	{
		if (!acsc_CompileBuffer(m_hHandle, m_mapMotorValue[axis].HomeBufferIndex, NULL))
		{
			LogError();
			return false;
		}
	}
	m_bConnectFlag = true;
	return true;
}

void ACSMotionControl::DeleteOtherConnections()
{
	int ConnectNum;
	ACSC_CONNECTION_DESC Connections[5];
	char app_name[] = "ACS.Framework.exe";
	if (!acsc_GetConnectionsList(Connections, 5, &ConnectNum))
	{
		LogError();
		return;
	}

	for (int i = 0; i < ConnectNum; i++)
	{
		string ConnectionsName = Connections[i].Application;
		string::size_type position;
		position = ConnectionsName.find("ACS.Framework.exe");
		if (position == ConnectionsName.npos)
		{
			if (acsc_TerminateConnection(&(Connections[i])))
			{
				LogError();
				return;
			}
		}
		// 			if(strcmp(Connections[i].Application, app_name) != 0)
		// 			{
		// 				acsc_TerminateConnection(&(Connections[i]));
		// 			}
	}
}

bool ACSMotionControl::WriteEFAC(Axis eAxis, int iEfac)
{
	int iPreEfac = 0;
	if (!ReadEFAC(eAxis, iPreEfac))
		return false;
	if (iPreEfac == iEfac)
		return true;

	Disable(eAxis);
	
	double dEfac = 1.0 / iEfac;
	if (!acsc_WriteReal(m_hHandle, ACSC_NONE, m_mapMotorValue[eAxis].Efac.data(),
		0, 0, ACSC_NONE, ACSC_NONE, &dEfac, NULL))
	{
		LogError();
		Enable(eAxis);
		ControllerSaveToFlash(eAxis);
		return false;
	}
	Enable(eAxis);
	return true;
}

bool ACSMotionControl::ReadEFAC(Axis eAxis, int& iEfac)
{
	double dEfac = 0.0;
	if (!acsc_ReadReal(m_hHandle, ACSC_NONE, m_mapMotorValue[eAxis].Efac.data(),
		0, 0, ACSC_NONE, ACSC_NONE, &dEfac, NULL))
	{
		LogError();
		return false;
	}
	iEfac = 1.0 / dEfac;
	return true;
}

bool ACSMotionControl::SetDiamaterXVEL(Axis eAxis, double dValue)
{
	string strXVEL = "XVEL" + boost::lexical_cast<string>(m_mapMotorValue[eAxis].AxisIndex);
	if (!acsc_WriteReal(m_hHandle, ACSC_NONE, strXVEL.data(),
		0, 0, ACSC_NONE, ACSC_NONE, &dValue, NULL))
	{
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::Punch(double fdDwellTime)
{
	return true;
}

void ACSMotionControl::ResetProgramCommand()
{
	m_strCommand = "";
}

void ACSMotionControl::BeginACSSegment(const Tool& tool)
{
	int iXIndex = m_mapMotorValue[enum_cast<Axis>(tool.m_strDirectionX).value()].AxisIndex;
	int iYIndex = m_mapMotorValue[enum_cast<Axis>(tool.m_strDirectionY).value()].AxisIndex;
	int iZIndex = m_mapMotorValue[Axis::Z].AxisIndex;
	string strXIndex = boost::lexical_cast<string>(iXIndex);
	string strYIndex = boost::lexical_cast<string>(iYIndex);
	string strZIndex = boost::lexical_cast<string>(iZIndex);
	string strDVel = boost::lexical_cast<string>(tool.m_dLineVelocity);
	string strFVel = boost::lexical_cast<string>(tool.m_dXsegEndVelocity);
	string strJVel = boost::lexical_cast<string>(tool.m_dJunctionVelocity);
	string strAngle = boost::lexical_cast<string>(tool.m_dJunctionAngle * 3.1415926 / 180);
	SetCuttingAccJerk(tool);
	string strEnd = "\n";
	string strZ0 = boost::lexical_cast<string>(tool.m_dCuttingHeight);

	string strLinkedIndex;
	if (tool.m_sLinkedDirection == "X")
		strLinkedIndex = strXIndex;
	else
		strLinkedIndex = strYIndex;

	string strLinkageA = boost::lexical_cast<string>(tool.m_dLinkageParameterA);// 坐标
	if (tool.m_bAxisZLinkage)
	{ 
		if (tool.m_iLinkedMode == 0)
		{
			string strLinkageB = boost::lexical_cast<string>(tool.m_dLinkageParameterB);	// tub/斜率
			m_strCommand += "MASTER MPOS(" + strZIndex + ")=" + "(" + strLinkageA + "-RPOS(" + strLinkedIndex + "))*" + "(" + strLinkageB + ")+" + strZ0 + strEnd;
			m_strCommand += "SLAVE/pt " + strZIndex + "," + "0" + "," + "50" + strEnd;
		}
		else if (tool.m_iLinkedMode == 1)
		{
			string strR0	= boost::lexical_cast<string>(tool.m_dLinkageParameterB / 2);
			string strR00	= boost::lexical_cast<string>((tool.m_dLinkageParameterB / 2) * (tool.m_dLinkageParameterB / 2));
			string strZMax	= boost::lexical_cast<string>(tool.m_dCuttingHeight + tool.m_dLinkageParameterB / 2);
			m_strCommand += "MASTER MPOS(" + strZIndex + ")=" + strZ0 + "+" + strR0 + "-SQRT(" + strR00 + "-POW((RPOS(" + strLinkedIndex + ")-" + strLinkageA + "),2))" + strEnd;
			m_strCommand += "SLAVE/pt " + strZIndex + "," + strZ0 + "," + strZMax + strEnd;
		}
		else
		{
			m_strCommand += tool.m_sLinkedFormula + strEnd;
		}

		double dDelay = tool.m_dLinkedDelay;
		string strDelay = boost::lexical_cast<string>(dDelay);
		m_strCommand += "WAIT " + strDelay + strEnd;
	}

	m_strCommand += "XSEG/VFJA (" + strXIndex + ", " + strYIndex + "), APOS" + strXIndex
		+ ", APOS" + strYIndex + ", " + strDVel + ", " + strFVel + ","
		+ strJVel + "," + strAngle + "\n";
}

void ACSMotionControl::EndProgramCommand(const Tool& tool)
{
	string strZIndex = boost::lexical_cast<string>(m_mapMotorValue[Axis::Z].AxisIndex);
	m_strCommand += "kill " + strZIndex + "\n";

	if (tool.m_bStopBlow)
	{
		if (tool.m_bBlow2)
			m_strCommand += m_mapDigitalOUT[DigitalOUT::Blow2].strIndex + "=0;\n";
		else
			m_strCommand += m_mapDigitalOUT[DigitalOUT::Blow].strIndex + "=0;\n";
	}
	bool bEnergySwitchUse = DT::getCustomerID() == "MaiTong"
		|| int(DT::getPermission()) > (int)PermissionLevel::Factory;
	if (bEnergySwitchUse && tool.m_bEnergySwitch)
		m_strCommand += "EnergySwitch=0\n";
	m_strCommand += "STOP\n";
}

bool ACSMotionControl::MoveZCutting(double dPos)
{
	string strPos = boost::lexical_cast<string>(dPos);
	string strZIndex = boost::lexical_cast<string>(m_mapMotorValue[Axis::Z].AxisIndex);
	string strVel = boost::lexical_cast<string>(m_mapMotorValue[Axis::Z].Velocity);
	m_strCommand += "PTP/EV (" + strZIndex + "), " + strPos + "," + strVel + "\n";
	return true;
}

void ACSMotionControl::OffsetLineTo(double dEndX, double dEndY, const Tool& tool)
{
	int iXIndex = m_mapMotorValue[enum_cast<Axis>(tool.m_strDirectionX).value()].AxisIndex;
	int iYIndex = m_mapMotorValue[enum_cast<Axis>(tool.m_strDirectionY).value()].AxisIndex;
	string strEndX = boost::lexical_cast<string>(dEndX);
	string strEndY = boost::lexical_cast<string>(dEndY);
	string strVel = boost::lexical_cast<string>(tool.m_dLineVelocity);
	string strXIndex = boost::lexical_cast<string>(iXIndex);
	string strYIndex = boost::lexical_cast<string>(iYIndex);

	m_strCommand += "LINE/V (" + strXIndex + ", " + strYIndex + "), " + strEndX + ", " + strEndY + ", " + strVel + "\n";
	m_strCommand += "IF GSFREE" + strXIndex + "<2; GO (" + strXIndex + ", " + strYIndex + "); END\n";

	m_dPreX = dEndX;
	m_dPreY = dEndY;
}

void ACSMotionControl::OffsetArcTo(double dEndX, double dEndY, double dCenterX, double dCenterY,
	bool bClockwise, const Tool& tool, double dIncX, double dIncY)
{
// 	if ((m_dPreX == dEndX) && (m_dPreY == dEndY))
// 		return;

	if (m_dPreY == dEndY)	// 在同一水平线上
	{
		dCenterX = (m_dPreX + dEndX) / 2;
	}
	else if (m_dPreX == dEndX)
	{
		dCenterY = (m_dPreY + dEndY) / 2;
	}
	else if ((fabs(m_dPreX + dEndX - 2 * dCenterX) <= 0.001 && fabs(m_dPreY + dEndY - 2 * dCenterY) <= 0.001))
	{
		dCenterX = (m_dPreX + dEndX) / 2;
		dCenterY = (m_dPreY + dEndY) / 2;
	}
	else
	{
		double B = 1;
		double A = (dEndX - m_dPreX) / (dEndY - m_dPreY);
		double C = -A * (m_dPreX + dEndX) / 2 - (m_dPreY + dEndY) / 2;
		double dTempCenterX = dCenterX;
		dCenterX = (B * B * dCenterX - A * B * dCenterY - A * C) / (A * A + B * B);
		dCenterY = (-A * B * dTempCenterX + A * A * dCenterY - B * C) / (A * A + B * B);
	}

	string strCenterX = boost::lexical_cast<string>(dCenterX);
	string strCenterY = boost::lexical_cast<string>(dCenterY);
	string strEndX = boost::lexical_cast<string>(dEndX);
	string strEndY = boost::lexical_cast<string>(dEndY);
	string strVel = boost::lexical_cast<string>(tool.m_dArcVelocity);
	int iXIndex = m_mapMotorValue[enum_cast<Axis>(tool.m_strDirectionX).value()].AxisIndex;
	int iYIndex = m_mapMotorValue[enum_cast<Axis>(tool.m_strDirectionY).value()].AxisIndex;
	string strXIndex = boost::lexical_cast<string>(iXIndex);
	string strYIndex = boost::lexical_cast<string>(iYIndex);
	string strArcDir = bClockwise ? "-" : "+";

	m_strCommand += "ARC1/V (" + strXIndex + ", " + strYIndex + "), " + strCenterX + ", " + strCenterY + ", "
		+ strEndX + ", " + strEndY + ", " + strArcDir + ", " + strVel + "\n";
	m_strCommand += "IF GSFREE" + strXIndex + "<2; GO (" + strXIndex + ", " + strYIndex + "); END\n";
	
	m_dPreX = dEndX;
	m_dPreY = dEndY;
}

void ACSMotionControl::OffsetArc2To(double dEndX, double dEndY, double dCenterX, double dCenterY, double dAngle, const Tool& tool)
{
	string strCenterX = boost::lexical_cast<string>(dCenterX);
	string strCenterY = boost::lexical_cast<string>(dCenterY);
	string strVel = boost::lexical_cast<string>(tool.m_dArcVelocity);
	int iXIndex = m_mapMotorValue[enum_cast<Axis>(tool.m_strDirectionX).value()].AxisIndex;
	int iYIndex = m_mapMotorValue[enum_cast<Axis>(tool.m_strDirectionY).value()].AxisIndex;
	string strXIndex = boost::lexical_cast<string>(iXIndex);
	string strYIndex = boost::lexical_cast<string>(iYIndex);
	string strAngle = boost::lexical_cast<string>(dAngle);
	m_strCommand += "ARC2/V (" + strXIndex + ", " + strYIndex + "), " + strCenterX + ", " + strCenterY + ", "
		+ strAngle + ", " + strVel + "\n";
	m_strCommand += "IF GSFREE" + strXIndex + "<2; GO (" + strXIndex + ", " + strYIndex + "); END\n";
	m_dPreX = dEndX;
	m_dPreY = dEndY;
}

void ACSMotionControl::JumpToTrough(const Tool& curTool, double time)
{
	string strWaitFirst = boost::lexical_cast<string>(curTool.m_dWaitFirst);
	string strWaitSecond = boost::lexical_cast<string>((int)(time * 1000));
	m_strCommand += "GLOBAL REAL waitFirst;\n";
	m_strCommand += "waitFirst = " + strWaitFirst + "\n";
	m_strCommand += "GLOBAL REAL waitSecond;\n";
	m_strCommand += "waitSecond = " + strWaitSecond + "\n";
}

// 改设置界面为旋转轴置位
void ACSMotionControl::JumpToSetAFPos(const Tool& curTool)
{
	if (curTool.m_bAZero) 
	{
		string strAIndex = boost::lexical_cast<string>(m_mapMotorValue[Axis::A].AxisIndex);
		string strAPos	 = boost::lexical_cast<string>(curTool.m_dAPos / 360 * PI * m_dDiameter);
		m_strCommand += "SET FPOS(" + strAIndex + ")=" + strAPos + "\n";
	}
	if (curTool.m_bA1Zero)
	{
		string strA1Index = boost::lexical_cast<string>(m_mapMotorValue[Axis::A1].AxisIndex);
		string strA1Pos   = boost::lexical_cast<string>(curTool.m_dA1Pos / 360 * PI * m_dDiameter);
		m_strCommand += "SET FPOS(" + strA1Index + ")=" + strA1Pos + "\n";
	}
}

void ACSMotionControl::JumpToIdleXYPosition(double dEndX, double dEndY, const Tool& curTool)
{
	Axis eDirectionX = enum_cast<Axis>(curTool.m_strDirectionX).value();
	Axis eDirectionY = enum_cast<Axis>(curTool.m_strDirectionY).value();

	//X 定位轴
	if (curTool.m_bXIsMove && eDirectionX != Axis::X && DT::IsAxisUse(Axis::X))
	{
		string strXIndex	= boost::lexical_cast<string>(m_mapMotorValue[Axis::X].AxisIndex);
		string strXPosition = boost::lexical_cast<string>(curTool.m_dXPosition);
		string strXVel		= boost::lexical_cast<string>(curTool.m_dIdleXVelocity);
		m_strCommand += "PTP/EV " + strXIndex + "," + strXPosition + "," + strXVel + "\n";
	}
	//X1 定位轴
	if (curTool.m_bX1IsMove && eDirectionX != Axis::X1 && DT::IsAxisUse(Axis::X1))
	{
		string strX1Index	 = boost::lexical_cast<string>(m_mapMotorValue[Axis::X1].AxisIndex);
		string strX1Position = boost::lexical_cast<string>(curTool.m_dX1Position);
		string strX1Vel		 = boost::lexical_cast<string>(curTool.m_dIdleX1Velocity);
		m_strCommand += "PTP/EV " + strX1Index + "," + strX1Position + "," + strX1Vel + "\n";
	}
	//A 定位轴
	if (curTool.m_bAIsMove && eDirectionY != Axis::A && DT::IsAxisUse(Axis::A))
	{
		string strAIndex	= boost::lexical_cast<string>(m_mapMotorValue[Axis::A].AxisIndex);
		string strAPosition = boost::lexical_cast<string>(curTool.m_dAPosition / 360 * PI * m_dDiameter);
		string strAVel		= boost::lexical_cast<string>(curTool.m_dIdleAVelocity);
		m_strCommand += "PTP/EV " + strAIndex + "," + strAPosition + "," + strAVel + "\n";
	}
	//A1 定位轴
	if (curTool.m_bA1IsMove && eDirectionY != Axis::A1 && DT::IsAxisUse(Axis::A1))
	{
		string strA1Index = boost::lexical_cast<string>(m_mapMotorValue[Axis::A1].AxisIndex);
		string strA1Position = boost::lexical_cast<string>(curTool.m_dA1Position / 360 * PI * m_dDiameter);
		string strA1Vel = boost::lexical_cast<string>(curTool.m_dIdleA1Velocity);
		m_strCommand += "PTP/EV " + strA1Index + "," + strA1Position + "," + strA1Vel + "\n";
	}
	//Y 定位轴
	if (curTool.m_bYIsMove && eDirectionY != Axis::Y && DT::IsAxisUse(Axis::Y))
	{
		string strYIndex	= boost::lexical_cast<string>(m_mapMotorValue[Axis::Y].AxisIndex);
		string strYPosition = boost::lexical_cast<string>(curTool.m_dYPosition);
		string strYVel		= boost::lexical_cast<string>(curTool.m_dIdleYVelocity);
		m_strCommand += "PTP/EV " + strYIndex + "," + strYPosition + "," + strYVel + "\n";
	}
	//Y1 定位轴
	if (curTool.m_bY1IsMove && eDirectionY != Axis::Y1 && DT::IsAxisUse(Axis::Y1))
	{
		string strY1Index	 = boost::lexical_cast<string>(m_mapMotorValue[Axis::Y1].AxisIndex);
		string strY1Position = boost::lexical_cast<string>(curTool.m_dY1Position);
		string strY1Vel		 = boost::lexical_cast<string>(curTool.m_dIdleY1Velocity);
		m_strCommand += "PTP/EV " + strY1Index + "," + strY1Position + "," + strY1Vel + "\n";
	}

	//空程起点坐标
	string strDirectionXIndex	= boost::lexical_cast<string>(m_mapMotorValue[eDirectionX].AxisIndex);
	string strDirectionYIndex	= boost::lexical_cast<string>(m_mapMotorValue[eDirectionY].AxisIndex);
	string strEndX				= boost::lexical_cast<string>(dEndX);
	string strEndY				= boost::lexical_cast<string>(dEndY);
	string strXDirectionVel		= boost::lexical_cast<string>(GetAxisIdleVel(eDirectionX, curTool));
	string strYDirectionVel		= boost::lexical_cast<string>(GetAxisIdleVel(eDirectionY, curTool));

	m_strCommand += "PTP/EV " + strDirectionXIndex + ", " + strEndX + ", " + strXDirectionVel + "\n";
	m_strCommand += "PTP/EV " + strDirectionYIndex + ", " + strEndY + ", " + strYDirectionVel + "\n";

	m_dPreX = dEndX;
	m_dPreY = dEndY;
}

void ACSMotionControl::JumpToIdleHeight(const Tool& curTool, double dCompensate)
{
	string strZIndex		= boost::lexical_cast<string>(m_mapMotorValue[Axis::Z].AxisIndex);
	string strZVel			= boost::lexical_cast<string>(curTool.m_dIdleZVelocity);
	string strIdleZHeight	= boost::lexical_cast<string>(curTool.m_dIdleZHeight + dCompensate);

	m_strCommand += "PTP/EV " + strZIndex + "," + strIdleZHeight + "," + strZVel + "\n";
	m_strCommand += "TILL ^MST(" + strZIndex + ").#MOVE" + "\n";
}

void ACSMotionControl::JumpToCuttingHeight(const Tool& curTool, double dCompensate)
{
	string strZIndex	= boost::lexical_cast<string>(m_mapMotorValue[Axis::Z].AxisIndex);
	string strZVel		= boost::lexical_cast<string>(curTool.m_dIdleZVelocity);
	string strZPosition = boost::lexical_cast<string>(curTool.m_dCuttingHeight + curTool.m_dCuttingHeightCompensate + dCompensate);

	m_strCommand += "PTP/EV " + strZIndex + "," + strZPosition + "," + strZVel + "\n";
	m_strCommand += "TILL ^MST(" + strZIndex + ").#MOVE" + "\n";
	bool bEnergySwitchUse = DT::getCustomerID() == "MaiTong"
		|| int(DT::getPermission()) > (int)PermissionLevel::Factory;
	if (bEnergySwitchUse && curTool.m_bEnergySwitch)
		m_strCommand += "EnergySwitch=1\n";
}

bool ACSMotionControl::SendCommand()
{
	//char* pcCommand = new char[m_strCommand.length() + 1];
	//strcpy_s(pcCommand, m_strCommand.length() + 1, m_strCommand.c_str());

	LOG_PROCESS_INFO("[Cutting]\n" + m_strCommand);
	LOG_PROCESS_REFRESH();
	if (!acsc_StopBuffer(m_hHandle, m_iProgramBufferIndex, ACSC_SYNCHRONOUS))
	{
		LogError();
		return false;
	}
	if (!acsc_LoadBuffer(m_hHandle, m_iProgramBufferIndex, m_strCommand.data(), m_strCommand.length(), ACSC_SYNCHRONOUS))
	{
		LogError();
		return false;
	}
	if (!acsc_CompileBuffer(m_hHandle, m_iProgramBufferIndex, ACSC_SYNCHRONOUS))
	{
		LogError();
		return false;
	}
	if (!acsc_RunBuffer(m_hHandle, m_iProgramBufferIndex, NULL, ACSC_SYNCHRONOUS))
	{
		LogError();
		return false;
	}
	//delete[]pcCommand;
	return true;
}

bool ACSMotionControl::SetCuttingAccJerk(const Tool& curTool)
{
	string strAcc = boost::lexical_cast<string>(curTool.m_dLineAcc);
	string strJerk = boost::lexical_cast<string>(curTool.m_dLineJerk);

	for (Axis axis : m_vecMotors)
	{
		string strIndex = boost::lexical_cast<string>(m_mapMotorValue[axis].AxisIndex);
		m_strCommand += "ACC" + strIndex + " = " + strAcc + ";";
		m_strCommand += "DEC" + strIndex + " = " + strAcc + ";";
		m_strCommand += "JERK" + strIndex + " = " + strJerk + ";\n";
	}

	return true;
}

bool ACSMotionControl::SetJumpAccJerk(const Tool& curTool)
{
	string strAcc = boost::lexical_cast<string>(curTool.m_dIdleXYAccDec);
	string strJerk = boost::lexical_cast<string>(curTool.m_dIdleXYJerk);

	for (Axis axis : m_vecMotors)
	{
		string strIndex = boost::lexical_cast<string>(m_mapMotorValue[axis].AxisIndex);
		m_strCommand += "ACC" + strIndex + " = " + strAcc + ";";
		m_strCommand += "DEC" + strIndex + " = " + strAcc + ";";
		m_strCommand += "JERK" + strIndex + " = " + strJerk + ";\n";
	}

	return true;
}

void ACSMotionControl::ProLaserControl(bool bLaser, bool bPso, const Tool& curTool, bool bAOUTFlag)
{
	string strLaserNum	= m_mapDigitalOUT[DigitalOUT::Laser].strIndex;
	string strAnalogNum = m_mapAnalogOUT[AnalogOUT::Laser].strIndex;
	string strEnd = "\n";

	if (bLaser)
	{
		string strLaserOnBWait = boost::lexical_cast<string>(m_dLaserOnBWait);
		string strLaserOnAWait = boost::lexical_cast<string>(m_dLaserOnAWait);
		string strBlowDelay = boost::lexical_cast<string>(m_dBlowDelay);

		if (curTool.m_bBlow2)
		{
			string strValue = m_mapDigitalOUT[DigitalOUT::Blow2].bInversion ? "0" : "1";
			m_strCommand += m_mapDigitalOUT[DigitalOUT::Blow2].strIndex + "=" + strValue + ";";
		}
		else
		{
			string strValue = m_mapDigitalOUT[DigitalOUT::Blow].bInversion ? "0" : "1";
			m_strCommand += m_mapDigitalOUT[DigitalOUT::Blow].strIndex + "=" + strValue + ";";
		}

		m_strCommand += strEnd + "WAIT " + strBlowDelay + strEnd;

		if (bPso)
		{
			string strTroughBuffer = boost::lexical_cast<string>(curTool.m_iTroughBuffer);
			m_strCommand += "START " + strTroughBuffer + ",1;\n";
			return;
		}
		else
		{
			m_strCommand += "WAIT " + strLaserOnBWait + strEnd;
			if (bAOUTFlag)
			{
				string strAnalogValue = boost::lexical_cast<string>(curTool.m_dAnalogLaserValue / 2.0);
				m_strCommand += strAnalogNum + "=" + strAnalogValue + ";";
				m_strCommand += "TILL " + strAnalogNum + ";";
			}
			else
			{
				string strLaserValue = m_mapDigitalOUT[DigitalOUT::Laser].bInversion ? "0" : "1";
				string strTill		 = m_mapDigitalOUT[DigitalOUT::Laser].bInversion ? "TILL ^" : "TILL ";
				m_strCommand += strLaserNum + "=" + strLaserValue + ";";
				m_strCommand += strTill + strLaserNum + ";";
			}
			m_strCommand += strEnd + "WAIT " + strLaserOnAWait + strEnd;
		}
	}
	else
	{
		if (bPso)
		{
			Axis eDirectionX = enum_cast<Axis>(curTool.m_strDirectionX).value();
			Axis eDirectionY = enum_cast<Axis>(curTool.m_strDirectionY).value();
			string strXIndex = boost::lexical_cast<string>(m_mapMotorValue[eDirectionX].AxisIndex);
			string strYIndex = boost::lexical_cast<string>(m_mapMotorValue[eDirectionY].AxisIndex);

			m_strCommand += "ENDS (" + strXIndex + ", " + strYIndex + ")" + strEnd;
			SetGO(curTool);
			m_strCommand += "SPLIT (" + strXIndex + ", " + strYIndex + ")" + strEnd;
		}
		else if (curTool.m_bTroughFlag)
			return;

		string strLaserOffBWait = boost::lexical_cast<string>(m_dLaserOffBWait);
		string strLaserOffAWait = boost::lexical_cast<string>(m_dLaserOffAWait);
		m_strCommand += "WAIT " + strLaserOffBWait + strEnd;
		if (bAOUTFlag)
		{
			m_strCommand += strAnalogNum + "=0;";
			m_strCommand += "TILL ^" + strAnalogNum + ";";
		}
		else
		{
			string strLaserValue = m_mapDigitalOUT[DigitalOUT::Laser].bInversion ? "1" : "0";
			string strTill		 = m_mapDigitalOUT[DigitalOUT::Laser].bInversion ? "TILL " : "TILL ^";
			m_strCommand += strLaserNum + "=" + strLaserValue + ";";
			m_strCommand += strTill + strLaserNum + ";";
		}
		m_strCommand += strEnd + "WAIT " + strLaserOffAWait + strEnd;
	}
}

int ACSMotionControl::GetPressureState()
{
	int temp;
	if (!acsc_GetInput(m_hHandle, 0, 0, &temp, NULL))
	{
		LogError();
		return false;
	}
	return temp;
}

bool ACSMotionControl::GetIsPressureState()
{
	return this->IsPressureMonitoring;
}

void ACSMotionControl::SetIsPressureState(bool bPressure)
{
	this->IsPressureMonitoring = bPressure;
}

bool ACSMotionControl::IsAxisStatusNormal(int& iFault)
{
	for (Axis axis : m_vecMotors)
	{
		if (!GetFault(m_mapMotorValue[axis].AxisIndex, iFault))
			return false;
	}

	if (!GetFault(ACSC_NONE, iFault))
		return false;

	return true;
}

bool ACSMotionControl::GetFault(int iAxis, int& Fault)
{
	if (!acsc_GetFault(m_hHandle, iAxis, &Fault, NULL))
	{
		LogError();
		return false;
	}
	return true;
}

double ACSMotionControl::GetAxisIdleVel(Axis eAxis, const Tool& curTool)
{
	switch (eAxis)
	{
	case Axis::X:	return curTool.m_dIdleXVelocity;
	case Axis::Y:	return curTool.m_dIdleYVelocity;
	case Axis::Z:	return curTool.m_dIdleZVelocity;
	case Axis::A:	return curTool.m_dIdleAVelocity;	//临时
	case Axis::X1:	return curTool.m_dIdleX1Velocity;
	case Axis::Y1:	return curTool.m_dIdleY1Velocity;
	case Axis::Z1:	return 0;
	case Axis::A1:	return 0;
	default:		return 0;
	}
}

bool ACSMotionControl::HaltMotor(Axis eMotor)
{
	if (!acsc_Halt(m_hHandle, m_mapMotorValue[eMotor].AxisIndex, NULL))
	{
		LogError();
		return false;
	}
	return true;
}

void ACSMotionControl::SetGO(const Tool& curTool)
{
	Axis eDirectionX = enum_cast<Axis>(curTool.m_strDirectionX).value();
	Axis eDirectionY = enum_cast<Axis>(curTool.m_strDirectionY).value();
	string strXIndex = boost::lexical_cast<string>(m_mapMotorValue[eDirectionX].AxisIndex);
	string strYIndex = boost::lexical_cast<string>(m_mapMotorValue[eDirectionY].AxisIndex);
	
	m_strCommand += "GO (" + strXIndex + ", " + strYIndex + ")\n";
}

bool ACSMotionControl::SetFPos(Axis eAxis, double dPos)
{
	if (!acsc_SetFPosition(m_hHandle, m_mapMotorValue[eAxis].AxisIndex, dPos, ACSC_SYNCHRONOUS))
	{
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::GetFPos(Axis eAxis, double& dPos)
{
	if (!acsc_GetFPosition(m_hHandle, m_mapMotorValue[eAxis].AxisIndex, &dPos, NULL))
	{
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::LoadApplication(string strStress)
{
	ACSC_APPSL_INFO* ainfo = NULL;
	if (!acsc_AnalyzeApplication(m_hHandle, strStress.c_str(), &ainfo, NULL))
	{
		LogError();
		return false;
	}
	if (!acsc_LoadApplication(m_hHandle, strStress.c_str(), ainfo, NULL))
	{
		LogError();
		return false;
	}
	return Reboot();
}

bool ACSMotionControl::ControllerSaveToFlash(Axis eAxis)
{
	int Axes[] = { m_mapMotorValue[eAxis].AxisIndex, -1 };
	if (!acsc_ControllerSaveToFlash(m_hHandle, Axes, NULL, NULL, NULL))
	{
		LogError();
		return false;
	}
	return true;
}

void ACSMotionControl::BeginACSSegmentSimple(const Tool& curTool)
{
	Axis	eDirectionX = enum_cast<Axis>(curTool.m_strDirectionX).value();
	Axis	eDirectionY = enum_cast<Axis>(curTool.m_strDirectionY).value();
	string	strXIndex	= boost::lexical_cast<string>(m_mapMotorValue[eDirectionX].AxisIndex);
	string	strYIndex	= boost::lexical_cast<string>(m_mapMotorValue[eDirectionY].AxisIndex);
	string	strZIndex	= boost::lexical_cast<string>(m_mapMotorValue[Axis::Z].AxisIndex);
	string	strDVel		= boost::lexical_cast<string>(curTool.m_dLineVelocity);
	string	strFVel		= boost::lexical_cast<string>(curTool.m_dXsegEndVelocity);
	string	strJVel		= boost::lexical_cast<string>(curTool.m_dJunctionVelocity);
	string	strAngle	= boost::lexical_cast<string>(curTool.m_dJunctionAngle * 3.1415926 / 180);
	SetCuttingAccJerk(curTool);

	m_strCommand += "XSEG/VFJA (" + strXIndex + ", " + strYIndex + "," + strZIndex + "), APOS" + strXIndex
		+ ", APOS" + strYIndex + ", " + strZIndex + "," + strDVel + ", " + strFVel + ","
		+ strJVel + "," + strAngle + "\n";
}

void ACSMotionControl::JumpToSimple(double dEndX, double dEndY, const Tool& curTool, double dFindTelos, double time)
{
	// 米制单位，精度到um
	double Precision = 1000;
	int iEndX = dEndX * Precision;
	dEndX = iEndX / Precision;
	int iEndY = dEndY * Precision;
	dEndY = iEndY / Precision;

	string strEndX = boost::lexical_cast<string>(dEndX);
	string strEndY = boost::lexical_cast<string>(dEndY);
	string strXVel = boost::lexical_cast<string>(curTool.m_dIdleXVelocity);
	string strYVel = boost::lexical_cast<string>(curTool.m_dIdleYVelocity);
	string strZVel = boost::lexical_cast<string>(curTool.m_dIdleZVelocity);
	string strX1Vel = boost::lexical_cast<string>(curTool.m_dIdleX1Velocity);
	string strY1Vel = boost::lexical_cast<string>(curTool.m_dIdleY1Velocity);

	Axis eDirectionX = enum_cast<Axis>(curTool.m_strDirectionX).value();
	Axis eDirectionY = enum_cast<Axis>(curTool.m_strDirectionY).value();
	string strXIndex = boost::lexical_cast<string>(m_mapMotorValue[eDirectionX].AxisIndex);
	string strYIndex = boost::lexical_cast<string>(m_mapMotorValue[eDirectionY].AxisIndex);
	string strZIndex = boost::lexical_cast<string>(m_mapMotorValue[Axis::Z].AxisIndex);
	string strZPosition = boost::lexical_cast<string>(curTool.m_dCuttingHeight + curTool.m_dCuttingHeightCompensate);
	string strIdleZHeight = boost::lexical_cast<string>(curTool.m_dIdleZHeight/*-m_dStandard+curTool.m_dCuttingHeightCompensate*/);
	string strYPosition = boost::lexical_cast<string>(curTool.m_dYPosition / 360 * PI * m_dDiameter);

	if (curTool.m_bAZero)
	{
		string strthetaIndex = boost::lexical_cast<string>(m_mapMotorValue[Axis::A].AxisIndex);
		m_strCommand += "SET FPOS(" + strthetaIndex + ")=" + strYPosition + "\n";
	}
	if (curTool.m_bA1Zero)
	{
		string strthetaIndex = boost::lexical_cast<string>(m_mapMotorValue[Axis::A1].AxisIndex);
		m_strCommand += "SET FPOS(" + strthetaIndex + ")=" + strYPosition + "\n";
	}

	m_strCommand += "LINE/V (" + strXIndex + "," + strYIndex + "," + strZIndex + ")," + "APOS" + strXIndex + ",APOS" + strYIndex + "," + strIdleZHeight + "," + strZVel + "\n";
	m_strCommand += "LINE/V (" + strXIndex + "," + strYIndex + "," + strZIndex + ")," + strEndX + ", " + strEndY + ",APOS" + strZIndex + "," + strXVel + "\n";
	m_strCommand += "LINE/V (" + strXIndex + "," + strYIndex + "," + strZIndex + ")," + "APOS" + strXIndex + ",APOS" + strYIndex + "," + strZPosition + "," + strZVel + "\n";

	m_dPreX = dEndX;
	m_dPreY = dEndY;
}

string ACSMotionControl::GetCuttingCommand()
{
	return m_strCommand;
}

void ACSMotionControl::OffsetArcToSimple(double dEndX, double dEndY, double dCenterX, double dCenterY, bool bClockwise, const Tool& curTool, double dIncX, double dIncY)
{
	double Precision = 1000;
	int iEndX = dEndX * Precision;
	dEndX = iEndX / Precision;
	int iEndY = dEndY * Precision;
	dEndY = iEndY / Precision;

	if ((m_dPreX == dEndX) && (m_dPreY == dEndY))
		return;

	if (m_dPreY == dEndY)	// 在同一水平线上
	{
		dCenterX = (m_dPreX + dEndX) / 2;
	}
	else if (m_dPreX == dEndX)
	{
		dCenterY = (m_dPreY + dEndY) / 2;
	}
	else if ((fabs(m_dPreX + dEndX - 2 * dCenterX) <= 0.001 && fabs(m_dPreY + dEndY - 2 * dCenterY) <= 0.001))
	{
		dCenterX = (m_dPreX + dEndX) / 2;
		dCenterY = (m_dPreY + dEndY) / 2;
	}
	else
	{
		double B = 1;
		double A = (dEndX - m_dPreX) / (dEndY - m_dPreY);
		double C = -A * (m_dPreX + dEndX) / 2 - (m_dPreY + dEndY) / 2;
		double dTempCenterX = dCenterX;
		dCenterX = (B * B * dCenterX - A * B * dCenterY - A * C) / (A * A + B * B);
		dCenterY = (-A * B * dTempCenterX + A * A * dCenterY - B * C) / (A * A + B * B);
	}

	string strCenterX	= boost::lexical_cast<string>(dCenterX);
	string strCenterY	= boost::lexical_cast<string>(dCenterY);
	string strEndX		= boost::lexical_cast<string>(dEndX);
	string strEndY		= boost::lexical_cast<string>(dEndY);
	string strVel		= boost::lexical_cast<string>(curTool.m_dArcVelocity);
	Axis   eDirectionX	= enum_cast<Axis>(curTool.m_strDirectionX).value();
	Axis   eDirectionY	= enum_cast<Axis>(curTool.m_strDirectionY).value();
	string strXIndex	= boost::lexical_cast<string>(m_mapMotorValue[eDirectionX].AxisIndex);
	string strYIndex	= boost::lexical_cast<string>(m_mapMotorValue[eDirectionY].AxisIndex);
	string strZIndex	= boost::lexical_cast<string>(m_mapMotorValue[Axis::Z].AxisIndex);
	string strArcDir	= bClockwise ? "-": "+";

	m_strCommand += "ARC1/V (" + strXIndex + ", " + strYIndex + "," + strZIndex + "), " + strCenterX + ", " + strCenterY + ","
		+ strEndX + ", " + strEndY + ", " + "APOS" + strZIndex + "," + strArcDir + ", " + strVel + "\n";
	m_strCommand += "STOPPER (" + strXIndex + ", " + strYIndex + "," + strZIndex + ")" + "\n";
	
	m_dPreX = dEndX;
	m_dPreY = dEndY;
}

void ACSMotionControl::OffsetLineToSimple(double dEndX, double dEndY, const Tool& curTool)
{
	double Precision = 1000;
	int iEndX = dEndX * Precision;
	dEndX = iEndX / Precision;
	int iEndY = dEndY * Precision;
	dEndY = iEndY / Precision;

	if ((m_dPreX == dEndX) && (m_dPreY == dEndY))
		return;

	string strEndX = boost::lexical_cast<string>(dEndX);
	string strEndY = boost::lexical_cast<string>(dEndY);
	string strVel = boost::lexical_cast<string>(curTool.m_dLineVelocity);
	Axis   eDirectionX = enum_cast<Axis>(curTool.m_strDirectionX).value();
	Axis   eDirectionY = enum_cast<Axis>(curTool.m_strDirectionY).value();
	string strXIndex = boost::lexical_cast<string>(m_mapMotorValue[eDirectionX].AxisIndex);
	string strYIndex = boost::lexical_cast<string>(m_mapMotorValue[eDirectionY].AxisIndex);
	string strZIndex = boost::lexical_cast<string>(m_mapMotorValue[Axis::Z].AxisIndex);

	m_strCommand += "LINE/V (" + strXIndex + ", " + strYIndex + "," + strZIndex + "), " + strEndX + ", " + strEndY + ", " + "APOS" + strZIndex + "," + strVel + "\n";
	m_strCommand += "STOPPER (" + strXIndex + ", " + strYIndex + "," + strZIndex + ")" + "\n";

	m_dPreX = dEndX;
	m_dPreY = dEndY;
}

void ACSMotionControl::ProLaserControlSimple(bool bLaser, bool bPso, const Tool& curTool)
{
	string strLaserNum	= m_mapDigitalOUT[DigitalOUT::Laser].strIndex;
	Axis   eDirectionX	= enum_cast<Axis>(curTool.m_strDirectionX).value();
	Axis   eDirectionY	= enum_cast<Axis>(curTool.m_strDirectionY).value();
	string strXIndex	= boost::lexical_cast<string>(m_mapMotorValue[eDirectionX].AxisIndex);
	string strYIndex	= boost::lexical_cast<string>(m_mapMotorValue[eDirectionY].AxisIndex);
	
	string strLaserOnBWait  = boost::lexical_cast<string>(m_dLaserOnBWait);
	string strLaserOnAWait  = boost::lexical_cast<string>(m_dLaserOnAWait);
	string strLaserOffBWait = boost::lexical_cast<string>(m_dLaserOffBWait);
	string strLaserOffAWait = boost::lexical_cast<string>(m_dLaserOffAWait);
	string strEnd = "\n";
	if (bLaser)
	{
		if (curTool.m_bBlow2)
		{
			string strValue = m_mapDigitalOUT[DigitalOUT::Blow2].bInversion ? "0" : "1";
			m_strCommand += m_mapDigitalOUT[DigitalOUT::Blow2].strIndex + "=" + strValue + ";";
		}
		else
		{
			string strValue = m_mapDigitalOUT[DigitalOUT::Blow].bInversion ? "0" : "1";
			m_strCommand += m_mapDigitalOUT[DigitalOUT::Blow].strIndex + "=" + strValue + ";";
		}

		string strBlowDelay	 = boost::lexical_cast<string>(m_dBlowDelay);
		string strLaserValue = m_mapDigitalOUT[DigitalOUT::Laser].bInversion ? "0" : "1";
		string strTill		 = m_mapDigitalOUT[DigitalOUT::Laser].bInversion ? "TILL ^" : "TILL ";
	
		m_strCommand += strEnd + "WAIT " + strBlowDelay + ";";
		m_strCommand += "WAIT " + strLaserOnBWait + strEnd;
		m_strCommand += strLaserNum + "=" + strLaserValue + ";";
		m_strCommand += strTill + strLaserNum + ";";
		m_strCommand += "WAIT " + strLaserOnAWait + strEnd;
	}
	else
	{
		string strLaserValue = m_mapDigitalOUT[DigitalOUT::Laser].bInversion ? "1" : "0";
		string strTill		 = m_mapDigitalOUT[DigitalOUT::Laser].bInversion ? "TILL " : "TILL ^";
		
		m_strCommand += "WAIT " + strLaserOffBWait + strEnd;
		m_strCommand += strLaserNum + "=" + strLaserValue + ";";
		m_strCommand += strTill + strLaserNum + ";";
		m_strCommand += "WAIT " + strLaserOffAWait + strEnd;
	}
}

void ACSMotionControl::EndProgramCommandSimple(const Tool& curTool)
{
	Axis   eDirectionX = enum_cast<Axis>(curTool.m_strDirectionX).value();
	Axis   eDirectionY = enum_cast<Axis>(curTool.m_strDirectionY).value();
	string strXIndex = boost::lexical_cast<string>(m_mapMotorValue[eDirectionX].AxisIndex);
	string strYIndex = boost::lexical_cast<string>(m_mapMotorValue[eDirectionY].AxisIndex);
	string strZIndex = boost::lexical_cast<string>(m_mapMotorValue[Axis::Z].AxisIndex);
	string strEnd = "\n";

	m_strCommand += "ENDS ("  + strXIndex + ", " + strYIndex + "," + strZIndex + ")" + strEnd;
	m_strCommand += "GO ("	  + strXIndex + ", " + strYIndex + "," + strZIndex + ")" + strEnd;
	m_strCommand += "SPLIT (" + strXIndex + ", " + strYIndex + "," + strZIndex + ")" + strEnd;

	if (curTool.m_bStopBlow)
	{
		if (curTool.m_bBlow2)
		{
			string strValue = m_mapDigitalOUT[DigitalOUT::Blow2].bInversion ? "1" : "0";
			m_strCommand += m_mapDigitalOUT[DigitalOUT::Blow2].strIndex + "=" + strValue + ";\n";
		}
		else
		{
			string strValue = m_mapDigitalOUT[DigitalOUT::Blow].bInversion ? "0" : "1";
			m_strCommand += m_mapDigitalOUT[DigitalOUT::Blow].strIndex + "=" + strValue + ";\n";
		}
	}
	m_strCommand += "STOP\n";
}

bool ACSMotionControl::StopMovingCuttingHead()
{
	string strZIndex = boost::lexical_cast<string>(m_mapMotorValue[Axis::Z].AxisIndex);

	m_strCommand += "STOP 6\n";
	m_strCommand += "MFLAGS (" + strZIndex + ").17=1\n";
	return true;
}

bool ACSMotionControl::StartMovingCuttingHead(const Tool& curTool)
{
	double dServoCuttingHeight = curTool.m_dServoCuttingHeight;
	string strServoCuttingHeight = boost::lexical_cast<string>(dServoCuttingHeight);
	m_strCommand += "qiegegaodu=" + strServoCuttingHeight + "\n";
	m_strCommand += "START 6,1\n";

	m_strCommand += "TILL NJHT_AIN0<=" + strServoCuttingHeight + "/10" + "*MaxPosition" + "+120" + "\n";
	//	m_strCommand += "TILL NJHT_AIN0<710\n";
	m_strCommand += "WAIT 10\n";
	return true;
}

bool ACSMotionControl::SetMFLAGSValue(Axis eAxis, int iValue)
{
	int iMFLAGS;
	string strIndex = boost::lexical_cast<string>(m_mapMotorValue[eAxis].AxisIndex);
	string strMFLAGS = "MFLAGS" + strIndex;
	if (!acsc_ReadInteger(m_hHandle, ACSC_NONE, strMFLAGS.data(),
		ACSC_NONE, ACSC_NONE, ACSC_NONE, ACSC_NONE, &iMFLAGS, NULL))
	{
		LogError();
		return false;
	}

	int bit = 131072;		//第17位
	if (iValue)
		iMFLAGS = iMFLAGS | bit;
	else
		iMFLAGS = iMFLAGS & (~bit);

	if (!acsc_WriteInteger(m_hHandle, ACSC_NONE, strMFLAGS.data(),
		ACSC_NONE, ACSC_NONE, ACSC_NONE, ACSC_NONE, &iMFLAGS, NULL))
	{
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::AcscReadReal(const string strCommand, double& dValue)
{
	string strCommands = strCommand;
	if (!acsc_ReadReal(m_hHandle, ACSC_NONE, strCommands.data(),
		ACSC_NONE, ACSC_NONE, ACSC_NONE, ACSC_NONE, &dValue, NULL))
	{
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::AcscWriteReal(const string strCommand, double dValue)
{
	string strCommands = strCommand;
	if (!acsc_WriteReal(m_hHandle, ACSC_NONE, strCommands.data(), 
		ACSC_NONE, ACSC_NONE, ACSC_NONE, ACSC_NONE, &dValue, NULL))
	{
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::AcscReadInt(const string strCommand, int& iValue)
{
	string strCommands = strCommand;
	if (!acsc_ReadInteger(m_hHandle, ACSC_NONE, strCommands.data(),
		ACSC_NONE, ACSC_NONE, ACSC_NONE, ACSC_NONE, &iValue, NULL))
	{
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::AcscWriteInt(const string strCommand, int iValue)
{
	string strCommands = strCommand;
	if (!acsc_WriteInteger(m_hHandle, ACSC_NONE, strCommands.data(),
		ACSC_NONE, ACSC_NONE, ACSC_NONE, ACSC_NONE, &iValue, NULL))
	{
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::CheckBuffer(int iBufferIndex, string& strCommand)
{
	if (!acsc_StopBuffer(m_hHandle, iBufferIndex, ACSC_SYNCHRONOUS))
	{
		LogError();
		return false;
	}
	if (!acsc_LoadBuffer(m_hHandle, iBufferIndex, strCommand.data(), strCommand.length(), ACSC_SYNCHRONOUS))
	{
		LogError();
		return false;
	}
	if (!acsc_CompileBuffer(m_hHandle, iBufferIndex, ACSC_SYNCHRONOUS))
	{
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::RunBuffer(int iBufferIndex)
{
	if (!acsc_RunBuffer(m_hHandle, iBufferIndex, NULL, ACSC_SYNCHRONOUS))
	{
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::PauseBuffer(int iBufferIndex)
{
	if (!acsc_SuspendBuffer(m_hHandle, iBufferIndex, ACSC_SYNCHRONOUS))
	{
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::GetBufferState(int iBufferIndex, int& iState)
{
	if (!acsc_GetProgramState(m_hHandle, iBufferIndex, &iState, ACSC_SYNCHRONOUS))
	{
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::LoadCommandAndRunBuffer(int iBufferIndex, string strCommand, int iTimeout)
{
	if (!CheckBuffer(iBufferIndex, strCommand))
		return false;

	if (!RunBuffer(iBufferIndex))
		return false;

	if (!acsc_WaitProgramEnd(m_hHandle, iBufferIndex, iTimeout))
	{
		LogError();
		return false;
	}
	return true;
}

bool ACSMotionControl::IsReachPos(Axis eAxis, bool bRelative, double dPos)
{
	if (bRelative)	//相对
	{
		double dNowPos;
		if (!GetActualPos(eAxis, dNowPos))
			return false;
		if (((dNowPos + dPos) >= m_mapMotorValue[eAxis].NegLimit) && ((dNowPos + dPos) <= m_mapMotorValue[eAxis].PosLimit))
			return true;
	}
	else			//绝对
	{
		if ((dPos >= m_mapMotorValue[eAxis].NegLimit) && (dPos <= m_mapMotorValue[eAxis].PosLimit))
			return true;
	}
	return false;
}

#pragma region FlightCutting
void ACSMotionControl::BeginACSSegmentForFlightCutting(const Tool& tool)
{
	int iXIndex = m_mapMotorValue[enum_cast<Axis>(tool.m_strDirectionX).value()].AxisIndex;
	int iYIndex = m_mapMotorValue[enum_cast<Axis>(tool.m_strDirectionY).value()].AxisIndex;
	string strXIndex = boost::lexical_cast<string>(iXIndex);
	string strYIndex = boost::lexical_cast<string>(iYIndex);
	string strDVel = boost::lexical_cast<string>(tool.m_dLineVelocity);
	string strFVel = boost::lexical_cast<string>(tool.m_dXsegEndVelocity);
	string strJVel = boost::lexical_cast<string>(tool.m_dJunctionVelocity);
	string strAngle = boost::lexical_cast<string>(tool.m_dJunctionAngle * 3.1415926 / 180);
	SetCuttingAccJerk(tool);
	string strEnd = "\n";

	string strLaserNum = m_mapDigitalOUT[DigitalOUT::Laser].strIndex;
	string strIO;
	if (strLaserNum.find(".") != string::npos)
		strIO = strLaserNum.substr(strLaserNum.find(".") + 1, strLaserNum.length());
	else
		strIO = "ERROR";
	int iIO = atoi(strIO.c_str());
	int iMask = 1 << iIO;
	string strDate = boost::lexical_cast<string>(iMask);

	m_strCommand += "GLOBAL INT VAL_ON(1)" + strEnd;
	m_strCommand += "GLOBAL INT VAL_OFF(1)" + strEnd;
	m_strCommand += "GLOBAL INT MASK(1)" + strEnd;

	m_strCommand += "VAL_ON(0) = " + strDate + strEnd;
	m_strCommand += "VAL_OFF(0) = 0" + strEnd;
	m_strCommand += "MASK(0) = " + strDate + strEnd;

	double dDelay = tool.m_dFlightCutting_MotorDelay;
	string strDelay = boost::lexical_cast<string>(dDelay);

	m_strCommand += "XSEG/VFJAQ (" + strXIndex + ", " + strYIndex + "), APOS" + strXIndex
		+ ", APOS" + strYIndex + ", " + strDVel + ", " + strFVel + ", "
		+ strJVel + ", " + strAngle + ", " + strDelay + strEnd;
}

void ACSMotionControl::OffsetFlightLineTo(double dEndX, double dEndY, const Tool& tool, bool bPolyGuide)
{
	int iXIndex = m_mapMotorValue[enum_cast<Axis>(tool.m_strDirectionX).value()].AxisIndex;
	int iYIndex = m_mapMotorValue[enum_cast<Axis>(tool.m_strDirectionY).value()].AxisIndex;
	string strXIndex = boost::lexical_cast<string>(iXIndex);
	string strYIndex = boost::lexical_cast<string>(iYIndex);

	string strLaserNum = m_mapDigitalOUT[DigitalOUT::Laser].strIndex;
	char prevCharAxis = '\0'; // 初始化为空字符
	size_t dotPos = strLaserNum.find(".");

	// 安全判断：确保字符串不为空，包含小数点，且小数点不是第一个字符
	if (!strLaserNum.empty() && dotPos != string::npos && dotPos > 0) {
		prevCharAxis = strLaserNum[dotPos - 1]; // 获取小数点前一位字符
	}

	double dVelocity = tool.m_dLineVelocity;

	if ((m_dPreX == dEndX) && (m_dPreY == dEndY))
	{
		return;
	}

	string strEndX = boost::lexical_cast<string>(dEndX);
	string strEndY = boost::lexical_cast<string>(dEndY);
	string strVel = boost::lexical_cast<string>(dVelocity);


	if (bPolyGuide)
	{
		m_strCommand += "LINE/vo (" + strXIndex + ", " + strYIndex + "), " + strEndX + ", " + strEndY + ", " + strVel + ", VAL_OFF, OUT, " + prevCharAxis + ", MASK" + "\n";
	}
	else
	{
		m_strCommand += "LINE/vo (" + strXIndex + ", " + strYIndex + "), " + strEndX + ", " + strEndY + ", " + strVel + ", VAL_ON, OUT, " + prevCharAxis + ", MASK" + "\n";
	}

	m_dPreX = dEndX;
	m_dPreY = dEndY;
}

void ACSMotionControl::OffsetFlightArcTo(double dEndX, double dEndY, double dCenterX, double dCenterY, bool bClockwise, const Tool& tool, double dIncX, double dIncY, bool bPolyGuide)
{
	int iXIndex = m_mapMotorValue[enum_cast<Axis>(tool.m_strDirectionX).value()].AxisIndex;
	int iYIndex = m_mapMotorValue[enum_cast<Axis>(tool.m_strDirectionY).value()].AxisIndex;
	string strXIndex = boost::lexical_cast<string>(iXIndex);
	string strYIndex = boost::lexical_cast<string>(iYIndex);

	string strLaserNum = m_mapDigitalOUT[DigitalOUT::Laser].strIndex;
	char prevCharAxis = '\0'; // 初始化为空字符
	size_t dotPos = strLaserNum.find(".");
	// 安全判断：确保字符串不为空，包含小数点，且小数点不是第一个字符
	if (!strLaserNum.empty() && dotPos != string::npos && dotPos > 0) {
		prevCharAxis = strLaserNum[dotPos - 1]; // 获取小数点前一位字符
	}

	double dVelocity = tool.m_dLineVelocity;

	if ((m_dPreX == dEndX) && (m_dPreY == dEndY))
	{
		return;
	}


	if (m_dPreY == dEndY)	// 在同一水平线上
	{
		dCenterX = (m_dPreX + dEndX) / 2;
	}
	else if (m_dPreX == dEndX)
	{
		dCenterY = (m_dPreY + dEndY) / 2;
	}
	else if ((fabs(m_dPreX + dEndX - 2 * dCenterX) <= 0.001 && fabs(m_dPreY + dEndY - 2 * dCenterY) <= 0.001))
	{
		dCenterX = (m_dPreX + dEndX) / 2;
		dCenterY = (m_dPreY + dEndY) / 2;
	}
	else
	{
		double B = 1;
		double A = (dEndX - m_dPreX) / (dEndY - m_dPreY);
		double C = -A * (m_dPreX + dEndX) / 2 - (m_dPreY + dEndY) / 2;
		double dTempCenterX = dCenterX;
		dCenterX = (B * B * dCenterX - A * B * dCenterY - A * C) / (A * A + B * B);
		dCenterY = (-A * B * dTempCenterX + A * A * dCenterY - B * C) / (A * A + B * B);
	}

	string strCenterX = boost::lexical_cast<string>(dCenterX);
	string strCenterY = boost::lexical_cast<string>(dCenterY);
	string strEndX = boost::lexical_cast<string>(dEndX);
	string strEndY = boost::lexical_cast<string>(dEndY);
	string strVel = boost::lexical_cast<string>(dVelocity);
	string strArcDir;
	if (bClockwise)
	{
		strArcDir = "-";
	}
	else
	{
		strArcDir = "+";
	}
	if (bPolyGuide)
	{
		m_strCommand += "ARC1/vo (" + strXIndex + ", " + strYIndex + "), " + strCenterX + ", " + strCenterY + ", "
			+ strEndX + ", " + strEndY + ", " + strArcDir + ", " + strVel + ", VAL_OFF, OUT, " + prevCharAxis + ", MASK" + "\n";
	}
	else
	{
		m_strCommand += "ARC1/vo (" + strXIndex + ", " + strYIndex + "), " + strCenterX + ", " + strCenterY + ", "
			+ strEndX + ", " + strEndY + ", " + strArcDir + ", " + strVel + ", VAL_ON, OUT, " + prevCharAxis + ", MASK" + "\n";
	}
	m_dPreX = dEndX;
	m_dPreY = dEndY;
}

void ACSMotionControl::OffsetFlightArc2To(double dCenterX, double dCenterY, double dAngle, const Tool& tool, bool bPolyGuide)
{
	string strCenterX = boost::lexical_cast<string>(dCenterX);
	string strCenterY = boost::lexical_cast<string>(dCenterY);
	string strVel = boost::lexical_cast<string>(tool.m_dArcVelocity);
	int iXIndex = m_mapMotorValue[enum_cast<Axis>(tool.m_strDirectionX).value()].AxisIndex;
	int iYIndex = m_mapMotorValue[enum_cast<Axis>(tool.m_strDirectionY).value()].AxisIndex;
	string strXIndex = boost::lexical_cast<string>(iXIndex);
	string strYIndex = boost::lexical_cast<string>(iYIndex);
	string strAngle = boost::lexical_cast<string>(dAngle);

	string strLaserNum = m_mapDigitalOUT[DigitalOUT::Laser].strIndex;
	char prevCharAxis = '\0'; // 初始化为空字符
	size_t dotPos = strLaserNum.find(".");
	// 安全判断：确保字符串不为空，包含小数点，且小数点不是第一个字符
	if (!strLaserNum.empty() && dotPos != string::npos && dotPos > 0) {
		prevCharAxis = strLaserNum[dotPos - 1]; // 获取小数点前一位字符
	}

	if (bPolyGuide)
	{
		m_strCommand += "ARC2/vo (" + strXIndex + ", " + strYIndex + "), " + strCenterX + ", " + strCenterY + ", "
			+ strAngle + ", " + strVel + ", VAL_OFF, OUT, " + prevCharAxis + ", MASK" + "\n";
	}
	else
	{
		m_strCommand += "ARC2/vo (" + strXIndex + ", " + strYIndex + "), " + strCenterX + ", " + strCenterY + ", "
			+ strAngle + ", " + strVel + ", VAL_ON, OUT, " + prevCharAxis + ", MASK" + "\n";
	}
}

void ACSMotionControl::EndProgramCommandForFlightCutting(const Tool& tool)
{
	int iXIndex = m_mapMotorValue[enum_cast<Axis>(tool.m_strDirectionX).value()].AxisIndex;
	int iYIndex = m_mapMotorValue[enum_cast<Axis>(tool.m_strDirectionY).value()].AxisIndex;
	string strXIndex = boost::lexical_cast<string>(iXIndex);
	string strYIndex = boost::lexical_cast<string>(iYIndex);
	string strEnd = "\n";

	
	m_strCommand += "ENDS (" + strXIndex + ", " + strYIndex + ")" + strEnd;
	m_strCommand += "SPLIT (" + strXIndex + ", " + strYIndex + ")" + strEnd;
	m_strCommand += m_mapDigitalOUT[DigitalOUT::Laser].strIndex + "=0;\n";
	if (tool.m_bStopBlow)
	{
		if (tool.m_bBlow2)
			m_strCommand += m_mapDigitalOUT[DigitalOUT::Blow2].strIndex + "=0;\n";
		else
			m_strCommand += m_mapDigitalOUT[DigitalOUT::Blow].strIndex + "=0;\n";
	}
	bool bEnergySwitchUse = DT::getCustomerID() == "MaiTong"
		|| int(DT::getPermission()) > (int)PermissionLevel::Factory;
	if (bEnergySwitchUse && tool.m_bEnergySwitch)
		m_strCommand += "EnergySwitch=0\n";
	m_strCommand += "STOP\n";
}

bool ACSMotionControl::LoadAndCompileBuffer(int iBufferID)
{
	char* pcCommand = new char[m_strCommand.length() + 1];
	strcpy(pcCommand, m_strCommand.c_str());

	bool success = true;
	acsc_StopBuffer(m_hHandle, iBufferID, ACSC_SYNCHRONOUS);
	int length = strlen(pcCommand);
	int iReturn = acsc_LoadBuffer(m_hHandle, iBufferID, pcCommand, strlen(pcCommand), ACSC_SYNCHRONOUS);
	if (!iReturn)
	{
		success = false;
		iReturn = acsc_GetLastError();
	}
	iReturn = acsc_CompileBuffer(m_hHandle, iBufferID, ACSC_SYNCHRONOUS);
	if (!iReturn)
	{
		success = false;
		iReturn = acsc_GetLastError();
	}
	delete[]pcCommand;
	return success;
}

bool ACSMotionControl::RunBufferForFlightCutting(int iBufferID)
{
	bool success = true;
	int iReturn = acsc_RunBuffer(m_hHandle, iBufferID, NULL, ACSC_SYNCHRONOUS);
	if (!iReturn)
	{
		success = false;
		iReturn = acsc_GetLastError();
	}
	return success;
}
#pragma endregion FlightCutting
