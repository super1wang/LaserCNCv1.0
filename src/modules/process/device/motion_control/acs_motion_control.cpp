#include "acs_motion_control.h"
//#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <cmath>
#include <fstream>
#include <limits>
//#include "CoreUtils.h"
#include "bdaqctrl.h"
#include "core/logging/logger.h"
#include "modules/process/system/process_numeric_constants.h"
#include "modules/process/runtime/process_runtime_configuration.h"

#include "magic_enum.hpp"

using namespace Automation::BDaq;

using lcnc::process::AnalogOUT;
using lcnc::process::Axis;
using lcnc::process::DigitalOUT;
using lcnc::process::PermissionLevel;
using std::ofstream;
using std::ios;
using std::string;
using std::vector;
using toml::table;
#define DEBUG_MODE

namespace {

// 安全地查表：避免 std::map::operator[] 在键缺失时插入默认 DigitalIOData{strIndex=""}。
// 该默认对象后续会被拼成形如 "=1;" 的残缺指令（参见 ProLaserControl / EndProgramCommand），
// 是切割文本中出现裸 "=0;" / "=1;" 的根因（Fix #3）。
// 返回 true 表示命中且 strIndex 非空。
template <class K>
bool resolveDigital(const std::map<K, DigitalIOData>& m, K key,
					DigitalIOData& out, const char* contextLabel)
{
	auto it = m.find(key);
	if (it == m.end() || it->second.strIndex.empty())
	{
		LCNC_WARN(lcnc::LogCode::Generic, "{}", std::string("ACS digital IO missing or unconfigured: ") + contextLabel
					+ " (key=" + std::string(magic_enum::enum_name(key)) + "); generated command skipped.");
		out = DigitalIOData{};
		return false;
	}
	out = it->second;
	return true;
}

} // namespace

ACSMotionControl::ACSMotionControl(lcnc::process::ProcessSettingsService& settings,
	                                 lcnc::process::ProcessRuntimeConfiguration& runtimeConfiguration)
	: MotionControl(settings, runtimeConfiguration)
	, m_bConnectFlag(false)
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
	// ProcessDeviceRuntime 正常销毁路径会先断开；析构兜底不能让 SDK 会话遗留。
	Disconnect();
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
		LCNC_ERR(lcnc::LogCode::Generic, "{}", ErrorStr);
	}
	else
		LCNC_ERR(lcnc::LogCode::Generic, "{}", boost::lexical_cast<string>(ErrorCode));
}

void ACSMotionControl::CreateMotor(Axis eAxis, const table& tAxis)
{
	SetAxisIndex(eAxis, tAxis.at("iIndex").as_integer());
	string strName = magic_enum::enum_name(eAxis).data();
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
	if (m_hHandle == ACSC_INVALID)
	{
		LogError();
		return false;
	}
	if (!acsc_StopBuffer(m_hHandle, ACSC_NONE, NULL))
	{
		LogError();
		acsc_CloseComm(m_hHandle);
		m_hHandle = ACSC_INVALID;
		return false;
	}
	if (!AfterOpenComm())
	{
		acsc_CloseComm(m_hHandle);
		m_hHandle = ACSC_INVALID;
		return false;
	}

	setlocale(LC_ALL, "Chinese-simplified");
	setlocale(LC_ALL, "C");

	LCNC_INFO(lcnc::LogCode::Generic, "{}", "ACS motion control is successfully connected.");
	return true;
}

bool ACSMotionControl::Disconnect()
{
	if (m_hHandle == ACSC_INVALID)
	{
		m_bConnectFlag = false;
		return true;
	}

	bool success = true;
	// 关闭激光
	if (!acsc_SetOutput(m_hHandle, 0, 4, 0, NULL))
	{
		LogError();
		success = false;
	}

	if (!StopMotion())
		success = false;
	constexpr int kStopTimeoutMs = 30000;
	int waitedMs = 0;
	while (IsAxisMoving() && waitedMs < kStopTimeoutMs) {
		Sleep(100);
		waitedMs += 100;
	}
	if (waitedMs >= kStopTimeoutMs)
	{
		LCNC_ERR(lcnc::LogCode::Generic, "{}", "ACS Disconnect timed out waiting for axis motion to stop.");
		success = false;
	}

	if (!acsc_CloseComm(m_hHandle))
	{
		LogError();
		return false;
	}
	m_bConnectFlag = false;
	m_hHandle = ACSC_INVALID;
	m_mapMotorValue.clear();
	return success;
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
	int arr[] = { 2, 1, 0, 3 };

	for (int i = 0; i < 4; ++i)
	{
		Axis eAxis = static_cast<Axis>(arr[i]);
		if (!m_runtimeConfiguration.isAxisEnabled(eAxis))
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
			LCNC_ERR(lcnc::LogCode::Generic, "{}", "Stop home over time.");
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
	bool bEnergySwitchUse = m_runtimeConfiguration.customerId() == "MaiTong"
		|| int(m_runtimeConfiguration.permission()) > int(PermissionLevel::Factory);
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

bool ACSMotionControl::SetFPosition(Axis eAxis, double dPos)
{
	// setfpos：将轴反馈位置寄存器直接重写为 dPos（用户单位），不产生运动。
	if (!acsc_SetFPosition(m_hHandle, m_mapMotorValue[eAxis].AxisIndex, dPos, NULL))
	{
		LogError();
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
	int iEfac = iResolutionRatio * 10000 / (lcnc::process::kPi * dDiamater);
	if (IsConnected())
	{
		if (!WriteEFAC(eAxis, iEfac))
			return false;

		if (eAxis == Axis::A)
			m_dDiameter = dDiamater;

		double dDiamaterTwo = 0;
	bool bXVSuccess = false;
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
			double dXVEL = lcnc::process::kPi * dDiamaterTwo * 5;		// 5为轴每秒旋转圈数
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
		LCNC_ERR(lcnc::LogCode::Generic, "{}", fmt::format("Set axis {} vel {} failed.", m_mapMotorValue[eAxis].Name, dVel));
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
		LCNC_ERR(lcnc::LogCode::Generic, "{}", fmt::format("Set axis {} acc {} failed.", m_mapMotorValue[eAxis].Name, dAcc));
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
		LCNC_ERR(lcnc::LogCode::Generic, "{}", fmt::format("Set axis {} dec {} failed.", m_mapMotorValue[eAxis].Name, dDec));
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
		LCNC_ERR(lcnc::LogCode::Generic, "{}", fmt::format("Set axis {} jerk {} failed.", m_mapMotorValue[eAxis].Name, dJerk));
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
		LCNC_ERR(lcnc::LogCode::Generic, "{}", fmt::format("Set axis {} neg limit {} failed.", m_mapMotorValue[eAxis].Name, dNegLimit));
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
		LCNC_ERR(lcnc::LogCode::Generic, "{}", fmt::format("Set axis {} pos limit {} failed.", m_mapMotorValue[eAxis].Name, dPosLimit));
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
	dTubeDiamater = iResolutionRatio * 10000 / (iEfac * lcnc::process::kPi);
	m_dDiameter = dTubeDiamater;
	return true;
}

bool ACSMotionControl::GetAxisVel(Axis eAxis, double& dVel)
{
	if (!acsc_GetVelocity(m_hHandle, m_mapMotorValue[eAxis].AxisIndex, &dVel, NULL))
	{
		dVel = m_mapMotorValue[eAxis].Velocity;
		LCNC_ERR(lcnc::LogCode::Generic, "{}", fmt::format("Get axis {} vel failed.", m_mapMotorValue[eAxis].Name));
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
		LCNC_ERR(lcnc::LogCode::Generic, "{}", fmt::format("Get axis {} acc failed.", m_mapMotorValue[eAxis].Name));
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
		LCNC_ERR(lcnc::LogCode::Generic, "{}", fmt::format("Get axis {} dec failed.", m_mapMotorValue[eAxis].Name));
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
		LCNC_ERR(lcnc::LogCode::Generic, "{}", fmt::format("Get axis {} jerk failed.", m_mapMotorValue[eAxis].Name));
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
		LCNC_ERR(lcnc::LogCode::Generic, "{}", fmt::format("Get axis {} neg limit failed.", m_mapMotorValue[eAxis].Name));
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
		LCNC_ERR(lcnc::LogCode::Generic, "{}", fmt::format("Get axis {} pos limit failed.", m_mapMotorValue[eAxis].Name));
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


// [P3 removed] ACSMotionControl::IsQueueFull

// [P3 removed] ACSMotionControl::IsQueueEmpty

bool ACSMotionControl::IsQueueActive()
{
	return true;
}

// [P3 removed] ACSMotionControl::ClearQueue

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

// [P3 removed] ACSMotionControl::StopQueue

// [P3 removed] ACSMotionControl::IsOffsetCutting

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

// [P3 removed] ACSMotionControl::Punch

void ACSMotionControl::ResetProgramCommand()
{
	m_strCommand = "";
}

// [P3 removed] ACSMotionControl::BeginACSSegment

void ACSMotionControl::EndProgramCommand(const Tool& tool)
{
	string strZIndex = boost::lexical_cast<string>(m_mapMotorValue[Axis::Z].AxisIndex);
	m_strCommand += "kill " + strZIndex + "\n";

	if (tool.m_bStopBlow)
	{
		DigitalIOData io;
		const DigitalOUT key = tool.m_bBlow2 ? DigitalOUT::Blow2 : DigitalOUT::Blow;
		if (resolveDigital(m_mapDigitalOUT, key, io, "EndProgramCommand Blow OFF"))
			m_strCommand += io.strIndex + "=0;\n";
	}
	bool bEnergySwitchUse = m_runtimeConfiguration.customerId() == "MaiTong"
		|| int(m_runtimeConfiguration.permission()) > int(PermissionLevel::Factory);
	if (bEnergySwitchUse && tool.m_bEnergySwitch)
		m_strCommand += "EnergySwitch=0\n";
	m_strCommand += "STOP\n";
}

// [P3 removed] ACSMotionControl::MoveZCutting

// [P3 removed] ACSMotionControl::OffsetLineTo

// [P3 removed] ACSMotionControl::OffsetArcTo

// [P3 removed] ACSMotionControl::OffsetArc2To

// [P3 removed] ACSMotionControl::JumpToTrough

// 改设置界面为旋转轴置位
// [P3 removed] ACSMotionControl::JumpToSetAFPos

void ACSMotionControl::JumpToIdleXYPosition(double dEndX, double dEndY, const Tool& curTool)
{
	Axis eDirectionX = magic_enum::enum_cast<Axis>(curTool.m_strDirectionX).value();
	Axis eDirectionY = magic_enum::enum_cast<Axis>(curTool.m_strDirectionY).value();

	//X 定位轴
	if (curTool.m_bXIsMove && eDirectionX != Axis::X && m_runtimeConfiguration.isAxisEnabled(Axis::X))
	{
		string strXIndex	= boost::lexical_cast<string>(m_mapMotorValue[Axis::X].AxisIndex);
		string strXPosition = boost::lexical_cast<string>(curTool.m_dXPosition);
		string strXVel		= boost::lexical_cast<string>(curTool.m_dIdleXVelocity);
		m_strCommand += "PTP/EV " + strXIndex + "," + strXPosition + "," + strXVel + "\n";
	}
	//X1 定位轴
	if (curTool.m_bX1IsMove && m_runtimeConfiguration.isExtensionAxis("X1"))
	{
		string strX1Index	 = boost::lexical_cast<string>(m_mapMotorValue[magic_enum::enum_cast<Axis>("X").value_or(Axis::X)].AxisIndex);
		string strX1Position = boost::lexical_cast<string>(curTool.m_dX1Position);
		string strX1Vel		 = boost::lexical_cast<string>(curTool.m_dIdleX1Velocity);
		m_strCommand += "PTP/EV " + strX1Index + "," + strX1Position + "," + strX1Vel + "\n";
	}
	//A 定位轴
	if (curTool.m_bAIsMove && eDirectionY != Axis::A && m_runtimeConfiguration.isAxisEnabled(Axis::A))
	{
		string strAIndex	= boost::lexical_cast<string>(m_mapMotorValue[Axis::A].AxisIndex);
		string strAPosition = boost::lexical_cast<string>(curTool.m_dAPosition / 360 * lcnc::process::kPi * m_dDiameter);
		string strAVel		= boost::lexical_cast<string>(curTool.m_dIdleAVelocity);
		m_strCommand += "PTP/EV " + strAIndex + "," + strAPosition + "," + strAVel + "\n";
	}
	//A1 定位轴
	if (curTool.m_bA1IsMove && m_runtimeConfiguration.isExtensionAxis("A1"))
	{
		string strA1Index = boost::lexical_cast<string>(m_mapMotorValue[magic_enum::enum_cast<Axis>("A").value_or(Axis::A)].AxisIndex);
		string strA1Position = boost::lexical_cast<string>(curTool.m_dA1Position / 360 * lcnc::process::kPi * m_dDiameter);
		string strA1Vel = boost::lexical_cast<string>(curTool.m_dIdleA1Velocity);
		m_strCommand += "PTP/EV " + strA1Index + "," + strA1Position + "," + strA1Vel + "\n";
	}
	//Y 定位轴
	if (curTool.m_bYIsMove && eDirectionY != Axis::Y && m_runtimeConfiguration.isAxisEnabled(Axis::Y))
	{
		string strYIndex	= boost::lexical_cast<string>(m_mapMotorValue[Axis::Y].AxisIndex);
		string strYPosition = boost::lexical_cast<string>(curTool.m_dYPosition);
		string strYVel		= boost::lexical_cast<string>(curTool.m_dIdleYVelocity);
		m_strCommand += "PTP/EV " + strYIndex + "," + strYPosition + "," + strYVel + "\n";
	}
	//Y1 定位轴
	if (curTool.m_bY1IsMove && m_runtimeConfiguration.isExtensionAxis("Y1"))
	{
		string strY1Index	 = boost::lexical_cast<string>(m_mapMotorValue[magic_enum::enum_cast<Axis>("Y").value_or(Axis::Y)].AxisIndex);
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

bool ACSMotionControl::SendCommand()
{
	//char* pcCommand = new char[m_strCommand.length() + 1];
	//strcpy_s(pcCommand, m_strCommand.length() + 1, m_strCommand.c_str());

	LCNC_INFO(lcnc::LogCode::Generic, "{}", "[Cutting]\n" + m_strCommand);
	if (!acsc_StopBuffer(m_hHandle, m_iProgramBufferIndex, ACSC_SYNCHRONOUS))
	{
		LogError();
		return false;
	}
	if (m_strCommand.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
		LCNC_ERR(lcnc::LogCode::InternalUnexpectedState,
		         "ACS command buffer exceeds SDK length limit");
		return false;
	}
	if (!acsc_LoadBuffer(m_hHandle, m_iProgramBufferIndex, m_strCommand.data(),
	                    static_cast<int>(m_strCommand.size()), ACSC_SYNCHRONOUS))
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
	// 切割段 ACC/JERK：优先取 Tool 工艺参数；若工艺参数无效（未配置、≤0、非有限值），
	// 退回每轴在 TOML 配置的 fAcc / fJerk（CreateMotor 时已读入 m_mapMotorValue[axis]）。
	const bool bToolAccValid  = std::isfinite(curTool.m_dLineAcc)  && curTool.m_dLineAcc  > 0.0;
	const bool bToolJerkValid = std::isfinite(curTool.m_dLineJerk) && curTool.m_dLineJerk > 0.0;

	for (Axis axis : m_vecMotors)
	{
		const auto& mv = m_mapMotorValue[axis];
		double dAcc  = bToolAccValid  ? curTool.m_dLineAcc  : mv.Acceleration;
		double dJerk = bToolJerkValid ? curTool.m_dLineJerk : mv.Jerk;
		if (!std::isfinite(dAcc)  || dAcc  <= 0.0) dAcc  = mv.Acceleration;
		if (!std::isfinite(dJerk) || dJerk <= 0.0) dJerk = mv.Jerk;

		string strIndex = boost::lexical_cast<string>(mv.AxisIndex);
		string strAcc   = boost::lexical_cast<string>(dAcc);
		string strJerk  = boost::lexical_cast<string>(dJerk);
		m_strCommand += "ACC"  + strIndex + " = " + strAcc  + ";";
		m_strCommand += "DEC"  + strIndex + " = " + strAcc  + ";";
		m_strCommand += "JERK" + strIndex + " = " + strJerk + ";\n";
	}

	if (!bToolAccValid || !bToolJerkValid)
		LCNC_WARN(lcnc::LogCode::Generic, "{}", "ACS SetCuttingAccJerk: tool m_dLineAcc/m_dLineJerk invalid, fell back to per-axis defaults.");

	return true;
}

bool ACSMotionControl::SetJumpAccJerk(const Tool& curTool)
{
	// 空程 ACC/JERK：同样回退到每轴默认。
	const bool bToolAccValid  = std::isfinite(curTool.m_dIdleXYAccDec) && curTool.m_dIdleXYAccDec > 0.0;
	const bool bToolJerkValid = std::isfinite(curTool.m_dIdleXYJerk)   && curTool.m_dIdleXYJerk   > 0.0;

	for (Axis axis : m_vecMotors)
	{
		const auto& mv = m_mapMotorValue[axis];
		double dAcc  = bToolAccValid  ? curTool.m_dIdleXYAccDec : mv.Acceleration;
		double dJerk = bToolJerkValid ? curTool.m_dIdleXYJerk   : mv.Jerk;
		if (!std::isfinite(dAcc)  || dAcc  <= 0.0) dAcc  = mv.Acceleration;
		if (!std::isfinite(dJerk) || dJerk <= 0.0) dJerk = mv.Jerk;

		string strIndex = boost::lexical_cast<string>(mv.AxisIndex);
		string strAcc   = boost::lexical_cast<string>(dAcc);
		string strJerk  = boost::lexical_cast<string>(dJerk);
		m_strCommand += "ACC"  + strIndex + " = " + strAcc  + ";";
		m_strCommand += "DEC"  + strIndex + " = " + strAcc  + ";";
		m_strCommand += "JERK" + strIndex + " = " + strJerk + ";\n";
	}

	if (!bToolAccValid || !bToolJerkValid)
		LCNC_WARN(lcnc::LogCode::Generic, "{}", "ACS SetJumpAccJerk: tool m_dIdleXYAccDec/m_dIdleXYJerk invalid, fell back to per-axis defaults.");

	return true;
}

void ACSMotionControl::ProLaserControl(bool bLaser, bool bPso, const Tool& curTool, bool bAOUTFlag)
{
	// 解析关键数字 IO，找不到时跳过对应分支（Fix #3）：避免拼出 "=1;" / "=0;" 之类残缺指令。
	DigitalIOData ioLaser, ioBlow;
	const bool bHasLaser = resolveDigital(m_mapDigitalOUT, DigitalOUT::Laser,
										ioLaser, "ProLaserControl Laser");
	const DigitalOUT blowKey = curTool.m_bBlow2 ? DigitalOUT::Blow2 : DigitalOUT::Blow;
	const char* blowLabel = curTool.m_bBlow2 ? "ProLaserControl Blow2" : "ProLaserControl Blow";
	const bool bHasBlow = resolveDigital(m_mapDigitalOUT, blowKey, ioBlow, blowLabel);

	const string strLaserNum  = ioLaser.strIndex;
	const string strAnalogNum = m_mapAnalogOUT.count(AnalogOUT::Laser)
		? m_mapAnalogOUT[AnalogOUT::Laser].strIndex : std::string();
	const string strEnd = "\n";

	if (bLaser)
	{
		string strLaserOnBWait = boost::lexical_cast<string>(m_dLaserOnBWait);
		string strLaserOnAWait = boost::lexical_cast<string>(m_dLaserOnAWait);
		string strBlowDelay = boost::lexical_cast<string>(m_dBlowDelay);

		if (bHasBlow)
		{
			string strValue = ioBlow.bInversion ? "0" : "1";
			m_strCommand += ioBlow.strIndex + "=" + strValue + ";";
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
				if (!strAnalogNum.empty())
				{
					string strAnalogValue = boost::lexical_cast<string>(curTool.m_dAnalogLaserValue / 2.0);
					m_strCommand += strAnalogNum + "=" + strAnalogValue + ";";
					m_strCommand += "TILL " + strAnalogNum + ";";
				}
				else
				{
					LCNC_WARN(lcnc::LogCode::Generic, "{}", "ACS ProLaserControl: Analog Laser OUT not configured, skipped.");
				}
			}
			else
			{
				if (bHasLaser)
				{
					string strLaserValue = ioLaser.bInversion ? "0" : "1";
					string strTill       = ioLaser.bInversion ? "TILL ^" : "TILL ";
					m_strCommand += strLaserNum + "=" + strLaserValue + ";";
					m_strCommand += strTill + strLaserNum + ";";
				}
			}
			m_strCommand += strEnd + "WAIT " + strLaserOnAWait + strEnd;
		}
	}
	else
	{
		if (bPso)
		{
			Axis eDirectionX = magic_enum::enum_cast<Axis>(curTool.m_strDirectionX).value();
			Axis eDirectionY = magic_enum::enum_cast<Axis>(curTool.m_strDirectionY).value();
			string strXIndex = boost::lexical_cast<string>(m_mapMotorValue[eDirectionX].AxisIndex);
			string strYIndex = boost::lexical_cast<string>(m_mapMotorValue[eDirectionY].AxisIndex);

			m_strCommand += "ENDS (" + strXIndex + ", " + strYIndex + ")" + strEnd;
			m_strCommand += "GO (" + strXIndex + ", " + strYIndex + ")\n";
			m_strCommand += "SPLIT (" + strXIndex + ", " + strYIndex + ")" + strEnd;
		}
		else if (curTool.m_bTroughFlag)
			return;

		string strLaserOffBWait = boost::lexical_cast<string>(m_dLaserOffBWait);
		string strLaserOffAWait = boost::lexical_cast<string>(m_dLaserOffAWait);
		m_strCommand += "WAIT " + strLaserOffBWait + strEnd;
		if (bAOUTFlag)
		{
			if (!strAnalogNum.empty())
			{
				m_strCommand += strAnalogNum + "=0;";
				m_strCommand += "TILL ^" + strAnalogNum + ";";
			}
		}
		else
		{
			if (bHasLaser)
			{
				string strLaserValue = ioLaser.bInversion ? "1" : "0";
				string strTill       = ioLaser.bInversion ? "TILL " : "TILL ^";
				m_strCommand += strLaserNum + "=" + strLaserValue + ";";
				m_strCommand += strTill + strLaserNum + ";";
			}
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
	default:	return 0;
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

// [P3 removed] ACSMotionControl::SetGO

// [P3 removed] ACSMotionControl::SetFPos

// [P3 removed] ACSMotionControl::GetFPos

// [P3 removed] ACSMotionControl::LoadApplication

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

// [P3 removed] ACSMotionControl::BeginACSSegmentSimple

// [P3 removed] ACSMotionControl::JumpToSimple

// [P3 removed] ACSMotionControl::GetCuttingCommand

// [P3 removed] ACSMotionControl::OffsetArcToSimple

// [P3 removed] ACSMotionControl::OffsetLineToSimple

// [P3 removed] ACSMotionControl::ProLaserControlSimple

// [P3 removed] ACSMotionControl::EndProgramCommandSimple

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

// [P3 removed] ACSMotionControl::SetMFLAGSValue

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

// [P3 removed] ACSMotionControl::CheckBuffer

// [P3 removed] ACSMotionControl::RunBuffer

bool ACSMotionControl::PauseBuffer(int iBufferIndex)
{
	if (!acsc_SuspendBuffer(m_hHandle, iBufferIndex, ACSC_SYNCHRONOUS))
	{
		LogError();
		return false;
	}
	return true;
}

// [P3 removed] ACSMotionControl::GetBufferState

// [P3 removed] ACSMotionControl::LoadCommandAndRunBuffer

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
// [P3 removed] ACSMotionControl::BeginACSSegmentForFlightCutting

// [P3 removed] ACSMotionControl::OffsetFlightLineTo

// [P3 removed] ACSMotionControl::OffsetFlightArcTo

// [P3 removed] ACSMotionControl::OffsetFlightArc2To

// [P3 removed] ACSMotionControl::EndProgramCommandForFlightCutting

// [P3 removed] ACSMotionControl::LoadAndCompileBuffer

// [P3 removed] ACSMotionControl::RunBufferForFlightCutting
#pragma endregion FlightCutting
