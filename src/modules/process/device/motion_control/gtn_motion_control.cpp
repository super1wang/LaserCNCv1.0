#include "gtn_motion_control.h"
#include <boost/lexical_cast.hpp>
#include <fstream>
//#include "bdaqctrl.h"
#include "core/logging/logger.h"
#include "modules/process/system/process_numeric_constants.h"
#include "modules/process/runtime/process_runtime_configuration.h"

#include "magic_enum.hpp"

using lcnc::process::AnalogOUT;
using lcnc::process::Axis;
using lcnc::process::DigitalOUT;
using std::string;
using std::vector;
using toml::table;

GTNMotionControl::GTNMotionControl(lcnc::process::ProcessSettingsService& settings,
	                                 lcnc::process::ProcessRuntimeConfiguration& runtimeConfiguration)
	: MotionControl(settings, runtimeConfiguration)
	, m_bConnectFlag(false)
	, m_strName("GTN")
	, m_iCore(1)
	, m_dPreX(0)
	, m_dPreY(0)
	, m_dPreZ(0)
	, m_dPreR1(0)
	, m_dPreR2(0)
	, m_bErrorOccurred(false)
	, m_dLaserOnBWait(0)
	, m_dLaserOnAWait(0)
	, m_dLaserOffBWait(0)
	, m_dLaserOffAWait(0)
	, m_dDiameter(1)
	
	, m_dBlowDelay(0)
	, m_bStop(false)
	, m_iWriteBuf(0)
	, m_inRunBuf(1)
	, m_bCrdStarted(false)
	
{
	m_vecMotors.reserve(8);  // 预分配8个元素空间
}

GTNMotionControl::~GTNMotionControl(void)
{
	ReleaseHomeParameters();
// 	if (!IsConnected())
// 	{
// 		return;
// 	}
// 	short sRtn;
// 	for (Axis axis : m_vecMotors)
// 	{
// 		string strIndex = boost::lexical_cast<string>(m_mapMotorValue[axis].AxisIndex);
// 		int iAxisIndex = m_mapMotorValue[axis].AxisIndex;
// 		//sRtn = GTN_LmtsOffEx(m_iCore, iAxisIndex, -1, 1);
// 		if (0 != GTN_LmtsOffEx(m_iCore, iAxisIndex, -1, 1))
// 		{
// 			LCNC_ERR(lcnc::LogCode::Generic, "{}", fmt::format("GTN Disconnect failed.retrun:{}", sRtn));
// 		}
// 	}
}

const string& GTNMotionControl::GetName() const
{
	return m_strName;
}

void GTNMotionControl::LogError(string strhandle, string command, string name, short error)
{
	string strLog = "GSN:" + strhandle+ command+ std::to_string(error);
	LCNC_ERR(lcnc::LogCode::Generic, "{}", fmt::format("GSN:{}-----{}----{}----- {}", strhandle, command, name, error));
	
}

long GTNMotionControl::AxisMaskByIndex(int iAxisIndex) const
{
	if (iAxisIndex < 1 || iAxisIndex > 8)
		return 0;
	return 1 << (iAxisIndex - 1);
}

long GTNMotionControl::AxisMask(Axis eAxis) const
{
	auto it = m_mapMotorValue.find(eAxis);
	if (it == m_mapMotorValue.end())
		return 0;
	return AxisMaskByIndex(it->second.AxisIndex);
}

void GTNMotionControl::ReleaseHomeParameters()
{
	for (auto& pair : m_mapMotorValue)
	{
		delete pair.second.pTHomePrm;
		pair.second.pTHomePrm = nullptr;
	}
}

bool GTNMotionControl::ClearGSNAlarm(Axis eAxis)
{
	short sRtn;
	int AxisIndex = m_mapMotorValue[eAxis].AxisIndex;
	THomePrm tHomePrm;
	sRtn = GTN_ClearAlarm(m_iCore, AxisIndex, 1, 1);
	if (0!= sRtn)
	{
		LogError("ClearGSNAlarm", "GTN_ClearAlarm", magic_enum::enum_name(eAxis).data(), sRtn);
		return false;
	}
	sRtn = GTN_ClrSts(m_iCore, AxisIndex, 1);
	if (0 != sRtn)
	{
		LogError("ClearGSNAlarm", "GTN_ClrSts", magic_enum::enum_name(eAxis).data(), sRtn);
		return false;
	}
	return true;
}

void GTNMotionControl::CreateMotor(Axis eAxis, const table& tAxis)
{
	m_mapMotorValue[eAxis].Name			= magic_enum::enum_name(eAxis).data();
	m_mapMotorValue[eAxis].AxisIndex	= tAxis.at("iIndex")		.as_integer();
	m_mapMotorValue[eAxis].Resolution	= tAxis.at("fResolution")	.as_floating();
	m_mapMotorValue[eAxis].Rotary		= tAxis.at("bRotation")		.as_boolean();
	m_mapMotorValue[eAxis].Velocity		= tAxis.at("fVel")			.as_floating();
	m_mapMotorValue[eAxis].Acceleration	= tAxis.at("fAcc")			.as_floating();
	m_mapMotorValue[eAxis].Deceleration	= tAxis.count("fDec") ? tAxis.at("fDec").as_floating() : tAxis.at("fAcc").as_floating();
	m_mapMotorValue[eAxis].SmoothTime	= tAxis.at("fJerk")			.as_floating();
	m_mapMotorValue[eAxis].NegLimit		= tAxis.at("fLeftLimit")	.as_floating();
	m_mapMotorValue[eAxis].PosLimit		= tAxis.at("fRightLimit")	.as_floating();

	if (tAxis.count("Home") && tAxis.at("Home").is_table())
		SetAxisHomePrm(eAxis, tAxis.at("Home").as_table());

	m_vecMotors.emplace_back(eAxis);
}

bool GTNMotionControl::Connect()
{
	short sRtn;
	if (m_bConnectFlag)
	{
		return true;
	}
	auto failConnect = [this](const string& command, short error) -> bool {
		LogError("Connect", command, "", error);
		GTN_Close();
		m_bConnectFlag = false;
		return false;
	};

	sRtn = GTN_Open(5, 2);//连接运动控制器
	if (sRtn != 0)
	{
		LogError("Connect", "GTN_Open", "", sRtn);
		m_bConnectFlag = false;
		return false;
	}
	sRtn = GTN_LoadConfig(m_iCore, const_cast < char*>("gtn_core1.cfg"));
	if (0 != sRtn)
	{
		return failConnect("GTN_LoadConfig", sRtn);
	}
	// 只对已配置的轴操作，避免对超出控制卡轴数的轴（如4轴卡的轴5-8）发送指令导致连接失败
	for (Axis axis : m_vecMotors)
	{
		int i = m_mapMotorValue[axis].AxisIndex;
		sRtn = GTN_ClrSts(m_iCore, i, 1);					//清除单轴状态
		if (sRtn != 0)
			return failConnect("GTN_ClrSts", sRtn);
		sRtn = GTN_SetStopDec(m_iCore, i, 1, 100);			//设置平滑停止减速度和急停减速度pulse/ms2
		if (sRtn != 0)
			return failConnect("GTN_SetStopDec", sRtn);
		sRtn = GTN_PrfTrap(m_iCore, i);						//设定点位运动
		if (sRtn != 0)
			return failConnect("GTN_PrfTrap", sRtn);
		sRtn = GTN_SetAxisMotionSmooth(m_iCore, i, 1, 0.5);
		if (sRtn != 0)
			return failConnect("GTN_SetAxisMotionSmooth", sRtn);
		sRtn = GTN_LmtsOnEx(m_iCore, i, -1, 1);				//控制轴限位有效
		if (sRtn != 0)
			return failConnect("GTN_LmtsOnEx", sRtn);
	}
	sRtn = GTN_ExtModuleInit(m_iCore);	
	if (sRtn != 0)
		return failConnect("GTN_ExtModuleInit", sRtn);

	m_bConnectFlag = true;
	return true;
}

bool GTNMotionControl::Disconnect()
{
	if (!m_bConnectFlag)
		return true;

	bool success = true;
	short sRtn;
	// 关闭激光
	sRtn = GTN_SetDoBit(m_iCore, MC_GPO, 4, 1);
	if (0!= sRtn)
	{
		LogError("Disconnect", "GTN_SetDoBit", "", sRtn);
		success = false;
	}
	if (!StopMotion())
		success = false;
	for (Axis axis : m_vecMotors)
	{
		string strIndex = boost::lexical_cast<string>(m_mapMotorValue[axis].AxisIndex);
		int iAxisIndex = m_mapMotorValue[axis].AxisIndex;
		sRtn = GTN_LmtsOffEx(m_iCore, iAxisIndex, -1, 1);
		if (0 != sRtn)
		{
			LogError("Disconnect", "GTN_LmtsOffEx:" + strIndex, "", sRtn);
			success = false;
		}
	}
	sRtn = GTN_Close();
	if (0 != sRtn)
	{
		LogError("Disconnect", "GTN_Close", "", sRtn);
		return false;
	}
	
	m_bConnectFlag = false;
	return success;
}

bool GTNMotionControl::IsConnected()
{
	return m_bConnectFlag;
}

bool GTNMotionControl::Reboot()
{
	if (!Disconnect())
	{
		LCNC_ERR(lcnc::LogCode::Generic, "{}", "GTN Reboot disconnect failed, try reconnect anyway.");
	}
	Sleep(500);
	m_bConnectFlag = false;
	return Connect();
}

bool GTNMotionControl::Home()
{
	// 顺序 Z -> Y -> X -> B -> C -> A。Axis 枚举只包含标准六轴，
	// 不再引用已移除的 X1/Y1/A1 扩展轴。
	int arr[] = { 2, 1, 0, 4, 5, 3 };

	for (const int axisValue : arr)
	{
		Axis eAxis = static_cast<Axis>(axisValue);
		if (!m_runtimeConfiguration.isAxisEnabled(eAxis))
			continue;

		if (m_bStop)
			return false;

		if (!Home(eAxis))
			return false;
		Sleep(10);

		if (m_bStop)
			return false;
	}

	return true;
}

bool GTNMotionControl::Home(Axis eAxis)
{
	short sRtn;
	int AxisIndex = m_mapMotorValue[eAxis].AxisIndex;
	if (!Enable(eAxis))
		return false;
	Sleep(2000);
	if (!ClearGSNAlarm(eAxis)) return false;

	sRtn = GTN_LmtsOffEx(m_iCore, AxisIndex, -1, 1);	//控制轴限位失效
	if (sRtn)
		return LogError("Home", "GTN_LmtsOffEx", magic_enum::enum_name(eAxis).data(), sRtn), false;
	// 限位已关闭，以下所有退出路径必须经过 GTN_LmtsOnEx 恢复限位
	bool bRet = false;
	try
	{
		do {
			sRtn = GTN_ZeroPos(m_iCore, AxisIndex, 1);
			if (sRtn) { LogError("Home", "GTN_ZeroPos", magic_enum::enum_name(eAxis).data(), sRtn); break; }

			THomePrm tHomePrm;
			if (m_mapMotorValue[eAxis].pTHomePrm)
				tHomePrm = *m_mapMotorValue[eAxis].pTHomePrm;	// 使用配置文件中读取的回零参数
			else
			{
				sRtn = GTN_GetHomePrm(m_iCore, AxisIndex, &tHomePrm);	// 退回读控制器默认参数
				if (sRtn != 0) { LogError("Home", "GTN_GetHomePrm", magic_enum::enum_name(eAxis).data(), sRtn); break; }
			}

			sRtn = GTN_GoHome(m_iCore, AxisIndex, &tHomePrm);	//启动Smart Home回原点
			if (sRtn != 0) { LogError("Home", "GTN_GoHome", magic_enum::enum_name(eAxis).data(), sRtn); break; }

			DWORD dwTimeout = GetTickCount64() + 120000;	// 120秒超时
			THomeStatus tHomeSts{};
			bool bAborted = false;
			do
			{
				if (m_bStop)
				{
					// 中文翻译：回原点被用户停止
					LogError("Home", "Return to origin stopped by user", magic_enum::enum_name(eAxis).data(), -1);
					bAborted = true; break;
				}
				if (GetTickCount64() > dwTimeout)
				{
					// 中文翻译：回原点超时
					LogError("Home", "Return to origin timeout", magic_enum::enum_name(eAxis).data(), -2);
					bAborted = true; break;
				}
				sRtn = GTN_GetHomeStatus(m_iCore, AxisIndex, &tHomeSts);	//获取回原点状态
				if (sRtn != 0)
				{
					LogError("Home", "GTN_GetHomeStatus", magic_enum::enum_name(eAxis).data(), sRtn);
					bAborted = true;
					break;
				}
				Sleep(20);
			} while (tHomeSts.run);	// 等待搜索原点停止
			if (bAborted) break;

			// 中文翻译：回原点报错
			if (tHomeSts.error) { LogError("Home", "Return to origin and report error", magic_enum::enum_name(eAxis).data(), tHomeSts.error); break; }

			sRtn = GTN_ZeroPos(m_iCore, AxisIndex, 1);
			if (sRtn != 0) { LogError("Home", "GTN_ZeroPos1", magic_enum::enum_name(eAxis).data(), sRtn); break; }

			bRet = true;
		} while (false);
	}
	catch (...)
	{
		// 中文翻译：回原点异常
		LogError("Home", "Return to origin exception", magic_enum::enum_name(eAxis).data(), -999);
		bRet = false;
	}

	sRtn = GTN_LmtsOnEx(m_iCore, AxisIndex, -1, 1);	//控制轴限位有效（无论成功/失败均须恢复）
	if (sRtn != 0)
		return LogError("Home", "GTN_LmtsOnEx", magic_enum::enum_name(eAxis).data(), sRtn), false;

	if (!IsEnabled(eAxis))
		return false;
	return bRet;
}

bool GTNMotionControl::IsHomed()
{
	for (Axis axis : m_vecMotors)
	{
		if (!IsHomed(axis))
			return false;
	}
	return true;
}

bool GTNMotionControl::IsHomed(Axis eAxis)
{
	if (!IsConnected())
	{
		return false;
	}
	int HomeState = 0;
	short sRtn;
	THomeStatus tHomeSts{};
	sRtn = GTN_GetHomeStatus(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &tHomeSts);//获取回原点状态 
	if (sRtn !=0 )
	{
		LogError("IsHomed", "GTN_GetHomeStatus", magic_enum::enum_name(eAxis).data(), sRtn);
		return false;
	}

	if (tHomeSts.stage != 100)
		return false;
	if (tHomeSts.error)
		return false;
	return true;
}

// [P3 removed] GTNMotionControl::IsHomeBufferRunning

bool GTNMotionControl::Enable()
{
	for (Axis axis : m_vecMotors)
	{
		if (!Enable(axis))
			return false;
	}
	return true;
}

bool GTNMotionControl::Enable(Axis eAxis)
{
	short sRtn = 0;
	if (IsEnabled(eAxis))
		return true;

	int AxisIndex = m_mapMotorValue[eAxis].AxisIndex;
	long mask = AxisMaskByIndex(AxisIndex);
	if (!mask)
		return LogError("Enable", "AxisMaskByIndex", magic_enum::enum_name(eAxis).data(), -1), false;
	sRtn = GTN_AxisOn(m_iCore, AxisIndex);
	if (sRtn != 0)
		return LogError("Enable", "GTN_AxisOn", magic_enum::enum_name(eAxis).data(), sRtn), false;
	double APos;
	sRtn = GTN_GetEncPos(m_iCore, AxisIndex, &APos);//实际位置
	if (0 != sRtn)
		return LogError("Enable", "GTN_GetEncPos", magic_enum::enum_name(eAxis).data(), sRtn), false;
	sRtn = GTN_SetPrfPos(m_iCore, AxisIndex, APos);//规划位置GTN_SetPrfPosEx
	if (0 != sRtn)
		return LogError("Enable", "GTN_SetPrfPos", magic_enum::enum_name(eAxis).data(), sRtn), false;
	sRtn = GTN_SynchAxisPos(m_iCore, mask);
	if (sRtn != 0)
		return LogError("Enable", "GTN_SynchAxisPos", magic_enum::enum_name(eAxis).data(), sRtn) ,false;
	return true;
}

bool GTNMotionControl::Disable()
{
	for (Axis axis : m_vecMotors)
	{
		if (!Disable(axis))
			return false;
	}
	return true;
}

bool GTNMotionControl::Disable(Axis eAxis)
{
	if (!IsEnabled(eAxis))
		return true;

	short sRtn = 0;
	int AxisIndex = m_mapMotorValue[eAxis].AxisIndex;
	sRtn = GTN_AxisOff(m_iCore, AxisIndex);
	if (sRtn != 0)
		return LogError("Disable", "GTN_AxisOff", magic_enum::enum_name(eAxis).data(), sRtn),false;
	return true;
}

bool GTNMotionControl::IsEnabled()
{
	for (Axis axis : m_vecMotors)
	{
		if (!IsEnabled(axis))
			return false;
	}
	return true;
}

bool GTNMotionControl::IsEnabled(Axis eAxis)
{
	if (!IsConnected())
	{
		return false;
	}
	long lAxisStatus;
	short sRtn;
	sRtn = GTN_GetSts(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &lAxisStatus);
	if (sRtn != 0)
		return LogError("IsEnabled", "GTN_GetSts", magic_enum::enum_name(eAxis).data(), sRtn), false;
	if (!(lAxisStatus & 0x200))
	{
		return false;
	}
	else
	{
		return true;
	}

}

bool GTNMotionControl::SetAxisEnable(Axis eAxis, bool bEnable)
{
	if (bEnable)
		return Enable(eAxis);
	else
		return Disable(eAxis);
}

bool GTNMotionControl::Jog(Axis eAxis, bool bDirection, double dVel)
{
	short sRtn;
	TJogPrm tJogPrm;
	sRtn = GTN_PrfJog(m_iCore,m_mapMotorValue[eAxis].AxisIndex);
	if (0 != sRtn)
		return LogError("Jog", "GTN_PrfJog", magic_enum::enum_name(eAxis).data(), sRtn),false;
	double dNewAcc, dNewDec;
	MillimeterToPulse(eAxis, m_mapMotorValue[eAxis].Acceleration / 1000000.00, dNewAcc);
	MillimeterToPulse(eAxis, m_mapMotorValue[eAxis].Deceleration / 1000000.00, dNewDec);
	tJogPrm.acc = dNewAcc;
	tJogPrm.dec = dNewDec;
	tJogPrm.smooth = m_mapMotorValue[eAxis].SmoothTime;
	sRtn = GTN_SetJogPrm(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &tJogPrm);
	if (0 != sRtn)
		return LogError("Jog", "GTN_SetJogPrm", magic_enum::enum_name(eAxis).data(), sRtn), false;
	double dNewVel;
	MillimeterToPulse(eAxis, dVel / 1000.0, dNewVel);
	double directedVel = bDirection ? dNewVel : -dNewVel;
	sRtn = GTN_SetVel(m_iCore, m_mapMotorValue[eAxis].AxisIndex, directedVel);
	if (0 != sRtn)
		return LogError("Jog", "GTN_SetVel", magic_enum::enum_name(eAxis).data(), sRtn), false;
	long mask = AxisMask(eAxis);
	if (!mask)
		return LogError("Jog", "AxisMask", magic_enum::enum_name(eAxis).data(), -1), false;
	sRtn = GTN_Update(m_iCore, mask);
	if (0 != sRtn)
		return LogError("Jog", "GTN_Update", magic_enum::enum_name(eAxis).data(), sRtn), false;

	return true;
}

bool GTNMotionControl::MoveRelative(Axis eAxis, double dPos, double dVel)
{
	short sRtn;
	double RelativePos;
	double APos;
	double Velocity = m_mapMotorValue[eAxis].Velocity;
	int iAxis = m_mapMotorValue[eAxis].AxisIndex;
	sRtn = GTN_PrfTrap(m_iCore, iAxis);//设定点位运动
	if (0 != sRtn)
		return LogError("MoveRelative", "GTN_PrfTrap", magic_enum::enum_name(eAxis).data(), sRtn), false;

	double dNewVel, dNewAcc, dNewDec;
	
	if (!SetAxisVelAccDecJerk(eAxis, m_mapMotorValue[eAxis].Velocity, m_mapMotorValue[eAxis].Acceleration, m_mapMotorValue[eAxis].Deceleration, m_mapMotorValue[eAxis].SmoothTime))
		return false;

	if (!MillimeterToPulse(eAxis, dVel / 1000, dNewVel))
		return false;
	sRtn = GTN_SetVel(m_iCore, iAxis, dNewVel);
	if (0 != sRtn)
		return LogError("MoveRelative", "GTN_SetVel", magic_enum::enum_name(eAxis).data(), sRtn), false;
	sRtn = GTN_GetPrfPos(m_iCore, iAxis, &APos);//规划位置
	if (0 != sRtn)
		return LogError("MoveRelative", "GTN_GetPrfPos", magic_enum::enum_name(eAxis).data(), sRtn), false;
	MillimeterToPulse(eAxis, dPos, RelativePos);
	sRtn = GTN_SetPos(m_iCore, iAxis, APos + RelativePos);
	if (0 != sRtn)
		return LogError("MoveRelative", "GTN_SetPos", magic_enum::enum_name(eAxis).data(), sRtn), false;
	long mask = AxisMaskByIndex(iAxis);
	if (!mask)
		return LogError("MoveRelative", "AxisMaskByIndex", magic_enum::enum_name(eAxis).data(), -1), false;
	sRtn = GTN_Update(m_iCore, mask);
	if (0 != sRtn)
		return LogError("MoveRelative", "GTN_Update", magic_enum::enum_name(eAxis).data(), sRtn), false;
	return true;
	
}

bool GTNMotionControl::MoveAbsolute(Axis eAxis, double dPos, double dVel)
{
	int iAxis = m_mapMotorValue[eAxis].AxisIndex;
	short sRtn;
	sRtn = GTN_PrfTrap(m_iCore, iAxis);
	if (0 != sRtn)
		return LogError("MoveAbsolute", "GTN_PrfTrap", magic_enum::enum_name(eAxis).data(), sRtn), false;
	double cachedVel = m_mapMotorValue[eAxis].Velocity;
	double vel = (dVel > 0) ? dVel : cachedVel;
	if (!SetAxisVelAccDecJerk(eAxis, vel, m_mapMotorValue[eAxis].Acceleration, m_mapMotorValue[eAxis].Deceleration, m_mapMotorValue[eAxis].SmoothTime))
		return false;
	if (dVel > 0)
		m_mapMotorValue[eAxis].Velocity = cachedVel;	// 恢复缓存速度，避免临时速度污染轴配置
	double dNewPos;
	MillimeterToPulse(eAxis, dPos, dNewPos);
	sRtn = GTN_SetPos(m_iCore, iAxis, (long)dNewPos);
	if (0 != sRtn)
		return LogError("MoveAbsolute", "GTN_SetPos", magic_enum::enum_name(eAxis).data(), sRtn), false;
	long mask = AxisMaskByIndex(iAxis);
	if (!mask)
		return LogError("MoveAbsolute", "AxisMaskByIndex", magic_enum::enum_name(eAxis).data(), -1), false;
	sRtn = GTN_Update(m_iCore, mask);
	if (0 != sRtn)
		return LogError("MoveAbsolute", "GTN_Update", magic_enum::enum_name(eAxis).data(), sRtn), false;
	return true;
}

bool GTNMotionControl::MoveMRelative(vector<Axis> vAxis, vector<double> dPos, double dVel)
{
	
	return true;
}

bool GTNMotionControl::MoveMAbsolute(vector<Axis> vAxis, vector<double> dPos, double dVel)
{
	
	return true;
}

bool GTNMotionControl::StopMotion()
{
	short sRtn, run;
	long allMask = 0;
	for (Axis axis : m_vecMotors)
		allMask |= AxisMask(axis);
	sRtn = GTN_Stop(m_iCore, allMask, 0x0);
	if (sRtn != 0)
	{
		LogError("StopMotion", "GTN_Stop","", sRtn);
		return false;
	}
	do 
	{
		Sleep(100);
	} while (IsAxisMoving());
	sRtn = GTN_CrdClear(m_iCore, 1, m_iWriteBuf);
	if (sRtn != 0)
	{
		LogError("StopMotion", "GTN_CrdClear","", sRtn);
		return false;
	}
	m_bStop = false;
	return true;
}

bool GTNMotionControl::StopMotion(Axis eAxis)
{
	short sRtn;
	long mask = AxisMask(eAxis);
	if (!mask)
		return LogError("StopMotion", "AxisMask", magic_enum::enum_name(eAxis).data(), -1), false;
	sRtn = GTN_Stop(m_iCore, mask, mask);//急停
	if (sRtn != 0)
		return LogError("StopMotion", "GTN_Stop", magic_enum::enum_name(eAxis).data(), sRtn),false;
	do
	{
		Sleep(100);
	} while (IsAxisMoving(eAxis));
	sRtn = GTN_CrdClear(m_iCore, 1, m_iWriteBuf);
	if (sRtn != 0)
		return LogError("StopMotion", "GTN_CrdClear", magic_enum::enum_name(eAxis).data(), sRtn), false;
	return true;
}

bool GTNMotionControl::IsAxisMoving()
{
	for (Axis axis : m_vecMotors)
	{
		if (IsAxisMoving(axis))
			return true;
	}
	return false;
}

bool GTNMotionControl::IsAxisMoving(Axis eAxis)
{
	long State;
	short sRtn;
	sRtn = GTN_GetSts(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &State);

	if (sRtn != 0)
		return LogError("IsAxisMoving", "GTN_GetSts", magic_enum::enum_name(eAxis).data(), sRtn), false;
	if (State & 0x400)
		return true;
	else
		return false;
}

bool GTNMotionControl::GetActualPos(Axis eAxis, double& dAPos)
{
	if (!IsConnected())
	{
		dAPos = -1;
		return false;
	}
	short sRtn;
	double APos;
	int iAxis = m_mapMotorValue[eAxis].AxisIndex;
	sRtn = GTN_GetPrfPos(m_iCore, iAxis, &APos);//规划位置
	if (sRtn != 0)
	{
		dAPos = -1;
		//LogError("GetActualPos", "GTN_GetPrfPos", magic_enum::enum_name(eAxis).data(), sRtn);
		return false;
	}
	PulseToMillimeter(eAxis, APos, dAPos);
	return true;
}

bool GTNMotionControl::GetFeedbackPos(Axis eAxis, double& dFPos)
{
	if (!IsConnected())
	{
		dFPos = -1;
		return false;
	}
	short sRtn;
	double APos;
	int iAxis = m_mapMotorValue[eAxis].AxisIndex;
	sRtn = GTN_GetEncPos(m_iCore, iAxis, &APos); // 编码器反馈位置
	if (sRtn != 0)
	{
		dFPos = -1;
		return false;
	}
	PulseToMillimeter(eAxis, APos, dFPos);
	return true;
}

bool GTNMotionControl::SetFPosition(Axis eAxis, double dPos)
{
	if (!IsConnected())
		return false;
	int iAxis = m_mapMotorValue[eAxis].AxisIndex;
	long mask = AxisMaskByIndex(iAxis);
	if (!mask)
		return LogError("SetFPosition", "AxisMaskByIndex", magic_enum::enum_name(eAxis).data(), -1), false;
	double dPulse = 0.0;
	if (!MillimeterToPulse(eAxis, dPos, dPulse))
		return false;
	// 对应 ACS setfpos：同步重写编码器反馈位置与规划位置，
	// 二者不一致会触发跟随误差报警，故必须同时设置并同步轴位置。
	short sRtn = GTN_SetEncPos(m_iCore, iAxis, (long)dPulse);
	if (0 != sRtn)
		return LogError("SetFPosition", "GTN_SetEncPos", magic_enum::enum_name(eAxis).data(), sRtn), false;
	sRtn = GTN_SetPrfPos(m_iCore, iAxis, (long)dPulse);
	if (0 != sRtn)
		return LogError("SetFPosition", "GTN_SetPrfPos", magic_enum::enum_name(eAxis).data(), sRtn), false;
	sRtn = GTN_SynchAxisPos(m_iCore, mask);
	if (0 != sRtn)
		return LogError("SetFPosition", "GTN_SynchAxisPos", magic_enum::enum_name(eAxis).data(), sRtn), false;
	return true;
}

void GTNMotionControl::SetAxisHomePrm(Axis eAxis, const toml::table& tableHome)
{
	if (!m_mapMotorValue[eAxis].pTHomePrm)
		m_mapMotorValue[eAxis].pTHomePrm = new THomePrm();
	THomePrm* p = m_mapMotorValue[eAxis].pTHomePrm;
	memset(p, 0, sizeof(THomePrm));
	p->triggerIndex = -1;	// 默认使用本轴触发器

	if (tableHome.count("iMode"))
		p->mode				= (short)tableHome.at("iMode").as_integer();
	if (tableHome.count("iMoveDir"))
		p->moveDir			= (short)tableHome.at("iMoveDir").as_integer();
	if (tableHome.count("iIndexDir"))
		p->indexDir			= (short)tableHome.at("iIndexDir").as_integer();
	if (tableHome.count("iEdge"))
		p->edge				= (short)tableHome.at("iEdge").as_integer();
	if (tableHome.count("iTriggerIndex"))
		p->triggerIndex		= (short)tableHome.at("iTriggerIndex").as_integer();
	if (tableHome.count("fVelHigh"))
		p->velHigh			= tableHome.at("fVelHigh").as_floating();
	if (tableHome.count("fVelLow"))
		p->velLow			= tableHome.at("fVelLow").as_floating();
	if (tableHome.count("fAcc"))
		p->acc				= tableHome.at("fAcc").as_floating();
	if (tableHome.count("fDec"))
		p->dec				= tableHome.at("fDec").as_floating();
	if (tableHome.count("iSmoothTime"))
		p->smoothTime		= (short)tableHome.at("iSmoothTime").as_integer();
	if (tableHome.count("iHomeOffset"))
		p->homeOffset		= (long)tableHome.at("iHomeOffset").as_integer();
	if (tableHome.count("iSearchHomeDistance"))
		p->searchHomeDistance	= (long)tableHome.at("iSearchHomeDistance").as_integer();
	if (tableHome.count("iSearchIndexDistance"))
		p->searchIndexDistance	= (long)tableHome.at("iSearchIndexDistance").as_integer();
	if (tableHome.count("iEscapeStep"))
		p->escapeStep		= (long)tableHome.at("iEscapeStep").as_integer();
}

bool GTNMotionControl::GetAxisHomePrm(Axis eAxis, toml::table& tableHome)
{
	if (!m_mapMotorValue.count(eAxis))
		return false;

	THomePrm tHomePrm;
	memset(&tHomePrm, 0, sizeof(THomePrm));

	if (IsConnected())
	{
		short sRtn = GTN_GetHomePrm(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &tHomePrm);
		if (sRtn != 0)
		{
			LogError("GetAxisHomePrm", "GTN_GetHomePrm", magic_enum::enum_name(eAxis).data(), sRtn);
			return false;
		}
		// 同步更新内存中存储的副本
		if (!m_mapMotorValue[eAxis].pTHomePrm)
			m_mapMotorValue[eAxis].pTHomePrm = new THomePrm();
		*m_mapMotorValue[eAxis].pTHomePrm = tHomePrm;
	}
	else if (m_mapMotorValue[eAxis].pTHomePrm)
	{
		tHomePrm = *m_mapMotorValue[eAxis].pTHomePrm;
	}
	else
	{
		return false;	// 未连接且无缓存参数
	}

	tableHome["iMode"]					= (int)tHomePrm.mode;
	tableHome["iMoveDir"]				= (int)tHomePrm.moveDir;
	tableHome["iIndexDir"]				= (int)tHomePrm.indexDir;
	tableHome["iEdge"]					= (int)tHomePrm.edge;
	tableHome["iTriggerIndex"]			= (int)tHomePrm.triggerIndex;
	tableHome["fVelHigh"]				= tHomePrm.velHigh;
	tableHome["fVelLow"]				= tHomePrm.velLow;
	tableHome["fAcc"]					= tHomePrm.acc;
	tableHome["fDec"]					= tHomePrm.dec;
	tableHome["iSmoothTime"]			= (int)tHomePrm.smoothTime;
	tableHome["iHomeOffset"]			= (int)tHomePrm.homeOffset;
	tableHome["iSearchHomeDistance"]	= (int)tHomePrm.searchHomeDistance;
	tableHome["iSearchIndexDistance"]	= (int)tHomePrm.searchIndexDistance;
	tableHome["iEscapeStep"]			= (int)tHomePrm.escapeStep;
	return true;
}

bool GTNMotionControl::SetAxisIndex(Axis eAxis, int iIndex)
{
	m_mapMotorValue[eAxis].AxisIndex = iIndex;
	return true;
}

bool GTNMotionControl::SetAxisIsRotary(Axis eAxis, bool bRotary)
{
	m_mapMotorValue[eAxis].Rotary = bRotary;
	return true;
}

bool GTNMotionControl::SetAxisResolution(Axis eAxis, int iResolution)
{
	m_mapMotorValue[eAxis].Resolution = iResolution;
	return true;
}

bool GTNMotionControl::SetAxisTubeDiamater(Axis eAxis, double dDiamater)
{
	if (dDiamater <= 0 || !m_mapMotorValue[eAxis].Rotary)
	{
		return false;
	}

	int iResolutionRatio = m_mapMotorValue[eAxis].Resolution;
	
	return false;
}

bool GTNMotionControl::SetAxisVel(Axis eAxis, double dVel)
{
	if (!IsConnected())
		return false;
	
	double dNewVel;
	MillimeterToPulse(eAxis, dVel/1000, dNewVel);
	short sRtn = GTN_SetVel(m_iCore, m_mapMotorValue[eAxis].AxisIndex, dNewVel);
	if (sRtn != 0)
	{
		return LogError("SetAxisVel", "GTN_SetVel", magic_enum::enum_name(eAxis).data(), sRtn),false;
	}
	m_mapMotorValue[eAxis].Velocity = dVel;
	return true;
}

bool GTNMotionControl::SetAxisAcc(Axis eAxis, double dAcc)
{
	if (!IsConnected())
		return false;
	short sRtn;
	double dNewAcc, dNewDec;

	// 将 AXIS 轴设为点位模式
	sRtn = GTN_PrfTrap(m_iCore, m_mapMotorValue[eAxis].AxisIndex);
	if (0 != sRtn)
	{
		return LogError("SetAxisAcc", "GTN_PrfTrap", magic_enum::enum_name(eAxis).data(), sRtn), false;
	}
	TTrapPrm trap;
	MillimeterToPulse(eAxis, dAcc / 1000000.00, dNewAcc);
	sRtn = GTN_GetTrapPrm(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &trap);
	if (0 != sRtn)
		return LogError("SetAxisAcc", "GTN_GetTrapPrm", magic_enum::enum_name(eAxis).data(), sRtn), false;
	trap.acc = dNewAcc;
	sRtn = GTN_SetTrapPrm(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &trap);
	if (0!= sRtn)
	{
		return LogError("SetAxisAcc", "GTN_SetTrapPrm", magic_enum::enum_name(eAxis).data(), sRtn),false;
	}
	m_mapMotorValue[eAxis].Acceleration = dAcc;
	return true;
}

bool GTNMotionControl::SetAxisDec(Axis eAxis, double dDec)
{
	if (!IsConnected())
		return false;
	short sRtn;
	double dNewAcc,dNewDec;
	TTrapPrm trap;
	MillimeterToPulse(eAxis, dDec / 1000000.00, dNewDec);
	sRtn = GTN_GetTrapPrm(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &trap);
	if (0 != sRtn)
		return LogError("SetAxisDec", "GTN_GetTrapPrm", magic_enum::enum_name(eAxis).data(), sRtn), false;
	trap.dec = dNewDec;
	sRtn = GTN_SetTrapPrm(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &trap);
	if (0 != sRtn)
	{
		return LogError("SetAxisDec", "GTN_SetTrapPrm", magic_enum::enum_name(eAxis).data(), sRtn), false;
	}
	m_mapMotorValue[eAxis].Deceleration = dDec;
	return true;
}

bool GTNMotionControl::SetAxisJerk(Axis eAxis, double dSmoothTime)
{
	if (!IsConnected())
		return false;
	short sRtn;
	TTrapPrm trap;
	sRtn = GTN_GetTrapPrm(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &trap);
	if (0 != sRtn)
		return LogError("SetAxisJerk", "GTN_GetTrapPrm", magic_enum::enum_name(eAxis).data(), sRtn), false;
	trap.smoothTime = dSmoothTime;
	sRtn = GTN_SetTrapPrm(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &trap);
	if (0 != sRtn)
	{
		return LogError("SetAxisJerk", "GTN_SetTrapPrm", magic_enum::enum_name(eAxis).data(), sRtn), false;
	}
	m_mapMotorValue[eAxis].SmoothTime = dSmoothTime;
	return true;
}

bool GTNMotionControl::SetAxisNegLimit(Axis eAxis, double dNegLimit)
{
	if (!IsConnected()/* || !IsHomed(eAxis)*/)
	{
		return false;
	}
	short sRtn;
	double dNewNegLimit;
	MillimeterToPulse(eAxis, dNegLimit, dNewNegLimit);
	long lPosLimit, lNegLimit;
	sRtn = GTN_GetSoftLimit(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &lPosLimit, &lNegLimit);
	if (0 != sRtn)
	{
		LogError("SetAxisNegLimit", "GTN_GetSoftLimit", magic_enum::enum_name(eAxis).data(), sRtn);
		return false;
	}
	lNegLimit = (long)dNewNegLimit;
	sRtn = GTN_SetSoftLimit(m_iCore, m_mapMotorValue[eAxis].AxisIndex, lPosLimit, lNegLimit);
	if (0 != sRtn)
	{
		LogError("SetAxisNegLimit", "GTN_SetSoftLimit", magic_enum::enum_name(eAxis).data(), sRtn);
		return false;
	}
	m_mapMotorValue[eAxis].NegLimit = dNegLimit;
	return true;
}

bool GTNMotionControl::SetAxisPosLimit(Axis eAxis, double dPosLimit)
{
	if (!IsConnected()/* || !IsHomed(eAxis)*/)
	{
		return false;
	}
	short sRtn;
	double dNewPosLimit;
	MillimeterToPulse(eAxis, dPosLimit, dNewPosLimit);
	long lPosLimit, lNegLimit;
	sRtn = GTN_GetSoftLimit(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &lPosLimit, &lNegLimit);
	if (0 != sRtn)
	{
		LogError("SetAxisNegLimit", "GTN_GetSoftLimit", magic_enum::enum_name(eAxis).data(), sRtn);
		return false;
	}
	lPosLimit = (long)dNewPosLimit;
	sRtn = GTN_SetSoftLimit(m_iCore, m_mapMotorValue[eAxis].AxisIndex, lPosLimit, lNegLimit);
	if (0 != sRtn)
	{
		LogError("SetAxisNegLimit", "GTN_SetSoftLimit", magic_enum::enum_name(eAxis).data(), sRtn);
		return false;
	}
	m_mapMotorValue[eAxis].PosLimit = dPosLimit;
	return true;
}

bool GTNMotionControl::SetAxisVelAccDecJerk(Axis eAxis, double dVel, double dAcc, double dDec, double dJerk)
{
	if (!SetAxisVel(eAxis, dVel))
		return false;

	short sRtn = GTN_PrfTrap(m_iCore, m_mapMotorValue[eAxis].AxisIndex);
	if (0 != sRtn)
	{
		return LogError("SetAxisAcc", "GTN_PrfTrap", magic_enum::enum_name(eAxis).data(), sRtn), false;
	}

	TTrapPrm trap;
	sRtn = GTN_GetTrapPrm(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &trap);
	if (0 != sRtn)
		return LogError("SetAxisAcc", "GTN_GetTrapPrm", magic_enum::enum_name(eAxis).data(), sRtn), false;
	MillimeterToPulse(eAxis, dAcc / 1000000.00, trap.acc);
	MillimeterToPulse(eAxis, dDec / 1000000.00, trap.dec);
	trap.smoothTime = dJerk;
	sRtn = GTN_SetTrapPrm(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &trap);
	if (0 != sRtn)
	{
		return LogError("SetAxisAcc", "GTN_SetTrapPrm", magic_enum::enum_name(eAxis).data(), sRtn), false;
	}
	return true;
}

bool GTNMotionControl::SetAxisSoftLimit(Axis eAxis, double dNegLimit, double dPosLimit)
{
	double dNegL, dPosL;
	MillimeterToPulse(eAxis, dNegLimit, dNegL);
	MillimeterToPulse(eAxis, dPosLimit, dPosL);

	short sRtn = GTN_SetSoftLimit(m_iCore, m_mapMotorValue[eAxis].AxisIndex, (long)dPosL, (long)dNegL);
	if (0 != sRtn)
		return LogError("SetAxisNegLimit", "GTN_SetSoftLimit", magic_enum::enum_name(eAxis).data(), sRtn), false;

	m_mapMotorValue[eAxis].NegLimit = dNegLimit;
	m_mapMotorValue[eAxis].PosLimit = dPosLimit;
	return true;
}

bool GTNMotionControl::GetAxisIndex(Axis eAxis, int& iIndex)
{
	iIndex = m_mapMotorValue[eAxis].AxisIndex;
	return true;
}

bool GTNMotionControl::GetAxisIsRotary(Axis eAxis, bool& bRotary)
{
	bRotary = m_mapMotorValue[eAxis].Rotary;
	return true;
}

bool GTNMotionControl::GetAxisResolution(Axis eAxis, int& iResolution)
{
	iResolution = m_mapMotorValue[eAxis].Resolution;
	return true;
}

bool GTNMotionControl::GetAxisTubeDiamater(Axis eAxis, double& dTubeDiamater)
{
	int iEfac = 0;
// 	if (!ReadEFAC(eAxis, iEfac))
// 		return false;
// 
// 	int iResolutionRatio = m_mapMotorValue[eAxis].Resolution;
// 	dTubeDiamater = iResolutionRatio * 10000 / (iEfac * lcnc::process::kPi);
// 	m_dDiameter = dTubeDiamater;
	return true;
}

bool GTNMotionControl::GetAxisVel(Axis eAxis, double& dVel)
{
	double Vel;
	TrapPrm trap;
	short sRtn;
	int iAxis = m_mapMotorValue[eAxis].AxisIndex;
	sRtn = GTN_GetVel(m_iCore, iAxis, &Vel);
	if (0!= sRtn)
	{
		return LogError("GetAxisVel", "GTN_GetVel", magic_enum::enum_name(eAxis).data(), sRtn),false;
	}
	PulseToMillimeter(eAxis, Vel, dVel);
	dVel = dVel * 1000;  // pulse/ms → mm/ms → mm/s
	return true;
}

bool GTNMotionControl::GetAxisAcc(Axis eAxis, double& dAcc)
{
	TrapPrm trap;
	short sRtn;
	int iAxis = m_mapMotorValue[eAxis].AxisIndex;
	sRtn = GTN_GetTrapPrm(m_iCore, iAxis, &trap);
	if (0 != sRtn)
	{
		return LogError("GetAxisAcc", "GTN_GetTrapPrm", magic_enum::enum_name(eAxis).data(), sRtn), false;
	}
	PulseToMillimeter(eAxis, trap.acc, dAcc);
	dAcc = dAcc * 1000000;
	return true;
}

bool GTNMotionControl::GetAxisDec(Axis eAxis, double& dDec)
{
	TrapPrm trap;
	short sRtn;
	int iAxis = m_mapMotorValue[eAxis].AxisIndex;
	sRtn = GTN_GetTrapPrm(m_iCore, iAxis, &trap);
	if (0 != sRtn)
	{
		return LogError("GetAxisDec", "GTN_GetTrapPrm", magic_enum::enum_name(eAxis).data(), sRtn), false;
	}
	PulseToMillimeter(eAxis, trap.dec, dDec);
	dDec = dDec*1000000;
	return true;
}

bool GTNMotionControl::GetAxisJerk(Axis eAxis, double& dSmoothTime)
{
	double Vel;
	TrapPrm trap;
	short sRtn;
	int iAxis = m_mapMotorValue[eAxis].AxisIndex;
	sRtn = GTN_GetTrapPrm(m_iCore, iAxis, &trap);
	if (0 != sRtn)
	{
		return LogError("GetAxisJerk", "GTN_GetTrapPrm", magic_enum::enum_name(eAxis).data(), sRtn), false;
	}
	dSmoothTime = trap.smoothTime;
	return true;
}

bool GTNMotionControl::GetAxisNegLimit(Axis eAxis, double& dNegLimit)
{
	long LLimit;
	long Rlimit;
	short sRtn;
	int iAxis = m_mapMotorValue[eAxis].AxisIndex;
	sRtn = GTN_GetSoftLimit(m_iCore, iAxis, &Rlimit, &LLimit);
	if (0 != sRtn)
	{
		return LogError("GetAxisNegLimit", "GTN_GetSoftLimit", magic_enum::enum_name(eAxis).data(), sRtn), false;
	}
	PulseToMillimeter(eAxis, LLimit, dNegLimit);
	
	return true;
}

bool GTNMotionControl::GetAxisPosLimit(Axis eAxis, double& dPosLimit)
{
	long LLimit;
	long Rlimit;
	short sRtn;
	int iAxis = m_mapMotorValue[eAxis].AxisIndex;
	sRtn = GTN_GetSoftLimit(m_iCore, iAxis, &Rlimit, &LLimit);
	if (0 != sRtn)
	{
		return LogError("GetAxisPosLimit", "GTN_GetSoftLimit", magic_enum::enum_name(eAxis).data(), sRtn), false;
	}
	PulseToMillimeter(eAxis, Rlimit, dPosLimit);
	
	return true;
}

bool GTNMotionControl::GetAxisVelAccDecJerk(Axis eAxis, double& dVel, double& dAcc, double& dDec, double& dJerk)
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

bool GTNMotionControl::GetAxisSoftLimit(Axis eAxis, double& dNegLimit, double& dPosLimit)
{
	int iResult = 0;
	if (!GetAxisNegLimit(eAxis, dNegLimit))
		iResult++;
	if (!GetAxisPosLimit(eAxis, dPosLimit))
		iResult++;
	return (bool)!iResult;
}

void GTNMotionControl::ReadAxisSoftLimit(Axis eAxis, double& dNegLimit, double& dPosLimit)
{
	dNegLimit = m_mapMotorValue[eAxis].NegLimit;
	dPosLimit = m_mapMotorValue[eAxis].PosLimit;
}

bool GTNMotionControl::DigitalOutputSet(DigitalIOData& IOData, int iValue, bool bLogError)
{
	//注意固高卡的ID从1开始，作为程序员痛恨此行为
	if (!IsConnected())
	{
		return false;
	}
	if (!IOData.iIO)
	{
		if (bLogError)
			LogError("DigitalOutputSet", "IOData.iIO = 0", IOData.qstrID.toStdString().data(), 0);
		return false;
	}
	short sRtn;
	int Value = iValue ? 0 : 1;	// 固高非扩展DO低电平有效，第一次取反做硬件校正
	
	if (IOData.bInversion)
		Value = Value ? 0 : 1;	// UI配置的逻辑翻转，在已校正值基础上再取反

	if (!IOData.bExpand)
	{
		if (IOData.iIO > 16 || IOData.iIO <= 0)
		{
			return true;
		}
		sRtn = GTN_SetDoBit(m_iCore, MC_GPO, IOData.iIO, Value);
		if (0 != sRtn)
			return LogError("DigitalOutputSet", "GTN_SetDoBit", IOData.qstrID.toStdString().data(), sRtn), false;
		
	}
	else
	{
		if (IOData.iIO > 12)
		{
			return true;
		}
// 		sRtn = GTN_ExtModuleInit(m_iCore);
// 		if (0 != sRtn)
// 			return LogError("DigitalOutputSet", "GTN_ExtModuleInit", IOData.qstrID.toStdString().data(), sRtn), false;
		Value = iValue;
		if (IOData.bInversion)
			Value = iValue ? 0 : 1;
		sRtn = GTN_SetExtDoBit(m_iCore, IOData.iIO, Value);		////按位设置核1的扩展模块第iIO路DO为iValue
		if (0 != sRtn)
			return LogError("DigitalOutputSet", "GTN_SetExtDoBit", IOData.qstrID.toStdString().data(), sRtn), false;
	}
	return true;
}

bool GTNMotionControl::DigitalOutputGet(DigitalIOData& IOData, int& iValue, bool bLogError)
{
	if (!IsConnected())
	{
		return false;
	}
	short sRtn;
	if (!IOData.iIO)
	{
		if (bLogError)
			LogError("DigitalOutputGet", "IOData.iIO = 0", IOData.qstrID.toStdString().data(), 0);
		return false; 
	}
	if (!IOData.bExpand)
	{
		short* pValue = new short();
		sRtn = GTN_ReadDigitalOutput(m_iCore, MC_GPO, IOData.iIO, pValue);
		if (0 != sRtn)
		{
			if (bLogError)
				LogError("DigitalOutputGet", "GTN_ReadDigitalOutput", IOData.qstrID.toStdString().data(), sRtn);
			
			delete pValue;
			pValue = nullptr;
			return false;
		}
		iValue = *pValue;
		delete pValue;
		pValue = nullptr;
		iValue = iValue ? 0 : 1;
		if (IOData.bInversion)
			iValue = iValue ? 0 : 1;
	}
	else
	{
		short* pValue = new short();
		sRtn = GTN_GetExtDoBit(m_iCore, IOData.iIO, pValue);
		
		if (0 != sRtn)
		{
			if (bLogError)
				LogError("DigitalOutputGet", "GTN_GetExtDoBit", IOData.qstrID.toStdString().data(), sRtn);
			
			delete pValue;
			pValue = nullptr;
			return false;
		}
		iValue = *pValue;
		if (IOData.bInversion)
			iValue = iValue ? 0 : 1;
	}
	
	return true;
}

bool GTNMotionControl::DigitalInputGet(DigitalIOData& IOData, int& iValue, bool bLogError)
{
	if (!IsConnected())
	{
		return false;
	}
	short sRtn;
	if (!IOData.bExpand)
	{
		if (!IOData.iIO)
		{
			if (bLogError)
				LogError("DigitalInputGet", "IOData.iIO = 0", IOData.qstrID.toStdString().data(), 0);
			return false;
		}
			
		short* pValue = new short();	
		sRtn = GTN_GetDiBit(m_iCore, MC_GPI, IOData.iIO, pValue);
		
		if (sRtn)
		{
			if (bLogError)
				LogError("DigitalInputGet", "GTN_GetDiBit", IOData.qstrID.toStdString().data(), sRtn);
			delete pValue;
			pValue = nullptr;
			return false;
		}
		iValue = *pValue;
		//搞个取反操作，固高的是低电平有效
		delete pValue;
		pValue = nullptr;
		iValue = iValue ? 0 : 1;
		if (IOData.bInversion)
			iValue = iValue ? 0 : 1;
	}
	else
	{
		if (!IOData.iIO && bLogError)
		{
			if (bLogError)
				LogError("DigitalInputGet", "IOData.iIO = 0", IOData.qstrID.toStdString().data(), 0);
			return  false;
		}
		short* pValue = new short();
		sRtn = GTN_GetExtDiBit(m_iCore, IOData.iIO, pValue);
		
		if (sRtn)
		{
			if (bLogError)
				LogError("DigitalInputGet", "GTN_GetExtDiBit", IOData.qstrID.toStdString().data(), sRtn);
			
			delete pValue;
			pValue = nullptr;
			return false;
		}
		iValue = *pValue;
		//拓展IO就不取反了，因为是高电平有效
		delete pValue;
		pValue = nullptr;
		if (IOData.bInversion)
			iValue = iValue ? 0 : 1;
	}
	return true;
}

bool GTNMotionControl::AnalogOutputSet(AnalogIOData& IOData, double dValue, bool bLogError)
{
	//暂定编写，需了解如何使用拓展和非拓展
	if (!IsConnected())
	{
		return false;
	}
	short sRtn;
	if (!IOData.iPort)
	{
		if (bLogError)
			LogError("AnalogOutputSet", "IOData.iPort = 0", IOData.qstrID.toStdString().data(), 0);
		return false;
	}
		
	if (!IOData.bExpand)
	{
		short pValue = (short)dValue;
		sRtn = GTN_SetDac(m_iCore, IOData.iPort, &pValue);
		
		if (sRtn)
		{
			if (bLogError)
				LogError("AnalogOutputSet", "GTN_SetDac", IOData.qstrID.toStdString().data(), sRtn);
			return false;
		}
	}
	else
	{
		short pValue = (short)dValue;
		sRtn = GTN_SetAuDac(m_iCore, IOData.iPort, &pValue);
		if (sRtn)
		{
			if (bLogError)
				LogError("AnalogOutputSet", "GTN_SetAuDac", IOData.qstrID.toStdString().data(), sRtn);
			
			return false;
		}
		
	}
	return true;
}

bool GTNMotionControl::AnalogOutputGet(AnalogIOData& IOData, double& dValue, bool bLogError)
{
	//暂定编写，需了解如何使用拓展和非拓展
	if (!IsConnected())
	{
		return false;
	}
	short sRtn;
	if (!IOData.iPort )
	{
		if (bLogError)
			LogError("AnalogOutputGet", "IOData.iPort = 0", IOData.qstrID.toStdString().data(), 0);
		return  false; 
	}
	if (!IOData.bExpand)
	{
		short* pValue = new short();
		sRtn = GTN_GetDac(m_iCore, IOData.iPort, pValue);
		if (sRtn != 0)
		{
			if (bLogError)
				LogError("AnalogOutputGet", "GTN_GetDac", IOData.qstrID.toStdString().data(), sRtn);
			delete pValue;
			pValue = nullptr;
			return false;
		}
		dValue = *pValue;
		delete pValue;
		pValue = nullptr;
		return true;
	}
	else
	{
		short* pValue = new short();
		sRtn = GTN_GetAuDac(m_iCore, IOData.iPort, pValue, 1);
		if (sRtn)
		{
			if (bLogError)
				LogError("AnalogOutputGet", "GTN_SetAuDac", IOData.qstrID.toStdString().data(), sRtn);
			
			delete pValue;
			pValue = nullptr;
			return false;
		}
		dValue = *pValue;
		delete pValue;
		pValue = nullptr;
		return true;
	}
}

bool GTNMotionControl::AnalogInputGet(AnalogIOData& IOData, double& dValue, bool bLogError)
{
	//暂定编写，需了解如何使用拓展和非拓展
	if (!IsConnected())
	{
		return false;
	}
	short sRtn;
	if (!IOData.iPort)
	{
		if (bLogError)
			LogError("AnalogInputGet", "IOData.iPort = 0", IOData.qstrID.toStdString().data(), 0);
		return false;
	}
	if (!IOData.bExpand)
	{
		double* pValue = new double();
		sRtn = GTN_GetAdc(m_iCore, IOData.iPort, pValue);
		if (sRtn != 0)
		{
			if (bLogError)
				LogError("AnalogInputGet", "GTN_GetDac", IOData.qstrID.toStdString().data(), sRtn);
			delete pValue;
			pValue = nullptr;
			return false;
		}
		dValue = *pValue;
		delete pValue;
		pValue = nullptr;
		return true;
	}
	else
	{
		double* pValue = new double();
		sRtn = GTN_GetAuAdc(m_iCore, IOData.iPort, pValue);
		if (sRtn != 0)
		{
			if (bLogError)
				LogError("AnalogInputGet", "GTN_GetDac", IOData.qstrID.toStdString().data(), sRtn);
			delete pValue;
			pValue = nullptr;
			return false;
		}
		dValue = *pValue;
		delete pValue;
		pValue = nullptr;
		return true;
	}
}

bool GTNMotionControl::SetShutterOnOffWaitTime(double dBeforeOn, double dAfterOn, double dBeforeOff, double dAfterOff, double dBlowDelay)
{
	m_dLaserOnBWait = dBeforeOn;
	m_dLaserOnAWait = dAfterOn;
	m_dLaserOffBWait = dBeforeOff;
	m_dLaserOffAWait = dAfterOff;
	m_dBlowDelay = dBlowDelay;
	return true;
}

bool GTNMotionControl::StopAllBuffer()
{
	short sRtn;
	long allMask = 0;
	for (Axis axis : m_vecMotors)
		allMask |= AxisMask(axis);
	sRtn = GTN_Stop(m_iCore, allMask, 0x0);
	if (sRtn)
		return LogError("StopAllBuffer", "GTN_Stop", "", sRtn), false;
	do
	{
		Sleep(100);
	} while (IsAxisMoving());
	sRtn = GTN_CrdClear(m_iCore, 1, m_iWriteBuf);
	if (sRtn)
		return LogError("StopAllBuffer", "GTN_CrdClear", "", sRtn), false;
	return true;
}

// [P3 removed] GTNMotionControl::IsOffsetCutting

bool GTNMotionControl::IsBufferRunning(int iBufferIndex)
{
	short run, sRtn;
	long segment;
	long pSpace;
	sRtn = GTN_CrdStatus(
		m_iCore,
		1, // 坐标系是坐标系1 
		&run, // 读取插补运动状态 
		&segment, // 读取当前已经完成的插补段数
		m_iWriteBuf); // 查询坐标系1的FIFO0缓存区

	if (sRtn)
		return LogError("IsBufferRunning", "GTN_CrdStatus","", sRtn), false;
	return run;
}

bool GTNMotionControl::ConfigureCuttingAxes(const std::array<Axis, 5>& axes, int dimension)
{
	if (dimension < 3 || dimension > 5)
		return false;
	m_cuttingAxes = axes;
	m_cuttingAxisCount = dimension;
	m_cuttingCoordinateReady = false;
	return true;
}

bool GTNMotionControl::MoveToPosition(Axis axis, double velocity, double position)
{
	return MovePostion(axis, velocity, position);
}

bool GTNMotionControl::OffsetLineTo(const std::array<double, 5>& target,
	                                 int dimension, const Tool& tool)
{
	if (!m_cuttingCoordinateReady || dimension != m_cuttingAxisCount) {
		return LogError("OffsetLineTo", "dynamic coordinate system is not ready", "", -1), false;
	}
	double dVelocity = tool.m_dLineVelocity;
	double LineAcc = tool.m_dLineAcc;

	if (m_hasPreviousCuttingPose
		&& (m_dPreX == target[0]) && (m_dPreY == target[1]) && (m_dPreZ == target[2])
		&& (dimension < 4 || m_dPreR1 == target[3])
		&& (dimension < 5 || m_dPreR2 == target[4]))
		return true;
	short sRtn;
	const int MAX_RETRY = 10;
	for (int retry = 0; retry < MAX_RETRY; retry++)
	{
		if (dimension == 3) {
			sRtn = GTN_LnXYZEx(m_iCore, 1, target[0], target[1], target[2],
			                     dVelocity, LineAcc, 0, 0, m_iWriteBuf);
		} else if (dimension == 4) {
			sRtn = GTN_LnXYZAEx(m_iCore, 1, target[0], target[1], target[2], target[3],
			                      dVelocity, LineAcc, 0, 0, m_iWriteBuf);
		} else {
			double position[5] = {target[0], target[1], target[2], target[3], target[4]};
			sRtn = GTN_LnXYZACEx(m_iCore, 1, position, 0x1f,
			                       dVelocity, LineAcc, 0, 0, m_iWriteBuf);
		}
		if (!sRtn) break;
		// 软件前瞻缓冲区已满：将已有数据刷入硬件FIFO后重试
		FlushToFifo();
	}
	if (sRtn)
		return LogError("OffsetLineTo", "GTN dynamic line interpolation", "", sRtn), false;
	m_dPreX = target[0];
	m_dPreY = target[1];
	m_dPreZ = target[2];
	if (dimension >= 4) m_dPreR1 = target[3];
	if (dimension >= 5) m_dPreR2 = target[4];
	m_hasPreviousCuttingPose = true;
	return true;
}

// [P3 removed] GTNMotionControl::OffsetArcTo

// 改设置界面为旋转轴置位
// [P3 removed] GTNMotionControl::JumpToSetAFPos

void GTNMotionControl::JumpToIdleXYPosition(double dEndX, double dEndY, const Tool& curTool)
{
	long sts;
	Axis eDirectionX = magic_enum::enum_cast<Axis>(curTool.m_strDirectionX).value();
	Axis eDirectionY = magic_enum::enum_cast<Axis>(curTool.m_strDirectionY).value();
	short sRtn;
	
	//X 定位轴
	if (curTool.m_bXIsMove && eDirectionX != Axis::X && m_runtimeConfiguration.isAxisEnabled(Axis::X))
	{
		MovePostion(Axis::X, curTool.m_dIdleXVelocity, curTool.m_dXPosition);
	}
	//A 定位轴
	if (curTool.m_bAIsMove && eDirectionY != Axis::A && m_runtimeConfiguration.isAxisEnabled(Axis::A))
	{
		MovePostion(Axis::A, curTool.m_dIdleAVelocity, curTool.m_dAPosition / 360 * lcnc::process::kPi * m_dDiameter);
		
	}
	//Y 定位轴
	if (curTool.m_bYIsMove && eDirectionY != Axis::Y && m_runtimeConfiguration.isAxisEnabled(Axis::Y))
	{
		MovePostion(Axis::Y, curTool.m_dIdleYVelocity, curTool.m_dYPosition);
		
	}
	
	//空程起点坐标
	MovePostion(eDirectionX, GetAxisIdleVel(eDirectionX, curTool), dEndX);
	MovePostion(eDirectionY, GetAxisIdleVel(eDirectionY, curTool), dEndY);

	m_dPreX = dEndX;
	m_dPreY = dEndY;
}

void GTNMotionControl::JumpToIdleHeight(const Tool& curTool, double dCompensate)
{
	MovePostion(Axis::Z, curTool.m_dIdleZVelocity, curTool.m_dIdleZHeight + dCompensate);
}

void GTNMotionControl::JumpToCuttingHeight(const Tool& curTool, double dCompensate)
{
	MovePostion(Axis::Z, curTool.m_dIdleZVelocity, curTool.m_dCuttingHeight + curTool.m_dCuttingHeightCompensate + dCompensate);
}

bool GTNMotionControl::SendCommand()
{
	short sRtn = 0;
	const int MAX_RETRY = 500;   // 最大重试次数
	int nRetryCount = 0;         // 当前重试次数
	// 将前瞻缓存区中的数据压入控制器
	while (nRetryCount++ < MAX_RETRY)
	{
		if (m_bStop)
			return true;
		sRtn = GTN_CrdDataEx(m_iCore, 1, NULL, m_iWriteBuf);//压入运动缓存区
		if (!sRtn)
			break; //确认GTN_CrdDataEx指令返回值为0，表示所有数据都压入控制器
		// 延时1ms再试（防止CPU占满）
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	if (sRtn)
		return LogError("SendCommand", "GTN_CrdDataEx", "", sRtn), false;

	// 若尚未启动插补运动，现在启动（小图形/未触发提前启动时）
	if (!m_bCrdStarted)
	{
		sRtn = GTN_CrdStart(m_iCore, 1, m_iWriteBuf);
		if (sRtn)
			return LogError("SendCommand", "GTN_CrdStart", "", sRtn), false;
		m_bCrdStarted = true;
	}
	return true;
}

// 将软件前瞻缓冲区数据刷入硬件FIFO。
// 若硬件FIFO已满导致GTN_CrdDataEx失败，则先启动插补运动消费FIFO后再重试。
// 用于 OffsetLineTo/OffsetArcTo 检测到软件缓冲区满时调用，实现流式压数据。
bool GTNMotionControl::FlushToFifo()
{
	short sRtn = 0;
	const int MAX_RETRY = 500;
	int nRetryCount = 0;
	while (nRetryCount++ < MAX_RETRY)
	{
		if (m_bStop) return true;
		sRtn = GTN_CrdDataEx(m_iCore, 1, NULL, m_iWriteBuf);
		if (!sRtn) return true; // 软件前瞻数据已全部压入硬件FIFO
		// GTN_CrdDataEx失败：硬件FIFO可能已满，启动插补运动消费FIFO腾出空间
		if (!m_bCrdStarted)
		{
			short run; long segment;
			GTN_CrdStatus(m_iCore, 1, &run, &segment, m_iWriteBuf);
			if (!run)
			{
				short s = GTN_CrdStart(m_iCore, 1, m_iWriteBuf);
				if (!s) m_bCrdStarted = true;
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	LogError("FlushToFifo", "GTN_CrdDataEx", "", sRtn);
	return false;
}

bool GTNMotionControl::InitCrd(const Tool& curTool)
{
	m_bCrdStarted = false; // 每次重新初始化前瞻时，重置启动状态
	m_hasPreviousCuttingPose = false;
	m_cuttingCoordinateReady = false;
	short sRtn;
	short crd = 1, fifo = 0;

	int axisIndex[5] = {};
	for (int dimension = 0; dimension < m_cuttingAxisCount; ++dimension) {
		const Axis axis = m_cuttingAxes[dimension];
		auto it = m_mapMotorValue.find(axis);
		if (it == m_mapMotorValue.end() || it->second.AxisIndex < 1 || it->second.AxisIndex > 8)
			return LogError("InitCrd", "five-axis cutting axis not configured", magic_enum::enum_name(axis).data(), -1), false;
		axisIndex[dimension] = it->second.AxisIndex;
	}
	for (int lhs = 0; lhs < m_cuttingAxisCount; ++lhs) {
		for (int rhs = lhs + 1; rhs < m_cuttingAxisCount; ++rhs) {
			if (axisIndex[lhs] == axisIndex[rhs])
				return LogError("InitCrd", "duplicate five-axis cutting axis", "", -1), false;
		}
	}
	//确保创建前瞻前没有轴系运动
	do 
	{
		Sleep(50);
	} while (IsAxisMoving());
	// 建立号坐标系，设置坐标系参数
	TCrdPrm crdPrm;
	memset(&crdPrm, 0, sizeof(crdPrm));
	//sRtn = GTN_GetCrdPrm(core,crd,&crdPrm);
	crdPrm.dimension = m_cuttingAxisCount;
	crdPrm.synVelMax = 500; // 最大合成速度：pulse/ms
	crdPrm.synAccMax = 10; // 最大加速度：pulse/ms^2
	crdPrm.evenTime = 50; // 最小匀速时间：ms
	for (int dimension = 0; dimension < m_cuttingAxisCount; ++dimension)
		crdPrm.profile[axisIndex[dimension] - 1] = dimension + 1;
	crdPrm.setOriginFlag = 1; // 通过originPos指定坐标系原点

	for (int dimension = 0; dimension < m_cuttingAxisCount; ++dimension)
		crdPrm.originPos[axisIndex[dimension] - 1] = 0;
	sRtn = GTN_SetCrdPrm(m_iCore, crd, &crdPrm);
	if(sRtn)
	{
		LCNC_ERR(lcnc::LogCode::Generic, "{}", fmt::format(
			"GTN_SetCrdPrm failed({}): core={} crd={} dimension={} profile=[{},{},{},{},{}] "
			"synVelMax={} synAccMax={} evenTime={} setOriginFlag={} "
			"originPos=[{},{},{},{},{}] axisIndex=[{},{},{},{},{}]",
			sRtn, m_iCore, crd,
			crdPrm.dimension, crdPrm.profile[0], crdPrm.profile[1], crdPrm.profile[2],
			crdPrm.profile[3], crdPrm.profile[4],
			crdPrm.synVelMax, crdPrm.synAccMax, crdPrm.evenTime, crdPrm.setOriginFlag,
			crdPrm.originPos[0], crdPrm.originPos[1], crdPrm.originPos[2],
			crdPrm.originPos[3], crdPrm.originPos[4],
			axisIndex[0], axisIndex[1], axisIndex[2], axisIndex[3], axisIndex[4]));
		return LogError("InitCrd", "GTN_SetCrdPrm", "", sRtn), false;
	}

	// 对齐C#示例：在SetupLookAheadCrd之前先清除坐标系缓冲区
	sRtn = GTN_CrdClear(m_iCore, crd, fifo);
	if (sRtn) return LogError("InitCrd", "GTN_CrdClear", "", sRtn), false;

	// ---- 初始化多轴前瞻模块 ----
	// 五轴机床模式：CAM 已提供五个轴的连续机床坐标，RTCP 保持关闭。
	int axisLimitMode[8] = {0};

	TLookAheadParameter lookAheadPara;
	memset(&lookAheadPara, 0, sizeof(lookAheadPara));
	lookAheadPara.lookAheadNum = 200;
	lookAheadPara.time = 0.01;
	lookAheadPara.radiusRatio = 0.5;
	for (Axis axis : m_vecMotors)
	{
		int iIndex = m_mapMotorValue[axis].AxisIndex - 1; // 0-based
		axisLimitMode[iIndex] = AXIS_LIMIT_MAX_VEL | AXIS_LIMIT_MAX_ACC | AXIS_LIMIT_MAX_DV;
		lookAheadPara.vMax[iIndex]  = 500;
		lookAheadPara.aMax[iIndex]  = 5000;
		lookAheadPara.DVMax[iIndex] = 300;
	}

	// 构建完整的 axisRelation[0..7]：保证所有8个槽位均非零且不重复。
	// 任何槽位为0都会导致 GTN_InitLookAheadEx 返回错误107。
	// 对齐C#示例：[1,2,3,4,5,6,7,8]（C#固定填满全部8个槽位）
	{
		bool used[8] = {};
		int slotIdx = 0;
		// 前五槽位严格对应坐标系 XYZ/R1/R2，保证 GTN_LnXYZACEx 的 pPos 顺序一致。
		for (int dimension = 0; dimension < m_cuttingAxisCount; ++dimension) {
			const int index = axisIndex[dimension];
			lookAheadPara.axisRelation[slotIdx++] = static_cast<short>(index);
			used[index - 1] = true;
		}
		// 剩余槽位填写其它已配置轴。
		for (Axis axis : m_vecMotors)
		{
			if (slotIdx >= 8) break;
			int idx = m_mapMotorValue[axis].AxisIndex;
			if (used[idx - 1]) continue;
			lookAheadPara.axisRelation[slotIdx++] = (short)idx;
			used[idx - 1] = true;
		}
		// 剩余空位：用1..8中尚未出现的轴号按升序填充
		for (int n = 1; n <= 8 && slotIdx < 8; n++)
		{
			if (!used[n - 1])
			{
				lookAheadPara.axisRelation[slotIdx++] = (short)n;
				used[n - 1] = true;
			}
		}
	}

	// scale：固高当前前瞻库要求各槽一致；按 X 轴的脉冲当量填充。
	{
		double dRes = m_mapMotorValue[m_cuttingAxes[0]].Resolution;
		for (int k = 0; k < 8; k++)
			lookAheadPara.scale[k] = dRes;
	}

	for (Axis axis : m_vecMotors)
	{
		int iCurAxisIdx = m_mapMotorValue[axis].AxisIndex;
		sRtn = GTN_SetAxisMotionSmooth(m_iCore, iCurAxisIdx, curTool.m_dAxisSmoothTime, curTool.m_dAxisSmoothK);
		if (sRtn)
			return LogError("SetContiInterpolation", "GTN_SetAxisMotionSmooth", "", sRtn), false;
	}

	// 对齐C#示例：SetupLookAheadCrd → InitLookAheadEx 之间不插入任何其它调用
	const EMachineMode machineMode = m_cuttingAxisCount == 3 ? NORMAL_THREE_AXIS
		: m_cuttingAxisCount == 4 ? MULTI_AXES : FIVE_AXIS;
	sRtn = GTN_SetupLookAheadCrd(m_iCore, crd, machineMode);
	if (sRtn) return LogError("SetContiInterpolation", "GTN_SetupLookAheadCrd", "", sRtn), false;

	sRtn = GTN_InitLookAheadEx(m_iCore, crd, &lookAheadPara, fifo, 0);
	if (sRtn) return LogError("SetContiInterpolation", "GTN_InitLookAheadEx", "", sRtn), false;

	// InitLookAheadEx 成功后再配置轴限制和速度有效模式
	sRtn = GTN_SetAxisLimitModeLa(m_iCore, crd, axisLimitMode);
	if (sRtn) return LogError("SetContiInterpolation", "GTN_SetAxisLimitModeLa", "", sRtn), false;
	long velValidMask = 0;
	for (int dimension = 0; dimension < m_cuttingAxisCount; ++dimension)
		velValidMask |= (1L << (axisIndex[dimension] - 1));
	sRtn = GTN_SetAxisVelValidModeLa(m_iCore, crd, velValidMask);
	if (sRtn) return LogError("SetContiInterpolation", "GTN_SetAxisVelValidModeLa", "", sRtn), false;

	sRtn = GTN_SetCrdJerkTime(m_iCore, 1, curTool.m_dCutSmoothTime, curTool.m_dCutSmoothK);
	if (sRtn) return LogError("SetContiInterpolation", "GTN_SetCrdJerkTime", "", sRtn), false;
	sRtn = GTN_CrdHsOn(m_iCore, 1, 0, 1, 300, 0);
	if (sRtn) return LogError("SetContiInterpolation", "GTN_CrdHsOn", "", sRtn), false;
	m_cuttingCoordinateReady = true;
	return true;
}

bool GTNMotionControl::PrfTrapAxis()
{
	short sRtn;
	for (Axis axis : m_vecMotors)
	{
		// 将 AXIS 轴设为点位模式
		sRtn = GTN_PrfTrap(m_iCore, m_mapMotorValue[axis].AxisIndex);
		if (sRtn)return LogError("SetJumpAccJerk", "GTN_PrfTrap", magic_enum::enum_name(axis).data(), sRtn), false;

	}
	return true;
}

bool GTNMotionControl::SetJumpAccJerk(const Tool& curTool)
{
	//空程参数暂时由各个轴系为相同值设定
	short sRtn;
	for (Axis axis : m_vecMotors)
	{
		// 将 AXIS 轴设为点位模式
		sRtn = GTN_PrfTrap(m_iCore, m_mapMotorValue[axis].AxisIndex);
		if (sRtn)return LogError("SetJumpAccJerk", "GTN_PrfTrap", magic_enum::enum_name(axis).data(), sRtn), false;
		TTrapPrm trap;
		double dAccx ;
		double dDecx ;
		MillimeterToPulse(axis, curTool.m_dIdleXYAccDec / 1000000.00, dAccx);
		MillimeterToPulse(axis, curTool.m_dIdleXYAccDec / 1000000.00, dDecx);
		trap.acc = dAccx;
		trap.dec = dDecx;
		trap.velStart = 0;
		trap.smoothTime = curTool.m_dIdleXYJerk;
		sRtn = GTN_SetTrapPrm(m_iCore, m_mapMotorValue[axis].AxisIndex, &trap);
		if (sRtn)return LogError("SetJumpAccJerk", "GTN_SetTrapPrm", magic_enum::enum_name(axis).data(), sRtn), false;
	}

	return true;
}

void GTNMotionControl::ProLaserControl(bool bLaser, bool bPso, const Tool& curTool, bool bAOUTFlag)
{
	short sRtn;
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
			int ivalue;
			//一路气关气
			if (!m_mapDigitalOUT[DigitalOUT::Blow].bExpand)
			{
				ivalue = m_mapDigitalOUT[DigitalOUT::Blow].bInversion ? 0 : 1;
				sRtn = GTN_BufDoBitEx(m_iCore, 1, MC_GPO, m_mapDigitalOUT[DigitalOUT::Blow].iIO, ivalue, m_iWriteBuf);
				if (sRtn) LogError("ProLaserControl", "GTN_BufDoBitEx_Blow","", sRtn);
			}
			else
			{
				ivalue = m_mapDigitalOUT[DigitalOUT::Blow].bInversion ? 1 : 0;
				sRtn = GTN_BufExtDoBitEx(m_iCore, 1, m_mapDigitalOUT[DigitalOUT::Blow].iIO, ivalue, m_iWriteBuf);
				if (sRtn) LogError("ProLaserControl", "GTN_BufDoBitEx_Expand_Blow", "", sRtn);
			}
			//开气
			if (!m_mapDigitalOUT[DigitalOUT::Blow2].bExpand)
			{
				ivalue = m_mapDigitalOUT[DigitalOUT::Blow2].bInversion ? 1 : 0;
				sRtn = GTN_BufDoBitEx(m_iCore, 1, MC_GPO, m_mapDigitalOUT[DigitalOUT::Blow2].iIO, ivalue, m_iWriteBuf);
				if (sRtn) LogError("ProLaserControl", "GTN_BufDoBitEx_Blow2", "", sRtn);
			}
			else
			{
				ivalue = m_mapDigitalOUT[DigitalOUT::Blow2].bInversion ? 0 : 1;
				sRtn = GTN_BufExtDoBitEx(m_iCore, 1, m_mapDigitalOUT[DigitalOUT::Blow2].iIO, ivalue, m_iWriteBuf);
				if (sRtn) LogError("ProLaserControl", "GTN_BufDoBitEx_Expand_Blow2", "", sRtn);
			}
		}
		else
		{
			int ivalue;
			//二路气关气
			if (!m_mapDigitalOUT[DigitalOUT::Blow2].bExpand)
			{
				ivalue = m_mapDigitalOUT[DigitalOUT::Blow2].bInversion ? 0 : 1;
				sRtn = GTN_BufDoBitEx(m_iCore, 1, MC_GPO, m_mapDigitalOUT[DigitalOUT::Blow2].iIO, ivalue, m_iWriteBuf);
				if (sRtn) LogError("ProLaserControl", "GTN_BufDoBitEx_Blow2", "", sRtn);
			}
			else
			{
				ivalue = m_mapDigitalOUT[DigitalOUT::Blow2].bInversion ? 1 : 0;
				sRtn = GTN_BufExtDoBitEx(m_iCore, 1, m_mapDigitalOUT[DigitalOUT::Blow2].iIO, ivalue, m_iWriteBuf);
				if (sRtn) LogError("ProLaserControl", "GTN_BufDoBitEx_Expand_Blow2", "", sRtn);
			}
			//开气
			if (!m_mapDigitalOUT[DigitalOUT::Blow].bExpand)
			{
				ivalue = m_mapDigitalOUT[DigitalOUT::Blow].bInversion ? 1 : 0;
				sRtn = GTN_BufDoBitEx(m_iCore, 1, MC_GPO, m_mapDigitalOUT[DigitalOUT::Blow].iIO, ivalue, m_iWriteBuf);
				if (sRtn) LogError("ProLaserControl", "GTN_BufDoBitEx_Blow", "", sRtn);
			}
			else
			{
				ivalue = m_mapDigitalOUT[DigitalOUT::Blow].bInversion ? 0 : 1;
				sRtn = GTN_BufExtDoBitEx(m_iCore, 1, m_mapDigitalOUT[DigitalOUT::Blow].iIO, ivalue, m_iWriteBuf);
				if (sRtn) LogError("ProLaserControl", "GTN_BufDoBitEx_Expand_Blow", "", sRtn);
			}
		}
		if (m_dBlowDelay >= 1)
		{
			sRtn = GTN_BufDelayEx(m_iCore, 1, m_dBlowDelay, m_iWriteBuf);
			if (sRtn) LogError("ProLaserControl", "GTN_BufDelayEx", "", sRtn);
		}
		
		if (m_dLaserOffBWait >= 1)
		{
			sRtn = GTN_BufDelayEx(m_iCore, 1, m_dLaserOffBWait, m_iWriteBuf);
			if (sRtn) LogError("ProLaserControl", "GTN_BufDelayEx_OffBWait","", sRtn);
		}

		if (bAOUTFlag)//模拟量判断
		{
			//编写有异议，后期考虑如何编写
			sRtn = GTN_BufLaserOnEx(m_iCore, 1, 0, m_iWriteBuf);// 打开激光通道 1 输出
			if (sRtn) LogError("ProLaserControl", "GTN_BufLaserOnEx", "", sRtn);
		}
		else
		{
			int ivalue;
			if (!m_mapDigitalOUT[DigitalOUT::Laser].bExpand)
			{
				ivalue = m_mapDigitalOUT[DigitalOUT::Laser].bInversion ? 1 : 0;
				sRtn = GTN_BufDoBitEx(m_iCore, 1, MC_GPO, m_mapDigitalOUT[DigitalOUT::Laser].iIO, ivalue, 0);
				if (sRtn) LogError("ProLaserControl", "GTN_BufDoBitEx_Laser", "", sRtn);
			}
			else
			{
				ivalue = m_mapDigitalOUT[DigitalOUT::Laser].bInversion ? 0 : 1;
				sRtn = GTN_BufExtDoBitEx(m_iCore, 1, m_mapDigitalOUT[DigitalOUT::Laser].iIO, ivalue, 0);
				if (sRtn) LogError("ProLaserControl", "GTN_BufExtDoBitEx_Laser", "", sRtn);
			}

		}
		if (m_dLaserOnAWait >= 1)
		{
			sRtn = GTN_BufDelayEx(m_iCore, 1, m_dLaserOnAWait, m_iWriteBuf);
			if (sRtn) LogError("ProLaserControl", "GTN_BufDelayEx_OnAWait", "", sRtn);
		}
	}
	else
	{
		if (m_dLaserOffBWait >= 1)
		{
			sRtn = GTN_BufDelayEx(m_iCore, 1, m_dLaserOffBWait, m_iWriteBuf);
			if (sRtn) LogError("ProLaserControl", "GTN_BufDelayEx_OffBWait", "", sRtn);
		}

		if (bAOUTFlag)
		{
			sRtn = GTN_BufLaserOffEx(m_iCore, 1, 0, m_iWriteBuf);// 关闭激光通道 1 输出
			if (sRtn) LogError("ProLaserControl", "GTN_BufLaserOffEx", "", sRtn);
		}
		else
		{
			int iValue;
			if (!m_mapDigitalOUT[DigitalOUT::Laser].bExpand)
			{
				iValue = m_mapDigitalOUT[DigitalOUT::Laser].bInversion ? 0 : 1;
				sRtn = GTN_BufDoBitEx(m_iCore, 1, MC_GPO, m_mapDigitalOUT[DigitalOUT::Laser].iIO, iValue, m_iWriteBuf);
				if (sRtn) LogError("ProLaserControl", "GTN_BufDoBitEx_Laser", "", sRtn);
			}
			else
			{
				iValue = m_mapDigitalOUT[DigitalOUT::Laser].bInversion ? 1 : 0;
				sRtn = GTN_BufExtDoBitEx(m_iCore, 1, m_mapDigitalOUT[DigitalOUT::Laser].iIO, iValue, m_iWriteBuf);
				if (sRtn) LogError("ProLaserControl", "GTN_BufExtDoBitEx_Laser", "", sRtn);
			}
		}

		if (m_dLaserOffAWait >= 1)
		{
			sRtn = GTN_BufDelayEx(m_iCore, 1, m_dLaserOffAWait, m_iWriteBuf);
			if (sRtn) LogError("ProLaserControl", "GTN_BufDelayEx_OffAWait", "", sRtn);
		}

		//切割完是否关气没有编写
	}
}

bool GTNMotionControl::IsAxisStatusNormal(int& iFault)
{
	
	return true;
}

double GTNMotionControl::GetAxisIdleVel(Axis eAxis, const Tool& curTool)
{
	switch (eAxis)
	{
	case Axis::X:	return curTool.m_dIdleXVelocity;
	case Axis::Y:	return curTool.m_dIdleYVelocity;
	case Axis::Z:	return curTool.m_dIdleZVelocity;
	case Axis::A:	return curTool.m_dIdleAVelocity;	//临时
	default:	return curTool.m_dIdleX1Velocity;
	}
}

// [P3 removed] GTNMotionControl::SetFPos

// [P3 removed] GTNMotionControl::GetFPos


bool GTNMotionControl::MovePostion(Axis aAxis, double dVel, double dPos)
{
	short sRtn;
	long sts;
	double dNewVel,dNewPos;
	MillimeterToPulse(aAxis, dVel / 1000.00, dNewVel);
	MillimeterToPulse(aAxis, dPos, dNewPos);
	sRtn = GTN_PrfTrap(m_iCore, m_mapMotorValue[aAxis].AxisIndex);
	if (sRtn) return LogError("MovePostion", "GTN_PrfTrap", magic_enum::enum_name(aAxis).data(), sRtn), false;
	sRtn = GTN_SetVel(m_iCore, m_mapMotorValue[aAxis].AxisIndex, dNewVel);
	if (sRtn) return LogError("MovePostion", "GTN_SetVel", magic_enum::enum_name(aAxis).data(), sRtn), false;

	sRtn = GTN_SetPos(m_iCore, m_mapMotorValue[aAxis].AxisIndex, (long)dNewPos);
	if (sRtn) return LogError("MovePostion", "GTN_SetPos", magic_enum::enum_name(aAxis).data(), sRtn), false;

	long mask = AxisMask(aAxis);
	if (!mask)
		return LogError("MovePostion", "AxisMask", magic_enum::enum_name(aAxis).data(), -1), false;
	sRtn = GTN_Update(m_iCore, mask);//启动轴运动
	if (sRtn) return LogError("MovePostion", "GTN_Update", magic_enum::enum_name(aAxis).data(), sRtn), false;

	do
	{
		sRtn = GTN_GetSts(m_iCore, m_mapMotorValue[aAxis].AxisIndex, &sts);
		if (sRtn)
			return LogError("MovePostion", "GTN_GetSts", magic_enum::enum_name(aAxis).data(), sRtn),false;
		if (m_bStop)
			return false;
	} while (sts & 0x400);// 等待AXIS轴规划停止
	return true;
}

bool GTNMotionControl::PulseToMillimeter(Axis aAxis, double dValue, double& dNewValue)
{
	if (m_mapMotorValue[aAxis].Rotary)
		dNewValue = dValue / m_mapMotorValue[aAxis].Resolution * (lcnc::process::kPi * m_dDiameter);
	else
		dNewValue = dValue / m_mapMotorValue[aAxis].Resolution;
	return true;
}

bool GTNMotionControl::MillimeterToPulse(Axis aAxis, double dValue, double& dNewValue)
{
	if (m_mapMotorValue[aAxis].Rotary)
		dNewValue = dValue * m_mapMotorValue[aAxis].Resolution / (lcnc::process::kPi * m_dDiameter);
	else
		dNewValue = dValue * m_mapMotorValue[aAxis].Resolution;
	return true;
}

// [P3 removed] GTNMotionControl::GetCuttingCommand

// [P3 removed] GTNMotionControl::SetMFLAGSValue

// [P3 removed] GTNMotionControl::GSN_SetLaserParameterApplication

// [P3 removed] GTNMotionControl::GSN_SetLaserEnablePro

// [P3 removed] GTNMotionControl::GSN_LaserOnStatus

// [P3 removed] GTNMotionControl::CheckBuffer

// [P3 removed] GTNMotionControl::RunBuffer

bool GTNMotionControl::PauseBuffer(int iBufferIndex)
{
	short sRtn;
	sRtn = GTN_Stop(m_iCore, 0xff, 0x0);
	if (sRtn) return LogError("PauseBuffer", "GTN_Stop","", sRtn), false;
	return true;
}

// [P3 removed] GTNMotionControl::GetBufferState

// [P3 removed] GTNMotionControl::LoadCommandAndRunBuffer

bool GTNMotionControl::IsReachPos(Axis eAxis, bool bRelative, double dPos)
{
	short sRtn;
	if (bRelative)	//相对
	{
		double dNowPos,dPlusePos;
		sRtn = GTN_GetPrfPos(m_iCore, m_mapMotorValue[eAxis].AxisIndex, &dPlusePos);//规划位置
		PulseToMillimeter(eAxis, dPlusePos, dNowPos);
		if (sRtn != 0)
			return LogError("IsReachPos", "GTN_GetPrfPos","", sRtn), false;
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

void GTNMotionControl::ClearAxisState()
{
	if (m_bConnectFlag)
		return;

	short sRtn = GTN_ClrSts(m_iCore, 1, 8);
	if (0 != sRtn)
		LogError("Connect", "GTN_ClrSts", "", sRtn);
}

void GTNMotionControl::StartCommand()
{
	short sRtn = GTN_CrdStart(m_iCore, 1, m_iWriteBuf);
	if (sRtn)
		LogError("StartCommand", "GTN_CrdStart", "", sRtn);
}

// Sink 入口：GTN 路径上 ResetProgramCommand 是 no-op，因为缓冲区的清空通过
// CrdClear/CrdData 在 InitCrd / SendCommand 中天然发生（详见 SendCommand 实现）。
void GTNMotionControl::ResetProgramCommand()
{
}

// Sink 入口：把工具的切割加速度/Jerk 推到坐标系前瞻 —— GTN 通过 GTN_SetCrdJerkTime
// 已在 InitCrd 里设置一次，每段刀路本身的 ACC/JERK 由 GTN_LnXYZACEx 内部按已建立的轨迹规划处理，
// 这里保留方法签名以匹配 IMotionCommandSink::applyToolMotionParams 调用入口，no-op 即可。
void GTNMotionControl::SetCuttingAccJerk(const Tool& tool)
{
	Q_UNUSED(tool);
}
