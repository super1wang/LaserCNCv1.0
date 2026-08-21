#pragma once

#include "motion_control.h"
#include "ACSC.h"
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <cmath>
#include <time.h>
#include <map>
#include <string>
#include <atomic>

#define     deviceDescription L"PCI-1730,BID#0"

//#define MM 10000
#define DEBUG_MODE

struct AcsAxisData
{
	AcsAxisData(int iIndex, 
				bool bRotary,
				int  iResolution,
				int iHomeIndex, 
				std::string strHomeName,
				std::string strRL,
				std::string strLL,
				std::string strEfac,
				double dVel,
				double dAcc,
				double dDec,
				double dJerk,
				double dNeg,
				double dPos,
				std::string strName):
				AxisIndex(iIndex),
				Rotary(bRotary),
				Resolution(iResolution),
				HomeBufferIndex(iHomeIndex),
				HomeName(strHomeName),
				SRLName(strRL),
				SLLName(strLL),
				Efac(strEfac),
				Velocity(dVel),
				Acceleration(dAcc),
				Deceleration(dDec),
				Jerk(dJerk),
				NegLimit(dNeg),
				PosLimit(dPos),
				Name(strName){}
	AcsAxisData(){}
	int		AxisIndex;
	bool	Rotary;
	int     Resolution;
	int		HomeBufferIndex;
	std::string	HomeName;
	std::string	SRLName;
	std::string	SLLName;
	std::string	Efac;
	double	Velocity;
	double	Acceleration;
	double	Deceleration;
	double	Jerk;
	double	NegLimit;
	double	PosLimit;
	std::string  Name;
};

class  ACSMotionControl :public MotionControl
{
private:
	double					m_dLaserOnBWait;
	double					m_dLaserOnAWait;
	double					m_dLaserOffBWait;
	double					m_dLaserOffAWait;

	double					m_dDiameter;
	bool					IsPressureMonitoring;
	double					m_dBlowDelay;

	std::atomic_bool		m_bStop{false};			// 停止轴系
protected:
	int						m_iProgramBufferIndex;
	bool					m_bConnectFlag;				//是否连接的标志状态
	bool					m_bErrorOccurred;
	std::string					m_strName;
	HANDLE					m_hHandle;					//与ACS控制器通讯的句柄
	std::map<lcnc::process::Axis,AcsAxisData>	m_mapMotorValue;

	std::string					m_strCommand;
	double					m_dPreX;					//上一个X的位置
	double					m_dPreY;					//上一个Y的位置

	double					m_dFrameLLX;
	double					m_dFrameLLY;
	double					m_dFrameURX;
	double					m_dFrameURY;
		
private:
	//std::string FindAxisSign(MotionControl::Axis);
public:
	ACSMotionControl(lcnc::process::ProcessSettingsService& settings,
	                 lcnc::process::ProcessRuntimeConfiguration& runtimeConfiguration);
	~ACSMotionControl(void);

	//MotionControl基类函数重写
	virtual const std::string& GetName() const;
	virtual void LogError();
	virtual void CreateMotor(lcnc::process::Axis eAxis, const toml::table& tAxis);
	virtual bool Connect();
	virtual bool Disconnect();
	virtual bool IsConnected();
	virtual bool Reboot();
	virtual bool Home();
	virtual bool Home(lcnc::process::Axis eAxis);
	virtual bool IsHomed();
	virtual bool IsHomed(lcnc::process::Axis eAxis);
	virtual bool StopHome();
	virtual bool Enable();
	virtual bool Enable(lcnc::process::Axis eAxis);
	virtual bool Disable();
	virtual bool Disable(lcnc::process::Axis eAxis);
	virtual bool IsEnabled();
	virtual bool IsEnabled(lcnc::process::Axis eAxis);
	virtual bool SetAxisEnable(lcnc::process::Axis eAxis, bool bEnable);
	virtual bool Jog(lcnc::process::Axis eAxis, bool bDirection, double dVel);
	virtual bool MoveRelative(lcnc::process::Axis eAxis, double dPos, double dVel);
	virtual bool MoveAbsolute(lcnc::process::Axis eAxis, double dPos, double dVel);
	virtual bool MoveMRelative(std::vector<lcnc::process::Axis> vAxis, std::vector<double> dPos, double dVel);
	virtual bool MoveMAbsolute(std::vector<lcnc::process::Axis> vAxis, std::vector<double> dPos, double dVel);
	virtual bool StopMotion();
	virtual bool StopMotion(lcnc::process::Axis eAxis);
	virtual bool IsAxisMoving();
	virtual bool IsAxisMoving(lcnc::process::Axis eAxis);
	virtual bool GetActualPos(lcnc::process::Axis eAxis, double& dFPos);
	virtual bool GetFeedbackPos(lcnc::process::Axis eAxis, double& dFPos);
	virtual bool SetFPosition(lcnc::process::Axis eAxis, double dPos);

	// 轴系基础参数设置
	virtual bool SetAxisIndex(lcnc::process::Axis eAxis, int iIndex);
	virtual bool SetAxisHomeBufferIndex(lcnc::process::Axis eAxis, int iHomeIndex);
	virtual bool SetAxisIsRotary(lcnc::process::Axis eAxis, bool bRotary);
	virtual bool SetAxisResolution(lcnc::process::Axis eAxis, int iResolution);
	virtual bool SetAxisTubeDiamater(lcnc::process::Axis eAxis, double dTubeDiamater);
	virtual bool SetAxisVel(lcnc::process::Axis eAxis, double dVel);
	virtual bool SetAxisAcc(lcnc::process::Axis eAxis, double dAcc);
	virtual bool SetAxisDec(lcnc::process::Axis eAxis, double dDec);
	virtual bool SetAxisJerk(lcnc::process::Axis eAxis, double dJerk);
	virtual bool SetAxisNegLimit(lcnc::process::Axis eAxis, double dNegLimit);
	virtual bool SetAxisPosLimit(lcnc::process::Axis eAxis, double dPosLimit);
	virtual bool SetAxisVelAccDecJerk(lcnc::process::Axis eAxis, double dVel, double dAcc, double dDec, double dJerk);
	virtual bool SetAxisSoftLimit(lcnc::process::Axis eAxis, double dNegLimit, double dPosLimit);

	virtual bool GetAxisIndex(lcnc::process::Axis eAxis, int& iIndex);
	virtual bool GetAxisHomeBufferIndex(lcnc::process::Axis eAxis, int& iIndex);
	virtual bool GetAxisIsRotary(lcnc::process::Axis eAxis, bool& bRotary);
	virtual bool GetAxisResolution(lcnc::process::Axis eAxis, int& iResolutionRatioRotation);
	virtual bool GetAxisTubeDiamater(lcnc::process::Axis eAxis, double& dTubeDiamater);
	virtual bool GetAxisVel(lcnc::process::Axis eAxis, double& dVel);
	virtual bool GetAxisAcc(lcnc::process::Axis eAxis, double& dAcc);
	virtual bool GetAxisDec(lcnc::process::Axis eAxis, double& dDec);
	virtual bool GetAxisJerk(lcnc::process::Axis eAxis, double& dJerk);
	virtual bool GetAxisNegLimit(lcnc::process::Axis eAxis, double& dNegLimit);
	virtual bool GetAxisPosLimit(lcnc::process::Axis eAxis, double& dPosLimit);
	virtual bool GetAxisVelAccDecJerk(lcnc::process::Axis eAxis, double& dVel, double& dAcc, double& dDec, double& dJerk);
	virtual bool GetAxisSoftLimit(lcnc::process::Axis eAxis, double& dNegLimit, double& dPosLimit);
	virtual void ReadAxisSoftLimit(lcnc::process::Axis eAxis, double& dNegLimit, double& dPosLimit);

	// IO部分设置
	virtual bool DigitalOutputSet	(DigitalIOData&	IOData,	int		iValue,	bool bLogError = false);
	virtual bool DigitalOutputGet	(DigitalIOData&	IOData,	int&	iValue, bool bLogError = false);
	virtual bool DigitalInputGet	(DigitalIOData&	IOData,	int&	iValue, bool bLogError = false);
	virtual bool AnalogOutputSet	(AnalogIOData&	IOData,	double  dValue, bool bLogError = false);
	virtual bool AnalogOutputGet	(AnalogIOData&	IOData,	double& dValue, bool bLogError = false);
	virtual bool AnalogInputGet		(AnalogIOData&	IOData,	double& dValue, bool bLogError = false);

	virtual bool SetShutterOnOffWaitTime(double dBeforeOn, double dAfterOn,
		double dBeforeOff, double dAfterOff, double dBlowDelay);

	/// @brief 由 AcsTextCommandSink 调用，把外部生成的 ACSPL+ 文本追加到当前程序缓冲。
	void AppendRawProgramText(const std::string& fragment) { m_strCommand += fragment; }
	/// @brief 提供给 sink 用于读取每轴默认 acc/vel/jerk（来自 TOML），以便在工具参数
	///        无效时回退到合理默认（Fix #2 的归一化入口）。
	const std::map<lcnc::process::Axis, AcsAxisData>& MotorMap() const { return m_mapMotorValue; }

	// IMotionCommandSink 切割管线入口（仅 AcsTextCommandSink 调用）
	virtual void ResetProgramCommand();
	virtual bool SetJumpAccJerk(const Tool&);
	virtual void JumpToIdleXYPosition(double dEndX, double dEndY, const Tool& curTool);
	virtual void ProLaserControl(bool, bool, const Tool&, bool);
	virtual bool SetCuttingAccJerk(const Tool&);
	virtual void EndProgramCommand(const Tool&);
	virtual bool SendCommand();
	virtual bool IsBufferRunning(int iBufferIndex);
	virtual bool PauseBuffer(int iBufferIndex);
	virtual bool StopBuffer(int iBufferIndex);
	virtual bool StopAllBuffer();
	virtual bool StopMovingCuttingHead();
	virtual bool StartMovingCuttingHead(const Tool&);

	virtual bool HaltMotor(lcnc::process::Axis eMotor);
	virtual bool IsAxisStatusNormal(int& iFault);
	virtual bool IsQueueActive();
	virtual bool ErrorOccurred() const;
	virtual int  GetPressureState();
	virtual bool GetIsPressureState();
	virtual void SetIsPressureState(bool);

protected:
	bool SetDiamaterXVEL(lcnc::process::Axis eAxis, double dValue);
	bool RunBufferTillEnd(int iBufferIndex, int iTimeout);
	bool AfterOpenComm();
	void DeleteOtherConnections();
	bool WriteEFAC(lcnc::process::Axis eAxis, int iEfac);
	bool ReadEFAC(lcnc::process::Axis eAxis, int &iEfac);
	bool ControllerSaveToFlash(lcnc::process::Axis eAxis);
	bool AcscReadReal(const std::string, double&);
	bool AcscWriteReal(const std::string, double);
	bool AcscReadInt(const std::string, int&);
	bool AcscWriteInt(const std::string, int);
	bool IsReachPos(lcnc::process::Axis eAxis, bool bRelative, double dPos);

private:
	bool GetFault(int iAxis, int &Fault);
	double GetAxisIdleVel(lcnc::process::Axis eAxis, const Tool& tool);
	bool IsHomeBufferRunning();
	bool InitCrd(const Tool&) { return true; };
	bool PrfTrapAxis() { return true; };
};
