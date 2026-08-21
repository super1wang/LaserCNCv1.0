#pragma once

#include "motion_control.h"
//#include "ACSC.h"
#include "gts.h"
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <cmath>
#include <time.h>
#include <map>
#include <array>
#include <atomic>
#include <string>

//#define     deviceDescription L"PCI-1730,BID#0"

//#define MM 10000
//#define DEBUG_MODE1

struct GSNAxisData
{
	GSNAxisData(int iIndex,
		bool bRotary,
		int  iResolution,
		//int iHomeIndex,
		//std::string strHomeName,
		//std::string strRL,
		//std::string strLL,
		//std::string strEfac,
		double dVel,
		double dAcc,
		double dDec,
		double dSmoothTime,
		double dNeg,
		double dPos,
		//int    dConversion,
		std::string strName,
		THomePrm *homeParameters) :
		AxisIndex(iIndex),
		Rotary(bRotary),
		Resolution(iResolution),
		//HomeBufferIndex(iHomeIndex),
		//HomeName(strHomeName),
		//SRLName(strRL),
		//SLLName(strLL),
		//Efac(strEfac),
		Velocity(dVel),
		Acceleration(dAcc),
		Deceleration(dDec),
		SmoothTime(dSmoothTime),
		NegLimit(dNeg),
		PosLimit(dPos),
		//Conversion(dConversion),
		Name(strName),
		pTHomePrm(homeParameters){}
	GSNAxisData() : AxisIndex(0), Rotary(false), Resolution(1),
		Velocity(0), Acceleration(0), Deceleration(0), SmoothTime(0),
		NegLimit(0), PosLimit(0), pTHomePrm(nullptr) {}
	int		AxisIndex;
	bool	Rotary;
	int     Resolution;
	//int		HomeBufferIndex;
	//std::string	HomeName;
	//std::string	SRLName;
	//std::string	SLLName;
	//std::string	Efac;
	double	Velocity;
	double	Acceleration;
	double	Deceleration;
	double	SmoothTime;
	double	NegLimit;
	double	PosLimit;
	//int     Conversion;
	std::string  Name;
    THomePrm *pTHomePrm;
};

class  GTNMotionControl :public MotionControl
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
	bool					m_bConnectFlag;				//是否连接的标志状态
	bool					m_bErrorOccurred;
	std::string					m_strName;
	std::map<lcnc::process::Axis, GSNAxisData>	m_mapMotorValue;

	//std::string					m_strCommand;
	double					m_dPreX;					//上一个X的位置
	double					m_dPreY;					//上一个Y的位置
	double					m_dPreZ;					//上一个Z的位置
	double					m_dPreR1;					//上一个第一旋转轴的位置
	double					m_dPreR2;					//上一个第二旋转轴的位置
	bool                    m_hasPreviousCuttingPose{false};
	bool                    m_cuttingCoordinateReady{false};
	// 五维坐标系的维度顺序。由 GtnBufferedCommandSink 按 AxisMap 注入，
	// 因而可覆盖 AC / BC 转台及摆头，而非硬编码物理轴号。
	std::array<lcnc::process::Axis, 5>     m_cuttingAxes{lcnc::process::Axis::X, lcnc::process::Axis::Y, lcnc::process::Axis::Z, lcnc::process::Axis::A, lcnc::process::Axis::C};
	int                     m_cuttingAxisCount{5};

	double					m_dFrameLLX;
	double					m_dFrameLLY;
	double					m_dFrameURX;
	double					m_dFrameURY;

	int                     m_iWriteBuf;				// 当前要写入的缓冲区（0/1交替）
	int						m_inRunBuf;					// 正在运行的缓冲区
	bool                    m_bCrdStarted;              // 当前图形是否已调用过 GTN_CrdStart
	int                     m_iCrd;                     //插补坐标系号
	int						m_iCore;					//与控制器核号

private:
	//std::string FindAxisSign(MotionControl::Axis);
public:
	GTNMotionControl(lcnc::process::ProcessSettingsService& settings,
	                 lcnc::process::ProcessRuntimeConfiguration& runtimeConfiguration);
	~GTNMotionControl(void);

	//MotionControl基类函数重写
	virtual const std::string& GetName() const;
	
	virtual void CreateMotor(lcnc::process::Axis eAxis, const toml::table& tAxis);
	virtual bool Connect();
	virtual bool Disconnect();
	virtual bool IsConnected();
	virtual bool Reboot();
	virtual bool Home();
	virtual bool Home(lcnc::process::Axis eAxis);
	virtual bool IsHomed();
	virtual bool IsHomed(lcnc::process::Axis eAxis);
	
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
	virtual void SetAxisHomePrm(lcnc::process::Axis eAxis, const toml::table& tableHome) override;
	virtual bool GetAxisHomePrm(lcnc::process::Axis eAxis, toml::table& tableHome) override;
	virtual bool SetAxisIndex(lcnc::process::Axis eAxis, int iIndex);
	
	virtual bool SetAxisIsRotary(lcnc::process::Axis eAxis, bool bRotary);
	virtual bool SetAxisResolution(lcnc::process::Axis eAxis, int iResolution);
	virtual bool SetAxisTubeDiamater(lcnc::process::Axis eAxis, double dTubeDiamater);
	virtual bool SetAxisVel(lcnc::process::Axis eAxis, double dVel);
	virtual bool SetAxisAcc(lcnc::process::Axis eAxis, double dAcc);
	virtual bool SetAxisDec(lcnc::process::Axis eAxis, double dDec);
	virtual bool SetAxisJerk(lcnc::process::Axis eAxis, double dSmoothTime);
	virtual bool SetAxisNegLimit(lcnc::process::Axis eAxis, double dNegLimit);
	virtual bool SetAxisPosLimit(lcnc::process::Axis eAxis, double dPosLimit);
	virtual bool SetAxisVelAccDecJerk(lcnc::process::Axis eAxis, double dVel, double dAcc, double dDec, double dJerk);
	virtual bool SetAxisSoftLimit(lcnc::process::Axis eAxis, double dNegLimit, double dPosLimit);

	virtual bool GetAxisIndex(lcnc::process::Axis eAxis, int& iIndex);
	
	virtual bool GetAxisIsRotary(lcnc::process::Axis eAxis, bool& bRotary);
	virtual bool GetAxisResolution(lcnc::process::Axis eAxis, int& iResolutionRatioRotation);
	virtual bool GetAxisTubeDiamater(lcnc::process::Axis eAxis, double& dTubeDiamater);
	virtual bool GetAxisVel(lcnc::process::Axis eAxis, double& dVel);
	virtual bool GetAxisAcc(lcnc::process::Axis eAxis, double& dAcc);
	virtual bool GetAxisDec(lcnc::process::Axis eAxis, double& dDec);
	virtual bool GetAxisJerk(lcnc::process::Axis eAxis, double& dSmoothTime);
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

	// IMotionCommandSink 切割管线入口（仅 GtnBufferedCommandSink 调用）
	virtual bool SetJumpAccJerk(const Tool&);
	virtual void JumpToIdleXYPosition(double dEndX, double dEndY, const Tool& curTool);
	virtual void ProLaserControl(bool, bool, const Tool&, bool);
	virtual bool InitCrd(const Tool& curTool);
	bool FlushToFifo();
	virtual bool PrfTrapAxis();
	virtual bool OffsetLineTo(const std::array<double, 5>& position,
	                          int dimension, const Tool& tool);
	/// Configure the XYZ/R1/R2-to-controller-axis mapping used by the five-axis
	/// interpolation coordinate system. Must be called before InitCrd().
	bool ConfigureCuttingAxes(const std::array<lcnc::process::Axis, 5>& axes, int dimension);
	/// 点位移动并等待到位，供 GTN 命令汇执行切割前的空程定位。
	bool MoveToPosition(lcnc::process::Axis axis, double velocity, double position);
	virtual void EndProgramCommand(const Tool&) {};
	virtual bool SendCommand();
	virtual bool IsBufferRunning(int iBufferIndex);
	virtual bool PauseBuffer(int iBufferIndex);
	virtual bool StopAllBuffer();
	virtual bool StopMovingCuttingHead() { return true; };
	virtual bool StartMovingCuttingHead(const Tool&) { return true; };

	virtual void ClearAxisState();
	virtual void StartCommand();

protected:
	bool MovePostion(lcnc::process::Axis aAxis,double dVel,double dPos);//移动位置
	bool PulseToMillimeter(lcnc::process::Axis aAxis, double dValue, double& dNewValue);
	bool MillimeterToPulse(lcnc::process::Axis aAxis, double dValue, double& dNewValue);

protected:
	bool IsReachPos(lcnc::process::Axis eAxis, bool bRelative, double dPos);

private:
	//获取当前工具轴索引的空程速度
	double GetAxisIdleVel(lcnc::process::Axis eAxis, const Tool& tool);
	void LogError(std::string strhandle, std::string command, std::string name, short error);
	bool ClearGSNAlarm(lcnc::process::Axis eAxis);//清除轴报警
	long AxisMask(lcnc::process::Axis eAxis) const;
	long AxisMaskByIndex(int iAxisIndex) const;
	void ReleaseHomeParameters();

public:
	// 指令汇所需的"切割管线"方法 —— 仅 GtnBufferedCommandSink 调用。
	// 这些方法把 GTN_BufXxx / GTN_LnXYZACEx / GTN_CrdDataEx 等写入 FIFO；mid-stream 不触发执行。
	void ResetProgramCommand();
	void SetCuttingAccJerk(const Tool&);

	// 实现在父类的"无调用"/"未用到"槽（保留以满足旧基类语义，但都是 no-op）。
	bool IsAxisStatusNormal(int& iFault);
	bool HaltMotor(lcnc::process::Axis) { return true; }
	int  GetPressureState() override { return 0; }
	bool GetIsPressureState() override { return true; }
	void SetIsPressureState(bool) override {}
	bool IsQueueActive() override { return true; }
	void LogError() override {}
	bool StopHome() override { return true; }
	bool SetAxisHomeBufferIndex(lcnc::process::Axis, int) override { return true; }
	bool GetAxisHomeBufferIndex(lcnc::process::Axis, int&) override { return true; }
	bool StopBuffer(int) override { return true; }
	bool AfterOpenComm() { return true; }
	bool ErrorOccurred() const override { return true; }
};
