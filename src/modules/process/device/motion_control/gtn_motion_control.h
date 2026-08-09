#ifndef _GSN_MOTION_CONTROL_
#define _GSN_MOTION_CONTROL_

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
#include <string>

//#define     deviceDescription L"PCI-1730,BID#0"

//using std::ios_base;
//using std::ofstream;
//using std::endl;
//using std::map;
//using std::string;
//using std::ostringstream;

//#define MM 10000
//#define DEBUG_MODE1

struct GSNAxisData
{
	GSNAxisData(int iIndex,
		bool bRotary,
		int  iResolution,
		//int iHomeIndex,
		//string strHomeName, 
		//string strRL, 
		//string strLL, 
		//string strEfac,
		double dVel,
		double dAcc,
		double dDec,
		double dSmoothTime,
		double dNeg,
		double dPos,
		//int    dConversion,
		string strName,
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
	//string	HomeName;
	//string	SRLName;
	//string	SLLName;
	//string	Efac;
	double	Velocity;
	double	Acceleration;
	double	Deceleration;
	double	SmoothTime;
	double	NegLimit;
	double	PosLimit;
	//int     Conversion;
	string  Name;
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

	bool					m_bStop;					// 停止轴系
protected:
	bool					m_bConnectFlag;				//是否连接的标志状态
	bool					m_bErrorOccurred;
	string					m_strName;
	map<Axis, GSNAxisData>	m_mapMotorValue;

	//string					m_strCommand;
	double					m_dPreX;					//上一个X的位置
	double					m_dPreY;					//上一个Y的位置
	double					m_dPreZ;					//上一个Z的位置
	double					m_dPreR1;					//上一个第一旋转轴的位置
	double					m_dPreR2;					//上一个第二旋转轴的位置
	bool                    m_hasPreviousCuttingPose{false};
	bool                    m_cuttingCoordinateReady{false};
	// 五维坐标系的维度顺序。由 GtnBufferedCommandSink 按 AxisMap 注入，
	// 因而可覆盖 AC / BC 转台及摆头，而非硬编码物理轴号。
	std::array<Axis, 5>     m_cuttingAxes{Axis::X, Axis::Y, Axis::Z, Axis::A, Axis::C};

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
	//string FindAxisSign(MotionControl::Axis);
public:
	GTNMotionControl(lcnc::process::ProcessSettingsService& settings,
	                 lcnc::process::ProcessRuntimeConfiguration& runtimeConfiguration);
	~GTNMotionControl(void);

	//MotionControl基类函数重写
	virtual const string& GetName() const;
	
	virtual void CreateMotor(Axis eAxis, const table& tAxis);
	virtual bool Connect();
	virtual bool Disconnect();
	virtual bool IsConnected();
	virtual bool Reboot();
	virtual bool Home();
	virtual bool Home(Axis eAxis);
	virtual bool IsHomed();
	virtual bool IsHomed(Axis eAxis);
	
	virtual bool Enable();
	virtual bool Enable(Axis eAxis);
	virtual bool Disable();
	virtual bool Disable(Axis eAxis);
	virtual bool IsEnabled();
	virtual bool IsEnabled(Axis eAxis);
	virtual bool SetAxisEnable(Axis eAxis, bool bEnable);
	virtual bool Jog(Axis eAxis, bool bDirection, double dVel);
	virtual bool MoveRelative(Axis eAxis, double dPos, double dVel);
	virtual bool MoveAbsolute(Axis eAxis, double dPos, double dVel);
	virtual bool MoveMRelative(vector<Axis> vAxis, vector<double> dPos, double dVel);
	virtual bool MoveMAbsolute(vector<Axis> vAxis, vector<double> dPos, double dVel);
	virtual bool StopMotion();
	virtual bool StopMotion(Axis eAxis);
	virtual bool IsAxisMoving();
	virtual bool IsAxisMoving(Axis eAxis);
	virtual bool GetActualPos(Axis eAxis, double& dFPos);
	virtual bool GetFeedbackPos(Axis eAxis, double& dFPos);
	virtual bool SetFPosition(Axis eAxis, double dPos);

	// 轴系基础参数设置
	virtual void SetAxisHomePrm(Axis eAxis, const toml::table& tableHome) override;
	virtual bool GetAxisHomePrm(Axis eAxis, toml::table& tableHome) override;
	virtual bool SetAxisIndex(Axis eAxis, int iIndex);
	
	virtual bool SetAxisIsRotary(Axis eAxis, bool bRotary);
	virtual bool SetAxisResolution(Axis eAxis, int iResolution);
	virtual bool SetAxisTubeDiamater(Axis eAxis, double dTubeDiamater);
	virtual bool SetAxisVel(Axis eAxis, double dVel);
	virtual bool SetAxisAcc(Axis eAxis, double dAcc);
	virtual bool SetAxisDec(Axis eAxis, double dDec);
	virtual bool SetAxisJerk(Axis eAxis, double dSmoothTime);
	virtual bool SetAxisNegLimit(Axis eAxis, double dNegLimit);
	virtual bool SetAxisPosLimit(Axis eAxis, double dPosLimit);
	virtual bool SetAxisVelAccDecJerk(Axis eAxis, double dVel, double dAcc, double dDec, double dJerk);
	virtual bool SetAxisSoftLimit(Axis eAxis, double dNegLimit, double dPosLimit);

	virtual bool GetAxisIndex(Axis eAxis, int& iIndex);
	
	virtual bool GetAxisIsRotary(Axis eAxis, bool& bRotary);
	virtual bool GetAxisResolution(Axis eAxis, int& iResolutionRatioRotation);
	virtual bool GetAxisTubeDiamater(Axis eAxis, double& dTubeDiamater);
	virtual bool GetAxisVel(Axis eAxis, double& dVel);
	virtual bool GetAxisAcc(Axis eAxis, double& dAcc);
	virtual bool GetAxisDec(Axis eAxis, double& dDec);
	virtual bool GetAxisJerk(Axis eAxis, double& dSmoothTime);
	virtual bool GetAxisNegLimit(Axis eAxis, double& dNegLimit);
	virtual bool GetAxisPosLimit(Axis eAxis, double& dPosLimit);
	virtual bool GetAxisVelAccDecJerk(Axis eAxis, double& dVel, double& dAcc, double& dDec, double& dJerk);
	virtual bool GetAxisSoftLimit(Axis eAxis, double& dNegLimit, double& dPosLimit);
	virtual void ReadAxisSoftLimit(Axis eAxis, double& dNegLimit, double& dPosLimit);

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
	virtual void JumpToIdleHeight(const Tool& curTool, double dCompensate = 0);
	virtual void JumpToCuttingHeight(const Tool& curTool, double dCompensate = 0);
	virtual void JumpToIdleXYPosition(double dEndX, double dEndY, const Tool& curTool);
	virtual void ProLaserControl(bool, bool, const Tool&, bool);
	virtual bool InitCrd(const Tool& curTool);
	bool FlushToFifo();
	virtual bool PrfTrapAxis();
	virtual void OffsetLineTo(double dEndX, double dEndY, double dEndZ,
	                          double dEndR1, double dEndR2, const Tool& tool);
	/// Configure the XYZ/R1/R2-to-controller-axis mapping used by the five-axis
	/// interpolation coordinate system. Must be called before InitCrd().
	void ConfigureCuttingAxes(Axis x, Axis y, Axis z, Axis r1, Axis r2);
	/// 点位移动并等待到位，供 GTN 命令汇执行切割前的空程定位。
	bool MoveToPosition(Axis axis, double velocity, double position);
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
	bool MovePostion(Axis aAxis,double dVel,double dPos);//移动位置
	bool PulseToMillimeter(Axis aAxis, double dValue, double& dNewValue);
	bool MillimeterToPulse(Axis aAxis, double dValue, double& dNewValue);

protected:
	bool IsReachPos(Axis eAxis, bool bRelative, double dPos);

private:
	//获取当前工具轴索引的空程速度
	double GetAxisIdleVel(Axis eAxis, const Tool& tool);
	void LogError(string strhandle, string command, string name, short error);
	bool ClearGSNAlarm(Axis eAxis);//清除轴报警
	long AxisMask(Axis eAxis) const;
	long AxisMaskByIndex(int iAxisIndex) const;
	void ReleaseHomeParameters();

public:
	// 指令汇所需的"切割管线"方法 —— 仅 GtnBufferedCommandSink 调用。
	// 这些方法把 GTN_BufXxx / GTN_LnXYZACEx / GTN_CrdDataEx 等写入 FIFO；mid-stream 不触发执行。
	void ResetProgramCommand();
	void SetCuttingAccJerk(const Tool&);

	// 实现在父类的"无调用"/"未用到"槽（保留以满足旧基类语义，但都是 no-op）。
	bool IsAxisStatusNormal(int& iFault);
	bool HaltMotor(Axis) { return true; }
	int  GetPressureState() override { return 0; }
	bool GetIsPressureState() override { return true; }
	void SetIsPressureState(bool) override {}
	bool IsQueueActive() override { return true; }
	void LogError() override {}
	bool StopHome() override { return true; }
	bool SetAxisHomeBufferIndex(Axis, int) override { return true; }
	bool GetAxisHomeBufferIndex(Axis, int&) override { return true; }
	bool StopBuffer(int) override { return true; }
	bool AfterOpenComm() { return true; }
	bool ErrorOccurred() const override { return true; }
};

#endif
