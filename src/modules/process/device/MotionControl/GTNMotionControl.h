#ifndef _GSN_MOTION_CONTROL_
#define _GSN_MOTION_CONTROL_

#include "MotionControl.h"
//#include "ACSC.h"
#include "gts.h"
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <cmath>
#include <time.h>
#include <map>
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
		THomePrm *pTHomePrm) :
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
		pTHomePrm(nullptr){}
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
	GTNMotionControl(void);
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

	virtual bool SetFPos(Axis eAxis, double dPos);
	virtual bool GetFPos(Axis eAxis, double& dPos);

	//切割流程
	string  GetCuttingCommand();
	virtual bool SetJumpAccJerk(const Tool&);
	virtual void JumpToSetAFPos(const Tool& curTool);
	
	virtual void JumpToIdleHeight(const Tool& curTool, double dCompensate = 0);
	virtual void JumpToCuttingHeight(const Tool& curTool, double dCompensate = 0);
	virtual void JumpToIdleXYPosition(double dEndX, double dEndY, const Tool& curTool);
	virtual void ProLaserControl(bool, bool, const Tool&, bool);
	
	virtual bool InitCrd(const Tool& curTool);
	bool FlushToFifo();       // 将软件前瞻缓冲区刷入硬件FIFO，若硬件FIFO满则提前启动插补
	virtual bool PrfTrapAxis();
	virtual void OffsetLineTo(double dEndX, double dEndY, const Tool& tool);
	virtual void OffsetArcTo(double dEndX, double dEndY, double dCenterX, double dCenterY, bool bClockwise, const Tool& tool, double dIncX, double dIncY);
	virtual void EndProgramCommand(const Tool&) {};
	virtual bool SendCommand();
	virtual bool IsOffsetCutting();
	virtual bool IsBufferRunning(int iBufferIndex);
	

	virtual bool CheckBuffer(int iBufferIndex, string& strCommand);
	virtual bool RunBuffer(int iBufferIndex);
	virtual bool PauseBuffer(int iBufferIndex);
	virtual bool GetBufferState(int iBufferIndex, int& iState);
	virtual bool LoadCommandAndRunBuffer(int iBufferIndex, string strCommand, int iTimeout);
	
	virtual bool StopAllBuffer();
	virtual bool SetMFLAGSValue(Axis, int);
	//随动
	virtual bool StopMovingCuttingHead() { return true; };
	virtual bool StartMovingCuttingHead(const Tool&) { return true; };
	virtual bool MoveZCutting(double dAbsolutePos) { return true; };//未用到内容为空

	//GSN PWM
	virtual bool GSN_SetLaserParameterApplication(double dFrequence, double dPulse, double dDelay);
	virtual bool GSN_SetLaserEnablePro(bool bState);
	virtual bool GSN_LaserOnStatus(int& iState);
	
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

protected:
#pragma region FlightCutting
	virtual void BeginACSSegmentForFlightCutting(const Tool& tool) {};
	virtual void OffsetFlightLineTo(double dEndX, double dEndY, const Tool& tool, bool bPolyGuide) {};
	virtual void OffsetFlightArcTo(double dEndX, double dEndY, double dCenterX, double dCenterY, bool bClockwise, const Tool& tool, double dIncX, double dIncY, bool bPolyGuide) {};
	virtual void OffsetFlightArc2To(double dCenterX, double dCenterY, double dAngle, const Tool& tool, bool bPolyGuide) {};
	virtual void EndProgramCommandForFlightCutting(const Tool& tool) {};
	virtual bool LoadAndCompileBuffer(int) { return true; };		//导入指令到指定的Buffer，并编译（NOT RUN）
	virtual bool RunBufferForFlightCutting(int) { return true; };    //直接Run指定的Buffer，请确保其已编译过
#pragma endregion FlightCutting

	//数据导出
	virtual bool ErrorOccurred() const { return true; };//无调用
	virtual bool Punch(double fdDwellTime) { return true; };//暂时没用到，返回true
	virtual void ResetProgramCommand() {};
	virtual void BeginACSSegment(const Tool&) {};	// ACS中，SEGMENT运动必须在之前设置加速度和加加速度
	virtual void BeginACSSegmentSimple(const Tool& tool) {};//界面导出有用到
	virtual void EndProgramCommandSimple(const Tool& tool) {};//界面导出有用到
	virtual void OffsetLineToSimple(double dEndX, double dEndY, const Tool& tool) {};//界面导出有用到
	virtual void OffsetArcToSimple(double dEndX, double dEndY,double dCenterX, double dCenterY,bool bClockwise, const Tool& tool,double dIncX, double dIncY){};//界面导出有用到
	virtual void JumpToSimple(double, double, const Tool&, double, double) {};//界面导出有用到
	virtual void ProLaserControlSimple(bool, bool, const Tool&) {};//界面导出有用到

	//没有被调用
	virtual int GetPressureState() { return 0; };       //气压监控接口   没有被调用
	virtual bool GetIsPressureState() { return true; };                      //获取是否需要气压监控标识   没有被调用
	virtual void SetIsPressureState(bool) {};                  //设置是否需要气压监控标识   没有被调用
	virtual void JumpToTrough(const Tool& curTool, double time) {};//未用到，内用为空
	virtual bool IsQueueFull() { return true; };//无调用
	virtual bool IsQueueEmpty() { return true; };//无调用
	virtual bool IsQueueActive() { return true; };//待筛查
	virtual bool ClearQueue() { return true; };//无调用
	//virtual bool CanCutting(CUTTING_CODE &);
	//ACS适用
	virtual void LogError() {};
	virtual bool StopHome() { return true; };
	virtual bool SetAxisHomeBufferIndex(Axis eAxis, int iHomeIndex) { return true; };
	virtual bool GetAxisHomeBufferIndex(Axis eAxis, int& iIndex) { return true; };
	bool RunBufferTillEnd(int iBufferIndex, int iTimeout) { return true; };
	virtual bool StopBuffer(int iBufferIndex) { return true; };
	virtual bool StopQueue() { return true; };//停止buffer9
	bool AfterOpenComm() { return true; };
	virtual bool SetCuttingAccJerk(const Tool&) { return true; };

	virtual bool IsAxisStatusNormal(int& iFault);//监控控制器状态
	virtual bool HaltMotor(Axis eMotor) { return true; };
	bool LoadApplication(string) { return true; };//SPI导入调用，但是SPI已经禁用

	bool ControllerSaveToFlash(Axis eAxis) { return true; };
	bool AcscReadReal(const string, double&) { return true; };
	bool AcscWriteReal(const string, double) { return true; };
	bool AcscReadInt(const string, int&) { return true; };
	bool AcscWriteInt(const string, int) { return true; };

	virtual void OffsetArc2To(double dEndX, double dEndY,
		double dCenterX, double dCenterY,
		double dAngle, const Tool& tool) {};//没有检索到有调用
	
};

#endif
