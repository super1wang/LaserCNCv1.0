#ifndef _ACS_MOTION_CONTROL_
#define _ACS_MOTION_CONTROL_

#include "MotionControl.h"
#include "ACSC.h"
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <cmath>
#include <time.h>
#include <map>
#include <string>

#define     deviceDescription L"PCI-1730,BID#0"

using std::ios_base;
using std::ofstream;
using std::endl;
using std::map;
using std::string;
using std::ostringstream;

//#define MM 10000
#define DEBUG_MODE

struct AcsAxisData
{
	AcsAxisData(int iIndex, 
				bool bRotary,
				int  iResolution,
				int iHomeIndex, 
				string strHomeName, 
				string strRL, 
				string strLL, 
				string strEfac,
				double dVel,
				double dAcc,
				double dDec,
				double dJerk,
				double dNeg,
				double dPos,
				string strName):
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
	string	HomeName;
	string	SRLName;
	string	SLLName;
	string	Efac;
	double	Velocity;
	double	Acceleration;
	double	Deceleration;
	double	Jerk;
	double	NegLimit;
	double	PosLimit;
	string  Name;
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

	bool					m_bStop;					// 停止轴系
protected:
	int						m_iProgramBufferIndex;
	bool					m_bConnectFlag;				//是否连接的标志状态
	bool					m_bErrorOccurred;
	string					m_strName;
	HANDLE					m_hHandle;					//与ACS控制器通讯的句柄
	map<Axis,AcsAxisData>	m_mapMotorValue;

	string					m_strCommand;
	double					m_dPreX;					//上一个X的位置
	double					m_dPreY;					//上一个Y的位置

	double					m_dFrameLLX;
	double					m_dFrameLLY;
	double					m_dFrameURX;
	double					m_dFrameURY;
		
private:
	//string FindAxisSign(MotionControl::Axis);
public:
	ACSMotionControl(void);
	~ACSMotionControl(void);

	//MotionControl基类函数重写
	virtual const string& GetName() const;
	virtual void LogError();
	virtual void CreateMotor(Axis eAxis, const table& tAxis);
	virtual bool Connect();
	virtual bool Disconnect();
	virtual bool IsConnected();
	virtual bool Reboot();
	virtual bool Home();
	virtual bool Home(Axis eAxis);
	virtual bool IsHomed();
	virtual bool IsHomed(Axis eAxis);
	virtual bool StopHome();
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
	virtual bool SetAxisIndex(Axis eAxis, int iIndex);
	virtual bool SetAxisHomeBufferIndex(Axis eAxis, int iHomeIndex);
	virtual bool SetAxisIsRotary(Axis eAxis, bool bRotary);
	virtual bool SetAxisResolution(Axis eAxis, int iResolution);
	virtual bool SetAxisTubeDiamater(Axis eAxis, double dTubeDiamater);
	virtual bool SetAxisVel(Axis eAxis, double dVel);
	virtual bool SetAxisAcc(Axis eAxis, double dAcc);
	virtual bool SetAxisDec(Axis eAxis, double dDec);
	virtual bool SetAxisJerk(Axis eAxis, double dJerk);
	virtual bool SetAxisNegLimit(Axis eAxis, double dNegLimit);
	virtual bool SetAxisPosLimit(Axis eAxis, double dPosLimit);
	virtual bool SetAxisVelAccDecJerk(Axis eAxis, double dVel, double dAcc, double dDec, double dJerk);
	virtual bool SetAxisSoftLimit(Axis eAxis, double dNegLimit, double dPosLimit);

	virtual bool GetAxisIndex(Axis eAxis, int& iIndex);
	virtual bool GetAxisHomeBufferIndex(Axis eAxis, int& iIndex);
	virtual bool GetAxisIsRotary(Axis eAxis, bool& bRotary);
	virtual bool GetAxisResolution(Axis eAxis, int& iResolutionRatioRotation);
	virtual bool GetAxisTubeDiamater(Axis eAxis, double& dTubeDiamater);
	virtual bool GetAxisVel(Axis eAxis, double& dVel);
	virtual bool GetAxisAcc(Axis eAxis, double& dAcc);
	virtual bool GetAxisDec(Axis eAxis, double& dDec);
	virtual bool GetAxisJerk(Axis eAxis, double& dJerk);
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

	virtual bool StopQueue();//停止buffer9

	virtual bool SetFPos(Axis eAxis, double dPos);
	virtual bool GetFPos(Axis eAxis, double& dPos);
	//编写buffer9
	string  GetCuttingCommand();
	virtual void ResetProgramCommand();
	virtual bool SetJumpAccJerk(const Tool&);
	virtual void JumpToSetAFPos(const Tool& curTool);
	virtual void JumpToTrough(const Tool& curTool, double time);
	virtual void JumpToIdleHeight(const Tool& curTool, double dCompensate = 0);
	virtual void JumpToCuttingHeight(const Tool& curTool, double dCompensate = 0);
	virtual void JumpToIdleXYPosition(double dEndX, double dEndY, const Tool& curTool);
	virtual void ProLaserControl(bool, bool, const Tool&, bool);
	virtual void SetGO(const Tool&);
	virtual bool SetCuttingAccJerk(const Tool&);
	
	virtual void OffsetLineTo(double dEndX, double dEndY, const Tool& tool);
	virtual void OffsetArcTo(double dEndX, double dEndY, double dCenterX, double dCenterY, bool bClockwise,
		const Tool& tool, double dIncX, double dIncY);
	virtual void EndProgramCommand(const Tool&);
	virtual bool SendCommand();
	virtual bool IsOffsetCutting();
	virtual bool IsBufferRunning(int iBufferIndex);

	virtual bool HaltMotor(Axis eMotor);

	virtual bool CheckBuffer(int iBufferIndex, string& strCommand);
	virtual bool RunBuffer(int iBufferIndex);
	virtual bool PauseBuffer(int iBufferIndex);
	virtual bool GetBufferState(int iBufferIndex, int& iState);
	virtual bool LoadCommandAndRunBuffer(int iBufferIndex, string strCommand, int iTimeout);
	virtual bool StopBuffer(int iBufferIndex);
	virtual bool StopAllBuffer();
	virtual bool SetMFLAGSValue(Axis, int);
	//随动
	virtual bool StopMovingCuttingHead();
	virtual bool StartMovingCuttingHead(const Tool&);
	virtual bool MoveZCutting(double dAbsolutePos);

	//GSN PWM
	virtual bool GSN_SetLaserParameterApplication(double dFrequence, double dPulse, double dDelay) { return true; };
	virtual bool GSN_SetLaserEnablePro(bool bState) { return true; };
	virtual bool GSN_LaserOnStatus(int& iState) { return true; };

	//监控控制器状态
	virtual bool IsAxisStatusNormal(int& iFault);

	virtual bool IsQueueFull();//无调用
	virtual bool IsQueueEmpty();//无调用
	virtual bool IsQueueActive();//待筛查
	virtual bool ClearQueue();//无调用
	//virtual bool CanCutting(CUTTING_CODE &);
	virtual bool ErrorOccurred() const;//无调用
	virtual bool Punch(double fdDwellTime);//暂时没用到，返回true
	
	virtual void BeginACSSegment(const Tool&);	// ACS中，SEGMENT运动必须在之前设置加速度和加加速度
	virtual void BeginACSSegmentSimple(const Tool& tool);//界面导出有用到
	virtual void EndProgramCommandSimple(const Tool& tool);//界面导出有用到
	virtual void OffsetLineToSimple(double dEndX, double dEndY, 
		const Tool& tool);//界面导出有用到
	virtual void OffsetArcToSimple(double dEndX, double dEndY, 
		double dCenterX, double dCenterY, 
		bool bClockwise, const Tool& tool,
		double dIncX, double dIncY);//界面导出有用到
	virtual void OffsetArc2To(double dEndX, double dEndY, 
		double dCenterX, double dCenterY,
		double dAngle,const Tool& tool);//没有检索到有调用
	virtual void JumpToSimple(double,double, const Tool&, double, double);//界面导出有用到
	virtual void ProLaserControlSimple(bool, bool , const Tool&);//界面导出有用到

	//virtual bool StartMovingCuttingHead(const Tool&);
	virtual int GetPressureState();       //气压监控接口   没有被调用
    virtual bool GetIsPressureState();                      //获取是否需要气压监控标识   没有被调用
	virtual void SetIsPressureState(bool);                  //设置是否需要气压监控标识   没有被调用

#pragma region FlightCutting
	virtual void BeginACSSegmentForFlightCutting(const Tool& tool);
	virtual void OffsetFlightLineTo(double dEndX, double dEndY, const Tool& tool, bool bPolyGuide);
	virtual void OffsetFlightArcTo(double dEndX, double dEndY, double dCenterX, double dCenterY, bool bClockwise, const Tool& tool, double dIncX, double dIncY, bool bPolyGuide);
	virtual void OffsetFlightArc2To(double dCenterX, double dCenterY, double dAngle, const Tool& tool, bool bPolyGuide);
	virtual void EndProgramCommandForFlightCutting(const Tool& tool);
	virtual bool LoadAndCompileBuffer(int);		//导入指令到指定的Buffer，并编译（NOT RUN）
	virtual bool RunBufferForFlightCutting(int);    //直接Run指定的Buffer，请确保其已编译过
#pragma endregion FlightCutting

	
	bool LoadApplication(string);//SPI导入调用，但是SPI已经禁用
protected:
	bool SetDiamaterXVEL(Axis eAxis, double dValue);	// 设置管径后设置限速

	bool RunBufferTillEnd(int iBufferIndex,int iTimeout);
	bool AfterOpenComm();
	void DeleteOtherConnections();
    
	bool WriteEFAC(Axis eAxis, int iEfac);
	bool ReadEFAC(Axis eAxis, int &iEfac);
	bool ControllerSaveToFlash(Axis eAxis);
	bool AcscReadReal(const string, double&);
	bool AcscWriteReal(const string, double);
	bool AcscReadInt(const string, int&);
	bool AcscWriteInt(const string, int);

	bool IsReachPos(Axis eAxis, bool bRelative, double dPos);

private:
	bool GetFault(int iAxis, int &Fault);
	//获取当前工具轴索引的空程速度
	double GetAxisIdleVel(Axis eAxis, const Tool& tool);
	bool IsHomeBufferRunning();

	//固高控制卡使用
	virtual bool InitCrd(const Tool& curTool) { return true; };
	virtual bool PrfTrapAxis() { return true; };
};

#endif