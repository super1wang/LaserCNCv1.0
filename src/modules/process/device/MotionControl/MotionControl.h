#pragma once
#include <vector>
#include <string>
#include <windows.h>
#include <regex>	//正则表达式库
#include "ToolFactory.h"
#include "Settings.h"
#include "MessageModule.h"

using std::vector;
using std::string;

struct DigitalIOData
{
	QString qstrID		= "";		// 名称
	string	strIndex	= "";		// 索引，buffer用
	string	strPort		= "";		// 地址位
	int		iPort;					// 地址位
	int		iIO			= 8;		// 数字量所在位
	bool	bExpand		= false;	// 拓展模块
	bool	bInversion	= false;	// 信号取反
};

struct AnalogIOData
{
	QString qstrID		= "";		// 名称
	string	strIndex	= "";		// 索引，buffer用
	string	strPort		= "";		// 地址位
	int		iPort;					// 地址位
	bool	bExpand		= false;	// 拓展模块
};

class  MotionControl
{
public:
	vector<Axis>					m_vecMotors;

	map<DigitalIN,	DigitalIOData>	m_mapDigitalIN;
	map<DigitalOUT, DigitalIOData>  m_mapDigitalOUT;
	map<AnalogIN,	AnalogIOData>	m_mapAnalogIN;
	map<AnalogOUT,	AnalogIOData>	m_mapAnalogOUT;

	map<QString,	DigitalIOData>	m_mapqDigitalIN;
	map<QString,	DigitalIOData>  m_mapqDigitalOUT;
	map<QString,	AnalogIOData>	m_mapqAnalogIN;
	map<QString,	AnalogIOData>	m_mapqAnalogOUT;

public:
	static const std::regex regex_DigitalIO;	// 匹配格式：(-) （N） 数字 . 数字 (数字)	，如N0.1、N1.23、-N0.1
	static const std::regex regex_AnalogIO;		// 匹配格式：(-) （N） 数字 (数字)		，如N1、N12、-N1

protected:
	void rebuildIOMap(int iType);	// 重构IO的qstring map

public:
	void rebuildAxes();				// 重构建轴系组
	virtual bool IsMotorCreated(Axis eAxis);
	virtual void CreateMotor(Axis eAxis, const table& tAxis) = 0;// 从setting初始化map
	// 参数下发层，父类直接实现
	virtual void SetMotionControlTable();
	virtual void SetMotionControlTable(const table& tMotion);
	virtual void SetAxisTable(Axis eAxis, const table& tMotion);
	virtual void SetAxisHomePrm(Axis eAxis, const table& tHome) {}
	virtual bool GetAxisHomePrm(Axis eAxis, table& tHome) { return false; }
	virtual void SetPipeDiameterTable();
	virtual void SetPipeDiameterTable(const table& tAxis);
	virtual void SetDigitalTable();
	virtual void SetDigitalTable(const table& tDigital);
	virtual void SetAnalogTable();
	virtual void SetAnalogTable(const table& tAnalog);
	virtual ErrorCode SetLaserParameterTable();
	virtual ErrorCode SetLaserParameterTable(const table& tLaser);

	// IO接口转义层
	virtual bool DigitalOutputSet	(DigitalOUT	eIndex,	int		iValue, bool bLogError = false);
	virtual bool DigitalOutputGet	(DigitalOUT	eIndex,	int&	iValue, bool bLogError = false);
	virtual bool DigitalInputGet	(DigitalIN	eIndex,	int&	iValue, bool bLogError = false);
	virtual bool AnalogOutputSet	(AnalogOUT	eIndex,	double  dValue, bool bLogError = false);
	virtual bool AnalogOutputGet	(AnalogOUT	eIndex,	double& dValue, bool bLogError = false);
	virtual bool AnalogInputGet		(AnalogIN	eIndex,	double& dValue, bool bLogError = false);

	virtual bool DigitalOutputSet	(QString	qstrName, int		iValue, bool bLogError = false);
	virtual bool DigitalOutputGet	(QString	qstrName, int&		iValue, bool bLogError = false);
	virtual bool DigitalInputGet	(QString	qstrName, int&		iValue, bool bLogError = false);
	virtual bool AnalogOutputSet	(QString	qstrName, double	dValue, bool bLogError = false);
	virtual bool AnalogOutputGet	(QString	qstrName, double&	dValue, bool bLogError = false);
	virtual bool AnalogInputGet		(QString	qstrName, double&	dValue, bool bLogError = false);

public:
	virtual void LogError() = 0;	// 记录控制器错误信息至日志
	/*
	* 函数名：	Connection
	* 函数介绍：与控制器建立连接
	* 输入参数：
	*     无
	* 返回值：
	*     true：	连接成功；
	*     false：连接失败。
	*/
	virtual bool Connect() = 0;

	/*
	* 函数名：	Reboot
	* 函数介绍：重启控制器并连接
	* 输入参数：
	*     无
	* 返回值：
	*     true：	连接成功；
	*     false：连接失败。
	*/
	virtual bool Reboot() = 0;

	/*
	* 函数名：	Disconnection
	* 函数介绍：与控制器断开连接
	* 输入参数：
	*     无
	* 返回值：
	*     true：	断开连接成功；
	*     false：断开连接失败。
	*/
	virtual bool Disconnect() = 0;
	/*
	* 函数名：	Home
	* 函数介绍：指定轴归零
	* 输入参数：
	*  eAxis：轴的枚举类型（Axis_X1、Axis_Y1、Axis_Z）
	* 返回值：
	*     true：归零命令发送成功；
	*     false：归零命令发送失败。
	*/
	virtual bool Home(Axis eAxis) = 0;

	/*
	* 函数名：	Home
	* 函数介绍：所有设备轴归零
	* 输入参数：
	*     无
	* 返回值：
	*     true：归零命令发送成功；
	*     false：归零命令发送失败。
	*/
	virtual bool Home() = 0;
	
	/*
	* 函数名：	StopHome
	* 函数介绍：停止Home
	* 输入参数：
	*     无
	* 返回值：
	*     true：停止Home成功；
	*     false：停止Home失败。	
	*/
	virtual bool StopHome() = 0;

	/*
	* 函数名：	IsHomed
	* 函数介绍：判断所有轴Home是否完成
	* 输入参数：
	*     无
	* 返回值：
	*     true：	完成；
	*     false：未完成。
	*/
	virtual bool IsHomed() = 0;

	/*
	* 函数名：	IsHomed
	* 函数介绍：判断指定轴Home是否完成
	* 输入参数：
	*   eAxis：轴的枚举类型（Axis_X1、Axis_Y1、Axis_Z）
	* 返回值：
	*     true：	完成；
	*     false：未完成。
	*/
	virtual bool IsHomed(Axis eAxis) = 0;
	
	/*
	* 函数名：	Enable
	* 函数介绍：给指定轴添加使能
	* 输入参数：
	*    eAxis：轴的枚举类型（Axis_X1、Axis_Y1、Axis_Z）
	* 返回值：
	*     true：	使能成功；
	*     false：使能失败。
	*/
	virtual bool Enable(Axis eAxis) = 0;

	/*
	* 函数名称：	Enable
	* 函数介绍：  给指所有轴添加使能
	* 输入参数：  空
	* 返回值：
	*           true：	使能成功；
	*           false：使能失败。
	*/
	virtual bool Enable() = 0;
	/*
	* 函数名：	Disable
	* 函数介绍：给指定轴去使能
	* 输入参数：
	*    eAxis：轴的枚举类型（Axis_X1、Axis_Y1、Axis_Z）
	* 返回值：
	*     true：	去使能成功；
	*     false：去使能失败。
	*/
	virtual bool Disable(Axis eAxis) = 0;

	/*
	* 函数名称：	Disable
	* 函数介绍：  给定所有轴去使能
	* 输入参数：  空
	* 返回值：
	*           true：	去使能成功；
	*           false：去使能失败。
	*/
	virtual bool Disable() = 0;

	/*
	* 函数名：	Jog
	* 函数介绍：指定轴持续运动
	* 输入参数：
	*    eAxis：轴的枚举类型（Axis_X1、Axis_Y1、Axis_Z）
	*    bDirection:运动方向（ture：正方向，false：负方向）
	* 返回值：
	*     true：	Jog命令发送成功；
	*     false：Jog命令发送失败。
	*/
	virtual bool Jog(Axis eAxis, bool bDirection, double dVel) = 0;

	/*
	* 函数名：	MoveRelative
	* 函数介绍：指定轴移动一个相对的距离
	* 输入参数：
	*	eAxis：轴的枚举类型（Axis_X1、Axis_Y1、Axis_Z）
	*	dPos：相对距离
	*   dVel: 速度
	* 返回值：
	*     true：	移动命令发送成功；
	*     false：移动命令发送失败。
	*/
	virtual bool MoveRelative(Axis eAxis, double dPos, double dVel) = 0;
	
	/*
	* 函数名：	MoveMRelative
	* 函数介绍：指定多个轴移动一个相对的距离
	* 输入参数：
	*	vAxes：轴枚举类型的容器
	*	dPos：对不同轴相对位置的容器
	* 返回值：
	*     true：	移动命令发送成功；
	*     false：移动命令发送失败。
	*/
	virtual bool MoveMRelative(vector<Axis> vAxis, vector<double> vPos, double dVel) = 0;

	/*
	* 函数名：	MoveAbsolute
	* 函数介绍：指定轴移动到一个绝对位置
	* 输入参数：
	*	eAxis：轴的枚举类型（Axis_X1、Axis_Y1、Axis_Z）
	*	dPos：轴要运动到的位置
	* 返回值：
	*     true：	移动命令发送成功；
	*     false：移动命令发送失败。
	*/
	virtual bool MoveAbsolute(Axis eAxis, double dPos, double dVel) = 0;

	/*
	* 函数名：	MoveMAbsolute
	* 函数介绍：指定多个轴移动到一个绝对位置
	* 输入参数：
	*	eAxes：轴枚举类型的容器
	*	dPos：对不同轴相对位置的容器
	* 返回值：
	*     true：	移动命令发送成功；
	*     false：移动命令发送失败。
	*/
	virtual bool MoveMAbsolute(vector<Axis> vAxis, vector<double> vPos, double dVel) = 0;

	/*
	* 函数名：	StopMotion
	* 函数介绍：停止指定轴运动
	* 输入参数：
	*	eAxis：轴的枚举类型（Axis_X1、Axis_Y1、Axis_Z）
	* 返回值：
	*     true：停止命令发送成功；
	*     false：停止命令发送失败。
	*/
	virtual bool StopMotion(Axis eAxis) = 0;

	/*
	* 函数名：	StopMotionM
	* 函数介绍：停止所有轴的运动
	* 输入参数：
	*	无
	* 返回值：
	*     true：	停止命令发送成功；
	*     false：停止命令发送失败。
	*/
	virtual bool StopMotion() = 0;

	/*
	* 函数名：	IsAxisMoving
	* 函数介绍：判断指定轴是否正在运动
	* 输入参数：
	*	eAxis：轴的枚举类型（Axis_X1、Axis_Y1、Axis_Z）
	* 返回值：
	*     true：	轴正在运动；
	*     false：轴不在运动。
	*/
	virtual bool IsAxisMoving(Axis eAxis) = 0;

	/*
	* 函数名：	IsAxisMoving
	* 函数介绍：判断所有轴是否正在运动
	* 输入参数：
	*	无
	* 返回值：
	*     true：	轴正在运动；
	*     false：轴不在运动。
	*/
	virtual bool IsAxisMoving() = 0;

	/*
	* 函数名：	GetActualPos
	* 函数介绍：得到指定轴的当前位置
	* 输入参数：
	*	eAxis：轴的枚举类型（Axis_X1、Axis_Y1、Axis_Z）
	* 返回值：
	*     指定轴的当前位置
	*/
	virtual bool GetActualPos(Axis eAxis, double& dFPos) = 0;
	virtual bool GetFeedbackPos(Axis eAxis, double& dFPos) = 0;

	virtual bool SetAxisIndex(Axis eAxis, int iIndex) = 0;
	virtual bool SetAxisHomeBufferIndex(Axis eAxis, int iHomeIndex) = 0;
	virtual bool SetAxisIsRotary(Axis eAxis, bool bRotary) = 0;
	virtual bool SetAxisResolution(Axis eAxis, int iResolution) = 0;
	virtual bool SetAxisTubeDiamater(Axis eAxis, double dTubeDiamater) = 0;
	virtual bool SetAxisVel(Axis eAxis, double dVel) = 0;
	virtual bool SetAxisAcc(Axis eAxis, double dAcc) = 0;
	virtual bool SetAxisDec(Axis eAxis, double dDec) = 0;
	virtual bool SetAxisJerk(Axis eAxis, double dJerk) = 0;
	virtual bool SetAxisNegLimit(Axis eAxis, double dNegLimit) = 0;
	virtual bool SetAxisPosLimit(Axis eAxis, double dPosLimit) = 0;
	virtual bool SetAxisVelAccDecJerk(Axis eAxis, double dVel, double dAcc, double dDec, double dJerk) = 0;
	virtual bool SetAxisSoftLimit(Axis eAxis, double dNegLimit, double dPosLimit) = 0;

	virtual bool GetAxisIndex(Axis eAxis, int& iIndex) = 0;
	virtual bool GetAxisHomeBufferIndex(Axis eAxis, int& iIndex) = 0;
	virtual bool GetAxisIsRotary(Axis eAxis, bool& bRotary) = 0;
	virtual bool GetAxisResolution(Axis eAxis, int& iResolution) = 0;
	virtual bool GetAxisTubeDiamater(Axis eAxis, double& dTubeDiamater) = 0;
	virtual bool GetAxisVel(Axis eAxis, double& dVel) = 0;
	virtual bool GetAxisAcc(Axis eAxis, double& dAcc) = 0;
	virtual bool GetAxisDec(Axis eAxis, double& dDec) = 0;
	virtual bool GetAxisJerk(Axis eAxis, double& dJerk) = 0;
	virtual bool GetAxisNegLimit(Axis eAxis, double& dNegLimit) = 0;
	virtual bool GetAxisPosLimit(Axis eAxis, double& dPosLimit) = 0;
	virtual bool GetAxisVelAccDecJerk(Axis eAxis, double& dVel, double& dAcc, double& dDec, double& dJerk) = 0;
	virtual bool GetAxisSoftLimit(Axis eAxis, double& dNegLimit, double& dPosLimit) = 0;
	virtual void ReadAxisSoftLimit(Axis eAxis, double& dNegLimit, double& dPosLimit) = 0;

	virtual bool DigitalOutputSet	(DigitalIOData&	IOData,	int		iValue,	bool bLogError = false) = 0;
	virtual bool DigitalOutputGet	(DigitalIOData&	IOData,	int&	iValue, bool bLogError = false) = 0;
	virtual bool DigitalInputGet	(DigitalIOData&	IOData,	int&	iValue, bool bLogError = false) = 0;
	virtual bool AnalogOutputSet	(AnalogIOData&	IOData,	double  dValue, bool bLogError = false) = 0;
	virtual bool AnalogOutputGet	(AnalogIOData&	IOData,	double& dValue, bool bLogError = false) = 0;
	virtual bool AnalogInputGet		(AnalogIOData&	IOData,	double& dValue, bool bLogError = false) = 0;

	//virtual bool IsQueueFull() = 0;

	/*
	* 函数名：IsQueueEmpty
	* 功能描述：判断缓冲区队列是否为空
	* 输入参数：
	*	无
	* 返回值：
	*	true：队列为空；
	*	false：队列不为空。
	*/
	//virtual bool IsQueueEmpty() = 0;
	virtual bool IsQueueActive() = 0;

	/*
	* 函数名：SetShutterOnOffWaitTime
	* 功能描述：设置激光开关等待时间
	* 输入参数：
	*	dBeforeOn：出光前延时
	*	dAfterOn：出光后延时
	*	dBeforeOff：关光前延时
	*	dAfterOff：关光后延时
	*   dBlowDelay: 吹气开启后的延时时间
	* 返回值：
	*	true：设置成功；
	*	false：设置失败。
	*/
	virtual bool SetShutterOnOffWaitTime(double dBeforeOn, double dAfterOn, 
	
									 double dBeforeOff, double dAfterOff, double dBlowDelay) = 0;

	/*
	* 函数名： MoveZCutting
	* 函数介绍：移动z轴到切割位置
	* 输入参数：
	*	AbsolutePos: Z轴移动的绝对位置
	* 返回值：
	*	true：调用成功
	*	false：调用失败
	*/
	virtual bool MoveZCutting(double dAbsolutePos) = 0;//没有调用

	/*
	* 函数名：	GetName
	* 函数介绍：获取控制器的名字
	* 输入参数：
	* 	无
	* 返回值：
	*	控制器名称的字符串
	*/
	virtual const string & GetName() const = 0;

	/*
	* 函数名：	IsConnected
	* 函数介绍：判断控制器是否已连接
	* 输入参数：
	* 	无
	* 返回值：
	*	true：控制器已连接；
	*   false：控制器未连接。
	*/
	virtual bool IsConnected() = 0;

	/*
	* 函数名：	IsEnable
	* 函数介绍：获取轴是否使能
	* 输入参数：
	* 	eAxis：轴的枚举类型（Axis_X1、Axis_Y1、Axis_Z）
	* 返回值：
	*	true：已使能
	*   false：未使能
	*/
	virtual bool IsEnabled(Axis eAxis) = 0;
	virtual bool IsEnabled() = 0;

	virtual bool ErrorOccurred() const = 0;//待定

	virtual bool StopQueue() = 0;

	virtual bool Punch(double fdDwellTime) = 0;//暂时没用到，返回true

	//Program模式
	virtual void ResetProgramCommand() = 0;//Aerotech有用
	virtual void BeginACSSegment(const Tool& tool) = 0;	// ACS中，SEGMENT运动必须在之前设置加速度和加加速度
	virtual void BeginACSSegmentSimple(const Tool& tool) = 0;	//界面导出有用到
	virtual void EndProgramCommand(const Tool& tool) = 0;
	virtual void EndProgramCommandSimple(const Tool& tool) = 0;	//界面导出有用到
	virtual void OffsetLineTo(double dEndX, double dEndY, const Tool& tool) = 0;
	virtual void OffsetLineToSimple(double dEndX, double dEndY, const Tool& tool) = 0;	//界面导出有用到
	virtual void OffsetArcTo(double dEndX, double dEndY, double dCenterX, double dCenterY, 
							bool bClockwise, const Tool& tool, double dIncX, double dIncY) = 0;
	virtual void OffsetArcToSimple(double dEndX, double dEndY, double dCenterX, double dCenterY, 
								bool bClockwise, const Tool& tool, double dIncX, double dIncY) = 0;
	virtual void OffsetArc2To(double dEndX, double dEndY, double dCenterX, double dCenterY, double dAngle, const Tool& tool) = 0;	//没有检索到有调用

	virtual bool IsOffsetCutting() = 0;
	virtual void JumpToSetAFPos(const Tool& curTool) = 0;
	virtual void JumpToTrough(const Tool& curTool, double time) = 0;
	virtual void JumpToIdleHeight(const Tool& curTool, double dCompensate = 0) = 0;
	virtual void JumpToCuttingHeight(const Tool& curTool, double dCompensate = 0) = 0;
	virtual void JumpToIdleXYPosition(double dEndX, double dEndY, const Tool& curTool) = 0;
	virtual void JumpToSimple(double, double, const Tool&, double, double) = 0;//界面导出有用到

	virtual bool SendCommand() = 0;
	virtual bool SetCuttingAccJerk(const Tool&) = 0; 
	virtual bool SetJumpAccJerk(const Tool&) = 0; 
	virtual bool StopMovingCuttingHead() = 0;
	virtual bool StartMovingCuttingHead(const Tool&) = 0;

	virtual void ProLaserControl(bool, bool , const Tool&, bool) = 0;
	virtual void ProLaserControlSimple(bool, bool , const Tool&) = 0;

	virtual int GetPressureState() = 0;
	virtual bool GetIsPressureState() =0;                      //获取是否需要气压监控标识
	virtual void SetIsPressureState(bool)=0;                   //设置是否需要气压监控标识

	virtual bool IsAxisStatusNormal(int &iFault)=0;

	virtual bool SetFPos(Axis, double) = 0;
	virtual bool GetFPos(Axis, double&) = 0;

	// 停止轴系
	virtual bool HaltMotor(Axis eMotor) = 0;

	virtual bool LoadApplication(string) = 0;
	virtual string GetCuttingCommand() = 0;
	virtual bool StopBuffer(int iBufferIndex) = 0;
	virtual bool StopAllBuffer() = 0;
	virtual bool RunBufferTillEnd(int,int) = 0;
	virtual bool IsBufferRunning(int iBufferIndex) = 0;
	virtual bool CheckBuffer(int iBufferIndex, string& strCommand) = 0;
	virtual bool RunBuffer(int iBufferIndex) = 0;
	virtual bool PauseBuffer(int iBufferIndex) = 0;
	virtual bool LoadCommandAndRunBuffer(int iBufferIndex, string strCommand, int iTimeout) = 0;

	virtual bool SetMFLAGSValue(Axis, int) = 0;
	virtual bool AcscReadReal(const string, double&) = 0;
	virtual bool AcscWriteReal(const string, double) = 0;
	virtual bool AcscReadInt(const string, int&) = 0;
	virtual bool AcscWriteInt(const string,  int) = 0;

	virtual bool IsReachPos(Axis eAxis, bool bRelative, double dPos) = 0; // 可达检测，相对模式+距离，绝对模式+位置
	virtual bool SetAxisEnable(Axis eAxis, bool bEnable) = 0;

	//建立插补坐标系，并且初始化前瞻
	virtual bool InitCrd(const Tool& curTool) = 0;
	virtual bool PrfTrapAxis() = 0;

	//GSN PWM
	virtual bool GSN_SetLaserParameterApplication(double dFrequence, double dPulse, double dDelay) = 0;
	virtual bool GSN_SetLaserEnablePro(bool bState) = 0;
	virtual bool GSN_LaserOnStatus(int& iState) = 0;

	virtual void ClearAxisState() {};
	virtual void StartCommand() {};


#pragma region FlightCutting
	virtual void BeginACSSegmentForFlightCutting(const Tool& tool) = 0;
	virtual void OffsetFlightLineTo(double dEndX, double dEndY, const Tool& tool, bool bPolyGuide) = 0;		//飞行切割独有的Line指令
	virtual void OffsetFlightArcTo(double dEndX, double dEndY, double dCenterX, double dCenterY, bool bClockwise, const Tool& tool, double dIncX, double dIncY, bool bPolyGuide) = 0;
	virtual void OffsetFlightArc2To(double dCenterX, double dCenterY, double dAngle, const Tool& tool, bool bPolyGuide) = 0;
	virtual void EndProgramCommandForFlightCutting(const Tool& tool) = 0;
	virtual bool LoadAndCompileBuffer(int) = 0;		//导入指令到指定的Buffer，并编译（NOT RUN）
	virtual bool RunBufferForFlightCutting(int) = 0;    //直接Run指定的Buffer，请确保其已编译过
#pragma endregion FlightCutting
};