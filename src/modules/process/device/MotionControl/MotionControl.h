#pragma once
#include <vector>
#include <string>
#include <windows.h>
#include <regex>
#include "ToolFactory.h"
#include "Settings.h"
#include "MessageModule.h"

using std::vector;
using std::string;

// 设备运行时层 —— 仅暴露 IO/jog/home/connect/axis-table 等设备控制面。
// 切割指令（buffer 文本拼装、ProLaserControl、JumpTo*、ACS XSEG/LINE/GTN crd 等）
// 已迁出到 IMotionCommandSink 体系，不再属于此基类的职责。

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

class MotionControl
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
	static const std::regex regex_DigitalIO;
	static const std::regex regex_AnalogIO;

	virtual ~MotionControl() = default;

protected:
	void rebuildIOMap(int iType);

public:
	void rebuildAxes();
	virtual bool IsMotorCreated(Axis eAxis);
	virtual void CreateMotor(Axis eAxis, const table& tAxis) = 0;

	// 参数下发层（父类提供默认实现，子类按需 override）
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

	// IO 接口转义层（父类实现）
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
	virtual void LogError() = 0;

	// 设备生命周期
	virtual bool Connect() = 0;
	virtual bool Reboot() = 0;
	virtual bool Disconnect() = 0;
	virtual bool IsConnected() = 0;

	// 回零
	virtual bool Home(Axis eAxis) = 0;
	virtual bool Home() = 0;
	virtual bool StopHome() = 0;
	virtual bool IsHomed() = 0;
	virtual bool IsHomed(Axis eAxis) = 0;

	// 使能
	virtual bool Enable(Axis eAxis) = 0;
	virtual bool Enable() = 0;
	virtual bool Disable(Axis eAxis) = 0;
	virtual bool Disable() = 0;
	virtual bool IsEnabled(Axis eAxis) = 0;
	virtual bool IsEnabled() = 0;
	virtual bool SetAxisEnable(Axis eAxis, bool bEnable) = 0;

	// 单 / 多轴运动
	virtual bool Jog(Axis eAxis, bool bDirection, double dVel) = 0;
	virtual bool MoveRelative(Axis eAxis, double dPos, double dVel) = 0;
	virtual bool MoveMRelative(vector<Axis> vAxis, vector<double> vPos, double dVel) = 0;
	virtual bool MoveAbsolute(Axis eAxis, double dPos, double dVel) = 0;
	virtual bool MoveMAbsolute(vector<Axis> vAxis, vector<double> vPos, double dVel) = 0;
	virtual bool StopMotion(Axis eAxis) = 0;
	virtual bool StopMotion() = 0;
	virtual bool HaltMotor(Axis eMotor) = 0;

	// 状态查询
	virtual bool IsAxisMoving(Axis eAxis) = 0;
	virtual bool IsAxisMoving() = 0;
	virtual bool GetActualPos(Axis eAxis, double& dFPos) = 0;
	virtual bool GetFeedbackPos(Axis eAxis, double& dFPos) = 0;
	virtual bool IsReachPos(Axis eAxis, bool bRelative, double dPos) = 0;
	virtual bool IsAxisStatusNormal(int& iFault) = 0;
	virtual bool ErrorOccurred() const = 0;
	virtual bool IsQueueActive() = 0;

	// 每轴参数 Set/Get
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

	// 基于 IOData 的 IO 操作
	virtual bool DigitalOutputSet	(DigitalIOData&	IOData,	int		iValue,	bool bLogError = false) = 0;
	virtual bool DigitalOutputGet	(DigitalIOData&	IOData,	int&	iValue, bool bLogError = false) = 0;
	virtual bool DigitalInputGet	(DigitalIOData&	IOData,	int&	iValue, bool bLogError = false) = 0;
	virtual bool AnalogOutputSet	(AnalogIOData&	IOData,	double  dValue, bool bLogError = false) = 0;
	virtual bool AnalogOutputGet	(AnalogIOData&	IOData,	double& dValue, bool bLogError = false) = 0;
	virtual bool AnalogInputGet		(AnalogIOData&	IOData,	double& dValue, bool bLogError = false) = 0;

	// 气压监控
	virtual int  GetPressureState() = 0;
	virtual bool GetIsPressureState() = 0;
	virtual void SetIsPressureState(bool) = 0;

	// 程序缓冲区控制（控制器侧 buffer #9 之类的整体启停，与具体指令内容无关）
	virtual bool IsBufferRunning(int iBufferIndex) = 0;
	virtual bool StopBuffer(int iBufferIndex) = 0;
	virtual bool StopAllBuffer() = 0;
	virtual bool PauseBuffer(int iBufferIndex) = 0;

	// 通用清理 / 标识
	virtual const string& GetName() const = 0;
	virtual void ClearAxisState() {}
	virtual void StartCommand() {}
};
