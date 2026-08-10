#pragma once
#include <vector>
#include <string>
#include <windows.h>
#include <map>
#include <regex>

#include "modules/process/device/process_io_types.h"
#include "modules/process/runtime/process_axis_types.h"
#include "modules/process/system/message_code.h"
#include "tool_factory.h"
#include "toml.hpp"

namespace lcnc::process { class ProcessSettingsService; class ProcessRuntimeConfiguration; }

// 设备运行时层 —— 仅暴露 IO/jog/home/connect/axis-toml::table 等设备控制面。
// 切割指令（buffer 文本拼装、ProLaserControl、JumpTo*、ACS XSEG/LINE/GTN crd 等）
// 已迁出到 IMotionCommandSink 体系，不再属于此基类的职责。

struct DigitalIOData
{
	QString qstrID		= "";		// 名称
	std::string	strIndex	= "";		// 索引，buffer用
	std::string	strPort		= "";		// 地址位
	int		iPort;					// 地址位
	int		iIO			= 8;		// 数字量所在位
	bool	bExpand		= false;	// 拓展模块
	bool	bInversion	= false;	// 信号取反
};

struct AnalogIOData
{
	QString qstrID		= "";		// 名称
	std::string	strIndex	= "";		// 索引，buffer用
	std::string	strPort		= "";		// 地址位
	int		iPort;					// 地址位
	bool	bExpand		= false;	// 拓展模块
};

class MotionControl
{
protected:
    MotionControl(lcnc::process::ProcessSettingsService& settings,
                  lcnc::process::ProcessRuntimeConfiguration& runtimeConfiguration)
        : m_settings(settings), m_runtimeConfiguration(runtimeConfiguration) {}

public:
	std::vector<lcnc::process::Axis>					m_vecMotors;

	std::map<lcnc::process::DigitalIN,	DigitalIOData>	m_mapDigitalIN;
	std::map<lcnc::process::DigitalOUT, DigitalIOData>  m_mapDigitalOUT;
	std::map<lcnc::process::AnalogIN,	AnalogIOData>	m_mapAnalogIN;
	std::map<lcnc::process::AnalogOUT,	AnalogIOData>	m_mapAnalogOUT;

	std::map<QString,	DigitalIOData>	m_mapqDigitalIN;
	std::map<QString,	DigitalIOData>  m_mapqDigitalOUT;
	std::map<QString,	AnalogIOData>	m_mapqAnalogIN;
	std::map<QString,	AnalogIOData>	m_mapqAnalogOUT;

public:
	static const std::regex regex_DigitalIO;
	static const std::regex regex_AnalogIO;

	virtual ~MotionControl() = default;

protected:
	lcnc::process::ProcessSettingsService& m_settings;
	lcnc::process::ProcessRuntimeConfiguration& m_runtimeConfiguration;
	void rebuildIOMap(int iType);

public:
	void rebuildAxes();
	virtual bool IsMotorCreated(lcnc::process::Axis eAxis);
	virtual void CreateMotor(lcnc::process::Axis eAxis, const toml::table& tAxis) = 0;

	// 参数下发层（父类提供默认实现，子类按需 override）
	virtual void setMotionControlTable();
	virtual void setMotionControlTable(const toml::table& tMotion);
	virtual void SetAxisTable(lcnc::process::Axis eAxis, const toml::table& tMotion);
	virtual void SetAxisHomePrm(lcnc::process::Axis, const toml::table&) {}
	virtual bool GetAxisHomePrm(lcnc::process::Axis, toml::table&) { return false; }
	virtual void SetPipeDiameterTable();
	virtual void SetPipeDiameterTable(const toml::table& tAxis);
	virtual void setDigitalTable();
	virtual void setDigitalTable(const toml::table& tDigital);
	virtual void setAnalogTable();
	virtual void setAnalogTable(const toml::table& tAnalog);
	virtual ErrorCode SetLaserParameterTable();
	virtual ErrorCode SetLaserParameterTable(const toml::table& tLaser);

	// IO 接口转义层（父类实现）
	virtual bool DigitalOutputSet	(lcnc::process::DigitalOUT	eIndex,	int		iValue, bool bLogError = false);
	virtual bool DigitalOutputGet	(lcnc::process::DigitalOUT	eIndex,	int&	iValue, bool bLogError = false);
	virtual bool DigitalInputGet	(lcnc::process::DigitalIN	eIndex,	int&	iValue, bool bLogError = false);
	virtual bool AnalogOutputSet	(lcnc::process::AnalogOUT	eIndex,	double  dValue, bool bLogError = false);
	virtual bool AnalogOutputGet	(lcnc::process::AnalogOUT	eIndex,	double& dValue, bool bLogError = false);
	virtual bool AnalogInputGet		(lcnc::process::AnalogIN	eIndex,	double& dValue, bool bLogError = false);

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
	virtual bool Home(lcnc::process::Axis eAxis) = 0;
	virtual bool Home() = 0;
	virtual bool StopHome() = 0;
	virtual bool IsHomed() = 0;
	virtual bool IsHomed(lcnc::process::Axis eAxis) = 0;

	// 使能
	virtual bool Enable(lcnc::process::Axis eAxis) = 0;
	virtual bool Enable() = 0;
	virtual bool Disable(lcnc::process::Axis eAxis) = 0;
	virtual bool Disable() = 0;
	virtual bool IsEnabled(lcnc::process::Axis eAxis) = 0;
	virtual bool IsEnabled() = 0;
	virtual bool SetAxisEnable(lcnc::process::Axis eAxis, bool bEnable) = 0;

	// 单 / 多轴运动
	virtual bool Jog(lcnc::process::Axis eAxis, bool bDirection, double dVel) = 0;
	virtual bool MoveRelative(lcnc::process::Axis eAxis, double dPos, double dVel) = 0;
	virtual bool MoveMRelative(std::vector<lcnc::process::Axis> vAxis, std::vector<double> vPos, double dVel) = 0;
	virtual bool MoveAbsolute(lcnc::process::Axis eAxis, double dPos, double dVel) = 0;
	virtual bool MoveMAbsolute(std::vector<lcnc::process::Axis> vAxis, std::vector<double> vPos, double dVel) = 0;
	virtual bool StopMotion(lcnc::process::Axis eAxis) = 0;
	virtual bool StopMotion() = 0;
	virtual bool HaltMotor(lcnc::process::Axis eMotor) = 0;

	// 置位：将轴当前位置寄存器直接重写为指定坐标（不产生运动）。
	// ACS 调用 acsc_SetFPosition(setfpos)；GTN 同步设置规划位置与编码器位置。
	virtual bool SetFPosition(lcnc::process::Axis eAxis, double dPos) = 0;

	// 状态查询
	virtual bool IsAxisMoving(lcnc::process::Axis eAxis) = 0;
	virtual bool IsAxisMoving() = 0;
	virtual bool GetActualPos(lcnc::process::Axis eAxis, double& dFPos) = 0;
	virtual bool GetFeedbackPos(lcnc::process::Axis eAxis, double& dFPos) = 0;
	virtual bool IsReachPos(lcnc::process::Axis eAxis, bool bRelative, double dPos) = 0;
	virtual bool IsAxisStatusNormal(int& iFault) = 0;
	virtual bool ErrorOccurred() const = 0;
	virtual bool IsQueueActive() = 0;

	// 每轴参数 Set/Get
	virtual bool SetAxisIndex(lcnc::process::Axis eAxis, int iIndex) = 0;
	virtual bool SetAxisHomeBufferIndex(lcnc::process::Axis eAxis, int iHomeIndex) = 0;
	virtual bool SetAxisIsRotary(lcnc::process::Axis eAxis, bool bRotary) = 0;
	virtual bool SetAxisResolution(lcnc::process::Axis eAxis, int iResolution) = 0;
	virtual bool SetAxisTubeDiamater(lcnc::process::Axis eAxis, double dTubeDiamater) = 0;
	virtual bool SetAxisVel(lcnc::process::Axis eAxis, double dVel) = 0;
	virtual bool SetAxisAcc(lcnc::process::Axis eAxis, double dAcc) = 0;
	virtual bool SetAxisDec(lcnc::process::Axis eAxis, double dDec) = 0;
	virtual bool SetAxisJerk(lcnc::process::Axis eAxis, double dJerk) = 0;
	virtual bool SetAxisNegLimit(lcnc::process::Axis eAxis, double dNegLimit) = 0;
	virtual bool SetAxisPosLimit(lcnc::process::Axis eAxis, double dPosLimit) = 0;
	virtual bool SetAxisVelAccDecJerk(lcnc::process::Axis eAxis, double dVel, double dAcc, double dDec, double dJerk) = 0;
	virtual bool SetAxisSoftLimit(lcnc::process::Axis eAxis, double dNegLimit, double dPosLimit) = 0;

	virtual bool GetAxisIndex(lcnc::process::Axis eAxis, int& iIndex) = 0;
	virtual bool GetAxisHomeBufferIndex(lcnc::process::Axis eAxis, int& iIndex) = 0;
	virtual bool GetAxisIsRotary(lcnc::process::Axis eAxis, bool& bRotary) = 0;
	virtual bool GetAxisResolution(lcnc::process::Axis eAxis, int& iResolution) = 0;
	virtual bool GetAxisTubeDiamater(lcnc::process::Axis eAxis, double& dTubeDiamater) = 0;
	virtual bool GetAxisVel(lcnc::process::Axis eAxis, double& dVel) = 0;
	virtual bool GetAxisAcc(lcnc::process::Axis eAxis, double& dAcc) = 0;
	virtual bool GetAxisDec(lcnc::process::Axis eAxis, double& dDec) = 0;
	virtual bool GetAxisJerk(lcnc::process::Axis eAxis, double& dJerk) = 0;
	virtual bool GetAxisNegLimit(lcnc::process::Axis eAxis, double& dNegLimit) = 0;
	virtual bool GetAxisPosLimit(lcnc::process::Axis eAxis, double& dPosLimit) = 0;
	virtual bool GetAxisVelAccDecJerk(lcnc::process::Axis eAxis, double& dVel, double& dAcc, double& dDec, double& dJerk) = 0;
	virtual bool GetAxisSoftLimit(lcnc::process::Axis eAxis, double& dNegLimit, double& dPosLimit) = 0;
	virtual void ReadAxisSoftLimit(lcnc::process::Axis eAxis, double& dNegLimit, double& dPosLimit) = 0;

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
	virtual const std::string& GetName() const = 0;
	virtual void ClearAxisState() {}
	virtual void StartCommand() {}
};
