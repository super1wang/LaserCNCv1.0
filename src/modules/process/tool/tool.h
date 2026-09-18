#pragma once
#include <string>
#include "toml.hpp"
using std::string;
using std::wstring;

class Tool
{
public:
	Tool() = default;

	/// 从工具的 TOML 表反序列化到本对象。表中找不到的键保持成员的默认值（已在类内初始化）。
	/// 返回写入字段数，便于排查未匹配的键。
	int SetFromTable(const ::toml::table& tab);
	/// Serialize all parameters consumed by SetFromTable for project snapshots.
	::toml::table toTable() const;

	// Value snapshots must copy every field, including smoothing/profile values.
	Tool& operator=(const Tool&) = default;
public:
	string m_strName;					// 工具名
	wstring m_strType;
	double m_dJunctionVelocity{0.0};			// 连接点速度
	double m_dJunctionAngle{0.0};			// 连接点角度
	double m_dXsegEndVelocity{0.0};			// XSEG末速度
	double m_dCutSmoothTime{0.0};			// 切割平滑时间
	double m_dCutSmoothK{0.0};				// 切割平滑系数
	double m_dAxisSmoothTime{0.0};			// 轴平滑时间
	double m_dAxisSmoothK{0.0};				// 轴平滑系数
	double m_dLineVelocity{0.0};				// 直线切割速度
	double m_dLineAcc{0.0};					// 直线切割加速度
	double m_dLineJerk{0.0};					// 直线切割加加速度
	double m_dArcVelocity{0.0};				// 圆弧切割速度
	double m_dArcAcc{0.0};					// 圆弧切割加速度
	double m_dArcJerk{0.0};					// 圆弧切割加加速度
	double m_dIdleXVelocity{0.0};			// X轴空程速度（Add）
	double m_dIdleAVelocity{0.0};			// A轴空程速度
	double m_dIdleA1Velocity{0.0};			// A1轴空程速度
	double m_dIdleCVelocity{0.0};			// C轴空程速度
	double m_dIdleYVelocity{0.0};			// Y轴空程速度
	double m_dIdleZVelocity{0.0};			// Z轴空程速度
	double m_dIdleX1Velocity{0.0};			// X1轴空程速度
	double m_dIdleY1Velocity{0.0};			// Y1轴空程速度
	double m_dIdleXYAccDec{0.0};				// XY轴空程加减速度（Add）
	double m_dIdleXYJerk{0.0};				// XY轴空程加加速度（Add）
	double m_dBeforeOn{0.0};					// 开激光前的等待时间
	double m_dAfterOn{0.0};					// 开激光后的等待时间
	double m_dBeforeOff{0.0};				// 关激光前的等待时间
	double m_dAfterOff{0.0};					// 关激光后的等待时间
	int m_dLaserFrequency{0};				// 激光的频率
	int m_dLaserPulseWidth{0};				// 激光的脉宽
	double m_dLaserEnergy{0.0};				// 激光的能量
	double m_dLaserAttenuatorPercentage{0.0};	// Pharos 衰减百分比
	double m_dLaserPpDivider{0.0};			// Pharos 分频参数
	int m_iLaserDelay{0};					// Pharos 参数下发延时
	double m_dAnalogValue{0.0};
	double m_dRadius{0.0};					// 光斑补偿半径
	wstring m_strOffsetType;			// 补偿类型
	double m_dOffsetDiameter{0.0};			// 补偿光斑直径
	double m_dOffsetDistance{0.0};			// 补偿间距
	double m_dOffsetIgnoreLength{0.0};		// 直线圆弧补偿时忽略长度
	string m_strDirectionX;
	string m_strDirectionY;

	bool m_bStopBlow{false};
	double m_dWaitFirst{0.0};
	double m_dWaitSecond{0.0};
	//Equal Power
	int m_iPDMode{0};
	double m_dPDScaleFactor{0.0};
	double m_dPDWidth{0.0};
	double m_dPDPosOffset{0.0};
	double m_dPDLowVelMax{0.0};
	double m_dPDPosLowVelMax{0.0};
	double m_dPDLowVelMax_1{0.0};
	double m_dPDPosLowVelMax_1{0.0};
	double m_dBlowDelay{0.0};				// 吹气打开后等待时间
	double m_dExtend{0.0};					// 延长线首端长度
	double m_dExtend_End{0.0};				// 延长线末段长度
	bool   m_bTroughFlag{false};				// 是否盲刻
	int	   m_iTroughBuffer{0};				// 盲刻执行buffer
	double m_dR0{0.0};
	double m_dZ0{0.0};
	double m_dFocusOffset{0.0};				// 切割时切割嘴距离钢片的距离
	bool   m_bPunch{false};					// 是否打点
	bool   m_bBlow2{false};					// 第二路气体

	int m_iPowerSetpoint{0};
	int m_iRepetitionRate{0};
	int m_iPulsePickerDivider{0};
	double m_dAnalogLaserValue{0.0};			// 激光器能量参数由模拟量模块儿下发

	bool m_bCuttingHead{false};
	bool m_bCrossBridge{false};
	double m_dServoCuttingHeight{0.0};

	bool m_bAxisZLinkage{false};
	double m_dLinkedDelay{0.0};
	string m_sLinkedDirection;
	int m_iLinkedMode{0};
	double m_dLinkageParameterA{0.0};
	double m_dLinkageParameterB{0.0};
	string m_sLinkedFormula;

	double m_dTroughDelay{0.0};				// 盲刻延时

	bool m_bAZero{false};
	double m_dAPos{0.0};
	bool m_bA1Zero{false};
	double m_dA1Pos{0.0};

	bool m_bXIsMove{false};
	double m_dXPosition{0.0};
	bool m_bX1IsMove{false};
	double m_dX1Position{0.0};
	bool m_bAIsMove{false};
	double m_dAPosition{0.0};
	bool m_bA1IsMove{false};
	double m_dA1Position{0.0};
	bool m_bYIsMove{false};
	double m_dYPosition{0.0};
	bool m_bY1IsMove{false};
	double m_dY1Position{0.0};

	bool m_bFlightCutting{false};						//飞切
	double m_dFlightCutting_MotorDelay{0.0};			//飞切延时
	bool m_bEnergySwitch{false};						// 工具能量切换标志
};
