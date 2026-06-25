#pragma once
#include <xstring>
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

	Tool & operator = (const Tool& _Tool){
		m_strName					= _Tool.m_strName;					// 工具名
		m_strType					= _Tool.m_strType;
		m_dJunctionVelocity			= _Tool.m_dJunctionVelocity;		// 连接点速度
		m_dJunctionAngle			= _Tool.m_dJunctionAngle;			// 连接点角度
		m_dXsegEndVelocity			= _Tool.m_dXsegEndVelocity;			// XSEG末速度
		m_dLineVelocity				= _Tool.m_dLineVelocity;			// 直线切割速度
		m_dLineAcc					= _Tool.m_dLineAcc;					// 直线切割加速度
		m_dLineJerk					= _Tool.m_dLineJerk;				// 直线切割加加速度
		m_dArcVelocity				= _Tool.m_dArcVelocity;				// 圆弧切割速度
		m_dArcAcc					= _Tool.m_dArcAcc;					// 圆弧切割加速度
		m_dArcJerk					= _Tool.m_dArcJerk;					// 圆弧切割加加速度
		m_dIdleXVelocity			= _Tool.m_dIdleXVelocity;			// X轴空程速度（Add）
		m_dIdleAVelocity			= _Tool.m_dIdleAVelocity;			// A轴空程速度
		m_dIdleA1Velocity			= _Tool.m_dIdleA1Velocity;			// A1轴空程速度
		m_dIdleYVelocity			= _Tool.m_dIdleYVelocity;			// Y轴空程速度
		m_dIdleZVelocity			= _Tool.m_dIdleZVelocity;			// Z轴空程速度
		m_dIdleX1Velocity			= _Tool.m_dIdleX1Velocity;			// X1轴空程速度
		m_dIdleY1Velocity			= _Tool.m_dIdleY1Velocity;			// Y1轴空程速度
		m_dIdleXYAccDec				= _Tool.m_dIdleXYAccDec;			// XY轴空程加减速度（Add）
		m_dIdleXYJerk				= _Tool.m_dIdleXYJerk;				// XY轴空程加加速度（Add）
		m_dBeforeOn					= _Tool.m_dBeforeOn;				// 开激光前的等待时间
		m_dAfterOn					= _Tool.m_dAfterOn;					// 开激光后的等待时间
		m_dBeforeOff				= _Tool.m_dBeforeOff;				// 关激光前的等待时间
		m_dAfterOff					= _Tool.m_dAfterOff;				// 关激光后的等待时间
		m_dLaserFrequency			= _Tool.m_dLaserFrequency;			// 激光的频率
		m_dLaserPulseWidth			= _Tool.m_dLaserPulseWidth;			// 激光的脉宽
		m_dLaserEnergy				= _Tool.m_dLaserEnergy;				// 激光的能量
		m_dLaserAttenuatorPercentage = _Tool.m_dLaserAttenuatorPercentage;
		m_dLaserPpDivider		= _Tool.m_dLaserPpDivider;
		m_iLaserDelay			= _Tool.m_iLaserDelay;
		m_dRadius					= _Tool.m_dRadius;					// 光斑补偿半径
		m_strOffsetType				= _Tool.m_strOffsetType;			//补偿类型
		m_dOffsetDiameter			= _Tool.m_dOffsetDiameter;			//补偿光斑直径
		m_dOffsetDistance			= _Tool.m_dOffsetDistance;			//补偿间距
		m_dOffsetIgnoreLength		= _Tool.m_dOffsetIgnoreLength;		//直线圆弧补偿时忽略长度
		m_dIdleZHeight				= _Tool.m_dIdleZHeight;		
		m_dCuttingHeight			= _Tool.m_dCuttingHeight;
		m_dCuttingHeightCompensate	= _Tool.m_dCuttingHeightCompensate;
		m_strDirectionX				= _Tool.m_strDirectionX;
		m_strDirectionY				= _Tool.m_strDirectionY;

		m_bStopBlow					= _Tool.m_bStopBlow;
		m_dWaitFirst				= _Tool.m_dWaitFirst;
		m_dWaitSecond				= _Tool.m_dWaitSecond;
		//Equal Power
		m_iPDMode					= _Tool.m_iPDMode;
		m_dPDScaleFactor			= _Tool.m_dPDScaleFactor;
		m_dPDWidth					= _Tool.m_dPDWidth;
		m_dPDPosOffset				= _Tool.m_dPDPosOffset;
		m_dPDLowVelMax				= _Tool.m_dPDLowVelMax;
		m_dPDPosLowVelMax			= _Tool.m_dPDPosLowVelMax;
		m_dPDLowVelMax_1			= _Tool.m_dPDLowVelMax_1;
		m_dPDPosLowVelMax_1			= _Tool.m_dPDPosLowVelMax_1;
		m_dBlowDelay				= _Tool.m_dBlowDelay;				//吹气打开后等待时间
		m_dExtend					= _Tool.m_dExtend;					//延长线长度
		m_dExtend_End				= _Tool.m_dExtend_End;
		m_bTroughFlag				= _Tool.m_bTroughFlag;				//是否盲刻
		m_iTroughBuffer				= _Tool.m_iTroughBuffer;
		m_dR0						= _Tool.m_dR0;
		m_dZ0						= _Tool.m_dZ0;
		m_dFocusOffset				= _Tool.m_dFocusOffset;				// 切割时切割嘴距离钢片的距离
		m_bPunch					= _Tool.m_bPunch;					//是否打点
		m_bBlow2					= _Tool.m_bBlow2;					//第二路气体
		m_iPowerSetpoint			= _Tool.m_iPowerSetpoint;
		m_iRepetitionRate			= _Tool.m_iRepetitionRate;
		m_iPulsePickerDivider		= _Tool.m_iPulsePickerDivider;
		m_dAnalogLaserValue			= _Tool.m_dAnalogLaserValue;
		m_dTroughDelay				= _Tool.m_dTroughDelay;

		m_bAZero					= _Tool.m_bAZero;
		m_dAPos						= _Tool.m_dAPos;
		m_bA1Zero					= _Tool.m_bA1Zero;
		m_dA1Pos					= _Tool.m_dA1Pos;

		m_bXIsMove					= _Tool.m_bXIsMove;
		m_dXPosition				= _Tool.m_dXPosition;
		m_bX1IsMove					= _Tool.m_bX1IsMove;
		m_dX1Position				= _Tool.m_dX1Position;
		m_bAIsMove					= _Tool.m_bAIsMove;
		m_dAPosition				= _Tool.m_dAPosition;
		m_bA1IsMove					= _Tool.m_bA1IsMove;
		m_dA1Position				= _Tool.m_dA1Position;
		m_bYIsMove					= _Tool.m_bYIsMove;
		m_dYPosition				= _Tool.m_dYPosition;
		m_bY1IsMove					= _Tool.m_bY1IsMove;
		m_dY1Position				= _Tool.m_dY1Position;

		m_bCuttingHead				= _Tool.m_bCuttingHead;
		m_bCrossBridge				= _Tool.m_bCrossBridge;
		m_dServoCuttingHeight		= _Tool.m_dServoCuttingHeight;

		m_bAxisZLinkage				= _Tool.m_bAxisZLinkage;
		m_dLinkedDelay				= _Tool.m_dLinkedDelay;
		m_sLinkedDirection			= _Tool.m_sLinkedDirection;
		m_iLinkedMode				= _Tool.m_iLinkedMode;
		m_dLinkageParameterA		= _Tool.m_dLinkageParameterA;
		m_dLinkageParameterB		= _Tool.m_dLinkageParameterB;
		m_sLinkedFormula			= _Tool.m_sLinkedFormula;

		m_bFlightCutting			= _Tool.m_bFlightCutting;
		m_dFlightCutting_MotorDelay = _Tool.m_dFlightCutting_MotorDelay;
		m_bEnergySwitch				= _Tool.m_bEnergySwitch;

		return *this;
	}
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
	double m_dIdleZHeight{0.0};
	double m_dCuttingHeight{0.0};
	double m_dCuttingHeightCompensate{0.0};  // 切割高度补偿
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