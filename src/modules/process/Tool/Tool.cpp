#include "Tool.h"

#include <cmath>
#include <string>

// 动态参数注册表所使用的工具字段映射。每个键如果不存在或类型不匹配
// 都保留 Tool 成员的默认值（已在类内 = 0/false 初始化）。

namespace {

using ::toml::table;
using ::toml::value;

bool tryGetDouble(const table& t, const char* key, double& dst)
{
	auto it = t.find(key);
	if (it == t.end())
		return false;
	try
	{
		const value& v = it->second;
		if (v.is_floating())      dst = v.as_floating();
		else if (v.is_integer())  dst = static_cast<double>(v.as_integer());
		else                       return false;
		if (!std::isfinite(dst))
			dst = 0.0;
		return true;
	}
	catch (...) { return false; }
}

bool tryGetInt(const table& t, const char* key, int& dst)
{
	auto it = t.find(key);
	if (it == t.end())
		return false;
	try
	{
		const value& v = it->second;
		if (v.is_integer())       dst = static_cast<int>(v.as_integer());
		else if (v.is_floating()) dst = static_cast<int>(v.as_floating());
		else                       return false;
		return true;
	}
	catch (...) { return false; }
}

bool tryGetBool(const table& t, const char* key, bool& dst)
{
	auto it = t.find(key);
	if (it == t.end())
		return false;
	try
	{
		const value& v = it->second;
		if (v.is_boolean())       dst = v.as_boolean();
		else if (v.is_integer())  dst = v.as_integer() != 0;
		else                       return false;
		return true;
	}
	catch (...) { return false; }
}

bool tryGetString(const table& t, const char* key, std::string& dst)
{
	auto it = t.find(key);
	if (it == t.end())
		return false;
	try
	{
		const value& v = it->second;
		if (!v.is_string()) return false;
		dst = v.as_string();
		return true;
	}
	catch (...) { return false; }
}

} // namespace

int Tool::SetFromTable(const ::toml::table& t)
{
	int n = 0;

	// 速度 / 加减速度 / 加加速度（切割段）
	if (tryGetDouble(t, "fLineVel",  m_dLineVelocity))   ++n;
	if (tryGetDouble(t, "fArcVel",   m_dArcVelocity))    ++n;
	if (tryGetDouble(t, "fCutAcc",   m_dLineAcc))        ++n;
	if (tryGetDouble(t, "fCutJerk",  m_dLineJerk))       ++n;
	// 直线/圆弧共用一套 ACC/JERK，给 m_dArcAcc/m_dArcJerk 也注入相同值，避免下游某些路径取 Arc 时为 0。
	m_dArcAcc  = m_dLineAcc;
	m_dArcJerk = m_dLineJerk;

	// 空程
	if (tryGetDouble(t, "fXVel",   m_dIdleXVelocity))  ++n;
	if (tryGetDouble(t, "fYVel",   m_dIdleYVelocity))  ++n;
	if (tryGetDouble(t, "fAVel",   m_dIdleAVelocity))  ++n;
	if (tryGetDouble(t, "fA1Vel",  m_dIdleA1Velocity)) ++n;
	if (tryGetDouble(t, "fCVel",   m_dIdleCVelocity))  ++n;
	if (tryGetDouble(t, "fX1Vel",  m_dIdleX1Velocity)) ++n;
	if (tryGetDouble(t, "fY1Vel",  m_dIdleY1Velocity)) ++n;
	if (tryGetDouble(t, "fZVel",   m_dIdleZVelocity))  ++n;
	if (tryGetDouble(t, "fIdelAcc",  m_dIdleXYAccDec)) ++n;
	if (tryGetDouble(t, "fIdelJerk", m_dIdleXYJerk))   ++n;

	// 激光参数
	if (tryGetDouble(t, "fEnergy",               m_dLaserEnergy))               ++n;
	if (tryGetDouble(t, "fAttenuatorPercentage", m_dLaserAttenuatorPercentage)) ++n;
	if (tryGetDouble(t, "fPpDivider",            m_dLaserPpDivider))            ++n;
	if (tryGetInt   (t, "iDelay",                m_iLaserDelay))                ++n;

	// 兼容旧字段：fPluse / fFrequency 既可能是 double 也可能是 int
	{
		int    iTmp = 0;
		double dTmp = 0.0;
		if (tryGetInt(t, "fPluse", iTmp))            { m_dLaserPulseWidth = iTmp; ++n; }
		else if (tryGetDouble(t, "fPluse", dTmp))    { m_dLaserPulseWidth = static_cast<int>(dTmp); ++n; }
		if (tryGetInt(t, "fFrequency", iTmp))        { m_dLaserFrequency  = iTmp; ++n; }
		else if (tryGetDouble(t, "fFrequency", dTmp)){ m_dLaserFrequency  = static_cast<int>(dTmp); ++n; }
	}

	// 激光延时
	if (tryGetDouble(t, "fBeforeOpenLaser",  m_dBeforeOn))  ++n;
	if (tryGetDouble(t, "fAfterOpenLaser",   m_dAfterOn))   ++n;
	if (tryGetDouble(t, "fBeforeCloseLaser", m_dBeforeOff)) ++n;
	if (tryGetDouble(t, "fAfterCloseLaser",  m_dAfterOff))  ++n;

	// ACS 拐角 / XSEG
	if (tryGetDouble(t, "fCornerVelocity", m_dJunctionVelocity)) ++n;
	if (tryGetDouble(t, "fCornerAngle",    m_dJunctionAngle))    ++n;
	if (tryGetDouble(t, "fXSEGVelocity",   m_dXsegEndVelocity))  ++n;

	// 平滑参数
	if (tryGetDouble(t, "fCutSmoothTime",  m_dCutSmoothTime))  ++n;
	if (tryGetDouble(t, "fCutSmoothK",     m_dCutSmoothK))     ++n;
	if (tryGetDouble(t, "fAxisSmoothTime", m_dAxisSmoothTime)) ++n;
	if (tryGetDouble(t, "fAxisSmoothK",    m_dAxisSmoothK))    ++n;

	// 高度
	if (tryGetDouble(t, "fCuttingHeight", m_dCuttingHeight)) ++n;
	if (tryGetDouble(t, "fIdleHeight",    m_dIdleZHeight))   ++n;

	// 通用开关
	if (tryGetBool(t, "bPunch",    m_bPunch))    ++n;
	if (tryGetBool(t, "bStopBlow", m_bStopBlow)) ++n;

	// 回零 / 移动
	if (tryGetBool  (t, "bSetPosA",   m_bAZero)) ++n;
	if (tryGetDouble(t, "fSetPosA",   m_dAPos))  ++n;
	if (tryGetBool  (t, "bSetPosA1",  m_bA1Zero)) ++n;
	if (tryGetDouble(t, "fSetPosA1",  m_dA1Pos))  ++n;
	if (tryGetBool  (t, "bMovePosX",  m_bXIsMove))    ++n;
	if (tryGetDouble(t, "fMovePosX",  m_dXPosition))  ++n;
	if (tryGetBool  (t, "bMovePosX1", m_bX1IsMove))   ++n;
	if (tryGetDouble(t, "fMovePosX1", m_dX1Position)) ++n;
	if (tryGetBool  (t, "bMovePosA",  m_bAIsMove))    ++n;
	if (tryGetDouble(t, "fMovePosA",  m_dAPosition))  ++n;
	if (tryGetBool  (t, "bMovePosA1", m_bA1IsMove))   ++n;
	if (tryGetDouble(t, "fMovePosA1", m_dA1Position)) ++n;
	if (tryGetBool  (t, "bMovePosY",  m_bYIsMove))    ++n;
	if (tryGetDouble(t, "fMovePosY",  m_dYPosition))  ++n;
	if (tryGetBool  (t, "bMovePosY1", m_bY1IsMove))   ++n;
	if (tryGetDouble(t, "fMovePosY1", m_dY1Position)) ++n;

	// 切割头 / 跨桥 / 随动
	if (tryGetBool  (t, "bCuttingHead",        m_bCuttingHead))       ++n;
	if (tryGetBool  (t, "bCrossBridge",        m_bCrossBridge))       ++n;
	if (tryGetDouble(t, "fServoCuttingHeight", m_dServoCuttingHeight)) ++n;

	// Z 联动
	if (tryGetBool  (t, "bAxisZLinkage",      m_bAxisZLinkage))      ++n;
	if (tryGetDouble(t, "fLinkedDelay",       m_dLinkedDelay))       ++n;
	if (tryGetString(t, "sLinkedDirection",   m_sLinkedDirection))   ++n;
	if (tryGetInt   (t, "iLinkedMode",        m_iLinkedMode))        ++n;
	if (tryGetDouble(t, "fLinkageParameterA", m_dLinkageParameterA)) ++n;
	if (tryGetDouble(t, "fLinkageParameterB", m_dLinkageParameterB)) ++n;
	if (tryGetString(t, "sLinkedFormula",     m_sLinkedFormula))     ++n;

	// 盲刻
	if (tryGetBool  (t, "bTrough",        m_bTroughFlag))                ++n;
	if (tryGetInt   (t, "iRunBuffer",     m_iTroughBuffer))              ++n;
	if (tryGetDouble(t, "fCHCompensate",  m_dCuttingHeightCompensate))   ++n;
	if (tryGetDouble(t, "fExtendSctart",  m_dExtend))                    ++n;
	if (tryGetDouble(t, "fExtendEnd",     m_dExtend_End))                ++n;
	if (tryGetDouble(t, "fDelay",         m_dTroughDelay))               ++n;

	// 飞切
	if (tryGetBool  (t, "bFlightCutting", m_bFlightCutting))             ++n;
	if (tryGetDouble(t, "fMotorDelay",    m_dFlightCutting_MotorDelay))  ++n;

	// 能量切换
	if (tryGetBool(t, "bEnergySwitch", m_bEnergySwitch)) ++n;

	// 工具名（可选；通常由调用方在外部 SetTool 索引处带入）
	if (tryGetString(t, "sName", m_strName)) ++n;

	return n;
}
