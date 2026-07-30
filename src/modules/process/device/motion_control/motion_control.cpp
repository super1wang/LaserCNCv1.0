#include "motion_control.h"
#include "modules/process/device/process_device_log.h"
#include "core/logging/logger.h"
#include "modules/process/settings/process_settings_service.h"
#include "modules/process/runtime/process_runtime_configuration.h"

const std::regex MotionControl::regex_DigitalIO("^(-)?(N)?\\d\\.\\d(\\d)?$");	// 匹配格式：(-) （N） 数字 . 数字 (数字)	，如N0.1、N1.23、-N0.1
const std::regex MotionControl::regex_AnalogIO("^(-)?(N)?\\d(\\d)?$");		// 匹配格式：(-) （N） 数字 (数字)		，如N1、N12、-N1


void MotionControl::rebuildAxes()
{
	for (const auto& eAxis : magic_enum::enum_values<Axis>())
	{
		if (!m_runtimeConfiguration.isAxisEnabled(eAxis))
			continue;

		if (!IsMotorCreated(eAxis))
		{
			string strAxis = enum_name(eAxis).data();
			table tableAxis = m_settings.axisRuntimeTable(QString::fromStdString(strAxis));
			CreateMotor(eAxis, tableAxis);
		}
	}
}
bool MotionControl::IsMotorCreated(Axis eAxis)
{
	for (Axis axis : m_vecMotors)
	{
		if (axis == eAxis)
			return true;
	}
	return false;
}

//void MotionControl::setMotionControlHomeTable()
//{
//	// 读取回零参数并下发（仅GTN控制器生效）
//	table tableAxisHome;
//	for (Axis eAxis : m_vecMotors)
//	{
//		string strAxis = enum_name(eAxis).data();
//		if (tableAxisHome.count("Home") && tableAxisHome.at("Home").is_table())
//			SetAxisHomePrm(eAxis, tableAxisHome.at("Home").as_table());
//	}
//}

void MotionControl::setMotionControlTable()
{
    table tableMotion;
	for (const Axis axis : m_vecMotors)
		tableMotion[enum_name(axis).data()] = m_settings.axisRuntimeTable(QString::fromStdString(enum_name(axis).data()));
	setMotionControlTable(tableMotion);
}

void MotionControl::setMotionControlTable(const table& tableMotion)
{
	if (!IsConnected())
		return;

	for (Axis eAxis : m_vecMotors)
	{
		string strAxis = enum_name(eAxis).data();
		if (tableMotion.count(strAxis))
			SetAxisTable(eAxis, tableMotion.at(strAxis).as_table());
	}
}

void MotionControl::SetAxisTable(Axis eAxis, const table& tableAxis)
{
	string strAxis = enum_name(eAxis).data();
	table t_Axis = m_settings.axisRuntimeTable(QString::fromStdString(strAxis));

	if (tableAxis.count("iIndex"))
	{
		int iIndex = t_Axis["iIndex"].as_integer();
		if (!SetAxisIndex(eAxis, iIndex))
			lcnc::process::logDeviceError(ErrorCode::ERROR_MC_SETTINGFAILED, QObject::tr("Set axis %1 sIndex %2 failed.").arg(strAxis.c_str()).arg(iIndex).toUtf8().data());
	}

	if (tableAxis.count("iHomeIndex"))
	{
		int iHomeIndex = t_Axis["iHomeIndex"].as_integer();
		if (!SetAxisHomeBufferIndex(eAxis, iHomeIndex))
			lcnc::process::logDeviceError(ErrorCode::ERROR_MC_SETTINGFAILED, QObject::tr("Set axis %1 home sIndex %2 failed.").arg(strAxis.c_str()).arg(iHomeIndex).toUtf8().data());
	}

	if (tableAxis.count("fResolution"))
	{
		double fResolution = t_Axis["fResolution"].as_floating();
		if (!SetAxisResolution(eAxis, (int)fResolution))
			lcnc::process::logDeviceError(ErrorCode::ERROR_MC_SETTINGFAILED, QObject::tr("Set axis %1 resolution %2 failed.").arg(strAxis.c_str()).arg(fResolution).toUtf8().data());
	}

	if (tableAxis.count("bRotation"))
	{
		bool bRotation = t_Axis["bRotation"].as_boolean();
		if (!SetAxisIsRotary(eAxis, bRotation))
			lcnc::process::logDeviceError(ErrorCode::ERROR_MC_SETTINGFAILED, QObject::tr("Set axis %1 is rotation %2 failed.").arg(strAxis.c_str()).arg(bRotation).toUtf8().data());
	}

	if (tableAxis.count("fVel"))
	{
		double fVel = t_Axis["fVel"].as_floating();
		if (!SetAxisVel(eAxis, fVel))
			lcnc::process::logDeviceError(ErrorCode::ERROR_MC_SETTINGFAILED, QObject::tr("Set axis %1 vel %2 failed.").arg(strAxis.c_str()).arg(fVel).toUtf8().data());
	}

	if (tableAxis.count("fAcc"))
	{
		double fAcc = t_Axis["fAcc"].as_floating();
		if (!SetAxisAcc(eAxis, fAcc))
			lcnc::process::logDeviceError(ErrorCode::ERROR_MC_SETTINGFAILED, QObject::tr("Set axis %1 acc %2 failed.").arg(strAxis.c_str()).arg(fAcc).toUtf8().data());
		if (!SetAxisDec(eAxis, fAcc))
			lcnc::process::logDeviceError(ErrorCode::ERROR_MC_SETTINGFAILED, QObject::tr("Set axis %1 dec %2 failed.").arg(strAxis.c_str()).arg(fAcc).toUtf8().data());
	}

	if (tableAxis.count("fJerk"))
	{
		double fJerk = t_Axis["fJerk"].as_floating();
		if (!SetAxisJerk(eAxis, fJerk))
			lcnc::process::logDeviceError(ErrorCode::ERROR_MC_SETTINGFAILED, QObject::tr("Set axis %1 jerk %2 failed.").arg(strAxis.c_str()).arg(fJerk).toUtf8().data());
	}
	
	bool bLimitChange = false;
	double fLeftLimit, fRightLimit;
    fLeftLimit = 0.0;
    fRightLimit = 0.0;

	if (tableAxis.count("fLeftLimit"))
	{
		bLimitChange = true;
		fLeftLimit = t_Axis["fLeftLimit"].as_floating();
	}

	if (tableAxis.count("fRightLimit"))
	{
		bLimitChange = true;
		fRightLimit = t_Axis["fRightLimit"].as_floating();
	}

	if (bLimitChange)
	{
		if (!SetAxisSoftLimit(eAxis, fLeftLimit, fRightLimit))
			lcnc::process::logDeviceError(ErrorCode::ERROR_MC_SETTINGFAILED, QObject::tr("Set axis %1 limit %2 ~ %3 failed.").arg(strAxis.c_str()).arg(fLeftLimit).arg(fRightLimit).toUtf8().data());
	}
}


void MotionControl::SetPipeDiameterTable()
{
	table tableAxis = m_settings.rawTable(lcnc::process::ProcessConfigArea::Operations, "Axis");
	SetPipeDiameterTable(tableAxis);
}

void MotionControl::SetPipeDiameterTable(const table& tableAxis)
{
	if (!IsConnected())
		return;

	for (Axis eAxis : m_vecMotors)
	{
		string strAxis = enum_name(eAxis).data();
		if (!tableAxis.count(strAxis))
			continue;
		bool bRotary;
		GetAxisIsRotary(eAxis, bRotary);
		if (bRotary && tableAxis.at(strAxis).count("fPipeDiameter"))
			if (!SetAxisTubeDiamater(eAxis, tableAxis.at(strAxis).at("fPipeDiameter").as_floating()))
				lcnc::process::logDeviceError(ErrorCode::ERROR_MC_SETTINGFAILED, QObject::tr("Set axis %1 pipe diameter failed.").arg(strAxis.c_str()).toUtf8().data());
	}
}

void MotionControl::rebuildIOMap(int iType)
{
	switch (iType)
	{
	case 11:
	{
		QStringList qlist;
		for (const auto& pair : m_mapDigitalIN)
		{
			m_mapqDigitalIN[pair.second.qstrID] = pair.second;
			qlist.append(pair.second.qstrID);
		}
		qlist.append("NONE");
		break;
	}
	case 10:
	{
		QStringList qlist;
		for (const auto& pair : m_mapDigitalOUT)
		{
			m_mapqDigitalOUT[pair.second.qstrID] = pair.second;
			qlist.append(pair.second.qstrID);
		}
		qlist.append("NONE");
		break;
	}
	case 21:
	{
		QStringList qlist;
		for (const auto& pair : m_mapAnalogIN)
		{
			m_mapqAnalogIN[pair.second.qstrID] = pair.second;
			qlist.append(pair.second.qstrID);
		}
		break;
	}
	case 20:
	{
		QStringList qlist;
		for (const auto& pair : m_mapAnalogOUT)
		{
			m_mapqAnalogOUT[pair.second.qstrID] = pair.second;
			qlist.append(pair.second.qstrID);
		}
	}
	default:
		break;
	}
}


void MotionControl::setDigitalTable()
{
	table tableDigital = m_settings.rawTable(lcnc::process::ProcessConfigArea::DigitalIo);
	setDigitalTable(tableDigital);
	rebuildIOMap(11);
	rebuildIOMap(10);
	// Custom channels have no legacy enum value.  Keep them in the string-keyed
	// map so workflow and UI references use their stable channel id directly.
	{
		auto append = [](map<QString, DigitalIOData>& destination, const lcnc::process::ProcessIoChannel& channel, bool input) {
			if (!channel.enabled || !std::regex_match(channel.hardwareIndex.toStdString(), MotionControl::regex_DigitalIO)) return;
			std::string index = channel.hardwareIndex.toStdString(); DigitalIOData data; data.qstrID = channel.id;
			if (!channel.activeHigh && !index.empty() && index.front() != '-') index.insert(index.begin(), '-');
			if (!index.empty() && index.front() == '-') { data.bInversion = true; index.erase(index.begin()); }
			if (!index.empty() && index.front() == 'N') { data.bExpand = true; index.erase(index.begin()); }
			data.iPort = std::stoi(index.substr(0, 1)); data.iIO = std::stoi(index.substr(2));
			data.strIndex = (input ? "IN" : "OUT") + index; destination[channel.id] = data;
		};
		for (const auto& channel : m_settings.ioChannels(lcnc::process::ProcessIoBucket::DigitalInput)) if (!channel.builtin) append(m_mapqDigitalIN, channel, true);
		for (const auto& channel : m_settings.ioChannels(lcnc::process::ProcessIoBucket::DigitalOutput)) if (!channel.builtin) append(m_mapqDigitalOUT, channel, false);
	}
}


// IO解析层，由array转map<enum, IOData>
namespace
{
	// 从一条 IO 记录里抽取 (name, index, active, enabled)。支持两种 schema：
	//   1. 新版 sub-table：{ name=..., index=..., active=true/false, enabled=true/false, ... }
	//   2. 旧版 array：[name, index]
	// 解析失败（既不是 table 也不是 array）时返回 false。
	struct IORecordFields
	{
		string sID;
		string sIndex;
		bool   bActive  { true };
		bool   bEnabled { true };
	};

	bool extractIORecord(const toml::value& v, IORecordFields& out)
	{
		if (v.is_table())
		{
			const auto& t = v.as_table();
			if (t.count("name"))
				out.sID = t.at("name").as_string();
			if (t.count("index"))
				out.sIndex = t.at("index").as_string();
			if (t.count("active"))
				out.bActive = t.at("active").as_boolean();
			if (t.count("enabled"))
				out.bEnabled = t.at("enabled").as_boolean();
			return true;
		}
		if (v.is_array())
		{
			const auto& a = v.as_array();
			if (a.size() >= 1 && a.at(0).is_string()) out.sID    = a.at(0).as_string();
			if (a.size() >= 2 && a.at(1).is_string()) out.sIndex = a.at(1).as_string();
			return true;
		}
		return false;
	}
}

void MotionControl::setDigitalTable(const table& tableDigital)
{
	table t_maps;
	if (tableDigital.count("DigitalIN"))
	{
		bool bRebuildMap = false;
		t_maps = tableDigital.at("DigitalIN").as_table();
		for (const auto& pair : t_maps)
		{
			string key = pair.first.data();
			auto eIndexOpt = enum_cast<DigitalIN>(key.substr(1, key.size() - 1));
			if (!eIndexOpt.has_value())
				continue;	// 未知扩展键，跳过
			DigitalIN eIndex = eIndexOpt.value();

			IORecordFields rec;
			if (!extractIORecord(pair.second, rec))
				continue;
			string sID    = rec.sID;
			string sIndex = rec.sIndex;
			// active=false 等价旧版索引前缀 '-'（取反）。
			if (!rec.bActive && !sIndex.empty() && sIndex.front() != '-')
				sIndex = "-" + sIndex;

			// enabled=false 当作记录无效，从 map 中移除。
			bool bInvalid = !rec.bEnabled || !std::regex_match(sIndex, regex_DigitalIO);
			if (!(sID.size() && sIndex.size()) || bInvalid)
			{
				if (rec.bEnabled && bInvalid && sIndex.size())
					LCNC_ERR(lcnc::LogCode::Generic, "{}", fmt::format("DigitalIN {} ID {} index {} is invalid", key, sID, sIndex));
				auto ei = m_mapDigitalIN.find(eIndex);
				if (ei != m_mapDigitalIN.end())
				{
					m_mapDigitalIN.erase(ei);
					bRebuildMap = true;
				}
				continue;
			}
			else if (sID.size() && sIndex.size())
			{
				// 索引没有该节点
				auto ei = m_mapDigitalIN.find(eIndex);
				if (ei == m_mapDigitalIN.end())
					bRebuildMap = true;
			}

			DigitalIOData IOData;
			IOData.qstrID = QString::fromStdString(sID);
			int iPos = 0;
			// 检查有无"-"设置信号取反
			if (sIndex.find('-') != std::string::npos)
			{
				IOData.bInversion = true;
				sIndex = sIndex.substr(1);
			}
			// 检查有无"N"走拓展模块
			if (sIndex.find('N') != std::string::npos)
			{
				IOData.bExpand = true;
				iPos++;
				IOData.strIndex = "NJHT_DI" + sIndex.substr(iPos);
				IOData.strPort	= "NJHT_DI" + sIndex.substr(iPos, sIndex.find('.') - 1);
				IOData.iPort	= stoi(sIndex.substr(iPos, 1));
				IOData.iIO		= stoi(sIndex.substr(iPos + 2));
			}
			else
			{
				IOData.strIndex = "IN" + sIndex.substr(iPos);
				IOData.iPort	= stoi(sIndex.substr(iPos, 1));
				IOData.iIO		= stoi(sIndex.substr(iPos + 2));
			}
			m_mapDigitalIN[eIndex] = IOData;
		}
		if(bRebuildMap)
			rebuildIOMap(11);
	}

	if (tableDigital.count("DigitalOUT"))
	{
		bool bRebuildMap = false;
		t_maps = tableDigital.at("DigitalOUT").as_table();
		for (const auto& pair : t_maps)
		{
			string key = pair.first.data();
			auto eIndexOpt = enum_cast<DigitalOUT>(key.substr(1, key.size() - 1));
			if (!eIndexOpt.has_value())
				continue;
			DigitalOUT eIndex = eIndexOpt.value();

			IORecordFields rec;
			if (!extractIORecord(pair.second, rec))
				continue;
			string sID    = rec.sID;
			string sIndex = rec.sIndex;
			if (!rec.bActive && !sIndex.empty() && sIndex.front() != '-')
				sIndex = "-" + sIndex;

			bool bInvalid = !rec.bEnabled || !std::regex_match(sIndex, regex_DigitalIO);
			if (!(sID.size() && sIndex.size()) || bInvalid)
			{
				if (rec.bEnabled && bInvalid && sIndex.size())
					LCNC_ERR(lcnc::LogCode::Generic, "{}", fmt::format("DigitalOUT {} ID {} index {} is invalid", key, sID, sIndex));
				auto ei = m_mapDigitalOUT.find(eIndex);
				if (ei != m_mapDigitalOUT.end())
				{
					m_mapDigitalOUT.erase(ei);
					bRebuildMap = true;
				}
				continue;
			}
			else if (sID.size() && sIndex.size())
			{
				// 索引没有该节点
				auto ei = m_mapDigitalOUT.find(eIndex);
				if (ei == m_mapDigitalOUT.end())
					bRebuildMap = true;
			}

			DigitalIOData IOData;
			IOData.qstrID = QString::fromStdString(sID);
			int iPos = 0;
			// 检查有无"-"设置信号取反
			if (sIndex.find('-') != std::string::npos)
			{
				IOData.bInversion = true;
				sIndex = sIndex.substr(1);
			}
			// 检查有无"N"走拓展模块
			if (sIndex.find('N') != std::string::npos)
			{
				IOData.bExpand = true;
				iPos++;
				IOData.strIndex = "NJHT_DO" + sIndex.substr(iPos);
				IOData.strPort	= "NJHT_DO" + sIndex.substr(iPos, sIndex.find('.') - 1);
				IOData.iIO		= stoi(sIndex.substr(iPos + 2));
			}
			else
			{
				IOData.strIndex = "OUT" + sIndex.substr(iPos);
				IOData.iPort	= stoi(sIndex.substr(iPos, 1));
				IOData.iIO		= stoi(sIndex.substr(iPos + 2));
			}
			m_mapDigitalOUT[eIndex] = IOData;
		}
		if (bRebuildMap)
			rebuildIOMap(10);
	}
}

void MotionControl::setAnalogTable()
{
	table tableAnalog = m_settings.rawTable(lcnc::process::ProcessConfigArea::AnalogIo);
	setAnalogTable(tableAnalog);
	rebuildIOMap(21);
	rebuildIOMap(20);
	{
		auto append = [](map<QString, AnalogIOData>& destination, const lcnc::process::ProcessIoChannel& channel, bool input) {
			if (!channel.enabled || !std::regex_match(channel.hardwareIndex.toStdString(), MotionControl::regex_AnalogIO)) return;
			std::string index = channel.hardwareIndex.toStdString(); AnalogIOData data; data.qstrID = channel.id;
			if (!index.empty() && index.front() == 'N') { data.bExpand = true; index.erase(index.begin()); }
			data.iPort = std::stoi(index); data.strIndex = (input ? "AIN" : "AOUT") + index; destination[channel.id] = data;
		};
		for (const auto& channel : m_settings.ioChannels(lcnc::process::ProcessIoBucket::AnalogInput)) if (!channel.builtin) append(m_mapqAnalogIN, channel, true);
		for (const auto& channel : m_settings.ioChannels(lcnc::process::ProcessIoBucket::AnalogOutput)) if (!channel.builtin) append(m_mapqAnalogOUT, channel, false);
	}
}

void MotionControl::setAnalogTable(const table& tableAnalog)
{
	table t_maps;

	if (tableAnalog.count("AnalogIN"))
	{
		bool bRebuildMap = false;
		t_maps = tableAnalog.at("AnalogIN").as_table();
		for (const auto& pair : t_maps)
		{
			string key = pair.first.data();
			auto eIndexOpt = enum_cast<AnalogIN>(key.substr(1, key.size() - 1));
			if (!eIndexOpt.has_value())
				continue;
			AnalogIN eIndex = eIndexOpt.value();

			IORecordFields rec;
			if (!extractIORecord(pair.second, rec))
				continue;
			string sID    = rec.sID;
			string sIndex = rec.sIndex;

			bool bInvalid = !rec.bEnabled || !std::regex_match(sIndex, regex_AnalogIO);
			if (!(sID.size() && sIndex.size()) || bInvalid)
			{
				if (rec.bEnabled && bInvalid && sIndex.size())
					LCNC_ERR(lcnc::LogCode::Generic, "{}", fmt::format("AnalogIN {} ID {} index {} is invalid", key, sID, sIndex));
				auto ei = m_mapAnalogIN.find(eIndex);
				if (ei != m_mapAnalogIN.end())
				{
					m_mapAnalogIN.erase(ei);
					bRebuildMap = true;
				}
				continue;
			}
			else if (sID.size() && sIndex.size())
			{
				// 索引没有该节点
				auto ei = m_mapAnalogIN.find(eIndex);
				if (ei == m_mapAnalogIN.end())
					bRebuildMap = true;
			}

			AnalogIOData IOData;
			IOData.qstrID = QString::fromStdString(sID);
			int iPos = 0;
			// 检查有无"N"走拓展模块
			if (sIndex.find('N') != std::string::npos)
			{
				IOData.bExpand = true;
				iPos++;
				IOData.strIndex = "NJHT_AIN" + sIndex.substr(iPos);
				IOData.strPort 	= "NJHT_AIN" + sIndex.substr(iPos);
				IOData.iPort = stoi(sIndex.substr(iPos));
			}
			else
			{
				IOData.strIndex = "AIN" + sIndex.substr(iPos);
				IOData.iPort	= stoi(sIndex.substr(iPos));
			}
			m_mapAnalogIN[eIndex] = IOData;
		}
		if (bRebuildMap)
			rebuildIOMap(21);
	}

	if (tableAnalog.count("AnalogOUT"))
	{
		bool bRebuildMap = false;
		t_maps = tableAnalog.at("AnalogOUT").as_table();
		for (const auto& pair : t_maps)
		{
			string key = pair.first.data();
			auto eIndexOpt = enum_cast<AnalogOUT>(key.substr(1, key.size() - 1));
			if (!eIndexOpt.has_value())
				continue;
			AnalogOUT eIndex = eIndexOpt.value();

			IORecordFields rec;
			if (!extractIORecord(pair.second, rec))
				continue;
			string sID    = rec.sID;
			string sIndex = rec.sIndex;

			bool bInvalid = !rec.bEnabled || !std::regex_match(sIndex, regex_AnalogIO);
			if (!(sID.size() && sIndex.size()) || bInvalid)
			{
				if (rec.bEnabled && bInvalid && sIndex.size())
					LCNC_ERR(lcnc::LogCode::Generic, "{}", fmt::format("AnalogOUT {} ID {} index {} is invalid", key, sID, sIndex));
				auto ei = m_mapAnalogOUT.find(eIndex);
				if (ei != m_mapAnalogOUT.end())
				{
					m_mapAnalogOUT.erase(ei);
					bRebuildMap = true;
				}
				continue;
			}
			else if (sID.size() && sIndex.size())
			{
				// 索引没有该节点
				auto ei = m_mapAnalogOUT.find(eIndex);
				if (ei == m_mapAnalogOUT.end())
					bRebuildMap = true;
			}

			AnalogIOData IOData;
			IOData.qstrID = QString::fromStdString(sID);
			int iPos = 0;
			// 检查有无"N"走拓展模块
			if (sIndex.find('N') != std::string::npos)
			{
				IOData.bExpand	= true;
				iPos++;
				IOData.strIndex = "NJHT_AOUT" + sIndex.substr(iPos);
				IOData.strPort	= "NJHT_AOUT" + sIndex.substr(iPos);
				IOData.iPort	= stoi(sIndex.substr(iPos));
			}
			else
			{
				IOData.strIndex = "AOUT" + sIndex.substr(iPos);
				IOData.iPort	= stoi(sIndex.substr(iPos));
			}
			m_mapAnalogOUT[eIndex] = IOData;
		}
		if (bRebuildMap)
			rebuildIOMap(20);
	}
}

ErrorCode MotionControl::SetLaserParameterTable()
{
	table tableLaser = m_settings.rawTable(lcnc::process::ProcessConfigArea::Devices, "Laser");
	return SetLaserParameterTable(tableLaser);
}

ErrorCode MotionControl::SetLaserParameterTable(const table& tableLaser)
{
    // GSN PWM 信号源已废弃。激光参数由 ProcessSettings → AcsMotionControllerAdapter / 实际激光器
    // 路径下发，这里保留接口以维持 Setting 层向下传 table 的语义兼容。
    Q_UNUSED(tableLaser);
    return ErrorCode::ERROR_NONE;
}

bool MotionControl::DigitalOutputSet(DigitalOUT eIOIndex, int iValue, bool bLogError)
{
	if (m_mapDigitalOUT.count(eIOIndex))
		return DigitalOutputSet(m_mapDigitalOUT[eIOIndex], iValue, bLogError);
	else
	{
		lcnc::process::logDeviceWarning(WarnCode::WARN_MC_NONEINDEX, QObject::tr("Digital OUT %1 is invalid").arg(enum_name(eIOIndex).data()));
		return false;
	}
}

bool MotionControl::DigitalOutputGet(DigitalOUT eIOIndex, int& iValue, bool bLogError)
{
	if (m_mapDigitalOUT.count(eIOIndex))
		return DigitalOutputGet(m_mapDigitalOUT[eIOIndex], iValue, bLogError);
	else
	{
		lcnc::process::logDeviceWarning(WarnCode::WARN_MC_NONEINDEX, QObject::tr("Digital OUT %1 is invalid").arg(enum_name(eIOIndex).data()));
		return false;
	}
}

bool MotionControl::DigitalInputGet(DigitalIN eIOIndex, int& iValue, bool bLogError)
{
	if (m_mapDigitalIN.count(eIOIndex))
		return DigitalInputGet(m_mapDigitalIN[eIOIndex], iValue, bLogError);
	else
	{
		lcnc::process::logDeviceWarning(WarnCode::WARN_MC_NONEINDEX, QObject::tr("Digital IN %1 is invalid").arg(enum_name(eIOIndex).data()));
		return false;
	}
}

bool MotionControl::AnalogOutputSet(AnalogOUT eIOIndex, double dValue, bool bLogError)
{
	if (m_mapAnalogOUT.count(eIOIndex))
		return AnalogOutputSet(m_mapAnalogOUT[eIOIndex], dValue, bLogError);
	else
	{
		lcnc::process::logDeviceWarning(WarnCode::WARN_MC_NONEINDEX, QObject::tr("Analog OUT %1 is invalid").arg(enum_name(eIOIndex).data()));
		return false;
	}
}

bool MotionControl::AnalogOutputGet(AnalogOUT eIOIndex, double& dValue, bool bLogError)
{
	if (m_mapAnalogOUT.count(eIOIndex))
		return AnalogOutputGet(m_mapAnalogOUT[eIOIndex], dValue, bLogError);
	else
	{
		lcnc::process::logDeviceWarning(WarnCode::WARN_MC_NONEINDEX, QObject::tr("Analog OUT %1 is invalid").arg(enum_name(eIOIndex).data()));
		return false;
	}
}

bool MotionControl::AnalogInputGet(AnalogIN eIOIndex, double& dValue, bool bLogError)
{
	if (m_mapAnalogIN.count(eIOIndex))
		return AnalogInputGet(m_mapAnalogIN[eIOIndex], dValue, bLogError);
	else
	{
		lcnc::process::logDeviceWarning(WarnCode::WARN_MC_NONEINDEX, QObject::tr("Analog IN %1 is invalid").arg(enum_name(eIOIndex).data()));
		return false;
	}
}

bool MotionControl::DigitalOutputSet(QString qstrName, int iValue, bool bLogError)
{
	if (m_mapqDigitalOUT.count(qstrName))
		return DigitalOutputSet(m_mapqDigitalOUT[qstrName], iValue, bLogError);
	else
	{
		lcnc::process::logDeviceWarning(WarnCode::WARN_MC_NONEINDEX, QObject::tr("Digital OUT %1 is invalid").arg(qstrName));
		return false;
	}
}

bool MotionControl::DigitalOutputGet(QString qstrName, int& iValue, bool bLogError)
{
	if (m_mapqDigitalOUT.count(qstrName))
		return DigitalOutputGet(m_mapqDigitalOUT[qstrName], iValue, bLogError);
	else
	{
		lcnc::process::logDeviceWarning(WarnCode::WARN_MC_NONEINDEX, QObject::tr("Digital OUT %1 is invalid").arg(qstrName));
		return false;
	}
}

bool MotionControl::DigitalInputGet(QString	qstrName, int& iValue, bool bLogError)
{
	if (m_mapqDigitalIN.count(qstrName))
		return DigitalInputGet(m_mapqDigitalIN[qstrName], iValue, bLogError);
	else
	{
		lcnc::process::logDeviceWarning(WarnCode::WARN_MC_NONEINDEX, QObject::tr("Digital IN %1 is invalid").arg(qstrName));
		return false;
	}
}

bool MotionControl::AnalogOutputSet(QString	qstrName, double dValue, bool bLogError)
{
	if (m_mapqAnalogOUT.count(qstrName))
		return AnalogOutputSet(m_mapqAnalogOUT[qstrName], dValue, bLogError);
	else
	{
		lcnc::process::logDeviceWarning(WarnCode::WARN_MC_NONEINDEX, QObject::tr("Analog OUT %1 is invalid").arg(qstrName));
		return false;
	}
}

bool MotionControl::AnalogOutputGet(QString	qstrName, double& dValue, bool bLogError)
{
	if (m_mapqAnalogOUT.count(qstrName))
		return AnalogOutputGet(m_mapqAnalogOUT[qstrName], dValue, bLogError);
	else
	{
		lcnc::process::logDeviceWarning(WarnCode::WARN_MC_NONEINDEX, QObject::tr("Analog OUT %1 is invalid").arg(qstrName));
		return false;
	}
}

bool MotionControl::AnalogInputGet(QString	qstrName, double& dValue, bool bLogError)
{
	if (m_mapqAnalogIN.count(qstrName))
		return AnalogInputGet(m_mapqAnalogIN[qstrName], dValue, bLogError);
	else
	{
		lcnc::process::logDeviceWarning(WarnCode::WARN_MC_NONEINDEX, QObject::tr("Analog IN %1 is invalid").arg(qstrName));
		return false;
	}
}
