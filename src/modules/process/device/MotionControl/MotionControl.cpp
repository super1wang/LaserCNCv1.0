#include "MotionControl.h"

const std::regex MotionControl::regex_DigitalIO("^(-)?(N)?\\d\\.\\d(\\d)?$");	// 匹配格式：(-) （N） 数字 . 数字 (数字)	，如N0.1、N1.23、-N0.1
const std::regex MotionControl::regex_AnalogIO("^(-)?(N)?\\d(\\d)?$");		// 匹配格式：(-) （N） 数字 (数字)		，如N1、N12、-N1


void MotionControl::rebuildAxes()
{
	for (const auto& eAxis : magic_enum::enum_values<Axis>())
	{
		if (!DT::IsAxisUse(eAxis))
			continue;

		if (!IsMotorCreated(eAxis))
		{
			string strAxis = enum_name(eAxis).data();
			table tableAxis = SETTINGS->GetTable(SettingSection::MotionControl, strAxis);
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

//void MotionControl::SetMotionControlHomeTable()
//{
//	// 读取回零参数并下发（仅GTN控制器生效）
//	table tableAxisHome;
//	for (Axis eAxis : m_vecMotors)
//	{
//		string strAxis = enum_name(eAxis).data();
//		tableAxisHome = SETTINGS->GetTable(SettingSection::MotionControl, strAxis);
//		if (tableAxisHome.count("Home") && tableAxisHome.at("Home").is_table())
//			SetAxisHomePrm(eAxis, tableAxisHome.at("Home").as_table());
//	}
//}

void MotionControl::SetMotionControlTable()
{
	table tableMotion = SETTINGS->GetTable(SettingSection::MotionControl);
	SetMotionControlTable(tableMotion);
}

void MotionControl::SetMotionControlTable(const table& tableMotion)
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
	table t_Axis = SETTINGS->GetTable(SettingSection::MotionControl, strAxis);

	if (tableAxis.count("iIndex"))
	{
		int iIndex = t_Axis["iIndex"].as_integer();
		if (!SetAxisIndex(eAxis, iIndex))
			SHOW_OPER_ERROR(ErrorCode::ERROR_MC_SETTINGFAILED, QObject::tr("Set axis %1 sIndex %2 failed.").arg(strAxis.c_str()).arg(iIndex).toUtf8().data());
	}

	if (tableAxis.count("iHomeIndex"))
	{
		int iHomeIndex = t_Axis["iHomeIndex"].as_integer();
		if (!SetAxisHomeBufferIndex(eAxis, iHomeIndex))
			SHOW_OPER_ERROR(ErrorCode::ERROR_MC_SETTINGFAILED, QObject::tr("Set axis %1 home sIndex %2 failed.").arg(strAxis.c_str()).arg(iHomeIndex).toUtf8().data());
	}

	if (tableAxis.count("fResolution"))
	{
		double fResolution = t_Axis["fResolution"].as_floating();
		if (!SetAxisResolution(eAxis, (int)fResolution))
			SHOW_OPER_ERROR(ErrorCode::ERROR_MC_SETTINGFAILED, QObject::tr("Set axis %1 resolution %2 failed.").arg(strAxis.c_str()).arg(fResolution).toUtf8().data());
	}

	if (tableAxis.count("bRotation"))
	{
		bool bRotation = t_Axis["bRotation"].as_boolean();
		if (!SetAxisIsRotary(eAxis, bRotation))
			SHOW_OPER_ERROR(ErrorCode::ERROR_MC_SETTINGFAILED, QObject::tr("Set axis %1 is rotation %2 failed.").arg(strAxis.c_str()).arg(bRotation).toUtf8().data());
	}

	if (tableAxis.count("fVel"))
	{
		double fVel = t_Axis["fVel"].as_floating();
		if (!SetAxisVel(eAxis, fVel))
			SHOW_OPER_ERROR(ErrorCode::ERROR_MC_SETTINGFAILED, QObject::tr("Set axis %1 vel %2 failed.").arg(strAxis.c_str()).arg(fVel).toUtf8().data());
	}

	if (tableAxis.count("fAcc"))
	{
		double fAcc = t_Axis["fAcc"].as_floating();
		if (!SetAxisAcc(eAxis, fAcc))
			SHOW_OPER_ERROR(ErrorCode::ERROR_MC_SETTINGFAILED, QObject::tr("Set axis %1 acc %2 failed.").arg(strAxis.c_str()).arg(fAcc).toUtf8().data());
		if (!SetAxisDec(eAxis, fAcc))
			SHOW_OPER_ERROR(ErrorCode::ERROR_MC_SETTINGFAILED, QObject::tr("Set axis %1 dec %2 failed.").arg(strAxis.c_str()).arg(fAcc).toUtf8().data());
	}

	if (tableAxis.count("fJerk"))
	{
		double fJerk = t_Axis["fJerk"].as_floating();
		if (!SetAxisJerk(eAxis, fJerk))
			SHOW_OPER_ERROR(ErrorCode::ERROR_MC_SETTINGFAILED, QObject::tr("Set axis %1 jerk %2 failed.").arg(strAxis.c_str()).arg(fJerk).toUtf8().data());
	}
	
	bool bLimitChange = false;
	double fLeftLimit, fRightLimit;
	SETTINGS->GetKeyValue("fLeftLimit",  fLeftLimit,  SettingSection::MotionControl, strAxis);
	SETTINGS->GetKeyValue("fRightLimit", fRightLimit, SettingSection::MotionControl, strAxis);

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
			SHOW_OPER_ERROR(ErrorCode::ERROR_MC_SETTINGFAILED, QObject::tr("Set axis %1 limit %2 ~ %3 failed.").arg(strAxis.c_str()).arg(fLeftLimit).arg(fRightLimit).toUtf8().data());
	}
}


void MotionControl::SetPipeDiameterTable()
{
	table tableAxis = SETTINGS->GetTable(SettingSection::Axis);
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
				SHOW_OPER_ERROR(ErrorCode::ERROR_MC_SETTINGFAILED, QObject::tr("Set axis %1 pipe diameter failed.").arg(strAxis.c_str()).toUtf8().data());
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
		DT::setDigitalINList(qlist);
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
		DT::setDigitalOUTList(qlist);
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
		DT::setAnalogINList(qlist);
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
		DT::setAnalogOUTList(qlist);
	}
	default:
		break;
	}
}


void MotionControl::SetDigitalTable()
{
	table tableDigital = SETTINGS->GetTable(SettingSection::Digital);
	SetDigitalTable(tableDigital);
	rebuildIOMap(11);
	rebuildIOMap(10);
}


// IO解析层，由array转map<enum, IOData>
void MotionControl::SetDigitalTable(const table& tableDigital)
{
	table t_maps;
	if (tableDigital.count("DigitalIN"))
	{
		bool bRebuildMap = false;
		t_maps = tableDigital.at("DigitalIN").as_table();
		for (const auto& pair : t_maps)
		{
			string key = pair.first.data();
			DigitalIN eIndex = enum_cast<DigitalIN>(key.substr(1, key.size() - 1)).value();

			string sID = pair.second[0].as_string();
			string sIndex = pair.second[1].as_string();

			// 昵称及索引必须均有，索引结构同理
			bool bInvalid = !std::regex_match(sIndex, regex_DigitalIO);
			if (!(sID.size() && sIndex.size()) || bInvalid)
			{
				if (bInvalid && sIndex.size())
					LOG_OPER_ERROR(fmt::format("DigitalIN {} ID {} index {} is invalid", key, sID, sIndex));
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
			DigitalOUT eIndex = enum_cast<DigitalOUT>(key.substr(1, key.size() - 1)).value();

			string sID = pair.second[0].as_string();
			string sIndex = pair.second[1].as_string();

			// 昵称及索引必须均有，索引结构同理
			bool bInvalid = !std::regex_match(sIndex, regex_DigitalIO);
			if (!(sID.size() && sIndex.size()) || bInvalid)
			{
				if (bInvalid && sIndex.size())
					LOG_OPER_ERROR(fmt::format("DigitalOUT {} ID {} index {} is invalid", key, sID, sIndex));
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

void MotionControl::SetAnalogTable()
{
	table tableAnalog = SETTINGS->GetTable(SettingSection::Analog);
	SetAnalogTable(tableAnalog);
	rebuildIOMap(21);
	rebuildIOMap(20);
}

void MotionControl::SetAnalogTable(const table& tableAnalog)
{
	table t_maps;

	if (tableAnalog.count("AnalogIN"))
	{
		bool bRebuildMap = false;
		t_maps = tableAnalog.at("AnalogIN").as_table();
		for (const auto& pair : t_maps)
		{
			string key = pair.first.data();
			AnalogIN eIndex = enum_cast<AnalogIN>(key.substr(1, key.size() - 1)).value();

			string sID = pair.second[0].as_string();
			string sIndex = pair.second[1].as_string();

			// 昵称及索引必须均有，索引结构同理
			bool bInvalid = !std::regex_match(sIndex, regex_AnalogIO);
			if (!(sID.size() && sIndex.size()) || bInvalid)
			{
				if (bInvalid && sIndex.size())
					LOG_OPER_ERROR(fmt::format("AnalogIN {} ID {} index {} is invalid", key, sID, sIndex));
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
			AnalogOUT eIndex = enum_cast<AnalogOUT>(key.substr(1, key.size() - 1)).value();

			string sID = pair.second[0].as_string();
			string sIndex = pair.second[1].as_string();

			// 昵称及索引缺一则无效，同时需要删除已有的
			bool bInvalid = !std::regex_match(sIndex, regex_AnalogIO);
			if (!(sID.size() && sIndex.size()) || bInvalid)
			{
				if (bInvalid && sIndex.size())
					LOG_OPER_ERROR(fmt::format("AnalogOUT {} ID {} index {} is invalid", key, sID, sIndex));
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
	table tableLaser = SETTINGS->GetTable(SettingSection::Laser);
	return SetLaserParameterTable(tableLaser);
}

ErrorCode MotionControl::SetLaserParameterTable(const table& tableLaser)
{
	ErrorCode eCode = ErrorCode::ERROR_NONE;
	if (tableLaser.count("Laser"))
	{
		table t_Laser = SETTINGS->GetTable(SettingSection::Laser, "Laser");
		/*if (tableLaser.count("fFrequency") && tableLaser.count("fPulseWidth"))*/
		{
			double dFrequency = t_Laser["fFrequency"].as_floating();
			double dPulseWidth = t_Laser["fPulseWidth"].as_floating();
			if (!GSN_SetLaserParameterApplication(dFrequency, dPulseWidth, 0.0))
			{
				eCode = ErrorCode::ERROR_LASER_SETTINGFAILED;
				SHOW_OPER_ERROR(ErrorCode::ERROR_LASER_SETTINGFAILED,
					QObject::tr("Set signal source frequency %1 pulse width %2 failed.")
					.arg(dFrequency).arg(dPulseWidth).toUtf8().data());
			}
		}
	}
	return eCode;
}

bool MotionControl::DigitalOutputSet(DigitalOUT eIOIndex, int iValue, bool bLogError)
{
	if (m_mapDigitalOUT.count(eIOIndex))
		return DigitalOutputSet(m_mapDigitalOUT[eIOIndex], iValue, bLogError);
	else
	{
		SHOW_SYS_WARN(WarnCode::WARN_MC_NONEINDEX, QObject::tr("Digital OUT %1 is invalid").arg(enum_name(eIOIndex).data()));
		return false;
	}
}

bool MotionControl::DigitalOutputGet(DigitalOUT eIOIndex, int& iValue, bool bLogError)
{
	if (m_mapDigitalOUT.count(eIOIndex))
		return DigitalOutputGet(m_mapDigitalOUT[eIOIndex], iValue, bLogError);
	else
	{
		SHOW_SYS_WARN(WarnCode::WARN_MC_NONEINDEX, QObject::tr("Digital OUT %1 is invalid").arg(enum_name(eIOIndex).data()));
		return false;
	}
}

bool MotionControl::DigitalInputGet(DigitalIN eIOIndex, int& iValue, bool bLogError)
{
	if (m_mapDigitalIN.count(eIOIndex))
		return DigitalInputGet(m_mapDigitalIN[eIOIndex], iValue, bLogError);
	else
	{
		SHOW_SYS_WARN(WarnCode::WARN_MC_NONEINDEX, QObject::tr("Digital IN %1 is invalid").arg(enum_name(eIOIndex).data()));
		return false;
	}
}

bool MotionControl::AnalogOutputSet(AnalogOUT eIOIndex, double dValue, bool bLogError)
{
	if (m_mapAnalogOUT.count(eIOIndex))
		return AnalogOutputSet(m_mapAnalogOUT[eIOIndex], dValue, bLogError);
	else
	{
		SHOW_SYS_WARN(WarnCode::WARN_MC_NONEINDEX, QObject::tr("Analog OUT %1 is invalid").arg(enum_name(eIOIndex).data()));
		return false;
	}
}

bool MotionControl::AnalogOutputGet(AnalogOUT eIOIndex, double& dValue, bool bLogError)
{
	if (m_mapAnalogOUT.count(eIOIndex))
		return AnalogOutputGet(m_mapAnalogOUT[eIOIndex], dValue, bLogError);
	else
	{
		SHOW_SYS_WARN(WarnCode::WARN_MC_NONEINDEX, QObject::tr("Analog OUT %1 is invalid").arg(enum_name(eIOIndex).data()));
		return false;
	}
}

bool MotionControl::AnalogInputGet(AnalogIN eIOIndex, double& dValue, bool bLogError)
{
	if (m_mapAnalogIN.count(eIOIndex))
		return AnalogInputGet(m_mapAnalogIN[eIOIndex], dValue, bLogError);
	else
	{
		SHOW_SYS_WARN(WarnCode::WARN_MC_NONEINDEX, QObject::tr("Analog IN %1 is invalid").arg(enum_name(eIOIndex).data()));
		return false;
	}
}

bool MotionControl::DigitalOutputSet(QString qstrName, int iValue, bool bLogError)
{
	if (m_mapqDigitalOUT.count(qstrName))
		return DigitalOutputSet(m_mapqDigitalOUT[qstrName], iValue, bLogError);
	else
	{
		SHOW_SYS_WARN(WarnCode::WARN_MC_NONEINDEX, QObject::tr("Digital OUT %1 is invalid").arg(qstrName));
		return false;
	}
}

bool MotionControl::DigitalOutputGet(QString qstrName, int& iValue, bool bLogError)
{
	if (m_mapqDigitalOUT.count(qstrName))
		return DigitalOutputGet(m_mapqDigitalOUT[qstrName], iValue, bLogError);
	else
	{
		SHOW_SYS_WARN(WarnCode::WARN_MC_NONEINDEX, QObject::tr("Digital OUT %1 is invalid").arg(qstrName));
		return false;
	}
}

bool MotionControl::DigitalInputGet(QString	qstrName, int& iValue, bool bLogError)
{
	if (m_mapqDigitalIN.count(qstrName))
		return DigitalInputGet(m_mapqDigitalIN[qstrName], iValue, bLogError);
	else
	{
		SHOW_SYS_WARN(WarnCode::WARN_MC_NONEINDEX, QObject::tr("Digital IN %1 is invalid").arg(qstrName));
		return false;
	}
}

bool MotionControl::AnalogOutputSet(QString	qstrName, double dValue, bool bLogError)
{
	if (m_mapqAnalogOUT.count(qstrName))
		return AnalogOutputSet(m_mapqAnalogOUT[qstrName], dValue, bLogError);
	else
	{
		SHOW_SYS_WARN(WarnCode::WARN_MC_NONEINDEX, QObject::tr("Analog OUT %1 is invalid").arg(qstrName));
		return false;
	}
}

bool MotionControl::AnalogOutputGet(QString	qstrName, double& dValue, bool bLogError)
{
	if (m_mapqAnalogOUT.count(qstrName))
		return AnalogOutputGet(m_mapqAnalogOUT[qstrName], dValue, bLogError);
	else
	{
		SHOW_SYS_WARN(WarnCode::WARN_MC_NONEINDEX, QObject::tr("Analog OUT %1 is invalid").arg(qstrName));
		return false;
	}
}

bool MotionControl::AnalogInputGet(QString	qstrName, double& dValue, bool bLogError)
{
	if (m_mapqAnalogIN.count(qstrName))
		return AnalogInputGet(m_mapqAnalogIN[qstrName], dValue, bLogError);
	else
	{
		SHOW_SYS_WARN(WarnCode::WARN_MC_NONEINDEX, QObject::tr("Analog IN %1 is invalid").arg(qstrName));
		return false;
	}
}

