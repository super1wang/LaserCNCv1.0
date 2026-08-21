/************************************************************************/
/*                            IPG激光器实现类                           */
/************************************************************************/

#include "ipg_laser_device.h"
#include "modules/process/device/process_device_log.h"
#include <string>
#include <stdlib.h>
//#include <cstringt.h>
#include <sstream>
#include <fstream>
using std::ostringstream;
using namespace std;
using toml::table;

IPGLaserDevice::IPGLaserDevice(lcnc::process::ProcessSettingsService& settings) : LaserDevice(settings), m_strName("IPG"), m_bIsInited(false), m_dMaxCurrent(0),
                                   m_dSimmerCurrent(0), m_iWaveShape(0), m_strTemperature("")
{
}

ErrorCode IPGLaserDevice::setLaserTable(const table& tableLaser)
{
	bool bConnectChange = false;

	// 串口
	ErrorCode eCode = ErrorCode::ERROR_NONE;
	if (tableLaser.count("ComSetting"))
	{
        table tCom = processLaserTable(QStringLiteral("ComSetting"));
		eCode = SetComTable(tCom, bConnectChange);
		if (eCode != ErrorCode::ERROR_NONE)
			return ErrorCode::ERROR_LASER_CONNECTIONFAILED;
	}

	// 参数
	if (tableLaser.count("Laser") || bConnectChange)
	{
		table tLaser;
		if (bConnectChange)
            tLaser = processLaserTable(QStringLiteral("Laser"));
		else
			tLaser = tableLaser.at("Laser").as_table();

		if (tLaser.count("fEnergy"))
		{
			double dEnergy = tLaser["fEnergy"].as_floating();
			if (!SetEnergy(dEnergy))
				eCode = ErrorCode::ERROR_LASER_SETTINGFAILED;
		}
		bool bSignalSource = false;
        bSignalSource = processLaserValue(QStringLiteral("SignalSource"), QStringLiteral("bSignal"), false).toBool();
		if (!bSignalSource && tLaser.count("fFrequency"))
		{
			double dFrequency = tLaser["fFrequency"].as_floating();
			if (!SetFrequency(dFrequency))
				eCode = ErrorCode::ERROR_LASER_SETTINGFAILED;
		}
		if (!bSignalSource && tLaser.count("fPulseWidth"))
		{
			double dPulseWidth = tLaser["fPulseWidth"].as_floating();
			if (!SetPulseWidth(dPulseWidth))
				eCode = ErrorCode::ERROR_LASER_SETTINGFAILED;
		}
	}
	if (eCode == ErrorCode::ERROR_NONE)
		m_bIsInited = true;
	else
		m_bIsInited = false;
	return eCode;
}

const string & IPGLaserDevice::GetName()
{
	return m_strName;
}

bool IPGLaserDevice::IsInited()
{
	return m_bIsInited;
}

bool IPGLaserDevice::IsAvailableData(const QByteArray& data)
{
	if (data.isEmpty())
		return false;

	// 检查是否以 \r 结尾
	if (data.endsWith('\r'))
		return true;

	return false;
}

bool IPGLaserDevice::StartLaser()
{
	return OnceData("EMON\r");
}

bool IPGLaserDevice::StopLaser()
{
	return OnceData("EMOFF\r");
}

bool IPGLaserDevice::StartAimingBeam()
{
	return OnceData("ABN\r");
}

bool IPGLaserDevice::StopAimingBeam()
{
	return OnceData("ABF\r");
}

bool IPGLaserDevice::SetEnergy(double dEnergy)
{
	double dSetEnergy;
	dSetEnergy = dEnergy;
	if ((dSetEnergy >= 0) && (dSetEnergy <= 100))
	{
		char cpEnergy[50];
		char buffer[_CVTBUFSIZE];
		ostringstream oss;
		string strEnergy;
		oss.str("");
		oss << dSetEnergy;
		strEnergy = oss.str();
		strcpy_s(buffer, strEnergy.c_str());
		const char* head = "SDC";
		const char* entr = "\r";
		strcpy_s(cpEnergy, head);
		strcat_s(cpEnergy, buffer);
		strcat_s(cpEnergy, entr);
		bool bResult = OnceData(cpEnergy);
		if (!bResult)
			lcnc::process::logDeviceError(ErrorCode::ERROR_LASER_SETTINGFAILED, QObject::tr("Set laser energy %1 failed.").arg(dEnergy).toUtf8().data());
		else
			LaserDevice::UpdateEnergy(dEnergy);
		return bResult;
	}
	else
	{
		lcnc::process::logDeviceError(ErrorCode::ERROR_LASER_SETTINGFAILED, QObject::tr("The laser energy %1 does not meet the requirement of 0 to 100.").arg(dEnergy).toUtf8().data());
		return false;
	}
}

bool IPGLaserDevice::SetFrequency(double dFrequency)
{
	char cpFrequency[50];
	char buffer[_CVTBUFSIZE];
	ostringstream oss;
	string strFrequency;
	oss.str("");
	oss << (int)dFrequency;
	strFrequency = oss.str();

	strcpy_s(buffer, strFrequency.c_str());
	const char* head = "SPRR";
	const char* entr = "\r";
	strcpy_s(cpFrequency, head);
	strcat_s(cpFrequency, buffer);
	strcat_s(cpFrequency, entr);
	bool bResult = OnceData(cpFrequency);
	if (!bResult)
		lcnc::process::logDeviceError(ErrorCode::ERROR_LASER_SETTINGFAILED, QObject::tr("Set laser frequency %1 failed.").arg(dFrequency).toUtf8().data());
	else
		LaserDevice::UpdateFrequency(dFrequency);
	return bResult;
}

bool IPGLaserDevice::SetPulseWidth(double dPulseWidth)
{
	char cpPulse[50];
	char buffer[_CVTBUFSIZE];
	ostringstream oss;
	string strPulse;
	oss.str("");
	oss << dPulseWidth / 1000.0;
	strPulse = oss.str();
	strcpy_s(buffer, strPulse.c_str());
	const char* head = "SPW";
	const char* entr = "\r";
	strcpy_s(cpPulse, head);
	strcat_s(cpPulse, buffer);
	strcat_s(cpPulse, entr);
	bool bResult = OnceData(cpPulse);
	if (!bResult)
		lcnc::process::logDeviceError(ErrorCode::ERROR_LASER_SETTINGFAILED, QObject::tr("Set laser pulse width %1 failed.").arg(dPulseWidth).toUtf8().data());
	else
		LaserDevice::UpdatePulseWidth(dPulseWidth);
	return bResult;
}

bool IPGLaserDevice::SetLaserParameter(const LaserParameter& parameter)
{
	if (!SetPulseWidth(parameter.dPulseWidth))
		return false;
	if (!SetFrequency(parameter.dFrequency))
		return false;
	if (!SetPulseWidth(parameter.dPulseWidth))
		return false;
	if (!SetFrequency(parameter.dFrequency))
		return false;
	if (!SetEnergy(parameter.dEnergy))
		return false;
	LaserDevice::UpdateLaserParameter(parameter);
	return true;
}

string IPGLaserDevice::GetEnergy()
{
	QByteArray strCommand("RCS\r");
	QByteArray strOut;
	bool success = OnceData(strCommand, strOut);
	success = success && !strOut.isEmpty();

	if (success && strOut.size() > 5)
	{
		string str = strOut.mid(5).toStdString();  // 从第5个字符开始到结尾
		return str;
	}
	else
	{
		return "NotConnected";
	}
}

string IPGLaserDevice::GetFrequency()
{
	QByteArray strCommand("RPRR\r");
	QByteArray strOut;
	bool success = OnceData(strCommand, strOut);
	success = success && !strOut.isEmpty();

	if (success && strOut.size() > 6)
	{
		string str = strOut.mid(6).toStdString();  // 从第6个字符开始到结尾
		return str;
	}
	else
	{
		return "NotConnected";
	}
}

string IPGLaserDevice::GetPulseWidth()
{
	QByteArray strCommand("RPW\r");
	QByteArray strOut;
	bool success = OnceData(strCommand, strOut);
	success = success && !strOut.isEmpty();

	if (success && strOut.size() > 5)
	{
		string str = strOut.mid(5).toStdString();  // 从第5个字符开始到结尾
		return str;
	}
	else
	{
		return "NotConnected";
	}
}

double IPGLaserDevice::GetAveragePower()
{
	QByteArray strCommand("ROP\r");
	QByteArray strOut;
	bool success = OnceData(strCommand, strOut);

	if (success && !strOut.isEmpty() && strOut.size() > 4)
	{
		QByteArray powerStr = strOut.mid(4);  // 从第4个字符开始

		if (powerStr.isEmpty() || powerStr.size() > 10)
		{
			return 0.0;
		}

		// 转换为double
		bool ok;
		double dPower = powerStr.toDouble(&ok);

		return ok ? dPower : 0.0;
	}

	return 0.0;
}

string IPGLaserDevice::GetTemperature()
{
	QByteArray strCommand("RCT\r");
	QByteArray strOut;
	bool success = OnceData(strCommand, strOut);

	if (success && !strOut.isEmpty() && strOut.size() > 4)
	{
		QByteArray tempStr = strOut.mid(4);  // 从第4个字符开始

		if (tempStr.isEmpty() || tempStr.size() > 10)
		{
			m_strTemperature = "0";
		}
		else
		{
			m_strTemperature = tempStr.toStdString();
		}
	}
	else
	{
		m_strTemperature = "NotConnected";
	}

	return m_strTemperature;
}

string IPGLaserDevice::GetTroubleshooting()
{
	QByteArray strCommand("STA\r");
	QByteArray strOut;
	bool success = OnceData(strCommand, strOut);
	success = success && !strOut.isEmpty();

	if (success && strOut.size() > 5)
	{
		string str = strOut.mid(5).toStdString();  // 从第5个字符开始到结尾
		return str;
	}
	else
	{
		return "NotConnected";
	}
}
