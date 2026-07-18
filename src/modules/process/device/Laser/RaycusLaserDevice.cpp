/************************************************************************/
/*                            锐科激光器实现类                           */
/************************************************************************/

#include "RaycusLaserDevice.h"
#include "MessageModule.h"
#include <string>
#include <stdlib.h>
//#include <cstringt.h>
#include <sstream>
#include <boost/lexical_cast.hpp>
using std::ostringstream;
using namespace std;

RaycusLaserDevice::RaycusLaserDevice(lcnc::process::ProcessSettingsService& settings) : LaserDevice(settings), m_strName("Raycus"), m_bIsInited(false),
										m_dMaxCurrent(0), m_dSimmerCurrent(0), m_iWaveShape(0)
{
}

ErrorCode RaycusLaserDevice::SetLaserTable(const table& tableLaser)
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
		bool	bChange		= false;
		double	dEnergy		= m_dEnergy;
		double	dFrequency	= m_dFrequency;
		double	dPulseWidth = m_dPulseWidth;

		table tLaser;
		if (bConnectChange)
            tLaser = processLaserTable(QStringLiteral("Laser"));
		else
			tLaser = tableLaser.at("Laser").as_table();
		
		if (tLaser.count("fEnergy"))
		{
			bChange = true;
			dEnergy = tLaser["fEnergy"].as_floating();
		}
		if (tLaser.count("fFrequency"))
		{
			bChange = true;
			dFrequency = tLaser["fFrequency"].as_floating();
		}
		if (tLaser.count("fPulseWidth"))
		{
			bChange = true;
			dPulseWidth = tLaser["fPulseWidth"].as_floating();
		}
		if (bChange)
		{
			if (!SetLaserParameter(LaserParameter(dEnergy, dFrequency, dPulseWidth)))
			{
				eCode = ErrorCode::ERROR_LASER_SETTINGFAILED;
			}
		}
	}
	if (eCode == ErrorCode::ERROR_NONE)
		m_bIsInited = true;
	else
		m_bIsInited = false;
	return eCode;
}

const string& RaycusLaserDevice::GetName()
{
	return m_strName;
}

bool RaycusLaserDevice::IsInited()
{
	return m_bIsInited;
}

bool RaycusLaserDevice::IsAvailableData(const QByteArray& data)
{
	if (data.isEmpty())
		return false;

	// 检查是否以 \x0D 结尾
	if (data.endsWith('\x0D'))
		return true;

	return false;
}

bool RaycusLaserDevice::StartLaser()
{
	return OnceData("\x1B\x4F\x0D");
}

bool RaycusLaserDevice::StopLaser()
{
	return OnceData("\x1B\x53\x0D");
}

bool RaycusLaserDevice::StartAimingBeam()
{
	return true;
}

bool RaycusLaserDevice::StopAimingBeam()
{
	return true;
}

bool RaycusLaserDevice::SetEnergy(double dEnergy)
{
	if (dEnergy > 0)
	{
		return SetLaserParameter(LaserParameter(dEnergy, m_dFrequency, m_dPulseWidth));
	}
	else
	{
		SHOW_OPER_ERROR(ErrorCode::ERROR_LASER_SETTINGFAILED,
			QObject::tr("The laser energy %1 does not meet the minimum requirement of 0.").arg(dEnergy).toUtf8().data());
		return false;
	}

}

bool RaycusLaserDevice::SetFrequency(double dFrequency)
{
	if (m_dFrequency >= 50.0)
	{
		return SetLaserParameter(LaserParameter(m_dEnergy, dFrequency, m_dPulseWidth));
	}
	else
	{
		SHOW_OPER_ERROR(ErrorCode::ERROR_LASER_SETTINGFAILED,
			QObject::tr("The laser frequency %1 does not meet the minimum requirement of 50.").arg(dFrequency).toUtf8().data());
		return false;
	}
}

bool RaycusLaserDevice::SetPulseWidth(double dPulseWidth)
{
	if (dPulseWidth * m_dFrequency / 10000 > 0)
	{
		return SetLaserParameter(LaserParameter(m_dEnergy, m_dFrequency, dPulseWidth));
	}
	else
	{
		SHOW_OPER_ERROR(ErrorCode::ERROR_LASER_SETTINGFAILED,
			QObject::tr("The laser pulse width %1 does not meet the requirement.").arg(dPulseWidth).toUtf8().data());
		return false;
	}
}

bool RaycusLaserDevice::SetLaserParameter(const LaserParameter& parameter)
{
	string strFrequency = "\x32";
	string strPulseWidth = "\x01";
	string strLaserEnergy = "\x01";

	double mdEnergy = parameter.dEnergy;
	double mdFrequency = parameter.dFrequency;
	double mdPulseWidth = parameter.dPulseWidth;

	char* Temp = new char[32];
	if (mdFrequency >= 50.0)
	{
		itoa((int)mdFrequency, Temp, 16);
		char* chs = hextochs(Temp);
		strFrequency = chs;
		free(chs);
		chs = NULL;
	}
	if (mdPulseWidth * mdFrequency / 10000 > 0)
	{
		int i = (mdPulseWidth * mdFrequency + 5000) / 10000;
		itoa(i, Temp, 16);
		char* chs = hextochs(Temp);
		strPulseWidth = chs;
		free(chs);
		chs = NULL;
	}
	if (mdEnergy > 0)
	{
		itoa(mdEnergy, Temp, 16);
		char* chs = hextochs(Temp);
		strLaserEnergy = chs;
		free(chs);
		chs = NULL;
	}
	string strHead = string("\x1B\x46");
	string strEnd = string("\x0D");
	string strMiddle1 = string("\x44");
	string strMiddle2 = string("\x50");

	string strCommand = strHead + strFrequency + strMiddle1 + strPulseWidth + strMiddle2 + strLaserEnergy + strEnd;
	bool success = OnceData(strCommand.c_str());
	Sleep(100);
	delete[]Temp;
	if (!success)
		SHOW_OPER_ERROR(ErrorCode::ERROR_LASER_SETTINGFAILED,
			QObject::tr("Set laser energy %1 frequency %2 pulse width %3 failed.")
			.arg(parameter.dEnergy).arg(parameter.dFrequency).arg(parameter.dPulseWidth).toUtf8().data());
	else
		LaserDevice::UpdateLaserParameter(parameter);
	return success;
}

string RaycusLaserDevice::GetEnergy()
{
	string strEnergy = std::to_string(m_dEnergy);
	return strEnergy;
}

string RaycusLaserDevice::GetFrequency()
{
	string strFrequency = std::to_string(m_dFrequency);
	return strFrequency;
}

string RaycusLaserDevice::GetPulseWidth()
{
	string strPulseWidth = std::to_string(m_dPulseWidth);
	return strPulseWidth;
}

char* RaycusLaserDevice::hextochs(char* ascii)
{
	int len = strlen(ascii);
	char* temp = nullptr;
	const char* src = ascii;

	// 处理奇数长度的情况
	if (len % 2 != 0) {
		temp = new char[len + 2];
		temp[0] = '0';
		strcpy(temp + 1, ascii);
		len++;
		src = temp;
	}

	char* chs = (char*)calloc(len / 2 + 1, sizeof(char));
	if (!chs) {
		delete[] temp;
		return nullptr;
	}

	for (int i = 0; i < len; i += 2) {
		char ch[2] = {
			(src[i] > 64) ? (src[i] % 16 + 9) : src[i] % 16,
			(src[i + 1] > 64) ? (src[i + 1] % 16 + 9) : src[i + 1] % 16
		};
		chs[i / 2] = (ch[0] * 16 + ch[1]);
	}

	delete[] temp;
	return chs;
}

char* RaycusLaserDevice::ftoa(double res, char* des, int type)
{
	int i_integer;
	double d_decimal;

	char c_integer[200], c_decimal[200], idx = 1;
	strcpy(des, "");
	if (res < 0)
	{
		res = -res;
		strcpy(des, "-");
	}

	i_integer = int(res); d_decimal = res - i_integer;
	itoa(i_integer, c_integer, type);
	strupr(c_integer);
	strcat(des, c_integer);

	c_decimal[0] = '.';
	while (d_decimal != 0.0 && idx < 20)//20表示精度
	{
		i_integer = int(d_decimal *= type);

		if (i_integer >= 0 && i_integer <= 9)
			c_decimal[idx++] = i_integer + 0x30;
		else
			c_decimal[idx++] = i_integer + 0x37;

		d_decimal -= i_integer;
	}
	c_decimal[idx] = '\0';

	return strcat(des, c_decimal);
}
