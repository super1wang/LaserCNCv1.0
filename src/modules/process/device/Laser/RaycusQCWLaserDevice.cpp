/************************************************************************/
/*                            锐科QCW激光器实现类                           */
/************************************************************************/

#include "RaycusQCWLaserDevice.h"
#include <string>
#include <stdlib.h>
#include <sstream>
#include <boost/lexical_cast.hpp>
using std::ostringstream;
using namespace std;

RaycusQCWLaserDevice::RaycusQCWLaserDevice() : m_strName("RaycusQCW"), m_bIsInited(false),
m_dMaxCurrent(0), m_dSimmerCurrent(0), m_iWaveShape(0)
{
}

ErrorCode RaycusQCWLaserDevice::SetLaserTable(const table& tableLaser)
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
		bool	bChange = false;
		double	dEnergy = m_dEnergy;
		double	dFrequency = m_dFrequency;
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

const string& RaycusQCWLaserDevice::GetName()
{
	return m_strName;
}

bool RaycusQCWLaserDevice::IsInited()
{
	return m_bIsInited;
}

bool RaycusQCWLaserDevice::IsAvailableData(const QByteArray& data)
{
	if (data.isEmpty())
		return false;

	// 检查是否以 \x0D 结尾
	if (data.endsWith(QByteArray("\x55\xAA", 2)))
		return true;

	return false;
}

bool RaycusQCWLaserDevice::StartLaser()
{
	const char chStar[12] = { 0xAA,0x55,0x00,0x06,0x00,0xff,0xE9,0x00,0x01,0xE9,0x55,0xAA };
	QByteArray data(chStar, 12);
	return OnceData(data);
}

bool RaycusQCWLaserDevice::StopLaser()
{
	char chStop[12] = { 0xAA,0x55,0x00,0x06,0x00,0xff,0xEA,0x00,0x01,0xE9,0x55,0xAA };
	QByteArray data(chStop, 12);
	return OnceData(data);
}

bool RaycusQCWLaserDevice::StartAimingBeam()
{
	return true;
}

bool RaycusQCWLaserDevice::StopAimingBeam()
{
	return true;
}

bool RaycusQCWLaserDevice::SetEnergy(double dEnergy)
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

bool RaycusQCWLaserDevice::SetFrequency(double dFrequency)
{
	if (m_dFrequency >= 10.0)
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

bool RaycusQCWLaserDevice::SetPulseWidth(double dPulseWidth)
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

bool RaycusQCWLaserDevice::SetLaserParameter(const LaserParameter& parameter)
{
	string strFrequency = "\x32";
	string strPulseWidth = "\x01";
	string strLaserEnergy = "\x01";

	double mdEnergy = parameter.dEnergy;
	double mdFrequency = parameter.dFrequency;
	double mdPulseWidth = parameter.dPulseWidth;
	char command[256] = { 0xAA,0x55,0x00,0x0D,0x00,0xFF,0xE2,0x00,0x04 };
	char* Temp = new char[32];
	//能量 百分比表示
	if (mdEnergy >= 0 && mdEnergy <= 100)
	{
		itoa(mdEnergy, Temp, 16);
		char* chs = hextochs(Temp);
		command[9] = chs[0];
		free(chs);
		chs = NULL;
	}
	//频率
	if (mdFrequency > 5000)
	{
		mdFrequency = 5000;
		itoa(mdFrequency, Temp, 16);
		char* chs = hextochs(Temp);
		command[10] = chs[0];
		command[11] = chs[1];
		free(chs);
		chs = NULL;
	}
	else if (mdFrequency < 1)
	{
		mdFrequency = 1;
		itoa(mdFrequency, Temp, 16);
		char* chs = hextochs(Temp);
		command[10] = 0x00;
		command[11] = chs[0];
		free(chs);
		chs = NULL;
	}
	else
	{
		itoa(mdFrequency, Temp, 16);
		char* chs = hextochs(Temp);
		if (mdFrequency < 256)
		{
			command[10] = 0x00;
			command[11] = chs[0];
		}
		else
		{
			command[10] = chs[0];
			command[11] = chs[1];
		}
		free(chs);
		chs = NULL;
	}

	//占空比 百分比表示
	int iDutyCycle = mdPulseWidth * mdFrequency / 10000;
	if (iDutyCycle >= 1 && iDutyCycle <= 50)
	{
		itoa(iDutyCycle, Temp, 16);
		char* chs = hextochs(Temp);
		command[12] = chs[0];
		free(chs);
		chs = NULL;
	}
	else
	{
		iDutyCycle = 1;
		itoa(iDutyCycle, Temp, 16);
		char* chs = hextochs(Temp);
		command[12] = chs[0];
		free(chs);
		chs = NULL;
	}

	//脉宽
	if (mdPulseWidth < 1)
	{
		mdPulseWidth = 1;
		itoa(mdPulseWidth, Temp, 16);
		char* chs = hextochs(Temp);
		command[13] = 0x00;
		command[14] = chs[0];
		free(chs);
		chs = NULL;
	}
	else
	{
		itoa(mdPulseWidth, Temp, 16);
		char* chs = hextochs(Temp);
		if (mdPulseWidth < 256)
		{
			command[13] = 0x00;
			command[14] = chs[0];
		}
		else
		{
			command[13] = chs[0];
			command[14] = chs[1];
		}
		free(chs);
		chs = NULL;
	}
	//校验字
	int iCheck = 753 + mdEnergy + mdFrequency + iDutyCycle + mdPulseWidth;
	itoa(iCheck, Temp, 16);
	char* chs1 = hextochs(Temp);
	command[15] = chs1[0];
	command[16] = chs1[1];
	free(chs1);
	chs1 = NULL;
	//结束码
	command[17] = 0x55;
	command[18] = 0xAA;
	//	command[19] = '\0';
	//发送指令
	QByteArray data(command, 19);
	bool success = OnceData(data);
	Sleep(100);
	success = StartLaser();
	delete[]Temp;
	if (!success)
		SHOW_OPER_ERROR(ErrorCode::ERROR_LASER_SETTINGFAILED,
			QObject::tr("Set laser energy %1 frequency %2 pulse width %3 failed.")
			.arg(parameter.dEnergy).arg(parameter.dFrequency).arg(parameter.dPulseWidth).toUtf8().data());
	else
		LaserDevice::UpdateLaserParameter(parameter);
	return success;
}

string RaycusQCWLaserDevice::GetEnergy()
{
	string strEnergy = std::to_string(m_dEnergy);
	return strEnergy;
}

string RaycusQCWLaserDevice::GetFrequency()
{
	string strFrequency = std::to_string(m_dFrequency);
	return strFrequency;
}

string RaycusQCWLaserDevice::GetPulseWidth()
{
	string strPulseWidth = std::to_string(m_dPulseWidth);
	return strPulseWidth;
}

char* RaycusQCWLaserDevice::hextochs(char* ascii)
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

char* RaycusQCWLaserDevice::ftoa(double res, char* des, int type)
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
