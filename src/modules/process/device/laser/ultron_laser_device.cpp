/************************************************************************/
/*                            奥创激光器实现类                           */
/************************************************************************/

#include "ultron_laser_device.h"

#include "core/logging/logger.h"

#include "modules/process/device/process_device_log.h"
#include <string>
#include <stdlib.h>
//#include <cstringt.h>
#include <sstream>
#include <fstream>
using std::ostringstream;
using namespace std;

ULTRONLaserDevice::ULTRONLaserDevice(lcnc::process::ProcessSettingsService& settings) : LaserDevice(settings), m_strName("ULTRON"), m_bIsInited(false), m_dMaxCurrent(0),
                                   m_dSimmerCurrent(0), m_iWaveShape(0), m_strTemperature("")
{
}

ErrorCode ULTRONLaserDevice::setLaserTable(const table& tableLaser)
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
	//激光器使能
	InitLaser();
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

const string & ULTRONLaserDevice::GetName()
{
	return m_strName;
}

bool ULTRONLaserDevice::IsInited()
{
	return m_bIsInited;
}

bool ULTRONLaserDevice::IsAvailableData(const QByteArray& data)
{
	if (data.isEmpty())
		return false;

	// 检查是否以 \r 结尾
	if (data.endsWith('\r'))
		return true;

	return false;
}

bool ULTRONLaserDevice::StartLaser()
{
	//声光开启
	char chLaserOff[] = { 0x01,0x06,0x06,0x07,0x00,0x01 };
	int itest = sizeof(chLaserOff);
	vector<uint8_t> crcCommand;
	for (int i = 0; i < sizeof(chLaserOff); ++i)
	{
		crcCommand.push_back(chLaserOff[i]);
	}
	uint16_t result = crc16(crcCommand);


	crcCommand.push_back((char)(result >> 8));
	crcCommand.push_back((char)(result & 0xFF));
	int vecSize = crcCommand.size();
	// 创建一个足够大的char数组来存储vector中的所有数据
	char* charArray = new char[128];

	// 使用std::copy函数将vector中的数据复制到char数组中
	std::copy(crcCommand.begin(), crcCommand.end(), charArray);
	QByteArray data(charArray, 8);
	bool bResult = OnceData(data);
	delete[]charArray;
	return bResult;
	
	
}

bool ULTRONLaserDevice::StopLaser()
{
	//关闭声光开关
	char chLaserOff[] = { 0x01,0x06,0x06,0x07,0x00,0x00 };
	int itest = sizeof(chLaserOff);
	vector<uint8_t> crcCommand;
	for (int i = 0; i < sizeof(chLaserOff); ++i)
	{
		crcCommand.push_back(chLaserOff[i]);
	}
	uint16_t result = crc16(crcCommand);

	crcCommand.push_back((char)(result >> 8));
	crcCommand.push_back((char)(result & 0xFF));
	int vecSize = crcCommand.size();
	// 创建一个足够大的char数组来存储vector中的所有数据
	char* charArray = new char[128];

	// 使用std::copy函数将vector中的数据复制到char数组中
	std::copy(crcCommand.begin(), crcCommand.end(), charArray);
	QByteArray data(charArray, 8);
	bool bResult = OnceData(data);
	delete[]charArray;
	return bResult;
	
}

bool ULTRONLaserDevice::SetEnergy(double dEnergy)
{
	double dSetEnergy;
	dSetEnergy = dEnergy;
	if ((dSetEnergy >= 0) && (dSetEnergy <= 100))
	{
		int iEnergyValue = dEnergy * 1000 / 100;
		char command[128] = { 0x01,0x06,0x06,0x06 };
		char* Temp = new char[32];
		itoa(iEnergyValue, Temp, 16);
		char* chs = hextochs(Temp);
		if (iEnergyValue < 256)
		{
			command[4] = 0x00;
			command[5] = chs[0];
		}
		else
		{
			command[4] = chs[0];
			command[5] = chs[1];
		}
		free(chs);
		chs = NULL;
		free(Temp);
		Temp = NULL;

		vector<uint8_t> crcCommand1;
		for (int i = 0; i < 6; ++i)
		{
			crcCommand1.push_back(command[i]);
		}
		uint16_t result1 = crc16(crcCommand1);
		crcCommand1.push_back((char)(result1 >> 8));
		crcCommand1.push_back((char)(result1 & 0xFF));
		int vecSize1 = crcCommand1.size();
		// 创建一个足够大的char数组来存储vector中的所有数据
		char* charArray = new char[128];
		// 使用std::copy函数将vector中的数据复制到char数组中
		std::copy(crcCommand1.begin(), crcCommand1.end(), charArray);
		QByteArray data(charArray, 8);
		bool bResult = OnceData(data);
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

bool ULTRONLaserDevice::SetFrequency(double dFrequency)
{
	int iFrequency = dFrequency * 1000 ;
	char command[128] = { 0x01,0x06,0x05,0xDF };
	char* Temp = new char[32];
	itoa(iFrequency, Temp, 16);
	char* chs = hextochs(Temp);
	if (iFrequency < 256)
	{
		command[4] = 0x00;
		command[5] = chs[0];
	}
	else
	{
		command[4] = chs[0];
		command[5] = chs[1];
	}
	free(chs);
	chs = NULL;
	free(Temp);
	Temp = NULL;

	vector<uint8_t> crcCommand1;
	for (int i = 0; i < 6; ++i)
	{
		crcCommand1.push_back(command[i]);
	}
	uint16_t result1 = crc16(crcCommand1);
	crcCommand1.push_back((char)(result1 >> 8));
	crcCommand1.push_back((char)(result1 & 0xFF));
	int vecSize1 = crcCommand1.size();
	// 创建一个足够大的char数组来存储vector中的所有数据
	char* charArray = new char[128];
	// 使用std::copy函数将vector中的数据复制到char数组中
	std::copy(crcCommand1.begin(), crcCommand1.end(), charArray);
	QByteArray data(charArray, 8);
	bool bResult = OnceData(data);
	if (!bResult)
		lcnc::process::logDeviceError(ErrorCode::ERROR_LASER_SETTINGFAILED, QObject::tr("Set laser frequency %1 failed.").arg(dFrequency).toUtf8().data());
	else
		LaserDevice::UpdateFrequency(dFrequency);
	return bResult;
}

bool ULTRONLaserDevice::SetLaserParameter(const LaserParameter& parameter)
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

bool ULTRONLaserDevice::SetPulseWidth(double dPulsePickerDivider)
{
	if (dPulsePickerDivider > 65535)
	{
		dPulsePickerDivider = 65535;
	}
	else if (dPulsePickerDivider < 0)
	{
		dPulsePickerDivider = 0;
	}
	int iDiv = dPulsePickerDivider;
	char command[128] = { 0x01,0x06,0x05,0xE1 };
	char* Temp = new char[32];
	_itoa(iDiv, Temp, 16);
	char* chs = hextochs(Temp);
	if (iDiv < 256)
	{
		command[4] = 0x00;
		command[5] = chs[0];
	}
	else
	{
		command[4] = chs[0];
		command[5] = chs[1];
	}
	free(chs);
	chs = NULL;
	free(Temp);
	Temp = NULL;

	vector<uint8_t> crcCommand1;
	for (int i = 0; i < 6; ++i)
	{
		crcCommand1.push_back(command[i]);
	}
	uint16_t result1 = crc16(crcCommand1);
	crcCommand1.push_back((char)(result1 >> 8));
	crcCommand1.push_back((char)(result1 & 0xFF));
	int vecSize1 = crcCommand1.size();
	// 创建一个足够大的char数组来存储vector中的所有数据
	char* charArray = new char[128];
	// 使用std::copy函数将vector中的数据复制到char数组中
	std::copy(crcCommand1.begin(), crcCommand1.end(), charArray);
	QByteArray data(charArray,8);
	bool bResult = OnceData(data);
	if (!bResult)
		lcnc::process::logDeviceError(ErrorCode::ERROR_LASER_SETTINGFAILED, QObject::tr("Set laser PulsePickerDivider %1 failed.").arg(dPulsePickerDivider).toUtf8().data());
	else
		LaserDevice::UpdatePulseWidth(dPulsePickerDivider);
	delete[]charArray;
	return bResult;
}

string ULTRONLaserDevice::GetEnergy()
{
	QByteArray strCommand;
	QByteArray strOut;
	char command[128] = { 0x01,0x03,0x06,0x06,0x00,0x01 };
	char* Temp = new char[32];
	vector<uint8_t> crcCommand1;
	for (int i = 0; i < 6; ++i)
	{
		crcCommand1.push_back(command[i]);
	}
	uint16_t result1 = crc16(crcCommand1);
	crcCommand1.push_back((char)(result1 >> 8));
	crcCommand1.push_back((char)(result1 & 0xFF));
	int vecSize1 = crcCommand1.size();
	// 创建一个足够大的char数组来存储vector中的所有数据
	char* charArray = new char[128];
	// 使用std::copy函数将vector中的数据复制到char数组中
	std::copy(crcCommand1.begin(), crcCommand1.end(), charArray);
	QByteArray data(charArray, 8);
	bool bResult = OnceData(data, strOut);
	QByteArray str112 = "010302";
	int i = strOut.contains(str112);
	if (bResult && i == -1)
	{
		return "NotConnected";
	}
	else
	{
		int iSize = str112.size();
		string sResult = strOut.mid(i + iSize, 4).toStdString();
		string str = hexStrToDecString(sResult);
		return str;
	}
}

string ULTRONLaserDevice::GetFrequency()
{
	return "NotConnected";
}

string ULTRONLaserDevice::GetPulseWidth()
{
	return "NotConnected";
}

bool ULTRONLaserDevice::InitLaser()
{
	QByteArray strOut;
	//激光器是能打开，设置外控模式
	//不考虑水冷机的情况下，打开激光器

	char chLaserOff[] = { 0x01,0x06,0x06,0x08,0x00,0x01 };
	vector<uint8_t> crcCommand;
	for (int i = 0; i < sizeof(chLaserOff); ++i)
	{
		crcCommand.push_back(chLaserOff[i]);
	}
	uint16_t result = crc16(crcCommand);
	crcCommand.push_back((char)(result >> 8));
	crcCommand.push_back((char)(result & 0xFF));
	int vecSize = crcCommand.size();
	// 创建一个足够大的char数组来存储vector中的所有数据
	char* charArray = new char[128];
	// 使用std::copy函数将vector中的数据复制到char数组中
	std::copy(crcCommand.begin(), crcCommand.end(), charArray);
	QByteArray data(charArray, 8);
	bool bResult = OnceData(data);

	int ivalue;
	const int MAX_RETRY = 30; // 3秒超时（100ms/次）
	int retryCount = 0;
	do
	{
		char chLaser[] = { 0x01,0x03,0x06,0x0D,0x00,0x01 };
		vector<uint8_t> crcCommand2;
		for (int i = 0; i < sizeof(chLaser); ++i)
		{
			crcCommand2.push_back(chLaser[i]);
		}
		uint16_t result2 = crc16(crcCommand2);
		crcCommand2.push_back((char)(result2 >> 8));
		crcCommand2.push_back((char)(result2 & 0xFF));
		int vecSize2 = crcCommand2.size();
		// 创建一个足够大的char数组来存储vector中的所有数据
		//char* charArray2 = new char[128];
		 delete[]charArray;
		// 使用std::copy函数将vector中的数据复制到char数组中
		std::copy(crcCommand2.begin(), crcCommand2.end(), charArray);
		QByteArray data1(charArray, 8);
		bResult = OnceData(data1,strOut);
		
		double dPower = 0;
		if (strOut.size() < 10)
		{
			dPower = 0;
		}
		else
		{
			QByteArray str112 = "010302";
			int i = strOut.contains(str112);
			if (i == -1)
			{
				dPower = 0;
				return false;
			}
			int itest = str112.size();
			std::string sResult = strOut.mid(i + itest, 4).toStdString();
			string str = hexStrToDecString(sResult);
			int ivalue = stoi(str);
		}
		
		Sleep(1000);
		retryCount++;
		if (retryCount > MAX_RETRY)
		{
			break;
		}
	} while (ivalue != 100);

	
	//设置为外控
	char chLaserOff1[] = { 0x01,0x06,0x06,0x0C,0x00,0x01 };
	vector<uint8_t> crcCommand1;
	for (int i = 0; i < sizeof(chLaserOff1); ++i)
	{
		crcCommand1.push_back(chLaserOff1[i]);
	}
	uint16_t result1 = crc16(crcCommand1);
	crcCommand1.push_back((char)(result1 >> 8));
	crcCommand1.push_back((char)(result1 & 0xFF));
	int vecSize1 = crcCommand1.size();
	// 创建一个足够大的char数组来存储vector中的所有数据
	//char* charArray1 = new char[128];
	delete[]charArray;
	// 使用std::copy函数将vector中的数据复制到char数组中
	std::copy(crcCommand1.begin(), crcCommand1.end(), charArray);
	QByteArray data2(charArray, 8);
	bResult = OnceData(data2);
}

uint16_t ULTRONLaserDevice::crc16(const std::vector<uint8_t>& data)
{
	uint16_t crc = 0xFFFF;
	for (std::vector<uint8_t>::const_iterator byte = data.begin(); byte != data.end(); ++byte)
	{
		crc ^= *byte;
		for (int i = 0; i < 8; i++)
		{
			if (crc & 0x0001) {
				crc = (crc >> 1) ^ 0xA001;
			}
			else {
				crc = crc >> 1;
			}
		}
	}
	uint16_t result = swap_bits(crc);
	return result;
}

uint16_t ULTRONLaserDevice::swap_bits(uint16_t& value)
{
	return ((value & 0x00FF) << 8) | ((value & 0xFF00) >> 8);
}
char* ULTRONLaserDevice::hextochs(char* ascii)
{
	int len = strlen(ascii);
	if (len % 2 == 0)
	{
		char* chs = NULL;
		chs = (char*)calloc(len / 2 + 1, sizeof(char));                // calloc chs

		int  i = 0;
		char ch[2] = { 0 };
		while (i < len)
		{
			ch[0] = ((int)ascii[i] > 64) ? (ascii[i] % 16 + 9) : ascii[i] % 16;
			ch[1] = ((int)ascii[i + 1] > 64) ? (ascii[i + 1] % 16 + 9) : ascii[i + 1] % 16;

			chs[i / 2] = (char)(ch[0] * 16 + ch[1]);
			i += 2;
		}

		return chs;            // chs 返回前未释放

	}
	else
	{
		char* chr = new char[len + 2];
		if (len % 2 != 0)
		{
			chr[0] = '0';
			int j = 0;
			while (j < len)
			{
				chr[j + 1] = ascii[j];
				j++;
			}
			chr[j + 1] = '\0';
			len++;
		}
		char* chs = NULL;
		chs = (char*)calloc(len / 2 + 1, sizeof(char));                // calloc chs

		int  i = 0;
		char ch[2] = { 0 };
		while (i < len)
		{
			ch[0] = ((int)chr[i] > 64) ? (chr[i] % 16 + 9) : chr[i] % 16;
			ch[1] = ((int)chr[i + 1] > 64) ? (chr[i + 1] % 16 + 9) : chr[i + 1] % 16;

			chs[i / 2] = (char)(ch[0] * 16 + ch[1]);
			i += 2;
		}

		delete[]chr;
		return chs;            // chs 返回前未释放

	}
}

string ULTRONLaserDevice::hexStrToDecString(const string& hexStr)
{
	try {
		// 步骤1：将16进制字符串转为数值
		stringstream ss;
		ss << hex << hexStr;  // hex指定以16进制解析输入
		unsigned int decNum;
		ss >> decNum;

		// 步骤2：将数值转为十进制字符串
		stringstream decSs;
		decSs << decNum;
		return decSs.str();
	}
	catch (const exception& e) {
		// 处理非法16进制字符串（如"G1"、空字符串）
		LCNC_ERR(lcnc::LogCode::Generic,
				 "ULTRONLaserDevice: invalid hexadecimal value '{}': {}",
                 hexStr,
				 e.what());
		return "";
	}
}
