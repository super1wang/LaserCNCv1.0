#include "JAPHLComp.h"
#include "qc_applicationwindow.h"

JAPHLComp::JAPHLComp() : m_strName("JAPHL"), m_bInited(false)
{
}

ErrorCode JAPHLComp::SetSpecialTable(bool bReconnect)
{
	bool bConnectChange = bReconnect;
	ErrorCode eCode = ErrorCode::ERROR_NONE;
	table tSpecial = SETTINGS->GetTable(SettingSection::Special);
	table tCom;
	if (!tSpecial.count("LPCOM"))
	{
		tCom["sPort"]		= "COM3";
		tCom["sBaudRate"]	= "9600";
		tCom["sDataBits"]	= "8";
		tCom["sStopBits"]	= "1";
		tCom["sParity"]		= "NONE";
		tCom["iTimeGo"]		= 1000;
		tCom["iTimeBack"]	= 1000;
		SETTINGS->SetTable(true, SettingSection::Special, tCom, "LPCOM");
	}
	else
		tCom = tSpecial["LPCOM"].as_table();

	m_iTimeGo	= tCom.count("iTimeGo")		? tCom["iTimeGo"]	.as_integer() : 1000;
	m_iTimeBack = tCom.count("iTimeBack")	? tCom["iTimeBack"]	.as_integer() : 1000;

	eCode = SetComTable(tCom, bConnectChange);
	if (eCode != ErrorCode::ERROR_NONE)
		return ErrorCode::ERROR_SPECIAL_LPCONNECTIONFAILED;

	eCode = InitMeasurement();
	return eCode;
}

ErrorCode JAPHLComp::InitMeasurement()
{
	ErrorCode eCode = ErrorCode::ERROR_NONE;
	MotionControl* pMC = QC_ApplicationWindow::getAppWindow()->getService()->GetMotionControl();
	
	m_bInited = true;
	return eCode;
}

ErrorCode JAPHLComp::MeasurementComp(CompValue& cValue, int iIndex)
{
	//返回一组值，同时向map写入，流程内对值对进行二次计算。确保是正常的

	// 添加其他测量辅助动作
	ErrorCode eCode = ErrorCode::ERROR_NONE;
	MotionControl* pMC = QC_ApplicationWindow::getAppWindow()->getService()->GetMotionControl();
	// 0左OUT4 1右OUT3 
	DigitalOUT eOUT = iIndex ? DigitalOUT::OUT3 : DigitalOUT::OUT4;
	pMC->DigitalOutputSet(eOUT, 1);
	boost::this_thread::sleep_for(boost::chrono::milliseconds(m_iTimeGo));
	eCode = ReadDistance(cValue.X, iIndex);	// 获取读数
	pMC->DigitalOutputSet(eOUT, 0);
	boost::this_thread::sleep_for(boost::chrono::milliseconds(m_iTimeBack));
	if (eCode != ErrorCode::ERROR_NONE)
		return eCode;
		
	cValue.Y = cValue.X;
	return eCode;
}

ErrorCode JAPHLComp::ReadDistance(QString& qstrDistance, int iIndex)
{
	ErrorCode eCode = ErrorCode::ERROR_NONE;
	QByteArray qbCommand, qbResponse;
	eCode = BuildCommand(qbCommand, iIndex);
	if (eCode != ErrorCode::ERROR_NONE)
		return eCode;

	for (int i = 0; i < 5; i++)
	{
		if (OnceData(qbCommand, qbResponse, 800))
		{
			eCode = ParseData(qbResponse, qstrDistance, iIndex);
			if (eCode == ErrorCode::ERROR_NONE)
				return eCode;	// 正常返回
		}
		else
			eCode = ErrorCode::ERROR_SPECIAL_LPSENDFAILED;

		Sleep(30);
	}
	return eCode;
}

ErrorCode JAPHLComp::BuildCommand(QByteArray& qbCommand, int iIndex)
{
	uint16_t p;
	if (iIndex == 0)
		p = '\x02';	// 从站地址，0左侧
	else if (iIndex == 1)
		p = '\x01';	// 从站地址，1右侧
	else
		return ErrorCode::ERROR_SPECIAL_LPNONEINDEX;

	qbCommand.append(p);
	qbCommand.append('\x04');		// 功能码，03为读距离
	qbCommand.append('\x00');		// 寄存器地址(高位)
	qbCommand.append('\x00');		// 寄存器地址(低位)
	qbCommand.append('\x00');		// 寄存器个数(低位)
	qbCommand.append('\x02');		// 寄存器数量(高位)

	ADD_CRC_MODBUS(qbCommand);
	return ErrorCode::ERROR_NONE;
}

ErrorCode JAPHLComp::ParseData(const QByteArray& data, QString& qstrDistance, int iIndex)
{
	if (data.length() < 7) // 最小响应长度
		return ErrorCode::ERROR_SPECIAL_LPRESPONSENONE;

	uint16_t p;
	if (iIndex == 0)
		p = '\x02';	// 从站地址，0左侧
	else if (iIndex == 1)
		p = '\x01';	// 从站地址，1右侧
	else
		return ErrorCode::ERROR_SPECIAL_LPNONEINDEX;

	if (data[0] == p && data[1] == 0x04 && data[2] == 0x04) // 校验位置及功能码
	{
		// 距离字节处理
		uint8_t byte0 = static_cast<uint8_t>(data[3]); // 符号位，00为正，01为负
		uint8_t byte1 = static_cast<uint8_t>(data[4]);
		uint8_t byte2 = static_cast<uint8_t>(data[5]);
		uint8_t byte3 = static_cast<uint8_t>(data[6]);

		uint32_t value = (static_cast<uint32_t>(byte1) << 16) | (static_cast<uint32_t>(byte2) << 8) | static_cast<uint32_t>(byte3);
		
		int32_t signedValue = value;
		if (byte0 == 0x01) // 01为负
			signedValue = -static_cast<int32_t>(value);
		
		qstrDistance = QString::number(signedValue);

		return ErrorCode::ERROR_NONE;
	}

	return ErrorCode::ERROR_SPECIAL_LPRESPONSENONE;
}

bool JAPHLComp::IsAvailableData(const QByteArray& data)
{
	// 句尾CRC校验
	if (data.size() < 3)
		return false;

	QByteArray dataCopy = data.left(data.size() - 2);
	ADD_CRC_MODBUS(dataCopy);

	bool bAvailable = (data.right(2) == dataCopy.right(2));
	return bAvailable;
}