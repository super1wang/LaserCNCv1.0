#include "CameraComp.h"
#include "boost/thread.hpp"
#include "System/LogModule.h"

CameraComp::CameraComp() : m_strName("Camera"), m_bInited(false)
{
}

ErrorCode CameraComp::SetSpecialTable(bool bReconnect)
{
	bool bConnectChange = bReconnect;
	ErrorCode eCode = ErrorCode::ERROR_NONE;
	table tSpecial = SETTINGS->GetTable(SettingSection::Special);
	table tCamera;
	if (!tSpecial.count("Camera"))
	{
		tCamera["sRemoteHost"] = "127.0.0.1";
		tCamera["iRemotePort"] = 6800;
		tCamera["iTimeout"] = 1000;
		// 这部分为额外相机指令设置
		tCamera["sCommand0"] = "T1\r\n";
		tCamera["sCommand1"] = "T2\r\n";
		tCamera["sCommand2"] = "T3\r\n";
		tCamera["sCommand3"] = "T4\r\n";
		SETTINGS->SetTable(true, SettingSection::Special, tCamera, "Camera");
	}
	else
		tCamera = tSpecial["Camera"].as_table();

	eCode = SetTCPTable(tCamera, bConnectChange);
	if (eCode != ErrorCode::ERROR_NONE)
		return ErrorCode::ERROR_SPECIAL_LPCONNECTIONFAILED;

	eCode = InitMeasurement();
	return eCode;
}

ErrorCode CameraComp::InitMeasurement()
{
	ErrorCode eCode = ErrorCode::ERROR_NONE;
	// 初始化选项
	table tCamera = SETTINGS->GetTable(SettingSection::Special, "Camera");
	m_qlistCommands.clear();
	m_qlistCommands.append(tCamera["sCommand0"].as_string().data());
	m_qlistCommands.append(tCamera["sCommand1"].as_string().data());
	m_qlistCommands.append(tCamera["sCommand2"].as_string().data());
	m_qlistCommands.append(tCamera["sCommand3"].as_string().data());
	DT::setCameraCommands(m_qlistCommands);
	m_bInited = true;
	return eCode;
}

ErrorCode CameraComp::MeasurementComp(CompValue& cValue, int iIndex)
{
	QByteArray qbRecv;
	QString command = m_qlistCommands[iIndex];

	if (!SendCommand(command.toUtf8(), qbRecv))
		return ErrorCode::ERROR_CAMERA_COMMUNICATIONFAILURE;

	QString response = QString::fromUtf8(qbRecv);
	QStringList parts = response.split(',', Qt::SkipEmptyParts);
	if (parts.size() >= 2) {
		cValue.X = parts[0].trimmed();
		cValue.Y = parts[1].trimmed();
	}

	if (cValue.X == "-9999")
		return ErrorCode::ERROR_CAMERA_READFAILED;

	return ErrorCode::ERROR_NONE;
}

bool CameraComp::IsInited()
{
	// 检查是否初始化过并且 TCP 连接是否正常
	return m_bInited && IsConnected();
}