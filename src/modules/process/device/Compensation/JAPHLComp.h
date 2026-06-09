#pragma once

#include "CompDevice.h"
#include "SerialPort.h"

class	JAPHLComp : public CompDevice, SerialPort
{
public:
	JAPHLComp();
	virtual const string&	GetName()			{ return m_strName; };
	virtual bool			IsInited()			{ return m_bInited; };
	virtual	ErrorCode		SetSpecialTable(bool bReconnect = false);
	virtual ErrorCode		InitMeasurement();
	virtual ErrorCode		MeasurementComp(CompValue& cValue, int iIndex = 0);

	virtual bool			IsAvailableData(const QByteArray& data);	// 返回数据校验方式

private:
	ErrorCode	ReadDistance(QString& qstrDistance, int iIndex);						// 读取距离值
	ErrorCode	BuildCommand(QByteArray& qbCommand, int iIndex);						// 命令构建
	ErrorCode	ParseData(const QByteArray& data, QString& qstrDistance, int iIndex);	// 数据解析

private:
	string	m_strName;
	bool	m_bInited;
	int		m_iTimeGo;
	int		m_iTimeBack;
};