#pragma once

#include "CompDevice.h"
#include "TCPClient.h"

class   CameraComp : public CompDevice, public TCPClient
{
public:
	CameraComp();

	virtual const string& GetName() { return m_strName; };
	virtual bool IsInited();
	virtual ErrorCode SetSpecialTable(bool bReconnect = false);
	virtual ErrorCode InitMeasurement();
	virtual ErrorCode MeasurementComp(CompValue& cValue, int iIndex = 0);

private:
	string m_strName;
	bool m_bInited;
	QStringList m_qlistCommands;
};