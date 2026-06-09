#pragma once

#include "CompDevice.h"

class   SimulatorComp : public CompDevice
{
public:
	SimulatorComp();

	virtual const string&	GetName()			{ return m_strName; };
	virtual bool			IsInited()			{ return m_bInited; };
	virtual	ErrorCode		SetSpecialTable(bool bReconnect = false);
	virtual ErrorCode		InitMeasurement();
	virtual ErrorCode		MeasurementComp(CompValue& cValue, int iIndex = 0);

private:
	string	m_strName;
	bool	m_bInited;
};