#pragma once

#include "laser_device.h"

class   SimulatorLaserDevice : public LaserDevice
{
public:
	explicit SimulatorLaserDevice(lcnc::process::ProcessSettingsService& settings);

	// 仿真激光器不打开真实串口，仅维护连接标志
	virtual bool			Connect() override;
	virtual void			Disconnect() override;

	virtual ErrorCode		setLaserTable(const table& tableLaser = table{});

	virtual const string&	GetName();
	virtual bool			IsInited();
	virtual bool			StartLaser();
	virtual bool			StopLaser();
	virtual bool			StartAimingBeam();
	virtual bool			StopAimingBeam();

	// 下发参数
	virtual bool			SetEnergy(double dEnergy);
	virtual bool			SetFrequency(double dFrequency);
	virtual bool			SetPulseWidth(double dPulseWidth);
	virtual bool			SetLaserParameter(const LaserParameter& parameter);

	// 获取实际参数 
	virtual string			GetEnergy();
	virtual string			GetFrequency();
	virtual string			GetPulseWidth();

private:  
	string					m_strName;
	bool					m_bIsInited;
	double					m_dMaxCurrent;
	double					m_dSimmerCurrent;
	int						m_iWaveShape;
	
public:
	// 非本型号函数
	virtual double GetAveragePower()	{ return 0; };
	virtual string GetTemperature()		{ return ""; };
	virtual string GetTroubleshooting() { return ""; };
	
};
