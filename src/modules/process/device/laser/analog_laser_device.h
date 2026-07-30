/************************************************************************/
/*                            Analog激光器实现类                           */
/************************************************************************/

#ifndef _ANALOG_LASER_DEVICE_
#define _ANALOG_LASER_DEVICE_

#include "laser_device.h"
#include <string>
using namespace std;

class AnalogLaserDevice : public LaserDevice
{
public:
	explicit AnalogLaserDevice(lcnc::process::ProcessSettingsService& settings);
	virtual ErrorCode		setLaserTable(const table& tableLaser = table{});

	virtual const string&	GetName();
	virtual bool			IsInited() { return true; };
	virtual bool			IsAvailableData(const QByteArray&) { return true; };
	virtual bool			StartLaser() { return true; };
	virtual bool			StopLaser() { return true; };
	virtual bool			StartAimingBeam() { return true; };
	virtual bool			StopAimingBeam() { return true; };

	// 下发参数
	virtual bool			SetEnergy(double) { return true; };
	virtual bool			SetFrequency(double) { return true; };
	virtual bool			SetPulseWidth(double) { return true; };
	virtual bool			SetLaserParameter(const LaserParameter& parameter) { LaserDevice::UpdateLaserParameter(parameter); return true; };

	// 获取实际参数 
	virtual string			GetEnergy() { return "NotConnected"; };
	virtual string			GetFrequency() { return "NotConnected"; };
	virtual string			GetPulseWidth() { return "NotConnected"; };

	//IPG
	virtual double			GetAveragePower() { return 0.0; };		//平均功率
	virtual string			GetTemperature() { return "NotConnected"; };		//激光器温度
	virtual string			GetTroubleshooting() { return "NotConnected"; };

	

private:
	string					m_strName;
	bool					m_bIsInited;
	double					m_dMaxCurrent;
	double					m_dSimmerCurrent;
	int						m_iWaveShape;
	string					m_strTemperature;

public:
	// 非本型号函数
	
};
#endif
