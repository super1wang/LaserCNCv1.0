/************************************************************************/
/*                        锐科风冷激光器实现类                           */
/************************************************************************/

#ifndef _RAYCUS_AIR_COOL_LASER_DEVICE_
#define _RAYCUS_AIR_COOL_LASER_DEVICE_

#include "laser_device.h"
#include <string>
using namespace std;

class RaycusAirCoolLaserDevice : public LaserDevice
{
public:
	explicit RaycusAirCoolLaserDevice(lcnc::process::ProcessSettingsService& settings);

	virtual ErrorCode		setLaserTable(const table& tableLaser = table{});

	virtual const string&	GetName();
	virtual bool			IsInited();
	virtual bool			IsAvailableData(const QByteArray& data);
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

	// IPG 风格查询接口
	virtual double			GetAveragePower();
	virtual string			GetTemperature();
	virtual string			GetTroubleshooting();

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
