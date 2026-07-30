/************************************************************************/
/*                            锐科激光器实现类                           */
/************************************************************************/

#ifndef _RAYCUS_LASER_DEVICE_
#define _RAYCUS_LASER_DEVICE_

#include "laser_device.h"
#include <string>
using namespace std;

class RaycusLaserDevice : public LaserDevice
{
public:
	explicit RaycusLaserDevice(lcnc::process::ProcessSettingsService& settings);

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

private:
	char*					hextochs(char* ascii);
	char*					ftoa(double res, char* des, int type);

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
	virtual string GetTroubleshooting()	{ return ""; };
	
};
#endif
