/************************************************************************/
/*                            IPG激光器实现类                           */
/************************************************************************/

#ifndef _IPG_LASER_DEVICE_
#define _IPG_LASER_DEVICE_

#include "LaserDevice.h"
#include <string>
using namespace std;

class IPGLaserDevice : public LaserDevice
{
public:
	explicit IPGLaserDevice(lcnc::process::ProcessSettingsService& settings);
	virtual ErrorCode		SetLaserTable(const table& tableLaser = table{});

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

	//IPG
	virtual double			GetAveragePower();		//平均功率
	virtual string			GetTemperature();		//激光器温度
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
