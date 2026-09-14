/************************************************************************/
/*                            锐科激光器实现类                           */
/************************************************************************/

#pragma once

#include "laser_device.h"
#include <string>
class RaycusLaserDevice final : public LaserDevice
{
public:
	explicit RaycusLaserDevice(lcnc::process::ProcessSettingsService& settings);

	ErrorCode setLaserTable(const toml::table& tableLaser = {}) override;

	const std::string& GetName() override;
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
	std::string GetEnergy() override;
	std::string GetFrequency() override;
	std::string GetPulseWidth() override;

private:
	std::string				m_strName;
	bool					m_bIsInited;
	double					m_dMaxCurrent;
	double					m_dSimmerCurrent;
	int						m_iWaveShape;
		
public:
	// 非本型号函数
	virtual double GetAveragePower()	{ return 0; };
	std::string GetTemperature() override { return {}; }
	std::string GetTroubleshooting() override { return {}; }
	
};
