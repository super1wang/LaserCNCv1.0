/************************************************************************/
/*                        锐科风冷激光器实现类                           */
/************************************************************************/

#pragma once

#include "laser_device.h"
#include <string>
class RaycusAirCoolLaserDevice final : public LaserDevice
{
public:
	explicit RaycusAirCoolLaserDevice(lcnc::process::ProcessSettingsService& settings);

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

	// IPG 风格查询接口
	virtual double			GetAveragePower();
	std::string GetTemperature() override;
	std::string GetTroubleshooting() override;

private:
	std::string				m_strName;
	bool					m_bIsInited;
	double					m_dMaxCurrent;
	double					m_dSimmerCurrent;
	int						m_iWaveShape;
	std::string				m_strTemperature;

public:
	// 非本型号函数
};
