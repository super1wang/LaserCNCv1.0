/************************************************************************/
/*                            Analog激光器实现类                           */
/************************************************************************/

#pragma once

#include "laser_device.h"
#include <string>
class AnalogLaserDevice final : public LaserDevice
{
public:
	explicit AnalogLaserDevice(lcnc::process::ProcessSettingsService& settings);
	ErrorCode setLaserTable(const toml::table& tableLaser = {}) override;

	const std::string& GetName() override;
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
	std::string GetEnergy() override { return "NotConnected"; }
	std::string GetFrequency() override { return "NotConnected"; }
	std::string GetPulseWidth() override { return "NotConnected"; }

	//IPG
	virtual double			GetAveragePower() { return 0.0; };		//平均功率
	std::string GetTemperature() override { return "NotConnected"; }		//激光器温度
	std::string GetTroubleshooting() override { return "NotConnected"; }

	

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
