/************************************************************************/
/*                               激光器基类                             */
/************************************************************************/
#pragma once 

#include "serial_port.h"
#include "toml.hpp"
#include "modules/process/settings/process_settings_service.h"

using namespace std;
using toml::table;

enum class LaserResponseMode
{
	Strict,
	FastVerified,
	SendOnly
};

struct LaserParameter
{
	LaserParameter()
		: dEnergy(0.0)
		, dFrequency(0.0)
		, dPulseWidth(0.0)
		, dAttenuatorPercentage(0.0)
		, dPpDivider(0.0)
		, iDelay(0)
		, eResponseMode(LaserResponseMode::Strict)
	{
	}

	LaserParameter(double dEnergyValue, double dFrequencyValue, double dPulseWidthValue,
		double dAttenuatorPercentageValue = 0.0, double dPpDividerValue = 0.0,
		int iDelayValue = 0, LaserResponseMode eResponseModeValue = LaserResponseMode::Strict)
		: dEnergy(dEnergyValue)
		, dFrequency(dFrequencyValue)
		, dPulseWidth(dPulseWidthValue)
		, dAttenuatorPercentage(dAttenuatorPercentageValue)
		, dPpDivider(dPpDividerValue)
		, iDelay(iDelayValue)
		, eResponseMode(eResponseModeValue)
	{
	}

	double dEnergy;
	double dFrequency;
	double dPulseWidth;
	double dAttenuatorPercentage;
	double dPpDivider;
	int iDelay;
	LaserResponseMode eResponseMode;
};

struct LaserCommunicationConfig
{
	LaserCommunicationConfig()
		: strHost("127.0.0.1")
		, iPort(20020)
		, strPath("v1/Basic")
		, iTimeOut(1000)
	{
	}

	string strHost;
	int iPort;
	string strPath;
	int iTimeOut;
};

class    LaserDevice : public SerialPort
{
public:
	// 手动更新激光参数缓存
	static void				UpdateEnergy(double dEnergy)			{ m_dEnergy		= dEnergy; m_lastLaserParameter.dEnergy = dEnergy;		};
	static void				UpdateFrequency(double dFrequency)		{ m_dFrequency	= dFrequency; m_lastLaserParameter.dFrequency = dFrequency;	};
	static void				UpdatePulseWidth(double dPulseWidth)	{ m_dPulseWidth = dPulseWidth; m_lastLaserParameter.dPulseWidth = dPulseWidth;	};
	static double			GetCurrentEnergy();
	static void				UpdateLaserParameter(const LaserParameter& parameter);
	static bool				IsChanged(const LaserParameter& parameter);

	// 下发
	virtual ErrorCode		setLaserTable();
	virtual ErrorCode		setLaserTable(const table& tableLaser) = 0;

	virtual const string&	GetName() = 0;
	virtual bool			IsInited() = 0;
	virtual bool			StartLaser() = 0;
	virtual bool			StopLaser() = 0;
	virtual bool			StartAimingBeam() = 0;
	virtual bool			StopAimingBeam() = 0;

	// 下发参数
	virtual bool			SetEnergy(double dEnergy) = 0;
	virtual bool			SetFrequency(double dFrequency) = 0;
	virtual bool			SetPulseWidth(double dPulseWidth) = 0;
	virtual bool			SetLaserParameter(const LaserParameter& parameter) = 0;
	virtual bool			SetPpDividerOnly(double,
		LaserResponseMode = LaserResponseMode::Strict) { return false; }

	// 获取实际参数 
	virtual string			GetEnergy() = 0;
	virtual string			GetFrequency() = 0;
	virtual string			GetPulseWidth() = 0;

	// IPG
	virtual double			GetAveragePower() = 0;
	virtual string			GetTemperature() = 0;
	virtual string			GetTroubleshooting() = 0;

	//// 设置最大电流（单位A）
	//virtual void SetLaserMaxCurrent(double dCurrent) = 0;
	//// 获取最大电流
	//virtual double GetLaserMaxCurrent() = 0;
	//// 设置Simmer电流
	//virtual bool SetSimmerEnergy(double dEnergy) = 0;
	//// 获取Simmer电流
	//virtual bool GetSimmerEnergy(double &dEnergy) = 0;
	//// 设置波形
	//virtual bool SetWaveShape(int iShape) = 0;
	//// 获取波形
	//virtual int  GetWaveShape() = 0;
	//virtual void SetLaserGate(string) = 0;
	//virtual void SetLaserModulation(string) = 0;

protected:
	explicit LaserDevice(lcnc::process::ProcessSettingsService& settings)
		: m_settings(settings) {}

	table processLaserTable(const QString& name = {}) const
	{
		return m_settings.rawTable(lcnc::process::ProcessConfigArea::Devices, name);
	}

	QVariant processLaserValue(const QString& tableName, const QString& key, const QVariant& fallback = {}) const
	{
		return m_settings.rawValue(lcnc::process::ProcessConfigArea::Devices, tableName, key, fallback);
	}

	lcnc::process::ProcessSettingsService& m_settings;
	static double			m_dEnergy;
	static double			m_dFrequency;
	static double			m_dPulseWidth;
	static LaserParameter	m_lastLaserParameter;
};
