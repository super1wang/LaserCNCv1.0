#pragma once

#include "laser_device.h"
#include "modules/process/device/connect/http_client.h"

class PharosLaserDevice final : public LaserDevice
{
	public:
	explicit PharosLaserDevice(lcnc::process::ProcessSettingsService& settings);

	ErrorCode setLaserTable(const toml::table& tableLaser = {}) override;

	const std::string& GetName() override;
	virtual bool			IsInited();
	virtual bool			StartLaser();
	virtual bool			StopLaser();
	virtual bool			StartAimingBeam();
	virtual bool			StopAimingBeam();

	virtual bool			SetEnergy(double dEnergy);
	virtual bool			SetFrequency(double dFrequency);
	virtual bool			SetPulseWidth(double dPulseWidth);
	virtual bool			SetLaserParameter(const LaserParameter& parameter);
	virtual bool			SetPpDividerOnly(double dPpDivider,
		LaserResponseMode eResponseMode = LaserResponseMode::Strict);

	std::string GetEnergy() override;
	std::string GetFrequency() override;
	std::string GetPulseWidth() override;

	virtual double			GetAveragePower();
	std::string GetTemperature() override;
	std::string GetTroubleshooting() override;

private:
	toml::table MergeLaserTable(const toml::table& tableLaser) const;
	void UpdateHttpCache(const toml::table& tableMerged);
	QString					BuildRequestUrl(const QString& suffix) const;
	bool					PutLaserParameter(const QString& suffix, double dValue,
		LaserResponseMode eResponseMode = LaserResponseMode::Strict);
	bool					PutTextBody(const QString& suffix, const QByteArray& body,
		LaserResponseMode eResponseMode = LaserResponseMode::Strict,
		bool bEnableHttpLog = true);

private:
	HTTPClient				m_httpClient;
	std::string				m_strName;
	bool					m_bIsInited;
	LaserCommunicationConfig m_communicationConfig;
	double					m_dAttenuatorPercentage;
	double					m_dPpDivider;
	int						m_iDelay;

	// GetAveragePower link state for log throttling.
	bool					m_bPowerLinkOk;
	qint64					m_iLastPowerErrorLogMs;
};
