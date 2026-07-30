#pragma once

#include "laser_device.h"
#include "http_client.h"

class PharosLaserDevice : public LaserDevice
{
	public:
	explicit PharosLaserDevice(lcnc::process::ProcessSettingsService& settings);

	virtual ErrorCode		setLaserTable(const table& tableLaser = table{});

	virtual const string&	GetName();
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

	virtual string			GetEnergy();
	virtual string			GetFrequency();
	virtual string			GetPulseWidth();

	virtual double			GetAveragePower();
	virtual string			GetTemperature();
	virtual string			GetTroubleshooting();

private:
	table					MergeLaserTable(const table& tableLaser) const;
	void					UpdateHttpCache(const table& tableMerged);
	QString					BuildRequestUrl(const QString& suffix) const;
	bool					PutLaserParameter(const QString& suffix, double dValue,
		LaserResponseMode eResponseMode = LaserResponseMode::Strict);
	bool					PutTextBody(const QString& suffix, const QByteArray& body,
		LaserResponseMode eResponseMode = LaserResponseMode::Strict,
		bool bEnableHttpLog = true);

private:
	HTTPClient				m_httpClient;
	string					m_strName;
	bool					m_bIsInited;
	LaserCommunicationConfig m_communicationConfig;
	double					m_dAttenuatorPercentage;
	double					m_dPpDivider;
	int						m_iDelay;

	// GetAveragePower link state for log throttling.
	bool					m_bPowerLinkOk;
	qint64					m_iLastPowerErrorLogMs;
};
