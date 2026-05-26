#include "PharosLaserDevice.h"
#include "LogModule.h"

#include <QDateTime>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>

namespace
{
	constexpr int kDefaultFastSendTimeoutMs = 50;
	constexpr int kPowerQueryTimeoutMs = 200;
	constexpr qint64 kPowerErrorLogIntervalMs = 10 * 1000;
	const char* const kPharosContentType = "text/plain";

	// Format a double as a stable ASCII text body for Pharos: fixed-point,
	// no scientific notation, trailing zeros and orphan decimal point removed.
	// 70.0 -> "70", 70.1 -> "70.1", 30.25 -> "30.25".
	QString FormatPharosNumber(double dValue, int iMaxDecimals = 6)
	{
		if (iMaxDecimals < 0)
			iMaxDecimals = 0;
		QString qstr = QString::number(dValue, 'f', iMaxDecimals);
		if (qstr.contains('.'))
		{
			while (qstr.endsWith('0'))
				qstr.chop(1);
			if (qstr.endsWith('.'))
				qstr.chop(1);
		}
		if (qstr == "-0")
			qstr = "0";
		return qstr;
	}
}

PharosLaserDevice::PharosLaserDevice()
	: m_httpClient()
	, m_strName("Pharos")
	, m_bIsInited(false)
	, m_communicationConfig()
	, m_dAttenuatorPercentage(0.0)
	, m_dPpDivider(0.0)
	, m_iDelay(0)
	, m_bPowerLinkOk(true)
	, m_iLastPowerErrorLogMs(0)
{
}

ErrorCode PharosLaserDevice::SetLaserTable(const table& tableLaser)
{
	table tableMerged = MergeLaserTable(tableLaser);
	UpdateHttpCache(tableMerged);
	m_httpClient.SetTimeout(m_communicationConfig.iTimeOut);

	LaserParameter parameter(m_dEnergy, m_dFrequency, m_dPulseWidth,
		m_dAttenuatorPercentage, m_dPpDivider, m_iDelay);
	if (tableMerged.count("Laser"))
	{
		table tLaser = tableMerged.at("Laser").as_table();
		if (tLaser.count("fEnergy"))
			parameter.dEnergy = tLaser.at("fEnergy").as_floating();
		if (tLaser.count("fFrequency"))
			parameter.dFrequency = tLaser.at("fFrequency").as_floating();
		if (tLaser.count("fPulseWidth"))
			parameter.dPulseWidth = tLaser.at("fPulseWidth").as_floating();
	}

	parameter.dAttenuatorPercentage = m_dAttenuatorPercentage;
	parameter.dPpDivider = m_dPpDivider;
	parameter.iDelay = m_iDelay;

	if (!SetLaserParameter(parameter))
	{
		m_bIsInited = false;
		return ErrorCode::ERROR_LASER_SETTINGFAILED;
	}

	m_bIsInited = true;
	return ErrorCode::ERROR_NONE;
}

const string& PharosLaserDevice::GetName()
{
	return m_strName;
}

bool PharosLaserDevice::IsInited()
{
	return m_bIsInited;
}

bool PharosLaserDevice::StartLaser()
{
	return PutTextBody("EnableOutput", QByteArray("0"), LaserResponseMode::Strict);
}

bool PharosLaserDevice::StopLaser()
{
	return PutTextBody("CloseOutput", QByteArray("0"), LaserResponseMode::Strict);
}

bool PharosLaserDevice::StartAimingBeam()
{
	return true;
}

bool PharosLaserDevice::StopAimingBeam()
{
	return true;
}

bool PharosLaserDevice::SetEnergy(double dEnergy)
{
	LaserDevice::UpdateEnergy(dEnergy);
	return true;
}

bool PharosLaserDevice::SetFrequency(double dFrequency)
{
	LaserDevice::UpdateFrequency(dFrequency);
	return true;
}

bool PharosLaserDevice::SetPulseWidth(double dPulseWidth)
{
	LaserDevice::UpdatePulseWidth(dPulseWidth);
	return true;
}

bool PharosLaserDevice::SetLaserParameter(const LaserParameter& parameter)
{
	if (parameter.dAttenuatorPercentage == 0.0 && parameter.dPpDivider == 0.0)
	{
		m_dAttenuatorPercentage = parameter.dAttenuatorPercentage;
		m_dPpDivider = parameter.dPpDivider;
		m_iDelay = parameter.iDelay;
		LaserDevice::UpdateLaserParameter(parameter);
		return true;
	}

	if (!PutLaserParameter("TargetAttenuatorPercentage", parameter.dAttenuatorPercentage, parameter.eResponseMode))
		return false;
	if (!PutLaserParameter("TargetPpDivider", parameter.dPpDivider, parameter.eResponseMode))
		return false;

	if (parameter.iDelay > 0)
		Sleep(static_cast<DWORD>(parameter.iDelay));

	m_dAttenuatorPercentage = parameter.dAttenuatorPercentage;
	m_dPpDivider = parameter.dPpDivider;
	m_iDelay = parameter.iDelay;
	LaserDevice::UpdateLaserParameter(parameter);
	return true;
}

bool PharosLaserDevice::SetPpDividerOnly(double dPpDivider, LaserResponseMode eResponseMode)
{
	if (!PutLaserParameter("TargetPpDivider", dPpDivider, eResponseMode))
		return false;

	m_dPpDivider = dPpDivider;
	LaserParameter parameter(m_dEnergy, m_dFrequency, m_dPulseWidth,
		m_dAttenuatorPercentage, dPpDivider, 0, eResponseMode);
	LaserDevice::UpdateLaserParameter(parameter);
	return true;
}

string PharosLaserDevice::GetEnergy()
{
	return std::to_string(m_dEnergy);
}

string PharosLaserDevice::GetFrequency()
{
	return std::to_string(m_dFrequency);
}

string PharosLaserDevice::GetPulseWidth()
{
	return std::to_string(m_dPulseWidth);
}

double PharosLaserDevice::GetAveragePower()
{
	QString qstrUrl = BuildRequestUrl("ActualOutputPower");

	// B方案: 缩短查询超时, 避免高频轮询时断线阻塞主线程; 调用结束后恢复.
	const int iSavedTimeout = m_httpClient.GetTimeout();
	m_httpClient.SetTimeout(kPowerQueryTimeoutMs);

	QByteArray response;
	const bool bGotOk = m_httpClient.GetSilent(qstrUrl, response);
	const QString qstrLastError = m_httpClient.GetLastError();

	m_httpClient.SetTimeout(iSavedTimeout);

	auto LogPowerFailure = [this, &qstrUrl](const QString& qstrReason) {
		const qint64 iNowMs = QDateTime::currentMSecsSinceEpoch();
		const bool bWasOk = m_bPowerLinkOk;
		m_bPowerLinkOk = false;
		if (bWasOk)
		{
			m_iLastPowerErrorLogMs = iNowMs;
			LOG_SYS_ERROR("Pharos GetAveragePower link down, url: " + qstrUrl.toStdString()
				+ ", reason: " + qstrReason.toStdString());
		}
		else if (iNowMs - m_iLastPowerErrorLogMs >= kPowerErrorLogIntervalMs)
		{
			m_iLastPowerErrorLogMs = iNowMs;
			LOG_SYS_WARN("Pharos GetAveragePower still failing, url: " + qstrUrl.toStdString()
				+ ", reason: " + qstrReason.toStdString());
		}
	};

	auto MarkPowerOk = [this, &qstrUrl]() {
		if (!m_bPowerLinkOk)
		{
			m_bPowerLinkOk = true;
			m_iLastPowerErrorLogMs = 0;
			LOG_SYS_INFO("Pharos GetAveragePower link recovered, url: " + qstrUrl.toStdString());
		}
	};

	if (!bGotOk)
	{
		LogPowerFailure(qstrLastError);
		return 0.0;
	}

	QByteArray trimmed = response.trimmed();
	if (trimmed.isEmpty())
	{
		LogPowerFailure(QStringLiteral("empty response"));
		return 0.0;
	}

	// Try plain numeric first.
	bool bOk = false;
	double dPower = QString::fromLatin1(trimmed).toDouble(&bOk);
	if (bOk)
	{
		MarkPowerOk();
		return dPower;
	}

	// Try JSON: { "value": <num> } or { "ActualOutputPower": <num> }.
	QJsonParseError jsonErr{};
	QJsonDocument doc = QJsonDocument::fromJson(trimmed, &jsonErr);
	if (jsonErr.error == QJsonParseError::NoError && doc.isObject())
	{
		QJsonObject obj = doc.object();
		QJsonValue val = obj.value("value");
		if (val.isUndefined() || val.isNull())
			val = obj.value("ActualOutputPower");
		if (val.isDouble())
		{
			MarkPowerOk();
			return val.toDouble();
		}
		if (val.isString())
		{
			double d = val.toString().toDouble(&bOk);
			if (bOk)
			{
				MarkPowerOk();
				return d;
			}
		}
	}

	LogPowerFailure(QStringLiteral("parse failed"));
	return 0.0;
}

string PharosLaserDevice::GetTemperature()
{
	return "";
}

string PharosLaserDevice::GetTroubleshooting()
{
	return "";
}

table PharosLaserDevice::MergeLaserTable(const table& tableLaser) const
{
	table tableMerged = SETTINGS->GetTable(SettingSection::Laser);
	for (const auto& section : tableLaser)
	{
		if (!section.second.is_table())
		{
			tableMerged[section.first] = section.second;
			continue;
		}

		table tableSection = {};
		if (tableMerged.count(section.first) && tableMerged.at(section.first).is_table())
			tableSection = tableMerged.at(section.first).as_table();

		for (const auto& item : section.second.as_table())
			tableSection[item.first] = item.second;

		tableMerged[section.first] = tableSection;
	}
	return tableMerged;
}

void PharosLaserDevice::UpdateHttpCache(const table& tableMerged)
{
	if (tableMerged.count("Laser"))
	{
		table tLaser = tableMerged.at("Laser").as_table();
		if (tLaser.count("fAttenuatorPercentage"))
			m_dAttenuatorPercentage = tLaser.at("fAttenuatorPercentage").as_floating();
		if (tLaser.count("fPpDivider"))
			m_dPpDivider = tLaser.at("fPpDivider").as_floating();
		if (tLaser.count("iDelay"))
			m_iDelay = static_cast<int>(tLaser.at("iDelay").as_integer());
	}

	if (tableMerged.count("HTTP"))
	{
		table tHTTP = tableMerged.at("HTTP").as_table();
		if (tHTTP.count("sHost"))
			m_communicationConfig.strHost = tHTTP.at("sHost").as_string();
		if (tHTTP.count("iPort"))
			m_communicationConfig.iPort = static_cast<int>(tHTTP.at("iPort").as_integer());
		if (tHTTP.count("sPath"))
			m_communicationConfig.strPath = tHTTP.at("sPath").as_string();
		if (tHTTP.count("iTimeOut"))
			m_communicationConfig.iTimeOut = static_cast<int>(tHTTP.at("iTimeOut").as_integer());
	}
}

QString PharosLaserDevice::BuildRequestUrl(const QString& suffix) const
{
	QString qstrPath = QString::fromStdString(m_communicationConfig.strPath).trimmed();
	while (qstrPath.startsWith('/'))
		qstrPath.remove(0, 1);
	while (qstrPath.endsWith('/'))
		qstrPath.chop(1);

	QString qstrUrl = QString("http://%1:%2")
		.arg(QString::fromStdString(m_communicationConfig.strHost))
		.arg(m_communicationConfig.iPort);
	if (!qstrPath.isEmpty())
		qstrUrl += "/" + qstrPath;
	qstrUrl += "/" + suffix;
	return qstrUrl;
}

bool PharosLaserDevice::PutLaserParameter(const QString& suffix, double dValue,
	LaserResponseMode eResponseMode)
{
	QString qstrData = FormatPharosNumber(dValue);
	return PutTextBody(suffix, qstrData.toLatin1(), eResponseMode, false);
}

bool PharosLaserDevice::PutTextBody(const QString& suffix, const QByteArray& body,
	LaserResponseMode eResponseMode, bool bEnableHttpLog)
{
	QString qstrUrl = BuildRequestUrl(suffix);
	QByteArray response;
	bool bOk = false;
	switch (eResponseMode)
	{
	case LaserResponseMode::Strict:
		bOk = m_httpClient.Put(qstrUrl, body, response, kPharosContentType, bEnableHttpLog);
		break;
	case LaserResponseMode::FastVerified:
		bOk = m_httpClient.PutFast(qstrUrl, body,
			kPharosContentType, kDefaultFastSendTimeoutMs, bEnableHttpLog);
		break;
	case LaserResponseMode::SendOnly:
		bOk = m_httpClient.PutSendOnly(qstrUrl, body, kPharosContentType, bEnableHttpLog);
		break;
	}

	const std::string strBody(body.constData(), body.size());
	if (!bOk)
	{
		LOG_SYS_ERROR("Pharos HTTP PUT failed, suffix: " + suffix.toStdString()
			+ ", value: " + strBody
			+ ", completion: " + m_httpClient.GetLastCompletionText().toStdString()
			+ ", error: " + m_httpClient.GetLastError().toStdString());
		return false;
	}
	return true;
}
