#include "SignalSource.h"
#include "modules/process/settings/process_settings_service.h"
#include <stdlib.h>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
using namespace std;

SignalSource::SignalSource() : m_strPulseWidth("30.0"), m_strFrequency("8000.0")
{
}

ErrorCode SignalSource::SetSignalSourceTable()
{
    table tableLaser = lcnc::process::ProcessSettingsService::current()
        ? lcnc::process::ProcessSettingsService::current()->rawTable(lcnc::process::ProcessConfigArea::Devices) : table{};
	return SetSignalSourceTable(tableLaser);
}

ErrorCode SignalSource::SetSignalSourceTable(const table& tableLaser)
{
	bool bConnectChange = false;

	// 串口
	ErrorCode eCode = ErrorCode::ERROR_NONE;
	if (tableLaser.count("SignalSource"))
	{
        table tCom = lcnc::process::ProcessSettingsService::current()
            ? lcnc::process::ProcessSettingsService::current()->rawTable(lcnc::process::ProcessConfigArea::Devices, "SignalSource") : table{};
		eCode = SetComTable(tCom, bConnectChange);
		if (eCode != ErrorCode::ERROR_NONE)
			return ErrorCode::ERROR_SIGNALSOURCE_CONNECTIONFAILED;
	}

	// 参数
	if (tableLaser.count("Laser") || bConnectChange)
	{
		bool	bChange = false;
		double  dFrequency	= m_dFrequency;
		double  dPulseWidth = m_dPulseWidth;
		
		table tLaser;
		if (bConnectChange)
            tLaser = lcnc::process::ProcessSettingsService::current()
                ? lcnc::process::ProcessSettingsService::current()->rawTable(lcnc::process::ProcessConfigArea::Devices, "Laser") : table{};
		else
			tLaser = tableLaser.at("Laser").as_table();

		if (tLaser.count("fFrequency"))
		{
			bChange = true;
			dFrequency = tLaser["fFrequency"].as_floating();
		}
		if (tLaser.count("fPulseWidth"))
		{
			bChange = true;
			dPulseWidth = tLaser["fPulseWidth"].as_floating();
		}
		if (bChange)
		{
			if (!SetFrequencyAndPulseWidth(dFrequency, dPulseWidth))
			{
				eCode = ErrorCode::ERROR_SIGNALSOURCE_SETTINGFAILED;
				SHOW_OPER_ERROR(ErrorCode::ERROR_SIGNALSOURCE_SETTINGFAILED, 
					QObject::tr("Set signal source frequency %1 pulse width %2 failed.")
					.arg(dFrequency).arg(dPulseWidth).toUtf8().data());
			}
		}
	}

	return eCode;
}

bool SignalSource::SetFrequency(double dFrequency)
{
	m_dFrequency = dFrequency;
	if (IsConnected())
		return SetFrequencyValue(dFrequency);
	return false;
}

bool SignalSource::SetPulseWidth(double dPulseWith)
{
	m_dPulseWidth = dPulseWith;
	if (IsConnected())
		return SetPulseWidthValue(dPulseWith);
	return false;
}

bool SignalSource::SetFrequencyAndPulseWidth(double dFrequency, double dPulseWidth)
{
	m_dFrequency  = dFrequency;
	m_dPulseWidth = dPulseWidth;
	if (IsConnected())
		return SetFrequencyAndPulseWidthValue(dFrequency, dPulseWidth);
	return false;
}

int SignalSource::GetFrequency()
{
	if (IsConnected())
		return GetFrequencyValue();
	return m_dFrequency;
}

int SignalSource::GetPulseWidth()
{
	if (IsConnected())
		return GetPulseWidthValue();
	return m_dPulseWidth;
}

void SignalSource::GetFrequencyAndPulseWidth()
{
	if (IsConnected())
	{
		GetFrequencyAndPulseWidthValue();
	}
}


bool SignalSource::IsAvailableData(const QByteArray& data)
{
	return data.size();
}

bool SignalSource::SetFrequencyValue(double dFrequency)
{
	if (!GetPulseWidth())
		return false;

	char szFrequency[15];
	char buffer[_CVTBUFSIZE];
	memset(buffer, 0, 10);
	if ((dFrequency > 0) && (dFrequency < 100000))
	{
		for (int i = 0; i < 5; ++i)
		{
			buffer[4 - i] = '0' + (int)dFrequency % 10;
			dFrequency /= 10;
		}
		const char* head = m_strPulseWidth.c_str();
		const char* entr = "0\r";
		strcpy(szFrequency, buffer);
		strcat(szFrequency, head);
		strcat(szFrequency, entr);

		if (OnceData(szFrequency))
		{
			GetFrequency();
			return true;
		}
		else
			return false;
	}
	else
		return false;
}

bool SignalSource::SetPulseWidthValue(double dPulseWidth)
{
	if (GetFrequency())
		return false;

	char szPulseWidth[15];
	char buffer[_CVTBUFSIZE];
	memset(buffer, 0, 10);
	if ((dPulseWidth > 0) && (dPulseWidth < 10000))
	{
		for (int i = 0; i < 4; ++i)
		{
			buffer[3 - i] = '0' + (int)dPulseWidth % 10;
			dPulseWidth /= 10;
		}
		const char* head = m_strFrequency.c_str();
		const char* entr = "0\r";
		strcpy(szPulseWidth, head);
		strcat(szPulseWidth, buffer);
		strcat(szPulseWidth, entr);

		if (OnceData(szPulseWidth))
		{
			GetPulseWidth();
			return true;
		}
		else
			return false;
	}
	else
		return false;
}

bool SignalSource::SetFrequencyAndPulseWidthValue(double dFrequency, double dPulseWidth)
{
	char szFrequencyAndPulseWidth[15];
	char bufferFrequency[_CVTBUFSIZE];
	memset(bufferFrequency, 0, 10);
	char bufferPulse[_CVTBUFSIZE];
	memset(bufferPulse, 0, 10);

	if ((dFrequency > 0) && (dFrequency < 100000))
	{
		for (int i = 0; i < 5; ++i)
		{
			bufferFrequency[4 - i] = '0' + (int)dFrequency % 10;
			dFrequency /= 10;
		}
	}
	else
		return false;

	if ((dPulseWidth > 0) && (dPulseWidth < 10000))
	{
		for (int j = 0; j < 4; ++j)
		{
			bufferPulse[3 - j] = '0' + (int)dPulseWidth % 10;
			dPulseWidth /= 10;
		}
	}
	else
		return false;

	const char* entr = "0\r";
	strcpy(szFrequencyAndPulseWidth, bufferFrequency);
	strcat(szFrequencyAndPulseWidth, bufferPulse);
	strcat(szFrequencyAndPulseWidth, entr);

	return OnceData(szFrequencyAndPulseWidth);
}

int SignalSource::GetFrequencyValue()
{
	m_strFrequency = string("");
	GetFrequencyAndPulseWidthValue();
	char mid[6];
	memset(mid, 0, 6);
	char m_chFrequency[128];
	strcpy(m_chFrequency, m_strFrequencyAndPulseWidth.c_str());
	mid[0] = m_chFrequency[0];
	mid[1] = m_chFrequency[1];
	mid[2] = m_chFrequency[2];
	mid[3] = m_chFrequency[3];
	mid[4] = m_chFrequency[4];
	mid[5] = '\0';
	m_strFrequency = mid;
	return atoi(m_strFrequency.c_str());
}

int SignalSource::GetPulseWidthValue()
{
	m_strPulseWidth = string("");
	GetFrequencyAndPulseWidthValue();
	char mid[5];
	memset(mid, 0, 5);
	char m_chPulse[128];
	strcpy(m_chPulse, m_strFrequencyAndPulseWidth.c_str());
	mid[0] = m_chPulse[5];
	mid[1] = m_chPulse[6];
	mid[2] = m_chPulse[7];
	mid[3] = m_chPulse[8];
	mid[4] = '\0';
	m_strPulseWidth = mid;
	return atoi(m_strPulseWidth.c_str());
}

void SignalSource::GetFrequencyAndPulseWidthValue()
{
	char cOrder[15] = "0000000002";
	const char* entr = "\r";
	strcat(cOrder, entr);
	
	QByteArray strOut;
	bool success = OnceData(cOrder, strOut);
	m_strFrequencyAndPulseWidth = strOut.toStdString();

	Sleep(100);
}
