#include "SimulatorLaserDevice.h"

SimulatorLaserDevice::SimulatorLaserDevice() : m_strName("Simulator"), m_bIsInited(false)
											, m_dMaxCurrent(0), m_dSimmerCurrent(0), m_iWaveShape(0)
{
	m_qstrPort		= "COM1";
	m_dwBaudRate	= 38400;
	m_iDataBits		= 8;
	m_iParity		= 0;
	m_iStopBits		= 1;
}

ErrorCode SimulatorLaserDevice::SetLaserTable(const table& tableLaser)
{
	// 参数
	if (tableLaser.count("Laser"))
	{
		table tLaser = tableLaser.at("Laser").as_table();
		if (tLaser.count("fEnergy"))
		{
			double dEnergy = tLaser["fEnergy"].as_floating();
			SetEnergy(dEnergy);
		}
		if (tLaser.count("fFrequency"))
		{
			double dFrequency = tLaser["fFrequency"].as_floating();
			SetFrequency(dFrequency);
		}
		if (tLaser.count("fPulseWidth"))
		{
			double dPulseWidth = tLaser["fPulseWidth"].as_floating();
			SetPulseWidth(dPulseWidth);
		}
	}
	m_bIsInited = true;
	return ErrorCode::ERROR_NONE;
}

const string & SimulatorLaserDevice::GetName()
{
	return m_strName;
}

bool SimulatorLaserDevice::IsInited()
{
	return m_bIsInited;
}

bool SimulatorLaserDevice::StartLaser()
{
	return WriteData("StartLaser");
}

bool SimulatorLaserDevice::StopLaser()
{
	return WriteData("StopLaser");
}

bool SimulatorLaserDevice::StartAimingBeam()
{
	return WriteData("StartAimingBeam");
}

bool SimulatorLaserDevice::StopAimingBeam()
{
	return WriteData("StopAimingBeam");
}

bool SimulatorLaserDevice::SetEnergy(double dEnergy)
{
	LaserDevice::UpdateEnergy(dEnergy);
	return true;
}

bool SimulatorLaserDevice::SetFrequency(double dFrequency)
{
	LaserDevice::UpdateFrequency(dFrequency);
	return true;
}

bool SimulatorLaserDevice::SetPulseWidth(double dPulseWidth)
{
	LaserDevice::UpdatePulseWidth(dPulseWidth);
	return true;
}

bool SimulatorLaserDevice::SetLaserParameter(const LaserParameter& parameter)
{
	LaserDevice::UpdateLaserParameter(parameter);
	return true;
}

string SimulatorLaserDevice::GetEnergy()
{
	string strEnerg = std::to_string(m_dEnergy);
	return strEnerg;
}

string SimulatorLaserDevice::GetFrequency()
{
	string strFrequency = std::to_string(m_dFrequency);
	return strFrequency;
}

string SimulatorLaserDevice::GetPulseWidth()
{
	string strPulseWidth = std::to_string(m_dPulseWidth);
	return strPulseWidth;
}