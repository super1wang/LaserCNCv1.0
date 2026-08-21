/************************************************************************/
/*                               激光器基类                             */
/************************************************************************/
#pragma once 

#include "laser_device.h"

using toml::table;

// Configuration access is provided by LaserDevice's injected settings reference.

double LaserDevice::m_dEnergy		= 0.0;
double LaserDevice::m_dFrequency	= 0.0;
double LaserDevice::m_dPulseWidth	= 0.0;
LaserParameter LaserDevice::m_lastLaserParameter;

double LaserDevice::GetCurrentEnergy()
{
	return m_dEnergy;
}

void LaserDevice::UpdateLaserParameter(const LaserParameter& parameter)
{
	m_lastLaserParameter = parameter;
	m_dEnergy = parameter.dEnergy;
	m_dFrequency = parameter.dFrequency;
	m_dPulseWidth = parameter.dPulseWidth;
}

bool LaserDevice::IsChanged(const LaserParameter& parameter)
{
	if (m_lastLaserParameter.dEnergy == parameter.dEnergy &&
		m_lastLaserParameter.dFrequency == parameter.dFrequency &&
		m_lastLaserParameter.dPulseWidth == parameter.dPulseWidth &&
		m_lastLaserParameter.dAttenuatorPercentage == parameter.dAttenuatorPercentage &&
		m_lastLaserParameter.dPpDivider == parameter.dPpDivider &&
		m_lastLaserParameter.iDelay == parameter.iDelay)
		return false;
	return true;
}

ErrorCode LaserDevice::setLaserTable()
{
    table tableLaser = processLaserTable();
	return setLaserTable(tableLaser);
}
