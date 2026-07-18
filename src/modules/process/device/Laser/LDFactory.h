#ifndef _LDFACTORY_
#define _LDFACTORY_

#include "LaserDevice.h"
#if defined(LCNC_PROCESS_HAS_REAL_LASER) && LCNC_PROCESS_HAS_REAL_LASER
#include "IPGLaserDevice.h"
#include "PharosLaserDevice.h"
#include "RaycusLaserDevice.h"
#include "RaycusAirCoolLaserDevice.h"
#include "RaycusQCWLaserDevice.h"
#include "ULTRONLaserDevice.h"
#endif
#include "SimulatorLaserDevice.h"
#include "AnalogLaserDevice.h"

class	LDFactory
{
public:
	explicit LDFactory(lcnc::process::ProcessSettingsService& settings);
	LaserDevice *		GetLaserDevice(const string & sLaserDeviceName);
	void					GetAll_LDName(vector<string> & vecName);
	
private:
	SimulatorLaserDevice m_simulator;
	#if defined(LCNC_PROCESS_HAS_REAL_LASER) && LCNC_PROCESS_HAS_REAL_LASER
	IPGLaserDevice		m_ipg;
	PharosLaserDevice	m_pharos;
	RaycusLaserDevice	m_raycus;
	RaycusAirCoolLaserDevice m_raycusAirCool;
	ULTRONLaserDevice	m_ultron;
	RaycusQCWLaserDevice m_raycusQCW;
	#endif
	AnalogLaserDevice	m_analog;
};
#endif
