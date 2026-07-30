#ifndef _LDFACTORY_
#define _LDFACTORY_

#include "laser_device.h"
#if defined(LCNC_PROCESS_HAS_REAL_LASER) && LCNC_PROCESS_HAS_REAL_LASER
#include "ipg_laser_device.h"
#include "pharos_laser_device.h"
#include "raycus_laser_device.h"
#include "raycus_air_cool_laser_device.h"
#include "raycus_qcw_laser_device.h"
#include "ultron_laser_device.h"
#endif
#include "simulator_laser_device.h"
#include "analog_laser_device.h"

class	LDFactory
{
public:
	explicit LDFactory(lcnc::process::ProcessSettingsService& settings);
	LaserDevice *		laserDevice(const string & sLaserDeviceName);
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
