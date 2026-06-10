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
	static LaserDevice *		GetLaserDevice(const string & sLaserDeviceName);
	static void					GetAll_LDName(vector<string> & vecName);
	
private:
	static SimulatorLaserDevice s_Simulator;
	#if defined(LCNC_PROCESS_HAS_REAL_LASER) && LCNC_PROCESS_HAS_REAL_LASER
	static IPGLaserDevice		s_IPG;
	static PharosLaserDevice	s_Pharos;
	static RaycusLaserDevice	s_Raycus;
	static RaycusAirCoolLaserDevice s_RaycusAirCool;
	static ULTRONLaserDevice	s_ULTRON;
	static RaycusQCWLaserDevice s_RaycusQCW;
	#endif
	static AnalogLaserDevice	s_Analog;
};
#endif
