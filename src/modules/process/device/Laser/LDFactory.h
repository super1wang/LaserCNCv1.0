#ifndef _LDFACTORY_
#define _LDFACTORY_

#include "LaserDevice.h"
#include "IPGLaserDevice.h"
#include "PharosLaserDevice.h"
#include "RaycusLaserDevice.h"
#include "RaycusQCWLaserDevice.h"
#include "SimulatorLaserDevice.h"
#include "ULTRONLaserDevice.h"
#include "AnalogLaserDevice.h"

class	LDFactory
{
public:
	static LaserDevice *		GetLaserDevice(const string & sLaserDeviceName);
	static void					GetAll_LDName(vector<string> & vecName);
	
private:
	static SimulatorLaserDevice s_Simulator;
	static IPGLaserDevice		s_IPG;
	static PharosLaserDevice	s_Pharos;
	static RaycusLaserDevice	s_Raycus;
	static ULTRONLaserDevice	s_ULTRON;
	static RaycusQCWLaserDevice s_RaycusQCW;
	static AnalogLaserDevice	s_Analog;
};
#endif
