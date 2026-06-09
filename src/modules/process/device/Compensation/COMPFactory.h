#ifndef _COMPFACTORY_
#define _COMPFACTORY_

#include "CompDevice.h"
#include "SimulatorComp.h"
#include "CameraComp.h"
#include "JAPHLComp.h"

class COMPFactory
{
public:
	static CompDevice*		GetCOMPDevice(const string& sCompName);
	static void				GetAllCompName(vector<string>& vecName);
private:
	static SimulatorComp	s_Simulator;
	static CameraComp		s_Camera;
	static JAPHLComp		s_JAPHL;
};

#endif // _COMPFACTORY_