#include "LDFactory.h"

SimulatorLaserDevice	LDFactory::s_Simulator;
IPGLaserDevice			LDFactory::s_IPG;
PharosLaserDevice		LDFactory::s_Pharos;
RaycusLaserDevice		LDFactory::s_Raycus;
ULTRONLaserDevice		LDFactory::s_ULTRON;
RaycusQCWLaserDevice	LDFactory::s_RaycusQCW;
AnalogLaserDevice		LDFactory::s_Analog;


LaserDevice * LDFactory::GetLaserDevice(const std::string & sLaserDeviceName)
{
	if (sLaserDeviceName.compare("Simulator") == 0)
		return &s_Simulator;
	else if (sLaserDeviceName.compare("IPG") == 0)
		return &s_IPG;
	else if (sLaserDeviceName.compare("Pharos") == 0)
		return &s_Pharos;
	else if (sLaserDeviceName.compare("Raycus") == 0)
		return &s_Raycus;
	else if (sLaserDeviceName.compare("ULTRON") == 0)
		return &s_ULTRON;	
	else if (sLaserDeviceName.compare("RaycusQCW") == 0)
		return &s_RaycusQCW;
	else if (sLaserDeviceName.compare("AnalogControl") == 0)
		return &s_Analog;
	return  &s_Simulator;
}

void LDFactory::GetAll_LDName(vector<string> &vecName)
{
	vector<string> vec;
	vec.push_back(s_Simulator.GetName());
	vec.push_back(s_IPG.GetName());
	vec.push_back(s_Pharos.GetName());
	vec.push_back(s_Raycus.GetName());
	vec.push_back(s_RaycusQCW.GetName());
	vecName.swap(vec);
}
