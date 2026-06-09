#include "COMPFactory.h"

SimulatorComp	COMPFactory::s_Simulator;
CameraComp		COMPFactory::s_Camera;
JAPHLComp		COMPFactory::s_JAPHL;

CompDevice* COMPFactory::GetCOMPDevice(const std::string& sCompDeviceName)
{
	if (sCompDeviceName.compare("Simulator") == 0)
		return &s_Simulator;
	else if (sCompDeviceName.compare("JAPHL") == 0)
		return &s_JAPHL;
	else if (sCompDeviceName.compare("SZK") == 0)
		return &s_Camera;

	return  &s_Simulator;
}

void COMPFactory::GetAllCompName(vector<string>& vecNames)
{
	vecNames.push_back("Simulator");
	vecNames.push_back("Camera");
	vecNames.push_back("JAPHL");
}