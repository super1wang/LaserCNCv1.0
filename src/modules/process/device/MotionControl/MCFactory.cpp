#include "MCFactory.h"

ACSMotionControl			MCFactory::m_ACSCMHP;
GTNMotionControl			MCFactory::m_GTN;
SimulateCMHPMotionControl	MCFactory::m_SimulatorCMHP;

 MotionControl* MCFactory::GetMotionController(const string &sName)
{
	if(sName.compare("ACSCMHP") == 0)
		return &m_ACSCMHP;
	if (sName.compare("GTN") == 0)
		return &m_GTN;
	if(sName.compare("SimulatorCMHP") == 0)
		return &m_SimulatorCMHP;
	return &m_SimulatorCMHP;
}

void MCFactory::GetAllMCName(std::vector<string> &vecName)
{
	vector<string> vec;
	vec.push_back(m_ACSCMHP.GetName());
	vec.push_back(m_SimulatorCMHP.GetName());
	vecName.swap(vec);
}