#include "MCFactory.h"

#if defined(LCNC_PROCESS_HAS_ACS) && LCNC_PROCESS_HAS_ACS
ACSMotionControl			MCFactory::m_ACSCMHP;
#endif
#if defined(LCNC_PROCESS_HAS_GTN) && LCNC_PROCESS_HAS_GTN
GTNMotionControl			MCFactory::m_GTN;
#endif
SimulateCMHPMotionControl	MCFactory::m_SimulatorCMHP;

 MotionControl* MCFactory::GetMotionController(const string &sName)
{
	#if defined(LCNC_PROCESS_HAS_ACS) && LCNC_PROCESS_HAS_ACS
	if(sName.compare("ACSCMHP") == 0)
		return &m_ACSCMHP;
	#endif
	#if defined(LCNC_PROCESS_HAS_GTN) && LCNC_PROCESS_HAS_GTN
	if (sName.compare("GTN") == 0)
		return &m_GTN;
	#endif
	if(sName.compare("SimulatorCMHP") == 0)
		return &m_SimulatorCMHP;
	return &m_SimulatorCMHP;
}

void MCFactory::GetAllMCName(std::vector<string> &vecName)
{
	vector<string> vec;
	#if defined(LCNC_PROCESS_HAS_ACS) && LCNC_PROCESS_HAS_ACS
	vec.push_back(m_ACSCMHP.GetName());
	#endif
	#if defined(LCNC_PROCESS_HAS_GTN) && LCNC_PROCESS_HAS_GTN
	vec.push_back(m_GTN.GetName());
	#endif
	vec.push_back(m_SimulatorCMHP.GetName());
	vecName.swap(vec);
}