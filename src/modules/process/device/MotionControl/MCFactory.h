#ifndef _MCFACTORY_
#define _MCFACTORY_

#include "MotionControl.h"
#include "SimulateCMHPMotionControl.h"
#if defined(LCNC_PROCESS_HAS_ACS) && LCNC_PROCESS_HAS_ACS
#include "ACSMotionControl.h"
#endif
#if defined(LCNC_PROCESS_HAS_GTN) && LCNC_PROCESS_HAS_GTN
#include "GTNMotionControl.h"
#endif

class  MCFactory
{
public:
	static MotionControl*				GetMotionController(const string &sMotionControlName);
	static void							GetAllMCName(vector<string> &vecName);
private:
	static SimulateCMHPMotionControl	m_SimulatorCMHP;
	#if defined(LCNC_PROCESS_HAS_ACS) && LCNC_PROCESS_HAS_ACS
	static ACSMotionControl				m_ACSCMHP;
	#endif
	#if defined(LCNC_PROCESS_HAS_GTN) && LCNC_PROCESS_HAS_GTN
	static GTNMotionControl				m_GTN;
	#endif
};

#endif // _MCFACTORY_