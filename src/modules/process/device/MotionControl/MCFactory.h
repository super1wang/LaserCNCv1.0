#ifndef _MCFACTORY_
#define _MCFACTORY_

#include "MotionControl.h"
#include "SimulateCMHPMotionControl.h"
#include "ACSMotionControl.h"
#include "GTNMotionControl.h"

class  MCFactory
{
public:
	static MotionControl*				GetMotionController(const string &sMotionControlName);
	static void							GetAllMCName(vector<string> &vecName);
private:
	static SimulateCMHPMotionControl	m_SimulatorCMHP;
	static ACSMotionControl				m_ACSCMHP;
	static GTNMotionControl				m_GTN;
};

#endif // _MCFACTORY_