#include "ld_factory.h"

using std::string;
using std::vector;

LDFactory::LDFactory(lcnc::process::ProcessSettingsService& settings)
    : m_simulator(settings)
#if defined(LCNC_PROCESS_HAS_REAL_LASER) && LCNC_PROCESS_HAS_REAL_LASER
    , m_ipg(settings)
    , m_pharos(settings)
    , m_raycus(settings)
    , m_raycusAirCool(settings)
    , m_ultron(settings)
    , m_raycusQCW(settings)
#endif
    , m_analog(settings)
{
}


LaserDevice * LDFactory::laserDevice(const std::string & sLaserDeviceName)
{
	if (sLaserDeviceName.compare("Simulator") == 0)
		return &m_simulator;
	#if defined(LCNC_PROCESS_HAS_REAL_LASER) && LCNC_PROCESS_HAS_REAL_LASER
	else if (sLaserDeviceName.compare("IPG") == 0)
		return &m_ipg;
	else if (sLaserDeviceName.compare("Pharos") == 0)
		return &m_pharos;
	else if (sLaserDeviceName.compare("Raycus") == 0)
		return &m_raycus;
	else if (sLaserDeviceName.compare("RaycusAirCool") == 0)
		return &m_raycusAirCool;
	else if (sLaserDeviceName.compare("ULTRON") == 0)
		return &m_ultron;
	else if (sLaserDeviceName.compare("RaycusQCW") == 0)
		return &m_raycusQCW;
	#endif
	else if (sLaserDeviceName.compare("AnalogControl") == 0)
		return &m_analog;
	return nullptr;
}

void LDFactory::GetAll_LDName(vector<string> &vecName)
{
	vector<string> vec;
	vec.push_back(m_simulator.GetName());
	#if defined(LCNC_PROCESS_HAS_REAL_LASER) && LCNC_PROCESS_HAS_REAL_LASER
	vec.push_back(m_ipg.GetName());
	vec.push_back(m_pharos.GetName());
	vec.push_back(m_raycus.GetName());
	vec.push_back(m_raycusAirCool.GetName());
	vec.push_back(m_raycusQCW.GetName());
	#endif
	vec.push_back(m_analog.GetName());
	vecName.swap(vec);
}
