/************************************************************************/
/*                            Analog激光器实现类                           */
/************************************************************************/

#include "AnalogLaserDevice.h"
#include <string>
#include <stdlib.h>
//#include <cstringt.h>
#include <sstream>
#include <fstream>
using std::ostringstream;
using namespace std;

AnalogLaserDevice::AnalogLaserDevice(lcnc::process::ProcessSettingsService& settings) : LaserDevice(settings), m_strName("AnalogControl"), m_bIsInited(false), m_dMaxCurrent(0),
                                   m_dSimmerCurrent(0), m_iWaveShape(0), m_strTemperature("")
{
}

ErrorCode AnalogLaserDevice::SetLaserTable(const table& tableLaser)
{
	bool bConnectChange = false;

	ErrorCode eCode = ErrorCode::ERROR_NONE;
	return eCode;
}

const string & AnalogLaserDevice::GetName()
{
	return m_strName;
}
