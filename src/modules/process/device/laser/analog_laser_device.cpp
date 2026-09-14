/************************************************************************/
/*                            Analog激光器实现类                           */
/************************************************************************/

#include "analog_laser_device.h"
#include <string>
#include <stdlib.h>
//#include <cstringt.h>
#include <sstream>
#include <fstream>
using std::ostringstream;
using namespace std;
using toml::table;

AnalogLaserDevice::AnalogLaserDevice(lcnc::process::ProcessSettingsService& settings) : LaserDevice(settings), m_strName("AnalogControl"), m_bIsInited(false), m_dMaxCurrent(0),
                                   m_dSimmerCurrent(0), m_iWaveShape(0), m_strTemperature("")
{
}

ErrorCode AnalogLaserDevice::setLaserTable(const table&)
{
	ErrorCode eCode = ErrorCode::ERROR_NONE;
	return eCode;
}

const string & AnalogLaserDevice::GetName()
{
	return m_strName;
}
