#pragma once
#include "modules/process/settings/process_settings.h"
namespace lcnc::process {
struct ProcessToolSettings { double laserEnergy{0}, laserFrequency{0}, laserPulseWidth{0}, feedRate{10.0}; QString laserDeviceName{"Simulator"}; };
struct ProcessTypedSettingsSnapshot { int schemaVersion{0}; ProcessToolSettings tool; };
struct ProcessDeviceSettings { QString motionControllerName; };
struct ProcessAuxSettings { ProcessMonitorSettings_Compat monitor; };
class ProcessSettingsSchema { public: static ProcessTypedSettingsSnapshot snapshotFrom(const ProcessSettings&) { return {}; } };
}
