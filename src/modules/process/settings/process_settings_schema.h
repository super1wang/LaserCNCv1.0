#pragma once
#include "modules/process/settings/process_settings.h"
#include "modules/process/toolpath/process_toolpath_service.h"

namespace lcnc::process {
struct ProcessTypedSettingsSnapshot { int schemaVersion{0}; ProcessToolSettings tool; };
struct ProcessDeviceSettings { QString motionControllerName; };
struct ProcessAuxSettings { ProcessMonitorSettings_Compat monitor; };
class ProcessSettingsSchema { public: static ProcessTypedSettingsSnapshot snapshotFrom(const ProcessSettings&) { return {}; } };
}
