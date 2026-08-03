#pragma once

#include "modules/process/runtime/device_command_queue.h"

#include <QStringList>

namespace lcnc::process {

struct ContourBoundaryHealth {
    bool connected{false};
    bool statusReadable{false};
    int faultCode{0};
    QStringList missingAxes;
    QStringList disabledAxes;
};

DeviceCommandResult evaluateContourBoundaryHealth(const ContourBoundaryHealth& health);

} // namespace lcnc::process
