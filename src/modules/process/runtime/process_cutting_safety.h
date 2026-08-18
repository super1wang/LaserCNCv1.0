#pragma once

#include "modules/cam/contracts/toolpath_export_dto.h"
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

/// Returns an operator-facing reason when the immutable CAM execution
/// snapshot is not safe to execute. Applies to single- and multi-contour jobs.
QString camExecutionBlockReason(const lcnc::cam::ToolpathExportSnapshot& snapshot);

} // namespace lcnc::process
