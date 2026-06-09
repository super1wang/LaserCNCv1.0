#pragma once

#include "modules/cam/contracts/toolpath_export_dto.h"

namespace lcnc::process {

struct ProcessToolSettings {
    double laserEnergy{0}, laserFrequency{0}, laserPulseWidth{0}, feedRate{10.0};
    QString laserDeviceName{"Simulator"};
};

class ProcessToolMatcher
{
public:
    static ProcessToolSettings match(const lcnc::cam::ToolpathExportContour& contour,
                                     QStringList* warnings = nullptr);
};

} // namespace lcnc::process
