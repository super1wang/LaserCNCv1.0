#pragma once

#include "modules/process/toolpath/process_toolpath_service.h"

namespace lcnc::process {

class ProcessToolMatcher
{
public:
    static ProcessToolSettings match(const lcnc::cam::ToolpathExportContour& contour,
                                     QStringList* warnings = nullptr);
};

} // namespace lcnc::process