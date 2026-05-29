#pragma once

#include "modules/cam/contracts/toolpath_export_dto.h"
#include "modules/process/settings/process_settings_schema.h"

namespace lcnc { class ProcessSettings; }

namespace lcnc::process {

/**
 * @brief Matches CAM layer/tool names to Process tool parameter snapshots.
 */
class ProcessToolMatcher
{
public:
    static ProcessToolSettings match(const lcnc::cam::ToolpathExportContour& contour,
                                     const lcnc::ProcessSettings& settings,
                                     QStringList* warnings = nullptr);
};

} // namespace lcnc::process