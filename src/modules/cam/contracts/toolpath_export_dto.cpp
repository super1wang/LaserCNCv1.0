#include "modules/cam/contracts/toolpath_export_dto.h"

namespace lcnc::cam {

bool ToolpathExportSnapshot::hasEnabledContours() const
{
    for (const ToolpathExportContour& contour : contours) {
        if (contour.enabled && contour.layerEnabled && contour.pointCount > 0)
            return true;
    }
    return false;
}

int ToolpathExportSnapshot::totalPointCount() const
{
    int total = 0;
    for (const ToolpathExportContour& contour : contours)
        total += contour.pointCount;
    return total;
}

} // namespace lcnc::cam