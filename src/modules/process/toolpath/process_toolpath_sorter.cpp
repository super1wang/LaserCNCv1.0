#include "modules/process/toolpath/process_toolpath_sorter.h"

#include <algorithm>

namespace lcnc::process {

QList<std::uint64_t> ProcessToolpathSorter::sortedContourIds(
    const lcnc::cam::ToolpathExportSnapshot& snapshot,
    ProcessToolpathSortStrategy strategy)
{
    QVector<lcnc::cam::ToolpathExportContour> contours = snapshot.contours;
    switch (strategy) {
    case ProcessToolpathSortStrategy::CamOrder:
        break;
    case ProcessToolpathSortStrategy::LayerThenContour:
        std::stable_sort(contours.begin(), contours.end(), [](const auto& left, const auto& right) {
            if (left.layerId != right.layerId)
                return left.layerId < right.layerId;
            return left.contourId < right.contourId;
        });
        break;
    case ProcessToolpathSortStrategy::ToolThenLayer:
        std::stable_sort(contours.begin(), contours.end(), [](const auto& left, const auto& right) {
            const int toolCompare = QString::localeAwareCompare(left.toolName, right.toolName);
            if (toolCompare != 0)
                return toolCompare < 0;
            if (left.layerId != right.layerId)
                return left.layerId < right.layerId;
            return left.contourId < right.contourId;
        });
        break;
    }

    QList<std::uint64_t> ids;
    for (const auto& contour : contours) {
        if (contour.enabled && contour.layerEnabled)
            ids.append(contour.contourId);
    }
    return ids;
}

} // namespace lcnc::process