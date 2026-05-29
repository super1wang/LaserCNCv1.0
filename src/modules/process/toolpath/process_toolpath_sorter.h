#pragma once

#include "modules/cam/contracts/toolpath_export_dto.h"

#include <QList>

namespace lcnc::process {

/**
 * @brief Sort strategies for Process execution contours.
 */
enum class ProcessToolpathSortStrategy
{
    CamOrder,
    LayerThenContour,
    ToolThenLayer
};

/**
 * @brief Builds stable contour execution orders from CAM snapshots.
 */
class ProcessToolpathSorter
{
public:
    static QList<std::uint64_t> sortedContourIds(const lcnc::cam::ToolpathExportSnapshot& snapshot,
                                                ProcessToolpathSortStrategy strategy);
};

} // namespace lcnc::process