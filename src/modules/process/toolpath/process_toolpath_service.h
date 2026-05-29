#pragma once

#include "modules/cam/contracts/toolpath_export_dto.h"
#include "modules/process/settings/process_settings_schema.h"

#include <QVector>
#include <memory>

namespace lcnc::cam { class ICamToolpathProvider; }
namespace lcnc { class ProcessSettings; }

namespace lcnc::process {

/**
 * @brief One contour selected for a Process job plan.
 */
struct ProcessJobContour
{
    lcnc::cam::ToolpathExportContour contour;
    QVector<lcnc::cam::ToolpathExportPoint> points;
    ProcessToolSettings toolSettings;
    QStringList warnings;
};

/**
 * @brief Controller-neutral toolpath execution plan before command translation.
 */
struct ProcessJobPlan
{
    std::uint64_t revision{0};
    QVector<ProcessJobContour> contours;
    QStringList warnings;
    bool valid{false};
    int totalPointCount{0};
};

/**
 * @brief Imports CAM snapshots, filters enabled contours and builds Process job plans.
 */
class ProcessToolpathService
{
public:
    explicit ProcessToolpathService(std::shared_ptr<lcnc::cam::ICamToolpathProvider> provider = {});

    void setProvider(std::shared_ptr<lcnc::cam::ICamToolpathProvider> provider);
    lcnc::cam::ToolpathExportSnapshot refreshSnapshot();
    const lcnc::cam::ToolpathExportSnapshot& currentSnapshot() const { return m_snapshot; }
    ProcessJobPlan buildJobPlan(const lcnc::ProcessSettings& settings) const;

private:
    std::shared_ptr<lcnc::cam::ICamToolpathProvider> m_provider;
    lcnc::cam::ToolpathExportSnapshot m_snapshot;
};

} // namespace lcnc::process