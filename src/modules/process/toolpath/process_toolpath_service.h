#pragma once

#include "modules/cam/contracts/toolpath_export_dto.h"

#include <QVector>
#include <memory>

namespace lcnc::cam { class ICamToolpathProvider; }

namespace lcnc::process {

struct ProcessToolSettings {
    double laserEnergy{0}, laserFrequency{0}, laserPulseWidth{0}, feedRate{10.0};
    QString laserDeviceName{"Simulator"};
};

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
    lcnc::cam::ToolpathExportSnapshot refreshSnapshotForOrder(
        const QVector<std::uint64_t>& orderedContourIds);
    const lcnc::cam::ToolpathExportSnapshot& currentSnapshot() const { return m_snapshot; }
    ProcessJobPlan buildJobPlan() const;

private:
    std::shared_ptr<lcnc::cam::ICamToolpathProvider> m_provider;
    lcnc::cam::ToolpathExportSnapshot m_snapshot;
};

} // namespace lcnc::process
