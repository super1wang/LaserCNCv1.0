#pragma once

#include "modules/cam/contracts/toolpath_export_dto.h"

#include <QVector>
#include <QReadWriteLock>
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
    /// Refresh only the catalog used for Process-side list/preflight display.
    lcnc::cam::ToolpathExportSnapshot refreshSnapshot();
    /// Refresh the exact execution snapshot already committed by CAM.  Process
    /// cannot supply an order or request a re-solve through this boundary.
    lcnc::cam::ToolpathExportSnapshot refreshCommittedExecutionSnapshot();
    lcnc::cam::ToolpathExportSnapshot currentSnapshot() const;
    ProcessJobPlan buildJobPlan() const;

private:
    std::shared_ptr<lcnc::cam::ICamToolpathProvider> m_provider;
    mutable QReadWriteLock m_snapshotLock;
    lcnc::cam::ToolpathExportSnapshot m_snapshot;
};

} // namespace lcnc::process
