#include "modules/cam/toolpath/toolpath_solve_service.h"

#include "core/kinematics/machine_kinematics.h"

#include <QSet>

#include <algorithm>

namespace lcnc::cam {

bool ToolpathSolveService::solveTransactionally(
    std::vector<LaserContour>* contours,
    const QVector<std::uint64_t>& orderedContourIds,
    MachineKinematics* planningKinematics,
    const MachineModeDefinition& modeDefinition,
    const WorkpieceSetupTransform& workpieceSetup,
    const HeadToolGeometry& headToolGeometry,
    QString* errorMessage)
{
    if (!contours || !planningKinematics) {
        if (errorMessage)
            *errorMessage = QStringLiteral("CAM ordered solve input is unavailable");
        return false;
    }

    std::vector<LaserContour> solvedContours = *contours;
    for (LaserContour& contour : solvedContours) {
        for (ToolpathPoint& point : contour.points)
            point.machineCoord = {};
        if (contour.leadInSolution.valid)
            contour.leadInSolution.point.machineCoord = {};
    }

    QSet<std::uint64_t> seenIds;
    std::vector<LaserContour*> orderedContours;
    orderedContours.reserve(static_cast<std::size_t>(orderedContourIds.size()));
    for (const std::uint64_t id : orderedContourIds) {
        if (id == 0 || seenIds.contains(id))
            continue;
        const auto contour = std::find_if(
            solvedContours.begin(), solvedContours.end(),
            [id](const LaserContour& item) { return item.contourId == id; });
        if (contour == solvedContours.end()) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("Cutting order references missing contour %1")
                    .arg(id);
            }
            return false;
        }
        seenIds.insert(id);
        orderedContours.push_back(&*contour);
    }

    if (!orderedContours.empty()
        && !LaserToolpathBuilder::solveToolpathForOrder(
            orderedContours, planningKinematics, gp_Trsf(), modeDefinition,
            workpieceSetup, headToolGeometry, errorMessage)) {
        return false;
    }

    *contours = std::move(solvedContours);
    return true;
}

} // namespace lcnc::cam
