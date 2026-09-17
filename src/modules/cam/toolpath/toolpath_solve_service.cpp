#include "modules/cam/toolpath/toolpath_solve_service.h"

#include "core/kinematics/machine_kinematics.h"
#include "modules/cam/toolpath/toolpath_generation_service.h"

#include <QSet>

#include <algorithm>

namespace lcnc::cam {

bool ToolpathSolveService::geometryHasCurrentSolve(const std::vector<LaserContour>& contours)
{
    return !contours.empty() && std::all_of(contours.begin(), contours.end(), [](const LaserContour& contour) {
        if (!contour.enabled) return true;
        return contour.geometrySamplingComplete && contour.geometrySamplingEvidence.complete()
            && !contour.needsRecalculation
            && !contour.points.empty()
            && std::all_of(contour.points.begin(), contour.points.end(), [](const ToolpathPoint& point) {
                return point.machineCoord.valid && point.machineCoord.solvedPose.valid;
            }) && (!contour.leadInSolution.valid
                || (contour.leadInSolution.point.machineCoord.valid
                    && contour.leadInSolution.point.machineCoord.solvedPose.valid));
    });
}

bool ToolpathSolveService::solveFrozen(std::vector<LaserContour>* contours,
    const QVector<std::uint64_t>& orderedContourIds,
    const MotionCompilationInput& input, QString* errorMessage)
{
    if (input.capturedContextHash != motionCompilationContextHash(input.context)) {
        if (errorMessage) *errorMessage = QStringLiteral("Frozen motion input identity is invalid");
        return false;
    }
    MachineKinematics planning;
    configureFrozenMachine(&planning, input);
    return solveTransactionally(contours, orderedContourIds, &planning,
        input.modeDefinition, input.workpieceSetup, input.headToolGeometry, errorMessage);
}

void ToolpathSolveService::configureFrozenMachine(MachineKinematics* machine,
    const MotionCompilationInput& input)
{
    machine->setAxes(input.machineAxes, input.machineConfigType);
    machine->setWorkpieceSetupTransform(input.kinematicSetup);
    for (auto it = input.workpieceMounts.cbegin(); it != input.workpieceMounts.cend(); ++it)
        machine->mountWorkpiece(it.key(), it.value());
    for (auto it = input.shapeAssignments.cbegin(); it != input.shapeAssignments.cend(); ++it)
        machine->assignShape(it.key(), it.value());
}

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
