#pragma once

#include "core/algorithms/cam/laser_toolpath.h"
#include "core/kinematics/machine_configuration_service.h"

#include <QVector>

#include <cstdint>
#include <vector>

class MachineKinematics;
class QString;

namespace lcnc::cam {

/// Executes the ordered machine-coordinate solve against a private contour
/// copy and commits traversal/coordinate mutations only after full success.
class ToolpathSolveService final
{
public:
    static bool solveTransactionally(
        std::vector<LaserContour>* contours,
        const QVector<std::uint64_t>& orderedContourIds,
        MachineKinematics* planningKinematics,
        const MachineModeDefinition& modeDefinition,
        const WorkpieceSetupTransform& workpieceSetup,
        const HeadToolGeometry& headToolGeometry,
        QString* errorMessage);
};

} // namespace lcnc::cam
