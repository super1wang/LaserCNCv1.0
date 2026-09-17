#pragma once

#include "core/algorithms/cam/laser_toolpath.h"
#include "core/algorithms/cam/continuous_motion_evaluator.h"
#include "core/algorithms/cam/dof_reduction.h"
#include "core/kinematics/machine_configuration_service.h"

#include <QVector>

#include <cstdint>
#include <vector>

class MachineKinematics;
class QString;

namespace lcnc::cam {
struct MotionCompilationInput;
struct ToolpathExportSnapshot;

/// Executes the ordered machine-coordinate solve against a private contour
/// copy and commits traversal/coordinate mutations only after full success.
class ToolpathSolveService final
{
public:
    static std::uint64_t optimizationInvocationCount();
    static bool optimizeMotionSnapshot(ToolpathExportSnapshot* snapshot,
        const MotionCompilationInput& input, QString* error,
        const std::function<bool()>& cancelled = {});
    static void configureFrozenMachine(MachineKinematics* machine, const MotionCompilationInput& input);
    static lcnc::cam_algo::MotionEvaluationContext physicalEvaluationContext(
        const MotionCompilationInput& input, const QString& workpieceEntry);
    static lcnc::cam_algo::ReductionEvaluation reductionEvaluationContext(
        const MotionCompilationInput& input, const QString& workpieceEntry);
    static bool geometryHasCurrentSolve(const std::vector<LaserContour>& contours);
    static bool solveFrozen(std::vector<LaserContour>* contours,
        const QVector<std::uint64_t>& orderedContourIds,
        const MotionCompilationInput& input, QString* errorMessage);
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
