#include "modules/cam/toolpath/toolpath_solve_service.h"

#include "core/kinematics/machine_kinematics.h"
#include "core/algorithms/kinematics/rtcp_reference_transform.h"
#include "modules/cam/toolpath/toolpath_generation_service.h"

#include <QSet>

#include <algorithm>
#include <cmath>
#include <gp_Ax1.hxx>
#include <gp_Vec.hxx>

namespace lcnc::cam {

lcnc::cam_algo::ReductionEvaluation ToolpathSolveService::reductionEvaluationContext(
    const MotionCompilationInput& input, const QString& workpieceEntry)
{
    lcnc::cam_algo::ReductionEvaluation result;
    result.motion = physicalEvaluationContext(input, workpieceEntry);
    if (!result.motion.evaluatePhysicalAxes) return result;
    // Exact translation bound for the existing planar/table FK families.
    // Head kinematics and scaled placements are deliberately not inferred.
    if (input.modeDefinition.mode != MachiningMode::Planar3Axis
        && input.modeDefinition.mode != MachiningMode::SimultaneousTable5Axis) return result;
    if (std::abs(input.kinematicSetup.ScaleFactor()) != 1.0) return result;
    MachineKinematics machine;
    configureFrozenMachine(&machine, input);
    int z = -1;
    QString zName;
    for (int i = 0; i < input.modeDefinition.interpolatedAxes.count; ++i)
        if (input.modeDefinition.interpolatedAxes.axes[i].role == MachineAxisRole::LinearZ) {
            z = i; zName = input.modeDefinition.interpolatedAxes.axes[i].name;
        }
    if (z < 0) return result;
    const auto* axis = machine.findAxis(zName);
    if (!axis || axis->motionType != MachineAxisDef::Linear) return result;
    // Workpiece/reference rotation must not acquire a Z-dependent translation.
    const QString mount = machine.mountedAxis(workpieceEntry);
    if (!mount.isEmpty() && machine.axisChain(mount).contains(zName)) return result;
    for (const auto& item : machine.axes())
        if (item.motionType == MachineAxisDef::Rotary && machine.axisChain(item.name).contains(zName)) return result;
    result.boundNumericalZ = [z](const CamMotionBlock& original, const CamMotionBlock& candidate, int changedAxis) {
        lcnc::cam_algo::NumericalZBound bound;
        if (changedAxis != z || original.physicalKnots.size() != candidate.physicalKnots.size()
            || original.hasEntryBoundary != candidate.hasEntryBoundary) return bound;
        const auto inspect = [&](const CamMotionNode& a, const CamMotionNode& b) {
            if (a.axisMask != b.axisMask) return false;
            for (int slot = 0; slot < MachineAxisLayout::kMaxAxes; ++slot)
                if (slot != z && a.axes[slot] != b.axes[slot]) return false;
            bound.maximumPositionDeviationMm = std::max(bound.maximumPositionDeviationMm,
                std::abs(a.axes[z] - b.axes[z]));
            return true;
        };
        if (original.hasEntryBoundary && !inspect(original.entryBoundary, candidate.entryBoundary)) return bound;
        for (int i = 0; i < original.physicalKnots.size(); ++i)
            if (!inspect(original.physicalKnots[i], candidate.physicalKnots[i])) return bound;
        // Affine physical Z delta is bounded by its endpoints; rigid FK rotates
        // this translation without changing its norm or beam orientation.
        bound.valid = true;
        return bound;
    };
    return result; // Controller/process admission remains unavailable.
}

lcnc::cam_algo::MotionEvaluationContext ToolpathSolveService::physicalEvaluationContext(
    const MotionCompilationInput& input, const QString& workpieceEntry)
{
    lcnc::cam_algo::MotionEvaluationContext model;
    if (input.capturedContextHash != motionCompilationContextHash(input.context)) return model;
    model.interpolationModelVersion = input.context.interpolationModelVersion;
    // S2 production capability: exact physical FK only. No qualified continuous
    // chord bound or process-tolerance authority is provided by this adapter.
    // Full must report its Conservative subset; test bounds are not qualification.
    // Detached values only. Each call owns its private kinematic model, so the
    // callback is usable in a worker without sharing a mutable QObject/pose.
    model.evaluatePhysicalAxes = [input, workpieceEntry](const auto& positions, std::uint8_t mask,
                                                       lcnc::cam_algo::EvaluatedMotionState* state) {
        MachineKinematics machine;
        configureFrozenMachine(&machine, input);
        for (auto it = input.modeDefinition.lockedAxisTargets.cbegin(); it != input.modeDefinition.lockedAxisTargets.cend(); ++it) {
            auto* axis = machine.findAxis(it.key());
            if (!axis || !std::isfinite(it.value()) || it.value() < axis->minVal || it.value() > axis->maxVal) return false;
            axis->currentPos = it.value();
        }
        const auto& layout = input.modeDefinition.interpolatedAxes;
        if (!layout.isValid() || mask != ((1u << layout.count) - 1u)) return false;
        for (int slot = 0; slot < layout.count; ++slot) {
            if (!(mask & (1u << slot))) continue;
            auto* axis = machine.findAxis(layout.axes[slot].name);
            if (!axis || !std::isfinite(positions[slot]) || positions[slot] < axis->minVal
                || positions[slot] > axis->maxVal) return false;
            axis->currentPos = positions[slot]; // No clamping or equivalent-angle rescue.
        }
        const MachineAxisDef* primary = nullptr;
        const MachineAxisDef* secondary = nullptr;
        const MachineAxisDef* toolCarrier = nullptr;
        for (const auto& axis : machine.axes()) {
            if (axis.role == MachineAxisRole::HeadTiltPrimary) primary = &axis;
            if (axis.role == MachineAxisRole::HeadTiltSecondary) secondary = &axis;
            if (axis.role == MachineAxisRole::LinearZ) toolCarrier = &axis;
        }
        if (input.modeDefinition.mode != MachiningMode::SimultaneousHead5Axis) {
            primary = nullptr;
            secondary = nullptr;
        }
        gp_Pnt tcp = machine.currentLinearPosition();
        gp_Vec normal(0, 0, 1);
        if (input.modeDefinition.mode == MachiningMode::Planar3Axis || primary || secondary) {
            gp_Vec linear(0, 0, 0);
            QVector<gp_Dir> directions;
            for (const auto& axis : machine.axes()) {
                if (axis.role != MachineAxisRole::LinearX && axis.role != MachineAxisRole::LinearY
                    && axis.role != MachineAxisRole::LinearZ) continue;
                for (const auto& direction : directions)
                    if (std::abs(direction.Dot(axis.direction)) > 1e-12) return false;
                directions.append(axis.direction);
                linear += gp_Vec(axis.direction) * axis.currentPos;
            }
            if (directions.size() != 3) return false;
            tcp = gp_Pnt(linear.X(), linear.Y(), linear.Z());
        } else {
            if (!toolCarrier || machine.axisChain(toolCarrier->name).isEmpty()) return false;
            tcp = gp_Pnt(0, 0, 0).Transformed(machine.computeAxisTransform(toolCarrier->name));
        }
        if (primary || secondary) {
            if (!primary || !secondary || input.headToolGeometry.focusLength <= 0) return false;
            constexpr double radians = 3.14159265358979323846 / 180.0;
            gp_Trsf first, second;
            first.SetRotation(gp_Ax1(primary->origin, primary->direction), primary->currentPos * radians);
            second.SetRotation(gp_Ax1(secondary->origin, secondary->direction), secondary->currentPos * radians);
            const gp_Trsf rotation = secondary->parentAxis == primary->name
                ? first.Multiplied(second) : second.Multiplied(first);
            gp_Vec beam(input.headToolGeometry.zeroBeamX, input.headToolGeometry.zeroBeamY,
                        input.headToolGeometry.zeroBeamZ);
            if (beam.Magnitude() <= 1e-10) return false;
            beam.Normalize();
            gp_Vec offset = beam * input.headToolGeometry.focusLength
                + gp_Vec(input.headToolGeometry.installationOffsetX, input.headToolGeometry.installationOffsetY,
                         input.headToolGeometry.installationOffsetZ);
            offset.Transform(rotation);
            tcp.Translate(offset);
            normal = -beam;
            normal.Transform(rotation);
        } else if (toolCarrier && input.modeDefinition.mode != MachiningMode::Planar3Axis) {
            normal.Transform(machine.computeAxisTransform(toolCarrier->name));
        } else if (!toolCarrier) {
            return false;
        }
        normal.Normalize();
        state->worldTcpX = tcp.X(); state->worldTcpY = tcp.Y(); state->worldTcpZ = tcp.Z();
        state->processDirectionX = normal.X(); state->processDirectionY = normal.Y(); state->processDirectionZ = normal.Z();
        gp_Pnt reference;
        state->referenceTcpValid = lcnc::kinematics::tableRtcpReferencePoint(tcp,
            machine.computeWpcTransform(workpieceEntry), machine.computeWpcTransformHome(workpieceEntry), &reference);
        if (state->referenceTcpValid) {
            state->referenceTcpX = reference.X(); state->referenceTcpY = reference.Y(); state->referenceTcpZ = reference.Z();
        }
        return true;
    };
    return model;
}

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
