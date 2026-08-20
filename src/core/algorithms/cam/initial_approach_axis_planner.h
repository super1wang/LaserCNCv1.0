#pragma once

#include "core/kinematics/machine_topology.h"
#include "core/project/cam/collision_validation_contracts.h"

#include <QString>
#include <QVector>

#include <cmath>
#include <cstdint>

namespace lcnc::cam_algo {

struct InitialApproachAxisWaypoint
{
    lcnc::SolvedMachinePose pose;
    lcnc::cam::RapidSegmentPhase phase{lcnc::cam::RapidSegmentPhase::Traverse};
};

struct InitialApproachAxisPlan
{
    QVector<InitialApproachAxisWaypoint> waypoints;
    double resolvedSafetyAxisZ{0.0};
    QString failureReason;

    bool isValid() const { return failureReason.isEmpty() && !waypoints.isEmpty(); }
};

enum class InitialApproachAxisMode : std::uint8_t
{
    Manual = 0,
    AutomaticSafeZone
};

inline bool shouldValidateInitialApproachMachine(
    bool machineModelLoaded,
    InitialApproachAxisMode mode,
    bool manualCollisionCheckEnabled)
{
    if (!machineModelLoaded)
        return false;
    return mode == InitialApproachAxisMode::AutomaticSafeZone
        || manualCollisionCheckEnabled;
}

/// Higher automatic safety-Z candidates extend the same physical retract ray.
/// A blocked retract prefix is therefore terminal, while a later SafeXY,
/// SafeAC or Approach collision may still be avoided by a higher candidate.
inline bool shouldRetryAutomaticSafetyZCandidate(
    lcnc::cam::CollisionValidationState state,
    bool hasBlockedPhase,
    lcnc::cam::RapidSegmentPhase firstBlockedPhase)
{
    if (state == lcnc::cam::CollisionValidationState::Indeterminate)
        return false;
    return !hasBlockedPhase
        || firstBlockedPhase != lcnc::cam::RapidSegmentPhase::Retract;
}

struct AutomaticSafetyZCandidates
{
    QVector<double> axisCoordinates;
    QString failureReason;

    bool isValid() const
    {
        return failureReason.isEmpty() && !axisCoordinates.isEmpty();
    }
};

/// Builds controller absolute-Z candidates ordered from the nearest physical
/// safe height toward the upper machine limit. Axis reversal changes only the
/// physical-height comparison; returned values remain signed controller
/// coordinates and are suitable for MoveAbsolute.
inline AutomaticSafetyZCandidates planAutomaticSafetyZCandidates(
    double sourceAxisZ,
    double targetAxisZ,
    double zAxisWorldZComponent,
    double minimumAxisZ,
    double maximumAxisZ,
    double requiredLiftMm,
    double candidateStepMm,
    int maximumCandidates = 128)
{
    AutomaticSafetyZCandidates result;
    if (!std::isfinite(sourceAxisZ) || !std::isfinite(targetAxisZ)
        || !std::isfinite(zAxisWorldZComponent)
        || std::abs(zAxisWorldZComponent) <= 1e-9
        || !std::isfinite(minimumAxisZ) || !std::isfinite(maximumAxisZ)
        || minimumAxisZ > maximumAxisZ
        || !std::isfinite(requiredLiftMm) || requiredLiftMm < 0.0
        || !std::isfinite(candidateStepMm) || candidateStepMm <= 0.0
        || maximumCandidates <= 0) {
        result.failureReason = QStringLiteral(
            "Automatic initial approach Z search input is invalid");
        return result;
    }
    const double sourceHeight = sourceAxisZ * zAxisWorldZComponent;
    const double targetHeight = targetAxisZ * zAxisWorldZComponent;
    const double requiredHeight = std::max(sourceHeight,
                                            targetHeight + requiredLiftMm);
    const double upperAxisZ = zAxisWorldZComponent > 0.0
        ? maximumAxisZ : minimumAxisZ;
    const double upperHeight = upperAxisZ * zAxisWorldZComponent;
    if (requiredHeight > upperHeight + 1e-9) {
        result.failureReason = QStringLiteral(
            "Automatic initial approach cannot find a safe Z height within machine limits");
        return result;
    }
    for (double height = requiredHeight;
         height <= upperHeight + 1e-9
             && result.axisCoordinates.size() < maximumCandidates;
         height += candidateStepMm) {
        result.axisCoordinates.append(std::clamp(
            height / zAxisWorldZComponent, minimumAxisZ, maximumAxisZ));
    }
    if (result.axisCoordinates.isEmpty()
        || std::abs(result.axisCoordinates.constLast() - upperAxisZ) > 1e-9) {
        result.axisCoordinates.append(upperAxisZ);
    }
    return result;
}

/// Builds an axis-space first-motion contract.  safetyAxisZ is already an
/// absolute controller coordinate and is never sign-normalized or converted.
/// zAxisWorldZComponent is used only to compare physical height when the
/// configured controller Z direction is reversed.
///
/// Manual:    Z safety coordinate -> all remaining axes -> cutting pose.
/// Automatic: Z safety region -> SafeXY -> SafeAC -> Z approach.
/// CAM may densify/validate these waypoints, but Process must execute the
/// resulting solved axes unchanged.
/// 中文翻译：安全 Z 是控制器绝对坐标，禁止取绝对值或按方向二次换算；轴方向
/// 只用于物理高低比较。自动模式严格按 Z 安全退回、SafeXY、SafeAC、Z 接近执行。
inline InitialApproachAxisPlan planInitialApproachAxes(
    const lcnc::MachineAxisLayout& layout,
    const lcnc::SolvedMachinePose& source,
    const lcnc::SolvedMachinePose& target,
    double safetyAxisZ,
    double zAxisWorldZComponent,
    InitialApproachAxisMode mode)
{
    InitialApproachAxisPlan result;
    if (layout.count == 0 || layout.count > lcnc::MachineAxisLayout::kMaxAxes
        || !source.valid || !target.valid
        || !std::isfinite(safetyAxisZ)
        || !std::isfinite(zAxisWorldZComponent)
        || std::abs(zAxisWorldZComponent) <= 1e-9) {
        result.failureReason = QStringLiteral("Initial approach axis input is invalid");
        return result;
    }

    int zIndex = -1;
    for (int index = 0; index < layout.count; ++index) {
        if (layout.axes[index].name.trimmed().isEmpty()) {
            result.failureReason = QStringLiteral("Initial approach axis layout is invalid");
            return result;
        }
        if (layout.axes[index].role == lcnc::MachineAxisRole::LinearZ) {
            zIndex = index;
            break;
        }
    }
    if (zIndex < 0
        || (source.activeMask & static_cast<std::uint8_t>(1u << zIndex)) == 0
        || (target.activeMask & static_cast<std::uint8_t>(1u << zIndex)) == 0) {
        result.failureReason = QStringLiteral("Initial approach requires an active linear Z axis");
        return result;
    }
    for (int index = 0; index < layout.count; ++index) {
        if (!std::isfinite(source.values[index])
            || !std::isfinite(target.values[index])) {
            result.failureReason = QStringLiteral("Initial approach contains a non-finite axis value");
            return result;
        }
    }

    constexpr double kTolerance = 1e-9;
    const double sourceZ = source.values[zIndex];
    const double targetZ = target.values[zIndex];
    const auto physicalHeight = [zAxisWorldZComponent](double axisZ) {
        return axisZ * zAxisWorldZComponent;
    };
    result.resolvedSafetyAxisZ = safetyAxisZ;
    const double traverseZ = mode == InitialApproachAxisMode::AutomaticSafeZone
        && physicalHeight(sourceZ) >= physicalHeight(safetyAxisZ) - kTolerance
        ? sourceZ : safetyAxisZ;
    if (physicalHeight(traverseZ) < physicalHeight(targetZ) - kTolerance) {
        result.failureReason = QStringLiteral(
            "Initial approach safety Z is below the first cutting height");
        return result;
    }

    const auto samePose = [&layout](const lcnc::SolvedMachinePose& lhs,
                                    const lcnc::SolvedMachinePose& rhs) {
        for (int index = 0; index < layout.count; ++index) {
            if (std::abs(lhs.values[index] - rhs.values[index]) > 1e-9)
                return false;
        }
        return true;
    };
    lcnc::SolvedMachinePose previous = source;
    lcnc::SolvedMachinePose retract = source;
    retract.values[zIndex] = traverseZ;
    if (!samePose(previous, retract)) {
        result.waypoints.append({retract, lcnc::cam::RapidSegmentPhase::Retract});
        previous = retract;
    }

    if (mode == InitialApproachAxisMode::Manual) {
        lcnc::SolvedMachinePose traverse = target;
        traverse.values[zIndex] = traverseZ;
        if (!samePose(previous, traverse)) {
            result.waypoints.append({traverse, lcnc::cam::RapidSegmentPhase::Traverse});
            previous = traverse;
        }
    } else {
        // SafeXY changes only the two linear planar axes.  Rotary axes retain
        // their measured position while the head stays in the safe-Z region.
        lcnc::SolvedMachinePose safeXy = previous;
        for (int index = 0; index < layout.count; ++index) {
            const auto role = layout.axes[index].role;
            if (role == lcnc::MachineAxisRole::LinearX
                || role == lcnc::MachineAxisRole::LinearY) {
                safeXy.values[index] = target.values[index];
            }
        }
        if (!samePose(previous, safeXy)) {
            result.waypoints.append({safeXy, lcnc::cam::RapidSegmentPhase::SafeXY});
            previous = safeXy;
        }

        // SafeAC then adopts the committed rotary branch without changing XYZ.
        lcnc::SolvedMachinePose safeAc = previous;
        for (int index = 0; index < layout.count; ++index) {
            const auto role = layout.axes[index].role;
            if (role != lcnc::MachineAxisRole::LinearX
                && role != lcnc::MachineAxisRole::LinearY
                && role != lcnc::MachineAxisRole::LinearZ) {
                safeAc.values[index] = target.values[index];
            }
        }
        if (!samePose(previous, safeAc)) {
            result.waypoints.append({safeAc, lcnc::cam::RapidSegmentPhase::SafeAC});
            previous = safeAc;
        }
    }

    // Retain an explicit final waypoint even when it is already reached.  A
    // non-empty transition is still required to establish the committed lead-
    // in boundary before the cutting program starts.
    // 中文翻译：即使已处于首切割位，仍保留显式终点来确立已提交的下刀边界。
    result.waypoints.append({target, lcnc::cam::RapidSegmentPhase::Approach});
    return result;
}

} // namespace lcnc::cam_algo
