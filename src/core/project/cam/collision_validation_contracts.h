#pragma once

#include "core/kinematics/machine_topology.h"

#include <QString>
#include <QVector>

#include <array>
#include <cstdint>

namespace lcnc::cam {

/// Result states are ordered by safety severity.  They intentionally contain
/// no OCC data so the exact same result can be consumed by CAM, Process and
/// the offline sandbox.
enum class CollisionValidationState : std::uint8_t
{
    Disabled = 0,
    Pending,
    Safe,
    Warning,
    Collision,
    Indeterminate
};

enum class CamMotionPhase : std::uint8_t
{
    Rapid,
    LeadIn,
    Cutting
};

enum class RapidSegmentPhase : std::uint8_t
{
    Retract = 0,
    Traverse,
    SafeXY,
    SafeAC,
    Approach
};

struct CollisionInterval
{
    int firstNode{-1};
    int lastNode{-1};
    CollisionValidationState state{CollisionValidationState::Pending};
    QString activeSource;
    QString passiveSource;
    QString activeEntity;
    QString passiveEntity;
    double minimumDistanceMm{-1.0};
    QString reason;
};

struct CollisionValidationSnapshot
{
    std::uint64_t key{0};
    CollisionValidationState state{CollisionValidationState::Pending};
    bool complete{false};
    bool blockWarning{true};
    QVector<CollisionValidationState> nodeStates;
    QVector<CollisionInterval> intervals;
    QString failureReason;

    bool blocksExecution(bool warningBlocksExecution = true) const
    {
        return state == CollisionValidationState::Collision
            || state == CollisionValidationState::Indeterminate
            || (warningBlocksExecution && state == CollisionValidationState::Warning)
            || state == CollisionValidationState::Pending;
    }
};

/// Final physical-coordinate motion.  It is the only representation accepted
/// by Process and simulation; no consumer may add tool heights or re-solve it.
struct CamMotionNode
{
    CamMotionPhase phase{CamMotionPhase::Cutting};
    /// Meaningful only when phase == Rapid.  It preserves CAM's strict
    /// retract/traverse/approach split for every read-only consumer.
    RapidSegmentPhase rapidPhase{RapidSegmentPhase::Traverse};
    std::uint64_t contourId{0};
    std::array<double, MachineAxisLayout::kMaxAxes> axes{};
    std::uint8_t axisMask{0};
    double tcpX{0.0};
    double tcpY{0.0};
    double tcpZ{0.0};
    double normalX{0.0};
    double normalY{0.0};
    double normalZ{1.0};
    double estimatedTimeMs{0.0};
};

struct CamMotionPlanSnapshot
{
    std::uint64_t revision{0};
    QVector<CamMotionNode> nodes;
    CollisionValidationSnapshot collision;
};

} // namespace lcnc::cam
