#pragma once

#include "core/kinematics/machine_topology.h"

#include <QHash>
#include <QString>
#include <QVector>

#include <array>
#include <cstdint>

namespace lcnc::cam {

// This contract deliberately contains no OCC or Process types.  CAM owns the
// geometry planning; Process only consumes the immutable, solved rapid plan.
enum class TravelPlanningMode : std::uint8_t
{
    None = 0,
    WorkpieceProxy,
    FullEnvironment
};

enum class RapidSynchronization : std::uint8_t
{
    Sequential = 0,
    Coordinated
};

/// The geometric intent of a rapid transition.  A surface-guided transition
/// is a sampled tool-centre trajectory outside the workpiece; it is not a
/// legacy Z-up/XY/Z-down jump.
enum class RapidPathKind : std::uint8_t
{
    None = 0,
    SurfaceOffset
};

struct RapidPose
{
    /// Process-facing semantic order: X, Y, Z, R1, R2.  This order is
    /// deliberately independent from MachineAxisLayout, whose physical order
    /// may for example be Y, X, Z, A, C.
    std::array<double, MachineAxisLayout::kMaxAxes> axes{};
    std::uint8_t activeMask{0};
    QString rotaryAxis1Name;
    QString rotaryAxis2Name;
    /// CAM-only physical-layout pose.  It is retained for kinematic collision
    /// checks and must never be consumed by a Process motion sink.
    std::array<double, MachineAxisLayout::kMaxAxes> kinematicAxes{};
    std::uint8_t kinematicAxisMask{0};
    double tcpX{0.0};
    double tcpY{0.0};
    double tcpZ{0.0};
    double surfaceNormalX{0.0};
    double surfaceNormalY{0.0};
    double surfaceNormalZ{1.0};
};

struct RapidMoveSegment
{
    RapidPose target;
    std::uint8_t movingAxisMask{0};
    RapidSynchronization synchronization{RapidSynchronization::Sequential};
    double estimatedTimeMs{0.0};
    double minimumClearanceMm{0.0};
};

/// Unsolved geometric surface path used only by the 3D preview.  It contains
/// no machine axes and is never consumed by Process.  The renderer may apply
/// a fixed illustrative normal offset without changing executable motion.
struct RapidSurfacePreviewPoint
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double normalX{0.0};
    double normalY{0.0};
    double normalZ{1.0};
};

struct RapidTransition
{
    std::uint64_t fromContourId{0};
    std::uint64_t toContourId{0};
    QVector<RapidMoveSegment> segments;
    QVector<RapidSurfacePreviewPoint> surfacePreviewPoints;
    double estimatedTimeMs{0.0};
    double pathLengthMm{0.0};
    double minimumClearanceMm{0.0};
    double maximumSurfaceOffsetMm{0.0};
    RapidPathKind pathKind{RapidPathKind::None};
    bool usedAbsoluteSafeFallback{false};
    QString failureReason;

    bool isValid() const { return failureReason.isEmpty() && !segments.isEmpty(); }
};

struct RapidMotionProfile
{
    std::array<double, MachineAxisLayout::kMaxAxes> velocity{};
    std::array<double, MachineAxisLayout::kMaxAxes> acceleration{};
    std::array<double, MachineAxisLayout::kMaxAxes> jerk{};
    std::uint8_t supportedCoordinatedMask{0};
};

struct TravelPlanKey
{
    std::uint64_t toolpathRevision{0};
    std::uint64_t orderHash{0};
    std::uint64_t environmentRevision{0};
    std::uint64_t motionProfileHash{0};
    double compensationOffsetX{0.0};
    double compensationOffsetY{0.0};

    bool operator==(const TravelPlanKey& other) const
    {
        return toolpathRevision == other.toolpathRevision
            && orderHash == other.orderHash
            && environmentRevision == other.environmentRevision
            && motionProfileHash == other.motionProfileHash
            && compensationOffsetX == other.compensationOffsetX
            && compensationOffsetY == other.compensationOffsetY;
    }
};

struct TravelPlanSnapshot
{
    TravelPlanKey key;
    TravelPlanningMode mode{TravelPlanningMode::None};
    QVector<RapidTransition> transitions;
    double totalEstimatedTimeMs{0.0};
    double totalLengthMm{0.0};
    double minimumClearanceMm{0.0};
    int fallbackCount{0};
    QString failureReason;
    bool stale{true};

    bool isExecutable() const { return !stale && failureReason.isEmpty(); }
    const RapidTransition* transitionTo(std::uint64_t contourId) const
    {
        for (const RapidTransition& transition : transitions) {
            if (transition.toContourId == contourId)
                return &transition;
        }
        return nullptr;
    }
};

} // namespace lcnc::cam
