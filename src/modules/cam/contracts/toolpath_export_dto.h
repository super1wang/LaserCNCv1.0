#pragma once

#include "core/kinematics/machine_topology.h"
#include "core/project/cam/collision_validation_contracts.h"
#include "core/project/cam/travel_plan_contracts.h"

#include <QHash>
#include <QString>
#include <QVector>

#include <cstdint>
#include <array>

namespace lcnc::cam {

/**
 * @brief One Process-facing sampled toolpath point without OCC dependencies.
 */
struct ToolpathExportPoint
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double normalX{0.0};
    double normalY{0.0};
    double normalZ{1.0};
    double tangentX{1.0};
    double tangentY{0.0};
    double tangentZ{0.0};
    double curveParam{0.0};
    int sourceEdgeIndex{-1}; ///< OCC wire-edge owner; -1 for non-source motion.
    int departureSourceEdgeIndex{-1};
    double departureSourceParameter{0.0};
    bool semanticHardBarrier{false}; ///< Required process/laser fence at this source knot.
    double machineX{0.0};
    double machineY{0.0};
    double machineZ{0.0};
    double machineR1{0.0};
    double machineR2{0.0};
    /// Table-zero reference TCP in the right-handed Z-up model (GTN MCS).
    /// Workpiece carrier motion has been removed, setup is retained. This is
    /// NOT the posed machine-world TCP used by collision/rendering, and NOT
    /// the controller-axis coordinate basis. Process consumes it unchanged.
    double tcpMcsX{0.0};
    double tcpMcsY{0.0};
    double tcpMcsZ{0.0};
    bool tcpMcsValid{false};
    QString rotaryAxis1Name;
    QString rotaryAxis2Name;
    bool machineCoordValid{false};
    std::array<double, MachineAxisLayout::kMaxAxes> machineAxes{};
    std::uint8_t machineAxisMask{0};
    QString machineFailureReason;
};

/**
 * @brief Process-facing contour metadata exported from CAM.
 */
struct ToolpathExportContour
{
    std::uint64_t contourId{0};
    std::uint64_t layerId{0};
    QString contourName;
    QString layerName;
    QString toolName;
    QString workpieceEntry;
    bool enabled{false};
    bool layerEnabled{false};
    bool needsRecalculation{false};
    QString recalculationReason;
    int pointCount{0};
    double cuttingOffsetMm{1.0};
    double rapidOffsetMm{5.0};

    // 世界坐标系下的几何端点，给空程规划/虚线绘制直接消费。
    // startX/Y/Z = lead-in 解有效 ? 下刀点 : points.front()
    // endX/Y/Z   = points.back()
    // cutStartX/Y/Z 始终是 points.front()，即真正的切割切入点
    double startX{0.0};
    double startY{0.0};
    double startZ{0.0};
    double endX{0.0};
    double endY{0.0};
    double endZ{0.0};
    double cutStartX{0.0};
    double cutStartY{0.0};
    double cutStartZ{0.0};
    ToolpathExportPoint leadInPoint;
    QString leadInError;
    bool hasLeadIn{false};
    bool endpointsValid{false};
};

/// Immutable collision-runtime identity captured together with the solved
/// motion. Process uses it as a real-machine preflight gate; it deliberately
/// contains no geometry or mutable service pointer.
struct CollisionSafetyExecutionSnapshot
{
    CollisionVerificationMode verificationMode{
        CollisionVerificationMode::Disabled};
    /// Compatibility projection. New execution gates use verificationMode.
    bool enabled{false};
    bool machinePackageRequired{false};
    bool machinePackageReady{false};
    bool packageBuildInProgress{false};
    bool jobOverlayRequired{false};
    bool jobOverlayReady{false};
    bool jobOverlayBuildInProgress{false};
    QString packageState;
    QString jobOverlayState;
    QString packageKeySha256;
    QString runtimeConfigurationSha256;
    QString failureReason;
    QString jobOverlayFailureReason;

    CollisionVerificationMode effectiveVerificationMode() const
    {
        // Compatibility with snapshots created by older in-process callers:
        // an explicit legacy enabled=true is interpreted fail-closed.
        return verificationMode == CollisionVerificationMode::Disabled && enabled
            ? CollisionVerificationMode::Required : verificationMode;
    }
};

/**
 * @brief Full immutable CAM toolpath snapshot consumed by Process runtime services.
 */
struct ToolpathExportSnapshot
{
    std::uint64_t revision{0};
    QVector<ToolpathExportContour> contours;
    QHash<std::uint64_t, QVector<ToolpathExportPoint>> pointsByContourId;
    QString description;
    MachiningMode machiningMode{MachiningMode::Planar3Axis};
    MachineAxisLayout machineAxisLayout;
    QString machineConfigurationFingerprint;
    QString solverId;
    int solverVersion{0};
    CollisionSafetyExecutionSnapshot collisionSafety;
    /// Derived only: never persisted in a .lcnc package.  A stale/failed plan
    /// is deliberately exported so Process can reject unsafe execution.
    TravelPlanSnapshot travelPlan;
    /// Canonical final-coordinate execution sequence.  It is derived by CAM
    /// and retained separately from the editable contour data.
    CamMotionPlanSnapshot motionPlan;

    bool hasEnabledContours() const;
    int totalPointCount() const;

    /// Refreshes collision-runtime state after asynchronous CAM verification
    /// without replacing the already committed solved coordinates and nodes.
    /// The adapter uses this when contour order and geometry revision are
    /// unchanged but Job Overlay readiness and edge certificates have advanced.
    bool mergeCollisionProofFrom(const ToolpathExportSnapshot& verified)
    {
        if (motionPlan.planHash.isEmpty()
            || verified.motionPlan.planHash != motionPlan.planHash
            || verified.motionPlan.contextHash != motionPlan.contextHash
            || verified.motionPlan.revision != motionPlan.revision) {
            return false;
        }
        collisionSafety = verified.collisionSafety;
        travelPlan = verified.travelPlan;
        motionPlan.collision = verified.motionPlan.collision;
        motionPlan.edgeCertificates = verified.motionPlan.edgeCertificates;
        return true;
    }
};

} // namespace lcnc::cam
