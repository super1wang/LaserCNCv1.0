#pragma once

#include "core/kinematics/machine_topology.h"
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
    double machineX{0.0};
    double machineY{0.0};
    double machineZ{0.0};
    double machineR1{0.0};
    double machineR2{0.0};
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
    /// Derived only: never persisted in a .lcnc package.  A stale/failed plan
    /// is deliberately exported so Process can reject unsafe execution.
    TravelPlanSnapshot travelPlan;

    bool hasEnabledContours() const;
    int totalPointCount() const;
};

} // namespace lcnc::cam
