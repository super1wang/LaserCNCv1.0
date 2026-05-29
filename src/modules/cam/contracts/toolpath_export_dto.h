#pragma once

#include <QHash>
#include <QString>
#include <QVector>

#include <cstdint>

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
    int pointCount{0};
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

    bool hasEnabledContours() const;
    int totalPointCount() const;
};

} // namespace lcnc::cam