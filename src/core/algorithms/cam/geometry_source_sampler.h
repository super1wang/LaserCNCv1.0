#pragma once

#include "core/algorithms/cam/laser_toolpath.h"

#include <functional>

namespace lcnc::cam_algo {

/// Bounds cover the entire source interval, not just sampled probe locations.
/// An adapter with no conservative bound must return available=false.
struct GeometryIntervalBound
{
    bool available{false};
    double chordDeviationMm{0.0};
    double tangentVariationDeg{0.0};
    double normalVariationDeg{0.0};
};

struct GeometrySamplingResult
{
    std::vector<ToolpathPoint> points;
    GeometrySamplingEvidence evidence;
    std::size_t maximumPointCount{0};
};

GeometrySamplingResult refineGeometrySource(
    const std::vector<ToolpathPoint>& seeds, bool sourceCoverageComplete,
    double chordToleranceMm, const GeometrySamplingPolicy& policy,
    const std::function<ToolpathPoint(int, double)>& sampleAt,
    const std::function<GeometryIntervalBound(int, double, double)>& boundInterval);

} // namespace lcnc::cam_algo
