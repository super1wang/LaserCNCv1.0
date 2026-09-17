#include "core/algorithms/cam/geometry_source_sampler.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lcnc::cam_algo {
namespace {
double angle(const gp_Dir& first, const gp_Dir& last)
{
    return std::acos(std::clamp(first.Dot(last), -1.0, 1.0))
        * (180.0 / 3.14159265358979323846);
}
bool finiteNonnegative(double value) { return std::isfinite(value) && value >= 0.0; }
}

GeometrySamplingResult refineGeometrySource(
    const std::vector<ToolpathPoint>& seeds, bool sourceCoverageComplete,
    double chordToleranceMm, const GeometrySamplingPolicy& policy,
    const std::function<ToolpathPoint(int, double)>& sampleAt,
    const std::function<GeometryIntervalBound(int, double, double)>& boundInterval)
{
    GeometrySamplingResult result;
    result.evidence.sourceCoverageComplete = sourceCoverageComplete && seeds.size() >= 2;
    result.evidence.requiredFeaturesPreserved = true;
    result.evidence.refinementCriteriaSatisfied = true;
    const auto fail = [&](const QString& reason) {
        if (result.evidence.failureReason.isEmpty())
            result.evidence.failureReason = reason;
    };
    if (!result.evidence.sourceCoverageComplete)
        fail(QStringLiteral("Required non-degenerate source interval was not sampled"));
    if (!(chordToleranceMm > 0.0) || !std::isfinite(chordToleranceMm)
        || !(policy.maxTangentStepDeg > 0.0) || !std::isfinite(policy.maxTangentStepDeg)
        || !(policy.maxNormalStepDeg > 0.0) || !std::isfinite(policy.maxNormalStepDeg)
        || policy.maxSubdivisionDepth < 0 || policy.maxSampleMultiplier < 1
        || !(policy.minSourceParameterSpan > 0.0) || !std::isfinite(policy.minSourceParameterSpan)) {
        result.evidence.refinementCriteriaSatisfied = false;
        result.evidence.requiredFeaturesPreserved = policy.processBarriers.empty();
        fail(QStringLiteral("Invalid geometry sampling policy"));
        return result;
    }
    const std::size_t multiplier = static_cast<std::size_t>(policy.maxSampleMultiplier);
    result.maximumPointCount = seeds.size() > std::numeric_limits<std::size_t>::max() / multiplier
        ? std::numeric_limits<std::size_t>::max() : seeds.size() * multiplier;
    std::vector<ToolpathPoint> guarded = seeds;
    // All original endpoints are reserved up front. Every inserted barrier or
    // recursive midpoint spends exactly one of the remaining output slots.
    std::size_t availableInsertions = result.maximumPointCount - seeds.size();
    for (const auto& barrier : policy.processBarriers) {
        bool resolved = false;
        if (barrier.sourceEdgeIndex >= 0 && std::isfinite(barrier.parameter)) {
            for (std::size_t index = 0; index < guarded.size(); ++index) {
                auto& point = guarded[index];
                if (point.sourceEdgeIndex != barrier.sourceEdgeIndex)
                    continue;
                if (point.param == barrier.parameter) {
                    point.semanticHardBarrier = true;
                    resolved = true;
                    break;
                }
                if (index + 1 == guarded.size()
                    || guarded[index + 1].sourceEdgeIndex != barrier.sourceEdgeIndex)
                    continue;
                const double last = guarded[index + 1].param;
                if (barrier.parameter <= std::min(point.param, last)
                    || barrier.parameter >= std::max(point.param, last))
                    continue;
                if (availableInsertions == 0)
                    break;
                ToolpathPoint inserted = sampleAt(barrier.sourceEdgeIndex, barrier.parameter);
                inserted.semanticHardBarrier = true;
                guarded.insert(guarded.begin() + static_cast<std::ptrdiff_t>(index + 1), inserted);
                --availableInsertions;
                resolved = true;
                break;
            }
        }
        if (!resolved) {
            result.evidence.requiredFeaturesPreserved = false;
            fail(QStringLiteral("Required source barrier is unresolved or exceeds the sample budget"));
        }
    }
    if (guarded.empty())
        return result;
    result.points.push_back(guarded.front());
    std::function<void(const ToolpathPoint&, const ToolpathPoint&, int)> refine;
    refine = [&](const ToolpathPoint& first, const ToolpathPoint& last, int depth) {
        if (first.sourceEdgeIndex != last.sourceEdgeIndex) {
            result.points.push_back(last);
            return;
        }
        const auto bound = boundInterval(first.sourceEdgeIndex, first.param, last.param);
        const bool validBound = bound.available && finiteNonnegative(bound.chordDeviationMm)
            && finiteNonnegative(bound.tangentVariationDeg) && finiteNonnegative(bound.normalVariationDeg);
        const bool accepted = validBound
            && bound.chordDeviationMm <= chordToleranceMm
            && bound.tangentVariationDeg <= policy.maxTangentStepDeg
            && bound.normalVariationDeg <= policy.maxNormalStepDeg
            && angle(first.tangent, last.tangent) <= policy.maxTangentStepDeg + 1e-10
            && angle(first.normal, last.normal) <= policy.maxNormalStepDeg + 1e-10;
        if (accepted) {
            result.points.push_back(last);
            return;
        }
        const double middleParameter = first.param + (last.param - first.param) * 0.5;
        if (!validBound || depth >= policy.maxSubdivisionDepth || availableInsertions == 0
            || std::abs(last.param - first.param) <= policy.minSourceParameterSpan
            || middleParameter == first.param || middleParameter == last.param) {
            result.evidence.refinementCriteriaSatisfied = false;
            fail(validBound ? QStringLiteral("Geometry refinement budget exhausted")
                            : QStringLiteral("No conservative source normal/curve interval bound is available"));
            result.points.push_back(last);
            return;
        }
        --availableInsertions;
        const auto middle = sampleAt(first.sourceEdgeIndex, middleParameter);
        refine(first, middle, depth + 1);
        refine(middle, last, depth + 1);
    };
    for (std::size_t index = 1; index < guarded.size(); ++index)
        refine(guarded[index - 1], guarded[index], 0);
    return result;
}

} // namespace lcnc::cam_algo
