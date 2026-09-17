#pragma once

#include "core/algorithms/cam/continuous_motion_evaluator.h"

#include <limits>

namespace lcnc::cam_algo {

enum class PoseOptimizationMode { Off, Conservative, Full };

struct Full5DPolicy
{
    PoseOptimizationMode mode{PoseOptimizationMode::Off};
    double orientationToleranceDeg{0.0};
    double maxRotaryStepDeg{5.0};
    // Zero means no chord-approximation request, not an invented tolerance.
    // Positive limits require the evaluator's whole-interval bound callback.
    // Full without this capability reports and retains the Conservative exact
    // path subset, without claiming that chord tolerances have been proven.
    double positionChordToleranceMm{0.0};
    double orientationChordToleranceDeg{0.0};
    int maxDepth{12};
    double minParameterSpan{1e-9};
    int maxKnotMultiplier{64};
    int maxKnotsPerBlock{65536};
    std::uint8_t rotaryMask{0};
    std::array<double, MachineAxisLayout::kMaxAxes> minimumAxes{};
    std::array<double, MachineAxisLayout::kMaxAxes> maximumAxes{};

    Full5DPolicy();
};

struct Full5DMetrics
{
    static constexpr bool dynamicsAuditOnly = true; // No dynamics threshold authority in S2.
    bool changed{false};
    int insertedKnots{0};
    int removedKnots{0};
    int maximumDepth{0};
    double rotaryTravelDegrees{0.0};
    double maximumRotaryStepDegrees{0.0};
    double maximumRotaryVelocityProxy{0.0};
    double maximumRotaryAccelerationProxy{0.0};
    int rotaryReversals{0};
    QStringList rejectedReasons;
};

/// Only wrapped *observations* may be lifted relative to a continuity seed.
/// Authoritative solver outputs already contain turns and must use false.
bool liftPeriodicObservation(double value, double previous, bool wrappedObservation,
                             double minimum, double maximum, double* result);

/// Transactional, collision-free Full5D reference construction. Physical-axis
/// subdivision is exactly the original affine path. Decimation requires exact
/// affine equality at the removed knot, proving equality on both subintervals
/// under arbitrary deterministic FK (not merely at a sampled midpoint).
/// Unsupported relaxed/RTCP paths are retained or rejected, never approximated.
bool optimizeFull5D(const lcnc::cam::CamMotionPlanSnapshot& raw,
                    const Full5DPolicy& policy,
                    const MotionEvaluationContext& evaluation,
                    lcnc::cam::CamMotionPlanSnapshot* optimized,
                    Full5DMetrics* metrics,
                    QString* error = nullptr,
                    const std::function<bool()>& cancelled = {},
                    const std::function<MotionEvaluationContext(const lcnc::cam::CamMotionBlock&)>& modelForBlock = {});

} // namespace lcnc::cam_algo
