#include "core/algorithms/cam/full5d_optimizer.h"

#include <QCoreApplication>
#include <QTextStream>

#include <cmath>

using namespace lcnc::cam;
using namespace lcnc::cam_algo;

namespace {
CamMotionPlanSnapshot fixture(const QVector<double>& angles)
{
    CamMotionPlanSnapshot plan;
    plan.context.interpolationModelVersion = 1;
    plan.context.optimizationPolicyHash = "strict-test";
    CamMotionBlock block;
    block.blockId = 1; block.contourId = 1; block.activeAxisMask = 0x1f;
    for (int i = 0; i < angles.size(); ++i) {
        CamMotionNode node;
        node.contourId = 1; node.axisMask = 0x1f; node.sourceEdgeIndex = 0;
        node.sourceParameter = i;
        node.axes[4] = angles[i]; node.axes[0] = i;
        node.tcpX = i * i;
        node.referenceTcpX = i * i + 10; node.referenceTcpValid = true;
        node.normalX = std::sin(angles[i] * 0.01) * (1.0 + 1e-15);
        node.normalZ = std::cos(angles[i] * 0.01) * (1.0 + 1e-15);
        node.estimatedTimeMs = 20;
        block.physicalKnots.append(node);
    }
    block.sourceSpans.append({1, 0, static_cast<int>(angles.size()) - 1, 0, static_cast<double>(angles.size() - 1), 0});
    block.fences.append({0, true, false, true});
    block.fences.append({static_cast<int>(angles.size()) - 1, false, true, false});
    plan.blocks.append(block);
    finalizeMotionPlan(&plan);
    return plan;
}

MotionEvaluationContext model()
{
    MotionEvaluationContext context;
    context.interpolationModelVersion = 1;
    context.evaluatePhysicalAxes = [](const auto& axes, std::uint8_t, EvaluatedMotionState* result) {
        // Nonlinear FK catches independently interpolated derived caches.
        result->worldTcpX = axes[0] * axes[0];
        result->referenceTcpX = axes[0] * axes[0] + 10;
        result->referenceTcpValid = true;
        result->processDirectionX = std::sin(axes[4] * 0.01);
        result->processDirectionZ = std::cos(axes[4] * 0.01);
        return true;
    };
    return context;
}
}

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    const auto require = [](bool condition, const char* message) {
        if (!condition) QTextStream(stderr) << message << '\n';
        return condition;
    };
    double lifted = 0;
    if (!require(liftPeriodicObservation(1, 359, true, -720, 720, &lifted) && lifted == 361,
                 "wrapped observation did not lift 359 -> 361")
        || !require(liftPeriodicObservation(711, 351, false, -720, 720, &lifted) && lifted == 711,
                    "authoritative multiple turn was lost")
        || !require(!liftPeriodicObservation(1, 359, true, 0, 360, &lifted), "near-limit lift escaped limit")
        || !require(!liftPeriodicObservation(711, 351, false, 0, 360, &lifted), "invalid solved turn was normalized")) return 1;

    Full5DPolicy policy;
    policy.mode = PoseOptimizationMode::Conservative; policy.rotaryMask = 0x10;
    policy.maxKnotMultiplier = 128;
    policy.minimumAxes[4] = -720; policy.maximumAxes[4] = 720;
    CamMotionPlanSnapshot result;
    Full5DMetrics metrics;
    QString error;
    auto source = fixture({351, 711});
    if (!require(optimizeFull5D(source, policy, model(), &result, &metrics, &error), qPrintable(error))
        || !require(metrics.maximumRotaryStepDegrees <= 5 && metrics.insertedKnots > 0, "hard rotary step not enforced")
        || !require(result.nodes.last().axes[4] == 711 && metrics.rotaryTravelDegrees == 360, "turn count changed")
        || !require(std::abs(result.nodes[1].tcpX - result.nodes[1].axes[0] * result.nodes[1].axes[0]) < 1e-12,
                    "inserted TCP not derived by evaluator")
        || !require(result.nodes[1].referenceTcpValid, "reference TCP not rebuilt")) return 1;
    const QByteArray hash = result.planHash;
    policy.mode = PoseOptimizationMode::Off;
    if (!require(!optimizeFull5D(source, policy, {}, &result, &metrics, &error)
                 && result.planHash == hash, "Off bypassed the hard rotary bound")) return 1;
    policy.mode = PoseOptimizationMode::Conservative;
    if (!require(optimizeFull5D(source, policy, model(), &result, &metrics, &error) && result.planHash == hash,
                 "repeat identity differs")) return 1;
    policy.maxKnotMultiplier = 1;
    if (!require(!optimizeFull5D(source, policy, model(), &result, &metrics, &error) && result.planHash == hash,
                 "budget failure partially published or relaxed step")) return 1;
    policy.maxKnotMultiplier = 128;
    if (!require(!optimizeFull5D(source, policy, model(), &result, &metrics, &error, [] { return true; })
                 && result.planHash == hash, "cancel partially published")) return 1;

    source = fixture({359, 360, 361});
    if (!require(optimizeFull5D(source, policy, model(), &result, &metrics, &error) && result.nodes.size() == 2,
                 "strict affine merge failed")) return 1;
    source.blocks[0].physicalKnots[1].axes[4] = 365;
    finalizeMotionPlan(&source);
    if (!require(optimizeFull5D(source, policy, model(), &result, &metrics, &error)
                 && result.nodes.size() >= 3 && metrics.rotaryReversals > 0, "midpoint/reversal counterexample merged")) return 1;
    source = fixture({359, 360, 361});
    source.blocks[0].fences.append({1, false, false, false});
    finalizeMotionPlan(&source);
    if (!require(optimizeFull5D(source, policy, model(), &result, &metrics, &error) && result.nodes.size() == 3,
                 "process fence was removed")) return 1;
    source.blocks[0].fences.removeLast();
    source.blocks[0].physicalKnots[1].semanticHardBarrier = true;
    finalizeMotionPlan(&source);
    if (!require(optimizeFull5D(source, policy, model(), &result, &metrics, &error) && result.nodes.size() == 3,
                 "semantic barrier was removed")) return 1;
    policy.orientationToleranceDeg = 0.1; policy.mode = PoseOptimizationMode::Full;
    if (!require(optimizeFull5D(source, policy, model(), &result, &metrics, &error)
                 && !metrics.rejectedReasons.isEmpty() && result.nodes.size() == 3,
                 "unproved nonzero smoothing silently admitted")) return 1;
    policy.mode = PoseOptimizationMode::Off;
    if (!require(optimizeFull5D(source, policy, {}, &result, &metrics, &error)
                 && result.nodes.size() == source.nodes.size()
                 && std::abs(std::hypot(result.nodes[1].normalX, result.nodes[1].normalZ) - 1) < 1e-15,
                 "Off did more than numerical normalization")) return 1;
    source.blocks[0].fences.append({999, false, false, false});
    finalizeMotionPlan(&source);
    if (!require(!optimizeFull5D(source, policy, {}, &result, &metrics, &error), "invalid fence accepted")) return 1;

    source = fixture({359, 360});
    auto next = source.blocks[0];
    next.blockId = 2; next.hasEntryBoundary = true;
    next.entryBoundary = source.nodes.last();
    next.physicalKnots = {next.entryBoundary};
    next.physicalKnots[0].sourceParameter = 2;
    next.physicalKnots[0].axes[4] = 361;
    next.sourceSpans = {{1, -1, -1, 1, 1, 0}, {1, 0, 0, 2, 2, 0}};
    next.fences = {{-1, true, false, true}, {0, false, true, false}};
    source.blocks.append(next);
    finalizeMotionPlan(&source);
    policy.mode = PoseOptimizationMode::Conservative;
    policy.orientationToleranceDeg = 0;
    if (!require(optimizeFull5D(source, policy, model(), &result, &metrics, &error)
                 && result.blocks.size() == 2 && result.nodes.size() == 3
                 && finalMotionPlanIdentityIsCurrent(result)
                 && result.blocks[1].sourceSpans[0].firstKnot == -1
                 && result.blocks[1].fences[0].knotIndex == -1,
                 "entry boundary ownership changed")) return 1;
    source.blocks[1].physicalKnots[0].axes[4] = 400;
    finalizeMotionPlan(&source);
    const auto boundaryHash = result.planHash;
    if (!require(!optimizeFull5D(source, policy, model(), &result, &metrics, &error)
                 && result.planHash == boundaryHash, "cross-boundary source ownership invented")) return 1;
    source = fixture({359, 361});
    policy.maximumAxes[4] = 360;
    if (!require(!optimizeFull5D(source, policy, model(), &result, &metrics, &error), "near-limit candidate accepted")) return 1;
    policy.maximumAxes[4] = 720;
    source = fixture({0, 0});
    policy.positionChordToleranceMm = 0.01;
    const auto unknownHash = result.planHash;
    if (!require(!optimizeFull5D(source, policy, model(), &result, &metrics, &error)
                 && result.planHash == unknownHash, "unknown chord bound certified")) return 1;
    auto bounded = model();
    bounded.boundPhysicalAxes = [](const CamMotionBlock&, double lo, double hi, MotionIntervalBound* interval) {
        // For x(u)=u^2, the maximum chord error is (hi-lo)^2/4.
        interval->minimumPhysicalAxes[0] = lo;
        interval->maximumPhysicalAxes[0] = hi;
        interval->minimumWorldTcpX = lo * lo;
        interval->maximumWorldTcpX = hi * hi;
        interval->maximumPositionErrorMm = (hi - lo) * (hi - lo) / 4;
        interval->maximumOrientationErrorDegrees = (hi - lo) * (hi - lo);
        interval->valid = true; interval->refinable = true;
        return true;
    };
    policy.orientationChordToleranceDeg = 0.01;
    if (!require(optimizeFull5D(source, policy, bounded, &result, &metrics, &error)
                 && result.nodes.size() == 17 && metrics.maximumDepth == 4,
                 "position/orientation adaptive bounds ignored")) return 1;
    policy.maxDepth = 1;
    if (!require(!optimizeFull5D(source, policy, bounded, &result, &metrics, &error), "depth exhaustion inflated tolerance")) return 1;
    policy.maxDepth = 12;
    policy.positionChordToleranceMm = 0;
    policy.orientationChordToleranceDeg = 0;
    source.blocks[0].physicalKnots[0].axes[0] = 5;
    if (!require(!optimizeFull5D(source, policy, model(), &result, &metrics, &error), "stale identity accepted")) return 1;
    return 0;
}
