#include "core/algorithms/cam/full5d_optimizer.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace lcnc::cam_algo {
namespace {
using namespace lcnc::cam;

bool fail(QString* error, const QString& reason)
{
    if (error) *error = reason;
    return false;
}

double rotaryStep(const CamMotionNode& a, const CamMotionNode& b, std::uint8_t mask)
{
    double result = 0;
    for (std::size_t axis = 0; axis < a.axes.size(); ++axis)
        if (mask & (1u << axis)) result = std::max(result, std::abs(b.axes[axis] - a.axes[axis]));
    return result;
}

bool withinLimits(const CamMotionNode& node, const Full5DPolicy& policy)
{
    for (std::size_t axis = 0; axis < node.axes.size(); ++axis)
        if ((node.axisMask & (1u << axis))
            && (!std::isfinite(node.axes[axis]) || node.axes[axis] < policy.minimumAxes[axis]
                || node.axes[axis] > policy.maximumAxes[axis])) return false;
    return true;
}

bool normalizeDirection(CamMotionNode* node)
{
    const double length = std::hypot(node->normalX, node->normalY, node->normalZ);
    if (!std::isfinite(length) || length == 0.0) return false;
    node->normalX /= length;
    node->normalY /= length;
    node->normalZ /= length;
    return true;
}
} // namespace

Full5DPolicy::Full5DPolicy()
{
    minimumAxes.fill(-std::numeric_limits<double>::infinity());
    maximumAxes.fill(std::numeric_limits<double>::infinity());
}

bool liftPeriodicObservation(double value, double previous, bool wrappedObservation,
                             double minimum, double maximum, double* result)
{
    if (!result || !std::isfinite(value) || !std::isfinite(previous)
        || std::isnan(minimum) || std::isnan(maximum) || minimum > maximum) return false;
    // Do not use equivalent-angle selection to rescue an out-of-limit input.
    if (value < minimum || value > maximum) return false;
    double lifted = value;
    if (wrappedObservation) {
        if (value < 0 || value >= 360) return false;
        lifted += 360.0 * std::round((previous - value) / 360.0);
        if (std::abs(lifted - previous) == 180.0) return false; // No direction authority.
    }
    if (!std::isfinite(lifted) || lifted < minimum || lifted > maximum) return false;
    *result = lifted;
    return true;
}

bool optimizeFull5D(const lcnc::cam::CamMotionPlanSnapshot& raw,
                    const Full5DPolicy& policy, const MotionEvaluationContext& evaluation,
                    lcnc::cam::CamMotionPlanSnapshot* optimized, Full5DMetrics* metrics,
                    QString* error, const std::function<bool()>& cancelled,
                    const std::function<MotionEvaluationContext(const CamMotionBlock&)>& modelForBlock)
{
    if (!optimized || !metrics || !finalMotionPlanIdentityIsCurrent(raw))
        return fail(error, QStringLiteral("Full5D input/output or frozen identity is invalid"));
    if (!std::isfinite(policy.maxRotaryStepDeg) || policy.maxRotaryStepDeg <= 0
        || (policy.mode != PoseOptimizationMode::Off && policy.mode != PoseOptimizationMode::Conservative
            && policy.mode != PoseOptimizationMode::Full)
        || !std::isfinite(policy.orientationToleranceDeg) || policy.orientationToleranceDeg < 0
        || !std::isfinite(policy.positionChordToleranceMm) || policy.positionChordToleranceMm < 0
        || !std::isfinite(policy.orientationChordToleranceDeg) || policy.orientationChordToleranceDeg < 0
        || policy.maxDepth < 0 || policy.maxDepth > 30 || policy.maxKnotMultiplier < 1
        || policy.maxKnotsPerBlock < 1 || !std::isfinite(policy.minParameterSpan)
        || policy.minParameterSpan <= 0 || policy.minParameterSpan > 1)
        return fail(error, QStringLiteral("Invalid Full5D policy"));
    for (std::size_t axis = 0; axis < policy.minimumAxes.size(); ++axis)
        if (std::isnan(policy.minimumAxes[axis]) || std::isnan(policy.maximumAxes[axis])
            || policy.minimumAxes[axis] > policy.maximumAxes[axis])
            return fail(error, QStringLiteral("Invalid frozen axis limits"));

    CamMotionPlanSnapshot result = raw;
    Full5DMetrics audit;
    std::array<double, MachineAxisLayout::kMaxAxes> minimum, maximum;
    minimum.fill(std::numeric_limits<double>::infinity());
    maximum.fill(-std::numeric_limits<double>::infinity());
    if (policy.orientationToleranceDeg > 0)
        audit.rejectedReasons.append(QStringLiteral(
            "Relaxed smoothing unavailable: no material-arc-length continuous proof; strict reference retained"));
    ContinuousMotionEvaluator evaluator;
    for (int blockIndex = 0; blockIndex < raw.blocks.size(); ++blockIndex) {
        if (cancelled && cancelled()) return fail(error, QStringLiteral("Full5D compilation cancelled"));
        const auto& source = raw.blocks[blockIndex];
        BoundMotionEvaluationContext bound;
        if (policy.mode != PoseOptimizationMode::Off
            && !bindMotionEvaluationContext(raw, modelForBlock ? modelForBlock(source) : evaluation,
                &bound, error)) return false;
        auto& block = result.blocks[blockIndex];
        if (source.interpolation != MotionInterpolationKind::PhysicalAxisLine)
            return fail(error, QStringLiteral("Full5D requires controller-independent physical interpolation"));
        const int count = source.physicalKnots.size();
        QVector<bool> protectedKnot(count, false);
        protectedKnot[0] = protectedKnot[count - 1] = true;
        for (const auto& fence : source.fences) {
            if (fence.knotIndex < -1 || fence.knotIndex >= count)
                return fail(error, QStringLiteral("Invalid Full5D fence index"));
            if (fence.knotIndex >= 0) protectedKnot[fence.knotIndex] = true;
        }
        for (const auto& span : source.sourceSpans) {
            if (span.firstKnot < -1 || span.lastKnot < span.firstKnot || span.lastKnot >= count)
                return fail(error, QStringLiteral("Invalid Full5D source span"));
            if (span.firstKnot >= 0) protectedKnot[span.firstKnot] = true;
            if (span.lastKnot >= 0) protectedKnot[span.lastKnot] = true;
        }
        QVector<int> originalIndex;
        QVector<CamMotionNode> knots;
        QVector<int> mapped(count, -1);
        const qint64 cap = std::min<qint64>(policy.maxKnotsPerBlock,
            static_cast<qint64>(count) * policy.maxKnotMultiplier);
        const int segmentCount = count - 1 + (source.hasEntryBoundary ? 1 : 0);
        for (int i = 0; i < count; ++i) {
            if (cancelled && cancelled()) return fail(error, QStringLiteral("Full5D compilation cancelled"));
            const auto& target = source.physicalKnots[i];
            if (target.estimatedTimeMs < 0)
                return fail(error, QStringLiteral("Full5D knot duration is negative"));
            if (!withinLimits(target, policy)) return fail(error, QStringLiteral("Full5D physical soft limit exceeded"));
            const CamMotionNode* previous = i > 0 ? &source.physicalKnots[i - 1]
                : source.hasEntryBoundary ? &source.entryBoundary : nullptr;
            int pieces = 1;
            if (previous) {
                if (!withinLimits(*previous, policy)) return fail(error, QStringLiteral("Full5D entry soft limit exceeded"));
                if (policy.mode == PoseOptimizationMode::Off
                    && rotaryStep(*previous, target, policy.rotaryMask) > policy.maxRotaryStepDeg)
                    return fail(error, QStringLiteral("Off cannot refine an edge exceeding the hard rotary step"));
                if (policy.mode != PoseOptimizationMode::Off) {
                    const double demand = rotaryStep(*previous, target, policy.rotaryMask);
                    int depth = 0;
                    for (;;) {
                        bool refine = demand / pieces > policy.maxRotaryStepDeg;
                        if (policy.positionChordToleranceMm > 0 || policy.orientationChordToleranceDeg > 0) {
                            for (int piece = 0; piece < pieces && !refine; ++piece) {
                                if (cancelled && cancelled()) return fail(error, QStringLiteral("Full5D compilation cancelled"));
                                const double start = i - 1 + (source.hasEntryBoundary ? 1 : 0);
                                MotionIntervalBound interval;
                                if (!evaluator.bound(source,
                                        (start + static_cast<double>(piece) / pieces) / segmentCount,
                                        (start + static_cast<double>(piece + 1) / pieces) / segmentCount,
                                        bound, &interval, error)) return false;
                                refine = (policy.positionChordToleranceMm > 0
                                        && interval.maximumPositionErrorMm > policy.positionChordToleranceMm)
                                    || (policy.orientationChordToleranceDeg > 0
                                        && interval.maximumOrientationErrorDegrees > policy.orientationChordToleranceDeg);
                            }
                        }
                        if (!refine) break;
                        if (depth >= policy.maxDepth || 0.5 / pieces / segmentCount < policy.minParameterSpan
                            || static_cast<qint64>(pieces) * 2 > cap)
                            return fail(error, QStringLiteral("Full5D refinement budget exhausted; candidate rejected"));
                        pieces *= 2;
                        ++depth;
                    }
                    audit.maximumDepth = std::max(audit.maximumDepth, depth);
                }
                // Incoming edges may belong to a different source/phase. Do not
                // invent source ownership for an inserted cross-boundary knot.
                if (pieces > 1 && (i == 0 || previous->sourceEdgeIndex != target.sourceEdgeIndex
                    || previous->contourId != target.contourId || previous->phase != target.phase))
                    return fail(error, QStringLiteral("Full5D boundary refinement requires explicit source ownership"));
                if (static_cast<qint64>(knots.size()) + pieces + count - i - 1 > cap)
                    return fail(error, QStringLiteral("Full5D knot budget exhausted; candidate rejected"));
                for (int piece = 1; piece < pieces; ++piece) {
                    if (cancelled && cancelled()) return fail(error, QStringLiteral("Full5D compilation cancelled"));
                    const double fraction = static_cast<double>(piece) / pieces;
                    const double u = (i - 1 + (source.hasEntryBoundary ? 1 : 0) + fraction) / segmentCount;
                    EvaluatedMotionState state;
                    if (!evaluator.evaluate(source, u, bound, &state, error)) return false;
                    CamMotionNode inserted = target;
                    inserted.axes = state.physicalAxes;
                    inserted.axisMask = state.physicalAxisMask;
                    inserted.tcpX = state.worldTcpX; inserted.tcpY = state.worldTcpY; inserted.tcpZ = state.worldTcpZ;
                    inserted.referenceTcpX = state.referenceTcpX; inserted.referenceTcpY = state.referenceTcpY;
                    inserted.referenceTcpZ = state.referenceTcpZ; inserted.referenceTcpValid = state.referenceTcpValid;
                    inserted.normalX = state.processDirectionX; inserted.normalY = state.processDirectionY;
                    inserted.normalZ = state.processDirectionZ;
                    inserted.sourceParameter = previous->sourceParameter
                        + (target.sourceParameter - previous->sourceParameter) * fraction;
                    inserted.semanticHardBarrier = false;
                    inserted.estimatedTimeMs = target.estimatedTimeMs / pieces;
                    if (!withinLimits(inserted, policy) || !normalizeDirection(&inserted))
                        return fail(error, QStringLiteral("Invalid evaluated Full5D knot"));
                    knots.append(inserted); originalIndex.append(-1); ++audit.insertedKnots;
                }
            }
            CamMotionNode retained = target;
            retained.estimatedTimeMs /= pieces;
            if (policy.mode != PoseOptimizationMode::Off) {
                EvaluatedMotionState state;
                const double u = segmentCount == 0 ? 0.0
                    : static_cast<double>(i + (source.hasEntryBoundary ? 1 : 0)) / segmentCount;
                if (!evaluator.evaluate(source, u, bound, &state, error)) return false;
                retained.tcpX = state.worldTcpX; retained.tcpY = state.worldTcpY; retained.tcpZ = state.worldTcpZ;
                retained.referenceTcpX = state.referenceTcpX; retained.referenceTcpY = state.referenceTcpY;
                retained.referenceTcpZ = state.referenceTcpZ; retained.referenceTcpValid = state.referenceTcpValid;
                retained.normalX = state.processDirectionX; retained.normalY = state.processDirectionY;
                retained.normalZ = state.processDirectionZ;
                audit.changed = audit.changed || retained.tcpX != target.tcpX || retained.tcpY != target.tcpY
                    || retained.tcpZ != target.tcpZ || retained.referenceTcpX != target.referenceTcpX
                    || retained.referenceTcpY != target.referenceTcpY || retained.referenceTcpZ != target.referenceTcpZ
                    || retained.referenceTcpValid != target.referenceTcpValid;
            }
            if (!normalizeDirection(&retained)) return fail(error, QStringLiteral("Invalid Full5D orientation"));
            audit.changed = audit.changed || retained.normalX != target.normalX
                || retained.normalY != target.normalY || retained.normalZ != target.normalZ;
            knots.append(retained); originalIndex.append(i);
        }
        // Only exact midpoint removal: identical axes on both affine halves is
        // a whole-interval FK equivalence proof, including nonlinear TCP motion.
        if (policy.mode != PoseOptimizationMode::Off
            && policy.positionChordToleranceMm == 0 && policy.orientationChordToleranceDeg == 0) {
            for (int i = 1; i + 1 < knots.size();) {
                if (cancelled && cancelled()) return fail(error, QStringLiteral("Full5D compilation cancelled"));
                const auto& a = knots[i - 1]; const auto& b = knots[i]; const auto& c = knots[i + 1];
                const int original = originalIndex[i];
                bool equivalent = !b.semanticHardBarrier && (original < 0 || !protectedKnot[original])
                    && a.axisMask == b.axisMask && b.axisMask == c.axisMask
                    && a.rapidPhase == b.rapidPhase && b.rapidPhase == c.rapidPhase
                    && a.sourceEdgeIndex == b.sourceEdgeIndex && b.sourceEdgeIndex == c.sourceEdgeIndex
                    && b.sourceParameter == a.sourceParameter + (c.sourceParameter - a.sourceParameter) * 0.5
                    && b.estimatedTimeMs == c.estimatedTimeMs
                    && rotaryStep(a, c, policy.rotaryMask) <= policy.maxRotaryStepDeg;
                for (std::size_t axis = 0; equivalent && axis < a.axes.size(); ++axis)
                    equivalent = b.axes[axis] == a.axes[axis] + (c.axes[axis] - a.axes[axis]) * 0.5;
                if (!equivalent) { ++i; continue; }
                knots[i + 1].estimatedTimeMs += b.estimatedTimeMs;
                knots.removeAt(i); originalIndex.removeAt(i); ++audit.removedKnots;
            }
        }
        for (int i = 0; i < originalIndex.size(); ++i)
            if (originalIndex[i] >= 0) mapped[originalIndex[i]] = i;
        for (auto& span : block.sourceSpans) {
            if (span.firstKnot >= 0) span.firstKnot = mapped[span.firstKnot];
            if (span.lastKnot >= 0) span.lastKnot = mapped[span.lastKnot];
        }
        for (auto& fence : block.fences)
            if (fence.knotIndex >= 0) fence.knotIndex = mapped[fence.knotIndex];
        block.physicalKnots = std::move(knots);
        block.feed.estimatedDurationMs = 0;
        for (const auto& node : block.physicalKnots) block.feed.estimatedDurationMs += node.estimatedTimeMs;
        block.optimizationState = MotionOptimizationState::Optimized;
        // Normalization of the predecessor must match its owning block exactly.
        if (block.hasEntryBoundary && blockIndex > 0)
            block.entryBoundary = result.blocks[blockIndex - 1].physicalKnots.constLast();
        std::array<double, MachineAxisLayout::kMaxAxes> priorVelocity{};
        std::array<int, MachineAxisLayout::kMaxAxes> priorDirection{};
        const CamMotionNode* previous = block.hasEntryBoundary ? &block.entryBoundary : nullptr;
        for (const auto& node : block.physicalKnots) {
            for (std::size_t axis = 0; axis < node.axes.size(); ++axis) {
                if (!(node.axisMask & (1u << axis))) continue;
                minimum[axis] = std::min(minimum[axis], node.axes[axis]);
                maximum[axis] = std::max(maximum[axis], node.axes[axis]);
            }
            if (previous) {
                audit.maximumRotaryStepDegrees = std::max(audit.maximumRotaryStepDegrees,
                    rotaryStep(*previous, node, policy.rotaryMask));
                for (std::size_t axis = 0; axis < node.axes.size(); ++axis) {
                    if (!(policy.rotaryMask & (1u << axis))) continue;
                    const double delta = node.axes[axis] - previous->axes[axis];
                    audit.rotaryTravelDegrees += std::abs(delta);
                    const int direction = (delta > 0) - (delta < 0);
                    if (direction && priorDirection[axis] && direction != priorDirection[axis]) ++audit.rotaryReversals;
                    if (direction) priorDirection[axis] = direction;
                    if (node.estimatedTimeMs > 0) {
                        const double seconds = node.estimatedTimeMs / 1000;
                        const double velocity = delta / seconds;
                        audit.maximumRotaryVelocityProxy = std::max(audit.maximumRotaryVelocityProxy, std::abs(velocity));
                        audit.maximumRotaryAccelerationProxy = std::max(audit.maximumRotaryAccelerationProxy,
                            std::abs(velocity - priorVelocity[axis]) / seconds);
                        priorVelocity[axis] = velocity;
                    }
                }
            }
            previous = &node;
        }
    }
    audit.changed = audit.changed || audit.insertedKnots != 0 || audit.removedKnots != 0;
    result.optimizerReport.knotsBefore = raw.nodes.size();
    result.optimizerReport.blocksBefore = raw.blocks.size();
    result.optimizerReport.blocksAfter = result.blocks.size();
    result.optimizerReport.rotaryTravelDegrees = audit.rotaryTravelDegrees;
    result.optimizerReport.rotaryReversals = audit.rotaryReversals;
    for (std::size_t axis = 0; axis < minimum.size(); ++axis)
        result.optimizerReport.axisSpans[axis] = std::isfinite(minimum[axis]) ? maximum[axis] - minimum[axis] : 0;
    result.optimizerReport.rejectedCandidateReasons.append(audit.rejectedReasons);
    if (cancelled && cancelled()) return fail(error, QStringLiteral("Full5D compilation cancelled"));
    if (!finalizeMotionPlan(&result, error)) return false;
    if (result.planHash != raw.planHash) {
        // Certificates refer to the original exact plan/edge identity.
        result.edgeCertificates.clear();
        result.collision.key = 0;
        result.collision.nodeStates.clear();
        result.collision.complete = result.context.collisionMode == CollisionVerificationMode::Disabled;
        result.collision.state = result.collision.complete ? CollisionValidationState::Disabled : CollisionValidationState::Pending;
    }
    result.optimizerReport.knotsAfter = result.nodes.size();
    result.optimizerReport.estimatedControllerCommands = static_cast<std::uint64_t>(std::max(0, static_cast<int>(result.nodes.size()) - 1));
    *optimized = std::move(result); *metrics = std::move(audit);
    return true;
}
} // namespace lcnc::cam_algo
