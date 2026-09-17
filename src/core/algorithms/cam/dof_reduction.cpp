#include "core/algorithms/cam/dof_reduction.h"

#include <QElapsedTimer>

#include <algorithm>
#include <cmath>
#include <tuple>

namespace lcnc::cam_algo {
namespace {
using namespace lcnc::cam;

int axisCount(unsigned mask)
{
    int result = 0;
    for (; mask; mask >>= 1) result += mask & 1u;
    return result;
}

MotionClass motionClass(unsigned mask)
{
    return static_cast<MotionClass>(std::max(1, axisCount(mask)) - 1);
}

const CamMotionNode& firstNode(const CamMotionBlock& block)
{
    return block.hasEntryBoundary ? block.entryBoundary : block.physicalKnots.first();
}

std::array<PhysicalAxisAnalysis, MachineAxisLayout::kMaxAxes> analyze(const CamMotionBlock& block)
{
    std::array<PhysicalAxisAnalysis, MachineAxisLayout::kMaxAxes> result{};
    const auto& first = firstNode(block);
    for (std::size_t axis = 0; axis < result.size(); ++axis) {
        auto& audit = result[axis];
        double low = first.axes[axis], high = low, last = low, velocity = 0;
        int direction = 0;
        for (const auto& node : block.physicalKnots) {
            const double value = node.axes[axis], delta = value - last;
            low = std::min(low, value); high = std::max(high, value);
            audit.maximumLocalDelta = std::max(audit.maximumLocalDelta, std::abs(delta));
            audit.freezeResidual = std::max(audit.freezeResidual, std::abs(value - first.axes[axis]));
            audit.travel += std::abs(delta);
            const int nextDirection = (delta > 0) - (delta < 0);
            if (direction && nextDirection && direction != nextDirection) ++audit.reversals;
            if (nextDirection) direction = nextDirection;
            if (node.estimatedTimeMs > 0) {
                const double seconds = node.estimatedTimeMs / 1000;
                const double nextVelocity = delta / seconds;
                audit.maximumVelocityProxy = std::max(audit.maximumVelocityProxy, std::abs(nextVelocity));
                audit.maximumAccelerationProxy = std::max(audit.maximumAccelerationProxy,
                    std::abs(nextVelocity - velocity) / seconds);
                velocity = nextVelocity;
            }
            last = value;
        }
        audit.span = high - low; audit.monotonic = audit.reversals == 0;
    }
    return result;
}

QString admissionError(const MotionCompilationContext& context, const ReductionAdmission& authority, unsigned mask)
{
    if (!controllerQualificationIsQualified(context.controllerQualification))
        return QStringLiteral("capability not qualified");
    if (context.controllerMode != ControllerMotionMode::PhysicalAxes
        || context.controllerQualification.requestedMode != context.controllerMode)
        return QStringLiteral("controller-mode unsupported");
    if (context.controllerCapabilityHash != controllerQualificationSnapshotHash(context.controllerQualification)
        || authority.controllerSnapshotHash != context.controllerCapabilityHash)
        return QStringLiteral("capability stale");
    if (!authority.exactPhysicalAxisLine || !authority.supportedActiveMasks[mask])
        return QStringLiteral("active-axis hold unsupported");
    if (!authority.feedAndDynamicsRepresentable || authority.dynamicsHash != context.dynamicsSemanticHash
        || authority.dynamicsHash.isEmpty()) return QStringLiteral("feed/dynamics not qualified");
    return {};
}

bool withinLimits(const CamMotionBlock& block, const Full5DPolicy& limits)
{
    auto previous = firstNode(block).axes;
    const auto& first = firstNode(block);
    for (std::size_t a = 0; a < previous.size(); ++a)
        if ((first.axisMask & (1u << a)) && (previous[a] < limits.minimumAxes[a]
            || previous[a] > limits.maximumAxes[a])) return false;
    for (const auto& node : block.physicalKnots) {
        if (node.estimatedTimeMs < 0) return false;
        for (std::size_t a = 0; a < previous.size(); ++a) {
            if (!(node.axisMask & (1u << a))) continue;
            if (node.axes[a] < limits.minimumAxes[a] || node.axes[a] > limits.maximumAxes[a]
                || ((limits.rotaryMask & (1u << a))
                    && std::abs(node.axes[a] - previous[a]) > limits.maxRotaryStepDeg)) return false;
        }
        previous = node.axes;
    }
    return true;
}

using ReductionCost = std::tuple<int, double, double, int, double, int, int, unsigned, int>;

ReductionCost selectionCost(const CamMotionBlock& block, const Full5DPolicy& limits, int ordinal)
{
    int stops = 0, reversals = 0;
    double duration = 0, limitRisk = 0, normalizedTravel = 0;
    for (const auto& fence : block.fences) if (!fence.laserEnabledAfterFence) ++stops;
    for (const auto& node : block.physicalKnots) duration += node.estimatedTimeMs;
    const auto axes = analyze(block);
    const auto& first = firstNode(block);
    for (std::size_t a = 0; a < axes.size(); ++a) {
        if (!(first.axisMask & (1u << a))) continue;
        const double range = limits.maximumAxes[a] - limits.minimumAxes[a];
        const bool finiteRange = std::isfinite(range) && range > 0;
        const double scale = finiteRange ? range : std::max(1.0, std::abs(first.axes[a]));
        normalizedTravel += axes[a].travel / scale;
        if (limits.rotaryMask & (1u << a)) reversals += axes[a].reversals;
        if (finiteRange) {
            const auto proximity = [&](double value) {
                return 1.0 - std::min(value - limits.minimumAxes[a], limits.maximumAxes[a] - value) / range;
            };
            limitRisk = std::max(limitRisk, proximity(first.axes[a]));
            for (const auto& node : block.physicalKnots) limitRisk = std::max(limitRisk, proximity(node.axes[a]));
        }
    }
    // All admitted S3 candidates consume zero process error budget. Numerical
    // rounding is reported separately. Axis count is only a late tie-break.
    return {stops, duration, limitRisk, reversals, normalizedTravel,
        static_cast<int>(block.physicalKnots.size()), axisCount(block.activeAxisMask), block.activeAxisMask, ordinal};
}
} // namespace

bool reduceMotionPlan(const lcnc::cam::CamMotionPlanSnapshot& reference,
    const Full5DPolicy& limits, const ReductionPolicy& policy,
    const std::function<ReductionEvaluation(const CamMotionBlock&)>& evaluation,
    CamMotionPlanSnapshot* selected, ReductionReport* report, QString* error,
    const std::function<bool()>& cancelled)
{
    const auto fail = [&](const QString& reason) { if (error) *error = reason; return false; };
    if (!selected || !report || !finalMotionPlanIdentityIsCurrent(reference))
        return fail(QStringLiteral("Reduction reference identity is stale"));
    if (policy.maximumCandidates < 1 || policy.maximumCandidates > 32 || policy.zAxis < -1
        || policy.zAxis >= MachineAxisLayout::kMaxAxes || policy.mode != limits.mode
        || (policy.mode != PoseOptimizationMode::Off && policy.mode != PoseOptimizationMode::Conservative
            && policy.mode != PoseOptimizationMode::Full)) return fail(QStringLiteral("Invalid reduction policy"));
    if (!std::isfinite(limits.maxRotaryStepDeg) || limits.maxRotaryStepDeg <= 0)
        return fail(QStringLiteral("Invalid rotary step"));
    for (std::size_t a = 0; a < limits.minimumAxes.size(); ++a)
        if (std::isnan(limits.minimumAxes[a]) || std::isnan(limits.maximumAxes[a])
            || limits.minimumAxes[a] > limits.maximumAxes[a]) return fail(QStringLiteral("Invalid axis limits"));
    QElapsedTimer timer; timer.start();
    auto result = reference;
    result.optimizerReport.reduction.clear();
    ReductionReport audit;
    for (int index = 0; index < reference.blocks.size(); ++index) {
        if (cancelled && cancelled()) return fail(QStringLiteral("Reduction cancelled"));
        const auto& source = reference.blocks[index];
        if (source.optimizationState != MotionOptimizationState::Optimized
            || source.interpolation != MotionInterpolationKind::PhysicalAxisLine)
            return fail(QStringLiteral("Reduction requires Optimized physical Full5D reference"));
        if (!withinLimits(source, limits)) return fail(QStringLiteral("Reference hard limit exceeded"));
        ReductionBlockReport blockReport;
        blockReport.blockId = source.blockId; blockReport.axes = analyze(source);
        blockReport.selectedMask = source.activeAxisMask;
        const auto reject = [&](const QString& reason) {
            if (!blockReport.rejectedReasons.contains(reason)) blockReport.rejectedReasons.append(reason);
        };
        const unsigned layoutMask = firstNode(source).axisMask;
        if (!layoutMask || layoutMask > 31) return fail(QStringLiteral("Invalid physical layout"));
        for (const auto& node : source.physicalKnots)
            if (node.axisMask != layoutMask) return fail(QStringLiteral("Physical layout changed inside block"));
        if (policy.mode != PoseOptimizationMode::Off) {
            const auto model = evaluation ? evaluation(source) : ReductionEvaluation{};
            if (policy.enableLaserZHold)
                reject(policy.mode == PoseOptimizationMode::Full
                    ? QStringLiteral("Z process envelope missing: no qualified whitelist authority")
                    : QStringLiteral("Process Z-hold requires Full mode"));
            auto numerical = source;
            bool zChanged = false;
            double zBudget = 0;
            NumericalZBound zProof;
            if (policy.zAxis >= 0 && (layoutMask & (1u << policy.zAxis))
                && !(limits.rotaryMask & (1u << policy.zAxis))) {
                const int z = policy.zAxis;
                const double anchor = firstNode(source).axes[z];
                zBudget = ReductionPolicy::numericalUlps * std::numeric_limits<double>::epsilon()
                    * std::max(1.0, std::abs(anchor));
                if (blockReport.axes[z].freezeResidual > 0 && blockReport.axes[z].freezeResidual <= zBudget) {
                    // Both canonical boundaries stay fixed, so the next block's
                    // incoming edge cannot change behind its proof/owner.
                    if (source.physicalKnots.last().axes[z] != anchor)
                        reject(QStringLiteral("Numerical Z boundary-hold conflict"));
                    else {
                        for (auto& node : numerical.physicalKnots) node.axes[z] = anchor;
                        if (model.boundNumericalZ) zProof = model.boundNumericalZ(source, numerical, z);
                        zChanged = zProof.valid && std::isfinite(zProof.maximumPositionDeviationMm)
                            && zProof.maximumPositionDeviationMm >= 0 && zProof.maximumPositionDeviationMm <= zBudget
                            && zProof.maximumOrientationDeviationDegrees == 0 && withinLimits(numerical, limits);
                        if (!zChanged) reject(QStringLiteral("Numerical Z continuous proof unavailable"));
                    }
                }
            }
            unsigned movingMask = 0;
            const auto analyzed = analyze(zChanged ? numerical : source);
            for (std::size_t a = 0; a < analyzed.size(); ++a)
                if ((layoutMask & (1u << a)) && analyzed[a].span != 0) movingMask |= 1u << a;
            auto winner = source;
            const auto referenceCost = selectionCost(source, limits, 1);
            auto bestCost = referenceCost;
            // All lower-dimensional physical subsets, including C-only and U+C.
            // Exact constant axes need no approximate threshold: affine knots
            // and their incoming endpoint prove a hold on every subinterval.
            for (unsigned mask = 1; mask <= layoutMask; ++mask) {
                if (cancelled && cancelled()) return fail(QStringLiteral("Reduction cancelled"));
                if ((mask & layoutMask) != mask || (mask & movingMask) != movingMask) continue;
                const bool reduced = mask != layoutMask;
                if ((reduced && !policy.enableDofReduction) || (!reduced && !zChanged)) continue;
                if (blockReport.candidates >= policy.maximumCandidates) {
                    reject(QStringLiteral("Compute budget exhausted"));
                    winner = source;
                    bestCost = referenceCost;
                    break;
                }
                ++blockReport.candidates;
                const QString denied = admissionError(reference.context, model.admission, mask);
                if (!denied.isEmpty()) { reject(denied); continue; }
                auto candidate = zChanged ? numerical : source;
                if (candidate.hasEntryBoundary) candidate.entryBoundary = result.blocks[index].entryBoundary;
                candidate.activeAxisMask = static_cast<std::uint8_t>(mask);
                candidate.motionClass = motionClass(mask);
                candidate.optimizationState = reduced ? MotionOptimizationState::Reduced : MotionOptimizationState::Optimized;
                const auto cost = selectionCost(candidate, limits, zChanged ? 0 : 1);
                if (cost >= bestCost) continue;
                winner = std::move(candidate); bestCost = cost;
            }
            if (bestCost < referenceCost) {
                // Evaluate a finalized candidate through the same physical FK.
                // Entry and terminal axes have not changed; source/fence/time
                // metadata is copied verbatim, never reconstructed heuristically.
                auto candidatePlan = result;
                candidatePlan.blocks[index] = winner;
                QString rebuildError;
                bool rebuildCancelled = false;
                const bool rebuilt = [&] {
                    if (!finalizeMotionPlan(&candidatePlan, &rebuildError)) return false;
                    BoundMotionEvaluationContext bound;
                    if (!bindMotionEvaluationContext(candidatePlan, model.motion, &bound, &rebuildError)) return false;
                    ContinuousMotionEvaluator evaluator;
                    const auto frozen = candidatePlan.blocks[index];
                    QVector<EvaluatedMotionState> states;
                    const auto checkCancelled = [&] {
                        rebuildCancelled = cancelled && cancelled();
                        return rebuildCancelled;
                    };
                    if (!evaluator.evaluatePhysicalKnots(frozen, bound, &states, &rebuildError, checkCancelled)) return false;
                    for (int knot = 0; knot < winner.physicalKnots.size(); ++knot) {
                        if (cancelled && cancelled()) { rebuildCancelled = true; return false; }
                        const auto& state = states[knot];
                        auto& node = winner.physicalKnots[knot];
                        node.tcpX = state.worldTcpX; node.tcpY = state.worldTcpY; node.tcpZ = state.worldTcpZ;
                        node.referenceTcpX = state.referenceTcpX; node.referenceTcpY = state.referenceTcpY;
                        node.referenceTcpZ = state.referenceTcpZ; node.referenceTcpValid = state.referenceTcpValid;
                        const double length = std::hypot(state.processDirectionX, state.processDirectionY, state.processDirectionZ);
                        if (!std::isfinite(length) || length == 0) {
                            rebuildError = QStringLiteral("Invalid derived direction"); return false;
                        }
                        node.normalX = state.processDirectionX / length; node.normalY = state.processDirectionY / length;
                        node.normalZ = state.processDirectionZ / length;
                    }
                    return true;
                }();
                if (rebuildCancelled) return fail(QStringLiteral("Reduction cancelled"));
                if (!rebuilt) reject(QStringLiteral("Candidate evaluator rejected: ") + rebuildError);
                else {
                    winner.toleranceProof.maximumPositionDeviationMm = std::max(
                        winner.toleranceProof.maximumPositionDeviationMm, zChanged ? zProof.maximumPositionDeviationMm : 0);
                    winner.toleranceProof.exactKnots = winner.toleranceProof.exactKnots && !zChanged;
                    result.blocks[index] = winner;
                    if (index + 1 < result.blocks.size()) result.blocks[index + 1].entryBoundary = winner.physicalKnots.last();
                    blockReport.selectedMask = winner.activeAxisMask; blockReport.numericalZApplied = zChanged;
                    audit.changed = true;
                }
            }
        }
        for (const auto& reason : blockReport.rejectedReasons)
            result.optimizerReport.rejectedCandidateReasons.append(QStringLiteral("block %1: %2").arg(source.blockId).arg(reason));
        audit.blocks.append(blockReport);
        MotionOptimizerReport::ReductionSummary summary;
        summary.blockId = blockReport.blockId; summary.selectedMask = blockReport.selectedMask;
        summary.candidates = blockReport.candidates; summary.numericalZApplied = blockReport.numericalZApplied;
        summary.rejectedReasons = blockReport.rejectedReasons;
        for (std::size_t a = 0; a < blockReport.axes.size(); ++a) summary.axisSpans[a] = blockReport.axes[a].span;
        result.optimizerReport.reduction.append(summary);
    }
    if (cancelled && cancelled()) return fail(QStringLiteral("Reduction cancelled"));
    result.optimizerReport.selectedClass = MotionClass::SingleAxis;
    for (const auto& block : result.blocks) {
        result.optimizerReport.selectedClass = std::max(result.optimizerReport.selectedClass, block.motionClass);
        result.optimizerReport.maximumPositionDeviationMm = std::max(result.optimizerReport.maximumPositionDeviationMm,
            block.toleranceProof.maximumPositionDeviationMm);
    }
    if (!finalizeMotionPlan(&result, error)) return false;
    std::array<double, MachineAxisLayout::kMaxAxes> low, high;
    low.fill(std::numeric_limits<double>::infinity());
    high.fill(-std::numeric_limits<double>::infinity());
    for (const auto& node : result.nodes)
        for (std::size_t a = 0; a < node.axes.size(); ++a) if (node.axisMask & (1u << a)) {
            low[a] = std::min(low[a], node.axes[a]); high[a] = std::max(high[a], node.axes[a]);
        }
    for (std::size_t a = 0; a < low.size(); ++a)
        result.optimizerReport.axisSpans[a] = std::isfinite(low[a]) ? high[a] - low[a] : 0;
    if (result.planHash != reference.planHash) {
        result.edgeCertificates.clear(); result.collision.key = 0;
        result.collision.nodeStates.clear(); result.collision.intervals.clear();
        result.collision.complete = result.context.collisionMode == CollisionVerificationMode::Disabled;
        result.collision.state = result.collision.complete ? CollisionValidationState::Disabled : CollisionValidationState::Pending;
    }
    audit.compileTimeMs = timer.elapsed();
    result.optimizerReport.reductionCompileTimeMs = audit.compileTimeMs;
    *selected = std::move(result); *report = std::move(audit);
    return true;
}
} // namespace lcnc::cam_algo
