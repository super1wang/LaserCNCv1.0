#include "core/algorithms/cam/continuous_motion_evaluator.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace lcnc::cam_algo {
namespace {

bool fail(QString* errorMessage, const QString& reason)
{
    if (errorMessage)
        *errorMessage = reason;
    return false;
}

bool validParameter(double u)
{
    return std::isfinite(u) && u >= 0.0 && u <= 1.0;
}

bool validState(const EvaluatedMotionState& state)
{
    for (double axis : state.physicalAxes) {
        if (!std::isfinite(axis))
            return false;
    }
    return std::isfinite(state.worldTcpX) && std::isfinite(state.worldTcpY)
        && std::isfinite(state.worldTcpZ)
        && std::isfinite(state.processDirectionX)
        && std::isfinite(state.processDirectionY)
        && std::isfinite(state.processDirectionZ)
        && (!state.referenceTcpValid
            || (std::isfinite(state.referenceTcpX)
                && std::isfinite(state.referenceTcpY)
                && std::isfinite(state.referenceTcpZ)))
        && validParameter(state.sourceParameter);
}

bool validBound(const MotionIntervalBound& bound)
{
    for (std::size_t axis = 0; axis < bound.minimumPhysicalAxes.size(); ++axis) {
        if (!std::isfinite(bound.minimumPhysicalAxes[axis])
            || !std::isfinite(bound.maximumPhysicalAxes[axis])
            || bound.minimumPhysicalAxes[axis] > bound.maximumPhysicalAxes[axis]) {
            return false;
        }
    }
    const std::array<double, 6> tcpBounds{
        bound.minimumWorldTcpX, bound.minimumWorldTcpY, bound.minimumWorldTcpZ,
        bound.maximumWorldTcpX, bound.maximumWorldTcpY, bound.maximumWorldTcpZ};
    for (double value : tcpBounds) {
        if (!std::isfinite(value))
            return false;
    }
    return bound.minimumWorldTcpX <= bound.maximumWorldTcpX
        && bound.minimumWorldTcpY <= bound.maximumWorldTcpY
        && bound.minimumWorldTcpZ <= bound.maximumWorldTcpZ
        && std::isfinite(bound.maximumPositionErrorMm)
        && bound.maximumPositionErrorMm >= 0.0
        && std::isfinite(bound.maximumOrientationErrorDegrees)
        && bound.maximumOrientationErrorDegrees >= 0.0;
}

} // namespace

bool BoundMotionEvaluationContext::supportsBlock(
    const lcnc::cam::CamMotionBlock& block) const
{
    return !block.blockHash.isEmpty()
        && block.blockHash == lcnc::cam::motionBlockHash(block)
        && m_blockHashes.contains(block.blockHash);
}

bool bindMotionEvaluationContext(
    const lcnc::cam::CamMotionPlanSnapshot& plan,
    MotionEvaluationContext callbacks,
    BoundMotionEvaluationContext* bound,
    QString* errorMessage)
{
    if (!bound)
        return fail(errorMessage, QStringLiteral("Bound motion evaluation context output is null"));
    if (!lcnc::cam::finalMotionPlanIdentityIsCurrent(plan))
        return fail(errorMessage, QStringLiteral("Motion evaluation plan identity is stale"));
    if (callbacks.interpolationModelVersion == 0
        || callbacks.interpolationModelVersion
            != plan.context.interpolationModelVersion) {
        return fail(errorMessage, QStringLiteral("Motion evaluation model version does not match the plan"));
    }
    BoundMotionEvaluationContext candidate;
    candidate.m_planHash = plan.planHash;
    candidate.m_contextHash = plan.contextHash;
    candidate.m_callbacks = std::move(callbacks);
    candidate.m_blockHashes.reserve(plan.blocks.size());
    for (const auto& block : plan.blocks)
        candidate.m_blockHashes.append(block.blockHash);
    *bound = std::move(candidate);
    if (errorMessage)
        errorMessage->clear();
    return true;
}

bool ContinuousMotionEvaluator::evaluate(
    const lcnc::cam::CamMotionBlock& block, double u,
    const BoundMotionEvaluationContext& context, EvaluatedMotionState* state,
    QString* errorMessage) const
{
    if (!state)
        return fail(errorMessage, QStringLiteral("ContinuousMotionEvaluator output is null"));
    if (!validParameter(u))
        return fail(errorMessage, QStringLiteral("ContinuousMotionEvaluator parameter is outside [0,1]"));
    if (!context.supportsBlock(block))
        return fail(errorMessage, QStringLiteral("Motion block is not part of the bound plan identity"));
    if (block.physicalKnots.isEmpty())
        return fail(errorMessage, QStringLiteral("Motion block has no physical knots"));

    const auto& callbacks = context.m_callbacks;
    EvaluatedMotionState evaluated;
    bool ok = false;
    if (block.interpolation == lcnc::cam::MotionInterpolationKind::PhysicalAxisLine) {
        if (!callbacks.evaluatePhysicalAxes)
            return fail(errorMessage, QStringLiteral("Physical-axis kinematics evaluator is unavailable"));
        const int evaluableKnots = block.physicalKnots.size()
            + (block.hasEntryBoundary ? 1 : 0);
        const int segmentCount = std::max(1, evaluableKnots - 1);
        const double scaled = u * segmentCount;
        const int firstIndex = std::min(
            static_cast<int>(std::floor(scaled)), segmentCount - 1);
        const int lastIndex = evaluableKnots == 1
            ? firstIndex : firstIndex + 1;
        const double local = evaluableKnots == 1
            ? 0.0 : scaled - firstIndex;
        const auto knotAt = [&block](int index) -> const lcnc::cam::CamMotionNode& {
            return block.hasEntryBoundary && index == 0
                ? block.entryBoundary
                : block.physicalKnots.at(index - (block.hasEntryBoundary ? 1 : 0));
        };
        const auto& first = knotAt(firstIndex);
        const auto& last = knotAt(lastIndex);
        std::array<double, MachineAxisLayout::kMaxAxes> axes{};
        for (std::size_t axis = 0; axis < axes.size(); ++axis)
            axes[axis] = first.axes[axis] + (last.axes[axis] - first.axes[axis]) * local;
        const std::uint8_t mask = first.axisMask | last.axisMask;
        ok = callbacks.evaluatePhysicalAxes(axes, mask, &evaluated);
        evaluated.physicalAxes = axes;
        evaluated.physicalAxisMask = mask;
        evaluated.sourceParameter = u;
    } else if (block.interpolation == lcnc::cam::MotionInterpolationKind::RtcpLine) {
        if (!callbacks.evaluateRtcp)
            return fail(errorMessage, QStringLiteral("Qualified RTCP interpolation evaluator is unavailable"));
        ok = callbacks.evaluateRtcp(block, u, &evaluated);
        evaluated.sourceParameter = u;
    } else {
        return fail(errorMessage, QStringLiteral("Motion block interpolation model is unsupported"));
    }
    if (!ok || !validState(evaluated))
        return fail(errorMessage, QStringLiteral("Motion interpolation evaluation failed"));
    *state = evaluated;
    if (errorMessage)
        errorMessage->clear();
    return true;
}

bool ContinuousMotionEvaluator::bound(
    const lcnc::cam::CamMotionBlock& block, double u0, double u1,
    const BoundMotionEvaluationContext& context, MotionIntervalBound* result,
    QString* errorMessage) const
{
    if (!result)
        return fail(errorMessage, QStringLiteral("ContinuousMotionEvaluator bound output is null"));
    if (!validParameter(u0) || !validParameter(u1) || u1 < u0)
        return fail(errorMessage, QStringLiteral("ContinuousMotionEvaluator bound interval is invalid"));
    if (!context.supportsBlock(block))
        return fail(errorMessage, QStringLiteral("Motion block is not part of the bound plan identity"));

    const auto& callbacks = context.m_callbacks;
    MotionIntervalBound bounded;
    bool ok = false;
    if (block.interpolation == lcnc::cam::MotionInterpolationKind::PhysicalAxisLine) {
        if (!callbacks.boundPhysicalAxes)
            return fail(errorMessage, QStringLiteral("Conservative physical-axis bound model is unavailable"));
        ok = callbacks.boundPhysicalAxes(block, u0, u1, &bounded);
    } else if (block.interpolation == lcnc::cam::MotionInterpolationKind::RtcpLine) {
        if (!callbacks.boundRtcp)
            return fail(errorMessage, QStringLiteral("Conservative qualified RTCP bound model is unavailable"));
        ok = callbacks.boundRtcp(block, u0, u1, &bounded);
    } else {
        return fail(errorMessage, QStringLiteral("Motion block interpolation model is unsupported"));
    }
    if (!ok || !bounded.valid || !validBound(bounded)) {
        return fail(errorMessage, QStringLiteral("Motion interpolation interval could not be conservatively bounded"));
    }
    *result = bounded;
    if (errorMessage)
        errorMessage->clear();
    return true;
}

} // namespace lcnc::cam_algo
