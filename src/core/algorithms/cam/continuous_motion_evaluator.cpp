#include "core/algorithms/cam/continuous_motion_evaluator.h"

#include <algorithm>
#include <cmath>

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
        && std::isfinite(state.sourceParameter);
}

} // namespace

bool ContinuousMotionEvaluator::evaluate(
    const lcnc::cam::CamMotionBlock& block, double u,
    const MotionEvaluationContext& context, EvaluatedMotionState* state,
    QString* errorMessage) const
{
    if (!state)
        return fail(errorMessage, QStringLiteral("ContinuousMotionEvaluator output is null"));
    if (!validParameter(u))
        return fail(errorMessage, QStringLiteral("ContinuousMotionEvaluator parameter is outside [0,1]"));
    if (context.interpolationModelVersion == 0)
        return fail(errorMessage, QStringLiteral("ContinuousMotionEvaluator model version is invalid"));
    if (block.physicalKnots.isEmpty())
        return fail(errorMessage, QStringLiteral("Motion block has no physical knots"));

    EvaluatedMotionState evaluated;
    bool ok = false;
    if (block.interpolation == lcnc::cam::MotionInterpolationKind::PhysicalAxisLine) {
        if (!context.evaluatePhysicalAxes)
            return fail(errorMessage, QStringLiteral("Physical-axis kinematics evaluator is unavailable"));
        const int segmentCount = std::max(
            1, static_cast<int>(block.physicalKnots.size()) - 1);
        const double scaled = u * segmentCount;
        const int firstIndex = std::min(
            static_cast<int>(std::floor(scaled)), segmentCount - 1);
        const int lastIndex = block.physicalKnots.size() == 1
            ? firstIndex : firstIndex + 1;
        const double local = block.physicalKnots.size() == 1
            ? 0.0 : scaled - firstIndex;
        const auto& first = block.physicalKnots.at(firstIndex);
        const auto& last = block.physicalKnots.at(lastIndex);
        std::array<double, MachineAxisLayout::kMaxAxes> axes{};
        for (std::size_t axis = 0; axis < axes.size(); ++axis)
            axes[axis] = first.axes[axis] + (last.axes[axis] - first.axes[axis]) * local;
        const std::uint8_t mask = first.axisMask | last.axisMask;
        ok = context.evaluatePhysicalAxes(axes, mask, &evaluated);
        evaluated.physicalAxes = axes;
        evaluated.physicalAxisMask = mask;
        evaluated.sourceParameter = u;
    } else if (block.interpolation == lcnc::cam::MotionInterpolationKind::RtcpLine) {
        if (!context.evaluateRtcp)
            return fail(errorMessage, QStringLiteral("Qualified RTCP interpolation evaluator is unavailable"));
        ok = context.evaluateRtcp(block, u, &evaluated);
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
    const MotionEvaluationContext& context, MotionIntervalBound* result,
    QString* errorMessage) const
{
    if (!result)
        return fail(errorMessage, QStringLiteral("ContinuousMotionEvaluator bound output is null"));
    if (!validParameter(u0) || !validParameter(u1) || u1 < u0)
        return fail(errorMessage, QStringLiteral("ContinuousMotionEvaluator bound interval is invalid"));
    if (context.interpolationModelVersion == 0)
        return fail(errorMessage, QStringLiteral("ContinuousMotionEvaluator model version is invalid"));

    MotionIntervalBound bounded;
    bool ok = false;
    if (block.interpolation == lcnc::cam::MotionInterpolationKind::PhysicalAxisLine) {
        if (!context.boundPhysicalAxes)
            return fail(errorMessage, QStringLiteral("Conservative physical-axis bound model is unavailable"));
        ok = context.boundPhysicalAxes(block, u0, u1, &bounded);
    } else if (block.interpolation == lcnc::cam::MotionInterpolationKind::RtcpLine) {
        if (!context.boundRtcp)
            return fail(errorMessage, QStringLiteral("Conservative qualified RTCP bound model is unavailable"));
        ok = context.boundRtcp(block, u0, u1, &bounded);
    } else {
        return fail(errorMessage, QStringLiteral("Motion block interpolation model is unsupported"));
    }
    if (!ok || !bounded.valid || !std::isfinite(bounded.maximumPositionErrorMm)
        || !std::isfinite(bounded.maximumOrientationErrorDegrees)) {
        return fail(errorMessage, QStringLiteral("Motion interpolation interval could not be conservatively bounded"));
    }
    *result = bounded;
    if (errorMessage)
        errorMessage->clear();
    return true;
}

} // namespace lcnc::cam_algo
