#include "core/project/cam/collision_validation_contracts.h"

#include <QCryptographicHash>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace lcnc::cam {
namespace {

void appendBytes(QByteArray* bytes, const QByteArray& value)
{
    bytes->append(QByteArray::number(value.size()));
    bytes->append(':');
    bytes->append(value);
    bytes->append(';');
}

void appendString(QByteArray* bytes, const QString& value)
{
    appendBytes(bytes, value.toUtf8());
}

template <typename Integer>
void appendInteger(QByteArray* bytes, Integer value)
{
    bytes->append(QByteArray::number(static_cast<qulonglong>(value)));
    bytes->append(';');
}

void appendDouble(QByteArray* bytes, double value)
{
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    bytes->append(QByteArray::number(bits, 16));
    bytes->append(';');
}

QByteArray sha256(const QByteArray& canonical)
{
    return QCryptographicHash::hash(canonical, QCryptographicHash::Sha256);
}

bool validNode(const CamMotionNode& node)
{
    if (!std::isfinite(node.tcpX) || !std::isfinite(node.tcpY)
        || !std::isfinite(node.tcpZ) || !std::isfinite(node.normalX)
        || !std::isfinite(node.normalY) || !std::isfinite(node.normalZ)
        || !std::isfinite(node.estimatedTimeMs)
        || !std::isfinite(node.sourceParameter)) {
        return false;
    }
    if (node.referenceTcpValid
        && (!std::isfinite(node.referenceTcpX)
            || !std::isfinite(node.referenceTcpY)
            || !std::isfinite(node.referenceTcpZ))) {
        return false;
    }
    for (double axis : node.axes) {
        if (!std::isfinite(axis))
            return false;
    }
    return true;
}

bool sameNode(const CamMotionNode& lhs, const CamMotionNode& rhs)
{
    return lhs.phase == rhs.phase && lhs.rapidPhase == rhs.rapidPhase
        && lhs.contourId == rhs.contourId
        && lhs.sourceEdgeIndex == rhs.sourceEdgeIndex
        && lhs.sourceParameter == rhs.sourceParameter
        && lhs.axes == rhs.axes
        && lhs.axisMask == rhs.axisMask && lhs.tcpX == rhs.tcpX
        && lhs.tcpY == rhs.tcpY && lhs.tcpZ == rhs.tcpZ
        && lhs.referenceTcpX == rhs.referenceTcpX
        && lhs.referenceTcpY == rhs.referenceTcpY
        && lhs.referenceTcpZ == rhs.referenceTcpZ
        && lhs.referenceTcpValid == rhs.referenceTcpValid
        && lhs.normalX == rhs.normalX && lhs.normalY == rhs.normalY
        && lhs.normalZ == rhs.normalZ
        && lhs.estimatedTimeMs == rhs.estimatedTimeMs;
}

void appendNode(QByteArray* canonical, const CamMotionNode& node)
{
    appendInteger(canonical, node.phase);
    appendInteger(canonical, node.rapidPhase);
    appendInteger(canonical, node.contourId);
    appendInteger(canonical, node.sourceEdgeIndex);
    appendDouble(canonical, node.sourceParameter);
    appendInteger(canonical, node.axisMask);
    for (double axis : node.axes)
        appendDouble(canonical, axis);
    appendDouble(canonical, node.tcpX);
    appendDouble(canonical, node.tcpY);
    appendDouble(canonical, node.tcpZ);
    appendDouble(canonical, node.referenceTcpX);
    appendDouble(canonical, node.referenceTcpY);
    appendDouble(canonical, node.referenceTcpZ);
    appendInteger(canonical, node.referenceTcpValid);
    appendDouble(canonical, node.normalX);
    appendDouble(canonical, node.normalY);
    appendDouble(canonical, node.normalZ);
    appendDouble(canonical, node.estimatedTimeMs);
}

} // namespace

QByteArray motionCompilationContextHash(const MotionCompilationContext& context)
{
    QByteArray canonical("lcnc.motion-context.v1;");
    appendInteger(&canonical, context.workspaceGeneration);
    appendInteger(&canonical, context.sourceToolpathRevision);
    appendBytes(&canonical, context.contourOrderHash);
    appendBytes(&canonical, context.machineKinematicsHash);
    appendBytes(&canonical, context.setupCalibrationHash);
    appendBytes(&canonical, context.toolProcessHash);
    appendBytes(&canonical, context.optimizationPolicyHash);
    appendInteger(&canonical, context.controllerMode);
    appendBytes(&canonical, context.controllerCapabilityHash);
    appendBytes(&canonical, context.dynamicsSemanticHash);
    appendInteger(&canonical, context.collisionMode);
    appendInteger(&canonical, context.interpolationModelVersion);
    return sha256(canonical);
}

QByteArray motionBlockHash(const CamMotionBlock& block)
{
    QByteArray canonical("lcnc.motion-block.v3;");
    appendInteger(&canonical, block.blockId);
    appendInteger(&canonical, block.phase);
    appendInteger(&canonical, block.contourId);
    appendInteger(&canonical, block.motionClass);
    appendInteger(&canonical, block.optimizationState);
    appendInteger(&canonical, block.interpolation);
    appendInteger(&canonical, block.activeAxisMask);
    appendInteger(&canonical, block.hasEntryBoundary);
    if (block.hasEntryBoundary)
        appendNode(&canonical, block.entryBoundary);
    appendInteger(&canonical, block.physicalKnots.size());
    for (const CamMotionNode& node : block.physicalKnots)
        appendNode(&canonical, node);
    appendInteger(&canonical, block.sourceSpans.size());
    for (const MotionSourceSpan& span : block.sourceSpans) {
        appendInteger(&canonical, span.contourId);
        appendInteger(&canonical, span.firstKnot);
        appendInteger(&canonical, span.lastKnot);
        appendDouble(&canonical, span.firstSourceParameter);
        appendDouble(&canonical, span.lastSourceParameter);
        appendInteger(&canonical, span.sourceEdgeIndex);
    }
    appendInteger(&canonical, block.fences.size());
    for (const MotionProcessFence& fence : block.fences) {
        appendInteger(&canonical, fence.knotIndex);
        appendInteger(&canonical, fence.blockStart);
        appendInteger(&canonical, fence.blockEnd);
        appendInteger(&canonical, fence.laserEnabledAfterFence);
    }
    appendDouble(&canonical, block.feed.nominalFeedPerMinute);
    appendDouble(&canonical, block.feed.estimatedDurationMs);
    appendBytes(&canonical, block.feed.profileHash);
    appendDouble(&canonical, block.toleranceProof.maximumPositionDeviationMm);
    appendDouble(&canonical,
                 block.toleranceProof.maximumOrientationDeviationDegrees);
    appendDouble(&canonical, block.toleranceProof.maximumRotaryDeviationDegrees);
    appendInteger(&canonical, block.toleranceProof.exactKnots);
    return sha256(canonical);
}

QByteArray finalMotionPlanHash(const CamMotionPlanSnapshot& plan)
{
    QByteArray canonical("lcnc.final-motion-plan.v2;");
    appendBytes(&canonical, motionCompilationContextHash(plan.context));
    appendString(&canonical, plan.solverId);
    appendInteger(&canonical, plan.solverVersion);
    appendInteger(&canonical, plan.blocks.size());
    for (const CamMotionBlock& block : plan.blocks)
        appendBytes(&canonical, motionBlockHash(block));
    return sha256(canonical);
}

bool finalizeMotionPlan(CamMotionPlanSnapshot* plan, QString* errorMessage)
{
    const auto fail = [errorMessage](const QString& reason) {
        if (errorMessage)
            *errorMessage = reason;
        return false;
    };
    if (!plan)
        return fail(QStringLiteral("FinalMotionPlan output is null"));
    if (plan->context.interpolationModelVersion == 0)
        return fail(QStringLiteral("FinalMotionPlan interpolation model version is invalid"));
    if (plan->blocks.isEmpty())
        return fail(QStringLiteral("FinalMotionPlan has no motion blocks"));

    QVector<CamMotionBlock> blocks = plan->blocks;
    QVector<CamMotionNode> derivedNodes;
    for (int blockIndex = 0; blockIndex < blocks.size(); ++blockIndex) {
        CamMotionBlock& block = blocks[blockIndex];
        if (block.physicalKnots.isEmpty())
            return fail(QStringLiteral("FinalMotionPlan block has no physical knots"));
        if (blockIndex == 0 && block.hasEntryBoundary)
            return fail(QStringLiteral("FinalMotionPlan first block has an unexpected entry boundary"));
        if (blockIndex > 0) {
            if (!block.hasEntryBoundary)
                return fail(QStringLiteral("FinalMotionPlan block is missing its canonical entry edge"));
            if (!validNode(block.entryBoundary)
                || !sameNode(block.entryBoundary,
                             blocks.at(blockIndex - 1).physicalKnots.constLast())) {
                return fail(QStringLiteral("FinalMotionPlan block entry boundary does not match its predecessor"));
            }
            const bool sourceCoversEntry = std::any_of(
                block.sourceSpans.cbegin(), block.sourceSpans.cend(),
                [](const MotionSourceSpan& span) { return span.firstKnot == -1; });
            const bool fencePrecedesEntryEdge = std::any_of(
                block.fences.cbegin(), block.fences.cend(),
                [](const MotionProcessFence& fence) {
                    return fence.knotIndex == -1 && fence.blockStart;
                });
            if (!sourceCoversEntry || !fencePrecedesEntryEdge) {
                return fail(QStringLiteral(
                    "FinalMotionPlan block entry edge is missing source-span or process-fence semantics"));
            }
        }
        for (const CamMotionNode& node : block.physicalKnots) {
            if (!validNode(node))
                return fail(QStringLiteral("FinalMotionPlan contains a non-finite motion value"));
            if (node.phase != block.phase || node.contourId != block.contourId)
                return fail(QStringLiteral("FinalMotionPlan block metadata disagrees with a physical knot"));
        }
        if (!std::isfinite(block.feed.nominalFeedPerMinute)
            || !std::isfinite(block.feed.estimatedDurationMs)
            || !std::isfinite(block.toleranceProof.maximumPositionDeviationMm)
            || !std::isfinite(
                block.toleranceProof.maximumOrientationDeviationDegrees)
            || !std::isfinite(
                block.toleranceProof.maximumRotaryDeviationDegrees)) {
            return fail(QStringLiteral("FinalMotionPlan block semantics contain a non-finite value"));
        }
        block.blockHash = motionBlockHash(block);
        derivedNodes += block.physicalKnots;
    }

    CamMotionPlanSnapshot candidate = *plan;
    candidate.blocks = std::move(blocks);
    candidate.contextHash = motionCompilationContextHash(candidate.context);
    candidate.planHash = finalMotionPlanHash(candidate);
    candidate.nodes = std::move(derivedNodes);
    candidate.derivedFromPlanHash = candidate.planHash;
    candidate.failureReason.clear();
    *plan = std::move(candidate);
    if (errorMessage)
        errorMessage->clear();
    return true;
}

bool finalMotionPlanIdentityIsCurrent(const CamMotionPlanSnapshot& plan)
{
    if (plan.blocks.isEmpty() || plan.contextHash.isEmpty() || plan.planHash.isEmpty()
        || plan.derivedFromPlanHash != plan.planHash
        || plan.contextHash != motionCompilationContextHash(plan.context)
        || plan.planHash != finalMotionPlanHash(plan)) {
        return false;
    }
    int projectedNodes = 0;
    for (const CamMotionBlock& block : plan.blocks) {
        if (block.blockHash != motionBlockHash(block))
            return false;
        for (const CamMotionNode& node : block.physicalKnots) {
            if (projectedNodes >= plan.nodes.size()
                || !sameNode(node, plan.nodes.at(projectedNodes))) {
                return false;
            }
            ++projectedNodes;
        }
    }
    return projectedNodes == plan.nodes.size();
}

} // namespace lcnc::cam
