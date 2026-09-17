#include "core/algorithms/cam/dof_reduction.h"

#include <QCoreApplication>
#include <QTextStream>

#include <cmath>
#include <algorithm>

using namespace lcnc::cam;
using namespace lcnc::cam_algo;

namespace {
CamMotionPlanSnapshot fixture(int count = 3)
{
    CamMotionPlanSnapshot plan;
    // Test-only qualification fixture, never a production controller source.
    plan.context.controllerQualification = {ControllerMotionMode::PhysicalAxes,
        ControllerQualificationState::Qualified, 7, "test-hold-capability", QStringLiteral("test-only/qualification")};
    plan.context.controllerCapabilityHash = controllerQualificationSnapshotHash(plan.context.controllerQualification);
    plan.context.dynamicsSemanticHash = "test-feed";
    plan.context.optimizationPolicyHash = "test-reduction-v1";
    CamMotionBlock block;
    block.blockId = 1; block.contourId = 1; block.activeAxisMask = 31;
    block.optimizationState = MotionOptimizationState::Optimized;
    for (int i = 0; i < count; ++i) {
        CamMotionNode node;
        node.contourId = 1; node.axisMask = 31; node.sourceEdgeIndex = 0;
        node.sourceParameter = i; node.axes[2] = 10; node.axes[4] = i * 0.01;
        node.estimatedTimeMs = 10; node.tcpZ = 10;
        block.physicalKnots.append(node);
    }
    block.sourceSpans = {{1, 0, count - 1, 0, double(count - 1), 0}};
    block.fences = {{0, true, false, true}, {count - 1, false, true, false}};
    plan.blocks = {block};
    finalizeMotionPlan(&plan);
    return plan;
}

ReductionEvaluation model(const CamMotionPlanSnapshot& source)
{
    ReductionEvaluation result;
    result.admission.controllerSnapshotHash = source.context.controllerCapabilityHash;
    result.admission.dynamicsHash = source.context.dynamicsSemanticHash;
    result.admission.exactPhysicalAxisLine = true;
    result.admission.feedAndDynamicsRepresentable = true;
    result.admission.supportedActiveMasks.fill(true);
    result.motion.interpolationModelVersion = 1;
    result.motion.evaluatePhysicalAxes = [](const auto& axes, std::uint8_t, EvaluatedMotionState* state) {
        state->worldTcpX = axes[0] + std::sin(axes[4]);
        state->worldTcpY = axes[1]; state->worldTcpZ = axes[2];
        state->referenceTcpZ = axes[2] + 3; state->referenceTcpValid = true;
        state->processDirectionZ = 1;
        return true;
    };
    result.boundNumericalZ = [](const CamMotionBlock& a, const CamMotionBlock& b, int z) {
        NumericalZBound bound; bound.valid = true;
        for (int i = 0; i < a.physicalKnots.size(); ++i)
            bound.maximumPositionDeviationMm = std::max(bound.maximumPositionDeviationMm,
                std::abs(a.physicalKnots[i].axes[z] - b.physicalKnots[i].axes[z]));
        return bound;
    };
    return result;
}
}

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    const auto require = [](bool value, const char* message) {
        if (!value) QTextStream(stderr) << message << '\n';
        return value;
    };
    auto source = fixture();
    Full5DPolicy limits; limits.mode = PoseOptimizationMode::Conservative; limits.rotaryMask = 24;
    ReductionPolicy policy; policy.mode = limits.mode; policy.enableDofReduction = true; policy.zAxis = 2;
    ReductionReport report;
    CamMotionPlanSnapshot output;
    QString error;
    auto authority = model(source);
    BoundMotionEvaluationContext batchContext;
    ContinuousMotionEvaluator batchEvaluator;
    QVector<EvaluatedMotionState> batchStates(1);
    batchStates[0].worldTcpZ = -42;
    if (!require(bindMotionEvaluationContext(source, authority.motion, &batchContext, &error), "batch bind failed")) return 1;
    auto staleBlock = source.blocks[0];
    staleBlock.physicalKnots[1].axes[0] = 1;
    int batchCalls = 0;
    if (!require(!batchEvaluator.evaluatePhysicalKnots(staleBlock, batchContext, &batchStates, &error)
                 && batchStates[0].worldTcpZ == -42, "batch accepted stale identity or mutated output")
        || !require(!batchEvaluator.evaluatePhysicalKnots(source.blocks[0], batchContext, &batchStates, &error,
                        [&] { return ++batchCalls == 2; }) && batchStates[0].worldTcpZ == -42,
                    "batch cancellation did not preserve output")
        || !require(batchEvaluator.evaluatePhysicalKnots(source.blocks[0], batchContext, &batchStates, &error)
                    && batchStates.size() == 3 && batchStates[1].worldTcpX == std::sin(0.01),
                    "batch physical FK failed")) return 1;
    const auto run = [&] { return reduceMotionPlan(source, limits, policy, [&](const auto&) { return authority; }, &output, &report, &error); };
    if (!require(run(), qPrintable(error))
        || !require(output.blocks[0].activeAxisMask == 16 && output.blocks[0].motionClass == MotionClass::SingleAxis,
                    "true C-only was not selected")
        || !require(output.nodes[1].axisMask == 31 && output.nodes[1].axes[4] == source.nodes[1].axes[4]
                    && output.nodes[1].tcpX == std::sin(source.nodes[1].axes[4])
                    && finalMotionPlanIdentityIsCurrent(output), "derived fields or full physical pose lost")) return 1;
    const auto stableHash = output.planHash;
    if (!require(run() && output.planHash == stableHash, "selection is not deterministic")) return 1;
    source.context.collisionMode = CollisionVerificationMode::Required;
    finalizeMotionPlan(&source);
    if (!require(run() && output.blocks[0].activeAxisMask == 16
                 && output.collision.state == CollisionValidationState::Pending,
                 "collision policy changed selection or retained stale proof")) return 1;
    source = fixture();
    auto failedModel = authority;
    failedModel.motion.evaluatePhysicalAxes = {};
    if (!require(reduceMotionPlan(source, limits, policy, [&](const auto&) { return failedModel; }, &output, &report, &error)
                 && output.planHash == source.planHash, "unproved evaluator candidate replaced reference")) return 1;
    if (!require(run(), "restoring evaluator failed")) return 1;
    auto forged = output;
    forged.blocks[0].physicalKnots[1].axes[0] = 1;
    if (!require(!finalizeMotionPlan(&forged), "finalizer admitted a moving held axis")) return 1;
    for (int i = 0; i < 3; ++i) source.blocks[0].physicalKnots[i].axes[0] = i;
    finalizeMotionPlan(&source);
    if (!require(run() && output.blocks[0].activeAxisMask == 17, "false C-only / true U+C misclassified")) return 1;
    for (int i = 0; i < 3; ++i) source.blocks[0].physicalKnots[i].axes[1] = i;
    finalizeMotionPlan(&source);
    if (!require(run() && output.blocks[0].motionClass == MotionClass::Coordinated3D, "generic 3D candidate failed")) return 1;
    for (int i = 0; i < 3; ++i) source.blocks[0].physicalKnots[i].axes[3] = i;
    finalizeMotionPlan(&source);
    if (!require(run() && output.blocks[0].motionClass == MotionClass::Reduced4D, "generic 4D candidate failed")) return 1;
    source = fixture(1001);
    for (int i = 0; i < 1001; ++i) source.blocks[0].physicalKnots[i].axes[0] = i * 1e-9;
    finalizeMotionPlan(&source);
    if (!require(run() && output.blocks[0].activeAxisMask == 17
                 && report.blocks[0].axes[0].span > 9e-7, "cumulative tiny motion was frozen")) return 1;
    source = fixture();
    source.blocks[0].physicalKnots[1].axes[2] += 1e-14;
    finalizeMotionPlan(&source);
    if (!require(run() && report.blocks[0].numericalZApplied && output.nodes[1].axes[2] == 10
                 && output.nodes[1].tcpZ == 10 && output.nodes[1].referenceTcpZ == 13,
                 "numerical Z did not rebuild derived coordinates")) return 1;
    policy.enableDofReduction = false;
    if (!require(run() && output.blocks[0].activeAxisMask == 31 && report.blocks[0].numericalZApplied,
                 "DOF flag incorrectly disabled numerical Z")) return 1;
    policy.mode = limits.mode = PoseOptimizationMode::Off;
    if (!require(run() && output.planHash == source.planHash && !report.changed, "Off changed the reference")) return 1;
    policy.mode = limits.mode = PoseOptimizationMode::Full;
    policy.enableLaserZHold = true;
    source.blocks[0].physicalKnots[1].axes[2] = 11;
    finalizeMotionPlan(&source);
    if (!require(run() && output.nodes[1].axes[2] == 11 && !report.blocks[0].rejectedReasons.isEmpty(),
                 "process Z without envelope silently applied")) return 1;
    policy.enableLaserZHold = false; policy.enableDofReduction = true;
    source = fixture();
    source.blocks[0].physicalKnots.last().axes[2] += 1e-14;
    finalizeMotionPlan(&source);
    if (!require(run() && !report.blocks[0].numericalZApplied && (output.blocks[0].activeAxisMask & 4),
                 "numeric Z changed the terminal boundary")) return 1;
    source = fixture();
    source.context.controllerQualification.state = ControllerQualificationState::Unavailable;
    source.context.controllerQualification.qualificationRevision = 0;
    source.context.controllerCapabilityHash = controllerQualificationSnapshotHash(source.context.controllerQualification);
    finalizeMotionPlan(&source);
    if (!require(run() && output.planHash == source.planHash
                 && report.blocks[0].rejectedReasons.contains(QStringLiteral("capability not qualified")),
                 "unqualified reduced candidate admitted")) return 1;
    source = fixture(); source.context.controllerMode = ControllerMotionMode::RTCP;
    finalizeMotionPlan(&source);
    if (!require(run() && output.planHash == source.planHash, "unsupported RTCP candidate admitted")) return 1;
    source = fixture(); ++source.context.controllerQualification.qualificationRevision;
    source.context.controllerCapabilityHash = controllerQualificationSnapshotHash(source.context.controllerQualification);
    finalizeMotionPlan(&source);
    if (!require(run() && output.planHash == source.planHash
                 && report.blocks[0].rejectedReasons.contains(QStringLiteral("capability stale")),
                 "stale qualification admitted")) return 1;
    source = fixture(); authority.admission.dynamicsHash = "old-feed";
    if (!require(run() && output.planHash == source.planHash, "stale feed semantics admitted")) return 1;
    authority = model(source);
    policy.maximumCandidates = 1;
    if (!require(run() && output.planHash == source.planHash, "budget exhaustion published incomplete search")) return 1;
    policy.maximumCandidates = 32;
    auto next = source.blocks[0]; next.blockId = 2; next.hasEntryBoundary = true;
    next.entryBoundary = source.blocks[0].physicalKnots.last();
    next.physicalKnots = {next.entryBoundary}; next.physicalKnots[0].axes[4] += 1;
    next.physicalKnots[0].sourceParameter = 3;
    next.sourceSpans = {{1, -1, 0, 2, 3, 0}};
    next.fences = {{-1, true, false, true}, {0, false, true, false}};
    source.blocks.append(next); finalizeMotionPlan(&source);
    if (!require(run() && finalMotionPlanIdentityIsCurrent(output) && output.nodes.size() == 4
                 && output.blocks[1].sourceSpans.first().firstKnot == -1
                 && output.blocks[1].fences.first().knotIndex == -1,
                 "entry boundary duplicated or changed")) return 1;
    const auto retainedHash = output.planHash;
    int checks = 0;
    if (!require(!reduceMotionPlan(source, limits, policy, [&](const auto&) { return authority; },
                     &output, &report, &error, [&] { return ++checks > 3; })
                 && output.planHash == retainedHash, "cancel published partial selection")) return 1;
    source.blocks[0].physicalKnots[0].axes[0] = 9;
    if (!require(!run() && output.planHash == retainedHash, "stale reference accepted")) return 1;
    source = fixture(); source.blocks[0].optimizationState = MotionOptimizationState::Raw; finalizeMotionPlan(&source);
    if (!require(!run(), "raw sampled fallback accepted")) return 1;
    return 0;
}
