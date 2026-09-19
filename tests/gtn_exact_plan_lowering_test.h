#pragma once

#include "modules/process/runtime/gtn_exact_plan_lowering.h"
#if LCNC_PROCESS_HAS_GTN
#include "modules/process/runtime/gtn_buffered_command_sink.h"
#endif

#include <cassert>
#include <cmath>
#include <limits>

namespace lcnc::process::gtn_lowering_test {

// Synthetic authority only: never installed into a production service.
struct Fixture {
    cam::ToolpathExportSnapshot snapshot;
    DeviceRunRecipe recipe;

    explicit Fixture(cam::ControllerMotionMode mode = cam::ControllerMotionMode::PhysicalAxes)
    {
        snapshot.revision = 8;
        auto& layout = snapshot.machineAxisLayout;
        layout.append("C", MachineAxisRole::TableSpin);
        layout.append("Z", MachineAxisRole::LinearZ);
        layout.append("X", MachineAxisRole::LinearX);
        layout.append("A", MachineAxisRole::TableTilt);
        layout.append("Y", MachineAxisRole::LinearY);
        auto& p = recipe.gtnLowering;
        p.state = cam::ControllerQualificationState::Qualified;
        p.revision = 11;
        p.sourceId = "test-only/GTN-qualification";
        p.groupSemanticsId = "test-only/Group-continuous-line-model";
        p.mode = mode;
        p.absoluteRotaryTurnsQualified = true;
        const int indices[] = {2, 4, 1, 3, 0};
        const int controllers[] = {7, 2, 9, 4, 6};
        for (int i = 0; i < 5; ++i) {
            const auto& axis = layout.axes[indices[i]];
            p.axes[i] = {indices[i], axis.name, axis.role, controllers[i], -10000, 10000};
            p.cells[i] = {true, true, true, true};
        }
        p.metric = mode == cam::ControllerMotionMode::PhysicalAxes
            ? GtnFeedMetric::WeightedMillimetres : GtnFeedMetric::LinearMillimetres;
        p.referenceRatios = mode == cam::ControllerMotionMode::PhysicalAxes
            ? std::array<double, 5>{1, 1, 1, 0.5, 0.25} : std::array<double, 5>{1, 1, 1, 0, 0};
        p.rapidFeedMmPerSecond = 200;
        FrozenToolExecutionRecipe tool;
        tool.toolName = "explicit";
        tool.sourceId = "test-only/recipe";
        tool.lineVelocity = 100;
        tool.lineAcceleration = 1000;
        tool.lineJerk = 10000;
        tool.rapidAcceleration = 2000;
        tool.rapidJerk = 20000;
        recipe.toolsByContour.insert(9, tool);
        recipe.sourceId = "test-only/recipe";
        recipe.processIoProfile = toml::value(toml::table{{"laserChannel", 5}});
        auto& plan = snapshot.motionPlan;
        plan.revision = snapshot.revision;
        plan.context.sourceToolpathRevision = snapshot.revision;
        plan.context.dynamicsSemanticHash = "test-only/dynamics";
        cam::CamMotionBlock b;
        b.blockId = 21;
        b.contourId = 9;
        b.motionClass = cam::MotionClass::Full5D;
        b.activeAxisMask = 31;
        b.optimizationState = cam::MotionOptimizationState::Optimized;
        b.interpolation = mode == cam::ControllerMotionMode::PhysicalAxes
            ? cam::MotionInterpolationKind::PhysicalAxisLine : cam::MotionInterpolationKind::RtcpLine;
        for (int i : {0, 1, 1, 2}) { // Intentional duplicate survives encoding.
            cam::CamMotionNode n;
            n.contourId = 9;
            n.axisMask = 31;
            n.axes = {720.0 + i * 360, 30.0 + i, 10.0 + i, -450.0 - i * 360, 20.0 + i};
            n.referenceTcpValid = true;
            n.referenceTcpX = 100 + i;
            n.referenceTcpY = 200 + i;
            n.referenceTcpZ = 300 + i;
            n.tcpX = 900 + i; // World TCP must never leak into RTCP commands.
            n.sourceEdgeIndex = 4;
            n.sourceParameter = i * 0.5;
            n.departureSourceEdgeIndex = 5;
            n.departureSourceParameter = i * 0.25;
            b.physicalKnots.append(n);
        }
        b.sourceSpans = {{9, 0, 3, 0, 1, 4}};
        b.fences = {{0, true, false, true, true, false}, {3, false, true, false, true, true}};
        plan.blocks = {b};
    }

    std::shared_ptr<const PreparedDeviceProgram> prepare()
    {
        auto& context = snapshot.motionPlan.context;
        const auto& p = recipe.gtnLowering;
        context.controllerMode = p.mode;
        context.controllerQualification = {p.mode, cam::ControllerQualificationState::Qualified,
            p.revision, gtnLoweringProfileHash(p), p.sourceId};
        context.controllerCapabilityHash = cam::controllerQualificationSnapshotHash(context.controllerQualification);
        QString error;
        assert(cam::finalizeMotionPlan(&snapshot.motionPlan, &error));
        recipe.planHash = snapshot.motionPlan.planHash;
        recipe.contextHash = snapshot.motionPlan.contextHash;
        recipe.revision = deviceRunRecipeHash(recipe);
        auto program = PreparedDeviceProgram::prepare(snapshot, recipe, 99, false, &error);
        assert(program);
        return program;
    }
};

inline void verify()
{
    QString error;
    Fixture physical;
    auto program = physical.prepare();
    const auto encoded = GtnEncodedSection::lower(*program, 0, {}, {}, &error);
    assert(encoded && encoded->commands().size() == 6);
    assert(encoded->planHash() == program->plan().planHash && encoded->runEpoch() == 99);
    assert(encoded->recipeRevision() == program->recipe().revision);
    assert(encoded->profile().axes[0].controllerAxis == 7);
    const auto& commands = encoded->commands();
    assert(commands[0].target == (std::array<double, 5>{10, 20, 30, -450, 720}));
    assert(commands[4].target == (std::array<double, 5>{12, 22, 32, -1170, 1440}));
    assert(commands[2].target == commands[3].target); // no duplicate filtering
    assert(commands[4].blockId == 21 && commands[4].knotIndex == 3);
    assert(commands[4].sourceEdgeIndex == 4 && commands[4].sourceParameter == 1);
    assert(commands[4].departureSourceEdgeIndex == 5 && commands[4].departureSourceParameter == 0.5);
    assert(commands[1].kind == GtnExactCommand::Kind::Fence && commands[1].fence.laserEnabledAfterFence);
    assert(commands[5].fence.requiredStop && !commands[5].fence.laserEnabledAfterFence);
    for (int i = 0; i < commands.size(); ++i) assert(commands[i].userTag == i + 1);
    assert(commands[0].dynamics.velocity == 100 && commands[0].dynamics.acceleration == 1000);
    assert(GtnEncodedSection::lower(*program, 0, {}, {}, &error)->encodingHash() == encoded->encodingHash());
    physical.recipe.gtnLowering.axes[0].controllerAxis = 22;
    physical.recipe.toolsByContour[9].lineVelocity = 999;
    assert(encoded->profile().axes[0].controllerAxis == 7 && encoded->tool().lineVelocity == 100);
    assert(GtnEncodedSection::lower(*program, 0, {}, {}, &error)->encodingHash() == encoded->encodingHash());
    assert(!GtnEncodedSection::lower(*program, 0, {}, [] { return true; }, &error));
    assert(!GtnEncodedSection::lower(*program, 1, {}, {}, &error));

    Fixture rtcp(cam::ControllerMotionMode::RTCP);
    int validated = 0;
    const GtnRtcpTargetValidator validator = [&](const auto& target, const auto& predicted, QString*) {
        ++validated;
        assert(target[0] - predicted[0] == 90 && target[1] - predicted[1] == 180);
        assert(target[3] == predicted[3] && target[4] == predicted[4]);
        return true;
    };
    auto rtcpProgram = rtcp.prepare();
    const auto rtcpEncoded = GtnEncodedSection::lower(*rtcpProgram, 0, validator, {}, &error);
    assert(rtcpEncoded && validated == 4);
    assert(rtcpEncoded->commands()[4].target == (std::array<double, 5>{102, 202, 302, -1170, 1440}));
    assert(rtcpEncoded->planHash() == rtcpProgram->plan().planHash); // no plan reselection
    assert(!GtnEncodedSection::lower(*rtcpProgram, 0, {}, {}, &error));
    assert(!GtnEncodedSection::lower(*rtcpProgram, 0,
        [](const auto&, const auto&, QString*) { return false; }, {}, &error));
    rtcp.snapshot.motionPlan.blocks[0].physicalKnots[1].referenceTcpValid = false;
    assert(!GtnEncodedSection::lower(*rtcp.prepare(), 0, validator, {}, &error));
    assert(error.contains("reference TCP"));
    rtcp.snapshot.motionPlan.blocks[0].physicalKnots[1].referenceTcpValid = true;
    for (auto& node : rtcp.snapshot.motionPlan.blocks[0].physicalKnots) {
        node.referenceTcpX = 100; node.referenceTcpY = 200; node.referenceTcpZ = 300;
    }
    assert(!GtnEncodedSection::lower(*rtcp.prepare(), 0,
        [](const auto&, const auto&, QString*) { return true; }, {}, &error));
    assert(error.contains("reference distance"));

    const auto refused = [&](auto mutate, const char* reason) {
        Fixture f;
        mutate(f);
        const auto p = f.prepare();
        assert(!GtnEncodedSection::lower(*p, 0, {}, {}, &error));
        assert(error.contains(QString::fromLatin1(reason)));
    };
    refused([](auto& f) { f.recipe.gtnLowering.state = cam::ControllerQualificationState::Unavailable; }, "qualification unavailable");
    refused([](auto& f) { f.recipe.gtnLowering.axes[0].physicalIndex = 1; }, "mapping");
    refused([](auto& f) { f.recipe.gtnLowering.axes[0].controllerAxis = 2; }, "mapping");
    refused([](auto& f) { f.recipe.gtnLowering.axes[0].name = "Y"; }, "mapping");
    refused([](auto& f) { f.recipe.gtnLowering.axes[4].maximum = 1000; }, "limits");
    refused([](auto& f) { f.recipe.gtnLowering.axes[4].minimum = std::numeric_limits<double>::quiet_NaN(); }, "limits");
    refused([](auto& f) { f.recipe.gtnLowering.cells[4].supported = false; }, "cell");
    refused([](auto& f) { f.recipe.gtnLowering.cells[4].continuousInterpolationQualified = false; }, "continuous");
    refused([](auto& f) { f.recipe.gtnLowering.cells[4].feedMappingQualified = false; }, "cell");
    refused([](auto& f) { f.recipe.gtnLowering.absoluteRotaryTurnsQualified = false; }, "rotary semantics");
    refused([](auto& f) { f.recipe.gtnLowering.metric = GtnFeedMetric::Unavailable; }, "metric");
    refused([](auto& f) { f.recipe.gtnLowering.referenceRatios[4] = 0; }, "ratios");
    refused([](auto& f) { f.recipe.gtnLowering.referenceRatios[4] = std::numeric_limits<double>::infinity(); }, "ratios");
    refused([](auto& f) { f.recipe.gtnLowering.mode = cam::ControllerMotionMode::RTCP; }, "interpolation");
    refused([](auto& f) {
        f.recipe.gtnLowering.metric = GtnFeedMetric::LinearMillimetres;
        f.recipe.gtnLowering.referenceRatios = {1, 1, 1, 0, 0};
    }, "Mixed physical");
    refused([](auto& f) { f.snapshot.motionPlan.context.interpolationModelVersion = 2; }, "model version");
    refused([](auto& f) { f.snapshot.motionPlan.blocks[0].feed.nominalFeedPerMinute = -1; }, "published feed");
    refused([](auto& f) { f.snapshot.motionPlan.blocks[0].feed.nominalFeedPerMinute = 60; }, "profile identity");
    auto staleRecipe = program->recipe();
    staleRecipe.gtnLowering.axes[0].controllerAxis = 22;
    assert(deviceRunRecipeHash(staleRecipe) != staleRecipe.revision);
    staleRecipe.revision = deviceRunRecipeHash(staleRecipe);
    Fixture base;
    base.prepare();
    auto stale = PreparedDeviceProgram::prepare(base.snapshot, staleRecipe, 99, false, &error);
    assert(stale && !GtnEncodedSection::lower(*stale, 0, {}, {}, &error));
    assert(error.contains("identity mismatch"));

    Fixture feed;
    feed.snapshot.motionPlan.blocks[0].feed.nominalFeedPerMinute = 1200;
    feed.snapshot.motionPlan.blocks[0].feed.profileHash = "test-only/dynamics";
    feed.recipe.feedOverride = 0.5;
    auto fed = GtnEncodedSection::lower(*feed.prepare(), 0, {}, {}, &error);
    assert(fed && fed->commands()[0].dynamics.velocity == 10); // mm/min -> mm/s, then override
    assert(fed->commands()[0].dynamics.acceleration == 1000); // velocity-only override
    Fixture rapid;
    rapid.snapshot.motionPlan.blocks[0].phase = cam::CamMotionPhase::Rapid;
    for (auto& node : rapid.snapshot.motionPlan.blocks[0].physicalKnots) node.phase = cam::CamMotionPhase::Rapid;
    auto rapidEncoded = GtnEncodedSection::lower(*rapid.prepare(), 0, {}, {}, &error);
    assert(rapidEncoded && rapidEncoded->commands()[0].dynamics.velocity == 200);
    assert(rapidEncoded->commands()[0].dynamics.acceleration == 2000);
    rapid.recipe.toolsByContour[9].rapidAcceleration = 0;
    assert(!GtnEncodedSection::lower(*rapid.prepare(), 0, {}, {}, &error));

    Fixture angular;
    auto& block = angular.snapshot.motionPlan.blocks[0];
    block.motionClass = cam::MotionClass::SingleAxis;
    block.optimizationState = cam::MotionOptimizationState::Reduced;
    block.activeAxisMask = 1; // C is layout slot 0, Group slot 4.
    for (auto& node : block.physicalKnots)
        for (int axis = 1; axis < 5; ++axis) node.axes[axis] = block.physicalKnots.front().axes[axis];
    auto& profile = angular.recipe.gtnLowering;
    profile.metric = GtnFeedMetric::RotaryDegrees;
    profile.referenceRatios = {0, 0, 0, 0, 1};
    profile.surfaceRadiusMm = 50;
    auto rotaryEncoded = GtnEncodedSection::lower(*angular.prepare(), 0, {}, {}, &error);
    assert(rotaryEncoded && rotaryEncoded->commands()[0].activeGroupSlotMask == 16);
    assert(rotaryEncoded->groupMembershipMask() == 31);
    const auto& d = rotaryEncoded->commands()[0].dynamics;
    assert(std::abs(d.velocity - 100 * 180 / (3.14159265358979323846 * 50)) < 1e-10);
    assert(std::abs(d.jerk / d.velocity - 100) < 1e-10);
    profile.surfaceRadiusMm = 0;
    assert(!GtnEncodedSection::lower(*angular.prepare(), 0, {}, {}, &error));
    profile.surfaceRadiusMm = 50;
    profile.cells[0].inactiveAxisHoldQualified = false;
    assert(!GtnEncodedSection::lower(*angular.prepare(), 0, {}, {}, &error));

    Fixture sections;
    auto second = sections.snapshot.motionPlan.blocks[0];
    second.blockId = 22;
    second.contourId = 10;
    second.hasEntryBoundary = true;
    second.entryBoundary = second.physicalKnots.back();
    second.physicalKnots = {second.entryBoundary};
    second.physicalKnots[0].contourId = 10;
    second.physicalKnots[0].axes[2] += 5;
    second.sourceSpans = {{10, -1, 0, 0, 1, 4}};
    second.fences = {{-1, true, false, true, true, false}, {0, false, true, false, true, true}};
    sections.snapshot.motionPlan.blocks.append(second);
    sections.recipe.toolsByContour.insert(10, sections.recipe.toolsByContour[9]);
    auto sectionProgram = sections.prepare();
    auto next = GtnEncodedSection::lower(*sectionProgram, 1, {}, {}, &error);
    assert(next && next->commands().size() == 3); // entry boundary is validation only
    assert(next->commands()[0].userTag == 7 && next->commands()[0].knotIndex == -1);
    assert(next->commands()[1].blockId == 22 && next->commands()[1].target[0] == 17);
    auto whole = GtnEncodedProgram::lower(*sectionProgram, {}, {}, &error);
    assert(whole && whole->sections().size() == 2);
    assert(whole->sections()[1]->encodingHash() == next->encodingHash());
    sections.snapshot.motionPlan.blocks[1].physicalKnots[0].axes[2] = 10001;
    assert(!GtnEncodedProgram::lower(*sections.prepare(), {}, {}, &error));
    assert(error.contains("limits"));
#if LCNC_PROCESS_HAS_GTN
    GtnBufferedCommandSink device(nullptr, AxisMap{});
    assert(!device.prepareExactSection(*program, 0, &error)); // offline fixture never device admission
    assert(!device.startExactSection(*program, 0, &error));
    assert(error.contains("not prepared")); // no legacy Start fallback without a sealed session
#endif
}
} // namespace lcnc::process::gtn_lowering_test
