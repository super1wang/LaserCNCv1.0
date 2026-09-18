#pragma once

#include "modules/process/runtime/prepared_device_program.h"
#include "modules/process/runtime/pure_simulation_sink.h"
#include "modules/process/tool/tool_factory.h"
#include "b2_s1_closeout_test.h"
#include <cassert>
#include <limits>

inline void verifyPreparedDeviceProgram()
{
    using namespace lcnc;
    using namespace lcnc::cam;
    using namespace lcnc::process;
    Tool source;
    source.m_strName = "explicit";
    source.SetFromTable(toml::table{{"fLineVel", 100.0}, {"fCutAcc", 1000.0},
                                  {"fCutJerk", 10000.0}, {"fCutSmoothTime", 2.0}});
    assert(source.m_dLineVelocity == 100);
    assert(source.m_dLineAcc == 1000 && source.m_dArcAcc == 1000);
    assert(source.m_dLineJerk == 10000 && source.m_dArcJerk == 10000);
    Tool assigned;
    assigned = source;
    assert(assigned.m_dCutSmoothTime == 2);
    ToolFactory factory;
    factory.SetTool(0, source, false);
    assert(ToolFactory::executionRecipes().isEmpty());
    assert(ToolFactory::restoreSnapshot(ToolFactory::snapshot()));
    assert(ToolFactory::executionRecipes().isEmpty());
    factory.SetTool(0, source);
    const auto capturedTools = ToolFactory::executionRecipes();
    factory.ToolClear();
    assert(capturedTools[QStringLiteral("explicit")].m_dCutSmoothTime == 2);

    ToolpathExportSnapshot snapshot;
    snapshot.revision = 7;
    snapshot.machineAxisLayout.append(QStringLiteral("X"), MachineAxisRole::LinearX);
    snapshot.machineAxisLayout.append(QStringLiteral("Y"), MachineAxisRole::LinearY);
    snapshot.machineAxisLayout.append(QStringLiteral("Z"), MachineAxisRole::LinearZ);
    auto& plan = snapshot.motionPlan;
    plan.revision = 7;
    plan.context.sourceToolpathRevision = 7;
    plan.context.toolProcessHash = "binding";
    plan.context.controllerCapabilityHash = controllerQualificationSnapshotHash(plan.context.controllerQualification);
    CamMotionNode node;
    node.axisMask = 7;
    node.contourId = 9;
    node.phase = CamMotionPhase::Rapid;
    CamMotionBlock first;
    first.blockId = 11;
    first.contourId = 9;
    first.phase = CamMotionPhase::Rapid;
    first.motionClass = MotionClass::Coordinated3D;
    first.activeAxisMask = 7;
    first.optimizationState = MotionOptimizationState::Optimized;
    first.physicalKnots = {node};
    first.fences = {{0, true, true, false, false, false}};
    CamMotionBlock second = first;
    second.blockId = 12;
    second.hasEntryBoundary = true;
    second.entryBoundary = node;
    second.phase = CamMotionPhase::LeadIn;
    node.phase = CamMotionPhase::LeadIn;
    node.axes[0] = 12;
    second.physicalKnots = {node};
    second.sourceSpans = {{9, -1, 0, 0, 1, -1}};
    second.fences = {{-1, true, false, false, false, false},
                     {0, false, true, false, false, false}};
    CamMotionBlock third = second;
    third.blockId = 13;
    third.phase = CamMotionPhase::Cutting;
    third.entryBoundary = node;
    node.phase = CamMotionPhase::Cutting;
    node.axes[0] = 24;
    third.physicalKnots = {node};
    third.fences = {{-1, true, false, true, true, false},
                    {0, false, true, false, true, true}};
    plan.blocks = {first, second, third};
    QString error;
    assert(finalizeMotionPlan(&plan, &error));
    DeviceRunRecipe recipe;
    recipe.sourceId = QStringLiteral("test/explicit-recipe");
    recipe.processIoProfile = toml::value(toml::table{{"laserChannel", 5}, {"profile", 42}});
    const auto frozenTool = freezeToolExecutionRecipe(
        capturedTools[QStringLiteral("explicit")], recipe.sourceId, &error);
    assert(frozenTool);
    assert(frozenTool->lineAcceleration == 1000 && frozenTool->lineJerk == 10000);
    for (auto field : {&Tool::m_dArcVelocity, &Tool::m_dArcAcc, &Tool::m_dArcJerk}) {
        for (double value : {1.0, std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::infinity()}) {
            Tool independentArc = source;
            independentArc.*field = value;
            assert(!freezeToolExecutionRecipe(independentArc, recipe.sourceId, &error));
        }
    }
    recipe.toolsByContour.insert(9, *frozenTool);
    const auto bind = [&] {
        recipe.planHash = plan.planHash;
        recipe.contextHash = plan.contextHash;
        recipe.revision = deviceRunRecipeHash(recipe);
    };
    bind();
    b2_s1_test::verify(snapshot, recipe);
    auto prepared = PreparedDeviceProgram::prepare(snapshot, recipe, 1, false, &error);
    assert(prepared);
    // Existing sinks cannot silently turn exact plans back into legacy lines.
    PureSimulationSink legacySink(nullptr, {}, AxisMap{});
    assert(!legacySink.prepareExactSection(*prepared, 0, &error));
    assert(error.contains("lowering is unavailable"));
    // Same unqualified snapshot is valid for offline consumption, never device admission.
    assert(!PreparedDeviceProgram::prepare(snapshot, recipe, 1, true, &error));
    assert(error.contains("unqualified"));
    // Test-only qualification proves collision-disabled is independent of
    // controller admission. Production has no such authority source.
    auto qualified = snapshot;
    auto& context = qualified.motionPlan.context;
    context.controllerQualification.state = ControllerQualificationState::Qualified;
    context.controllerQualification.qualificationRevision = 1;
    context.controllerQualification.sourceId = QStringLiteral("test-only");
    context.controllerQualification.capabilityFingerprint = "test-only-capability";
    context.controllerCapabilityHash = controllerQualificationSnapshotHash(context.controllerQualification);
    assert(finalizeMotionPlan(&qualified.motionPlan, &error));
    auto qualifiedRecipe = recipe;
    qualifiedRecipe.planHash = qualified.motionPlan.planHash;
    qualifiedRecipe.contextHash = qualified.motionPlan.contextHash;
    qualifiedRecipe.revision = deviceRunRecipeHash(qualifiedRecipe);
    assert(PreparedDeviceProgram::prepare(qualified, qualifiedRecipe, 1, true, &error));
    qualifiedRecipe.toolsByContour[9].lineAcceleration = 0;
    qualifiedRecipe.revision = deviceRunRecipeHash(qualifiedRecipe);
    assert(!PreparedDeviceProgram::prepare(qualified, qualifiedRecipe, 1, true, &error));
    assert(!PreparedDeviceProgram::prepare(snapshot, recipe, 0, false, &error));

    struct Trace final : ExactPlanConsumer {
        QStringList events;
        double feed{0};
        bool discarded{false};
        bool begin(const PreparedDeviceProgram& p, QString*) override {
            feed = p.recipe().feedOverride; events << "begin"; return true;
        }
        bool block(const CamMotionBlock& b, const FrozenToolExecutionRecipe& t, QString*) override {
            assert(t.cutSmoothTime == 2);
            events << QStringLiteral("b%1").arg(b.blockId); return true;
        }
        bool knot(const CamMotionBlock& b, int i, QString*) override {
            events << QStringLiteral("x%1").arg(b.physicalKnots[i].axes[0]); return true;
        }
        bool fence(const CamMotionBlock&, const MotionProcessFence& f, QString*) override {
            events << (f.changesLaserState ? (f.laserEnabledAfterFence ? "on" : "off") : "structure");
            if (f.requiredStop) events << "stop";
            return true;
        }
        bool seal(QString*) override { events << "seal"; return true; }
        void discard() noexcept override { discarded = true; }
    } trace;
    // UI/settings/source mutations after prepare cannot affect the run.
    recipe.feedOverride = 0.5;
    recipe.toolsByContour[9].cutSmoothTime = 999;
    recipe.processIoProfile["laserChannel"] = 99;
    plan.blocks[1].physicalKnots[0].axes[0] = 999;
    assert(consumeExactPlan(*prepared, trace, {}, &error));
    assert(trace.feed == 1 && !trace.discarded);
    assert(trace.events == QStringList({"begin", "b11", "x0", "structure", "b12", "structure", "x12", "structure", "b13", "on", "x24", "off", "stop", "seal"}));
    assert(prepared->recipe().processIoProfile.at("laserChannel").as_integer() == 5);
    Trace cancelled;
    int calls = 0;
    assert(!consumeExactPlan(*prepared, cancelled, [&] { return ++calls > 4; }, &error));
    assert(cancelled.discarded && !cancelled.events.contains("seal"));
    assert(!PreparedDeviceProgram::prepare(snapshot, recipe, 2, false, &error));
    plan = prepared->plan();
    recipe = prepared->recipe();
    auto stale = recipe;
    stale.toolsByContour[9].lineVelocity++;
    assert(!PreparedDeviceProgram::prepare(snapshot, stale, 2, false, &error));
    stale = recipe;
    stale.toolsByContour.clear();
    stale.revision = deviceRunRecipeHash(stale);
    assert(!PreparedDeviceProgram::prepare(snapshot, stale, 2, false, &error));
    // Legacy points, nodes or travel readiness cannot replace the final blocks.
    auto legacy = snapshot;
    legacy.motionPlan.blocks.clear();
    assert(!PreparedDeviceProgram::prepare(legacy, recipe, 2, false, &error));
    // Mutable safety policy does not override committed Disabled/Optional.
    snapshot.collisionSafety.verificationMode = CollisionVerificationMode::Required;
    plan.collision.state = CollisionValidationState::Collision;
    assert(PreparedDeviceProgram::prepare(snapshot, recipe, 2, false, &error));
    plan.context.collisionMode = CollisionVerificationMode::Optional;
    assert(finalizeMotionPlan(&plan, &error)); bind();
    assert(PreparedDeviceProgram::prepare(snapshot, recipe, 2, false, &error));
    plan.context.collisionMode = CollisionVerificationMode::Required;
    assert(finalizeMotionPlan(&plan, &error)); bind();
    snapshot.collisionSafety.verificationMode = CollisionVerificationMode::Disabled;
    assert(!PreparedDeviceProgram::prepare(snapshot, recipe, 2, false, &error));
    plan.collision.state = CollisionValidationState::Safe;
    plan.collision.complete = true;
    CamMotionEdgeCertificate certificate;
    certificate.firstNode = 0; certificate.lastNode = 1;
    certificate.state = CamMotionCertificateState::CertifiedSafe;
    plan.edgeCertificates = {certificate};
    certificate.firstNode = 1; certificate.lastNode = 2;
    plan.edgeCertificates.append(certificate);
    snapshot.travelPlan.verifiedMotionPlanHash = "wrong-plan";
    assert(!PreparedDeviceProgram::prepare(snapshot, recipe, 2, false, &error));
    snapshot.travelPlan.verifiedMotionPlanHash = plan.planHash;
    assert(PreparedDeviceProgram::prepare(snapshot, recipe, 2, false, &error));
}
