#pragma once
#include "gtn_exact_plan_lowering_test.h"
#include "modules/process/runtime/gtn_exact_session.h"
#include "modules/process/runtime/coordinated_motion_command_sink.h"
#include "modules/process/runtime/process_interrupt_context.h"
#include <cstdio>

namespace lcnc::process::gtn_session_test {
inline gtn_lowering_test::Fixture fixture(cam::ControllerMotionMode mode)
{
    gtn_lowering_test::Fixture f(mode);
    auto& p = f.recipe.gtnLowering;
    auto& g = p.group;
    g.finiteListAndIoQualified = true;
    g.groupIndex = g.listIndex = 1;
    g.kinematics.modelType = 0;
    g.kinematics.directionMode = 1;
    g.kinematics.machineKinematicsFingerprint = "test-only/machine";
    g.kinematics.calibrationFingerprint = "test-only/calibration";
    g.kinematics.calibrationMachineVerified = true;
    for (int i = 0; i < 5; ++i) {
        g.kinematics.physicalAxisIndices[i] = p.axes[i].controllerAxis;
        g.kinematics.scales[i].count = 1;
        g.kinematics.scales[i].alpha = 1;
        g.kinematics.scales[i].beta = 1000;
        g.kinematics.axisVectorsMcs[i][i % 3] = 1;
        g.axisVelocity[i] = 10000;
        g.axisAcceleration[i] = 100000;
        g.axisJerk[i] = 1000000;
        g.axisDvMax[i] = 500;
    }
    g.pathVelocityLimit = g.orientationVelocity = 1000;
    g.pathAccelerationLimit = g.orientationAcceleration = 10000;
    g.pathJerkLimit = g.orientationJerk = 100000;
    g.smoothTimeMs = 20; g.smoothK = 15;
    g.lookAheadSegments = 200; g.lookAheadTime = .01; g.lookAheadRadiusRatio = .1;
    g.startPositionTolerance = g.rtcpAxisTolerance = .01;
    g.outputs[0] = {1, false, 0, 1}; g.outputs[1] = {2, false, 0, 1};
    return f;
}
struct Backend final : IGtnExactBackend {
    int acquired = 0, appended = 0, sealed = 0, started = 0, released = 0, transforms = 0;
    int pending = 0, failAppend = -1;
    bool admission = true, acquisition = true, startOk = true, releaseOk = true;
    bool sealOk = true, pollOk = true, moving = false, latched = false;
    bool trace = false;
    bool admit(const PreparedDeviceProgram&, int, QString*) override { return admission; }
    bool acquire(const PreparedDeviceProgram&, QString*) override { ++acquired; return acquisition; }
    bool validateRtcp(const std::array<double,5>&, const std::array<double,5>&, QString*) override { ++transforms; return true; }
    bool append(const GtnEncodedSection&, const GtnExactCommand& c, QString*) override {
        if (trace) std::printf("S3 test-only command tag=%d knot=%d kind=%d target=[%.9g,%.9g,%.9g,%.9g,%.9g] velocity=%.9g acceleration=%.9g jerk=%.9g stop=%d laser_change=%d laser_after=%d\n",
            int(c.userTag), c.knotIndex, int(c.kind), c.target[0], c.target[1], c.target[2], c.target[3], c.target[4],
            c.dynamics.velocity, c.dynamics.acceleration, c.dynamics.jerk, int(c.fence.requiredStop), int(c.fence.changesLaserState), int(c.fence.laserEnabledAfterFence));
        return ++appended != failAppend;
    }
    GtnSealResult seal(QString*) override { ++sealed; return !sealOk ? GtnSealResult::Failed : pending-- > 0 ? GtnSealResult::Pending : GtnSealResult::Sealed; }
    bool start(QString*) override { ++started; return startOk; }
    bool poll(bool& running, QString*) override { running = moving; return pollOk; }
    bool stopRelease(bool latch, QString*) override { ++released; latched |= latch; return releaseOk; }
};
struct Sink final : IMotionCommandSink {
    Backend backend;
    GtnExactSession session{backend};
    DeviceCommandQueue& queue;
    ProcessInterruptContext& interrupt;
    bool stopDuringFill{false};
    int fills{0};
    Sink(DeviceCommandQueue& q, ProcessInterruptContext& i) : queue(q), interrupt(i) {}
    QString id() const override { return "test-only/exact-session"; }
    bool supportsBatchProgram() const override { return true; }
    void setCancellation(ProcessInterruptContext*) override {}
    bool prepareExactSection(const PreparedDeviceProgram& p, int i, QString* e) override { assert(queue.isWorkerThread()); return session.prepare(p, i, {}, e); }
    bool continueExactPreparation(const PreparedDeviceProgram& p, int i, bool& done, QString* e) override {
        assert(queue.isWorkerThread()); ++fills;
        const bool ok = session.fill(p, i, done, {}, e);
        if (stopDuringFill && fills == 1) assert(queue.submitStop([this] { interrupt.requestStop(); QString reason; session.abort(&reason); }));
        return ok;
    }
    bool startExactSection(const PreparedDeviceProgram& p, int i, QString* e) override { assert(queue.isWorkerThread()); return session.start(p, i, {}, e); }
    bool isExactSectionRunning(const PreparedDeviceProgram& p, int i, QString* e) override { assert(queue.isWorkerThread()); bool running = false; return session.poll(p, i, running, {}, e) && running; }
    // A call to any legacy execution API is a test failure.
    void resetProgram() override { assert(false); }
    bool startProgram(QString*) override { assert(false); return false; }
    bool isProgramRunning(QString*) override { assert(false); return false; }
    bool flush(QString*) override { assert(false); return false; }
    bool executeRapidSegment(const cam::RapidMoveSegment&, const Tool&, QString*) override { assert(false); return false; }
    void startCuttingHead(const Tool&) override { assert(false); }
    void stopCuttingHead() override { assert(false); }
    void setShutterTimings(double,double,double,double,double) override { assert(false); }
    bool lineTo(const MachinePose5&, const Tool&, QString*) override { assert(false); return false; }
    bool beginSegment(const MachinePose5&, const Tool&, QString*) override { assert(false); return false; }
    void endSegment(const Tool&) override { assert(false); }
    void laserOn(const Tool&) override { assert(false); }
    void laserOff(const Tool&) override { assert(false); }
    void endProgram(const Tool&) override { assert(false); }
    void applyToolMotionParams(const Tool&, bool) override { assert(false); }
};
inline void verify()
{
    QString error;
    for (auto mode : {cam::ControllerMotionMode::PhysicalAxes, cam::ControllerMotionMode::RTCP}) {
        auto f = fixture(mode);
        auto program = f.prepare();
        Backend b; b.pending = 2; b.trace = true;
        std::printf("S3 test-only mode=%s source=test-only/GTN-qualification revision=11\n", mode == cam::ControllerMotionMode::RTCP ? "RTCP" : "PhysicalAxes");
        GtnExactSession s(b);
        assert(s.prepare(*program, 0, {}, &error));
        bool complete = false;
        for (int i = 0; i < 3; ++i) { assert(s.fill(*program, 0, complete, {}, &error)); assert(complete == (i == 2)); }
        assert(b.started == 0 && b.sealed == 3);
        f.recipe.gtnLowering.group.pathVelocityLimit = 1; // frozen program is independent
        assert(s.start(*program, 0, {}, &error));
        bool running = false; b.moving = true;
        assert(s.poll(*program, 0, running, {}, &error) && running);
        b.moving = false;
        assert(s.poll(*program, 0, running, {}, &error) && !running);
        assert(s.state() == GtnSessionState::Complete && !s.ownsGroup());
        assert(b.released == 1 && !b.latched && b.started == 1);
        assert(mode != cam::ControllerMotionMode::RTCP || b.transforms >= 4);
        assert(!s.start(*program, 0, {}, &error) && b.started == 1);
        const auto audit = gtnParameterAudit("speed", 100, 100, "mm/s", program->recipe().gtnLowering);
        assert(audit.contains("requested=100 effective=100 unit=mm/s source=test-only/") && audit.contains("revision=11"));
    }
    // Each injected fault is terminal and cannot replay Start or silently retry release.
    for (int fault = 0; fault < 8; ++fault) {
        auto f = fixture(cam::ControllerMotionMode::PhysicalAxes);
        auto p = f.prepare(); Backend b; GtnExactSession s(b);
        if (fault == 0) b.acquisition = false;
        const bool prepared = s.prepare(*p, 0, {}, &error);
        if (fault == 0) assert(!prepared); else {
            assert(prepared);
            if (fault == 1) b.failAppend = 2;
            if (fault == 2) b.sealOk = false;
            bool complete = false;
            const bool filled = s.fill(*p, 0, complete, [=] { return fault == 3; }, &error);
            if (fault <= 3) assert(!filled); else {
                assert(filled && complete);
                if (fault == 4) b.admission = false;
                if (fault == 5) b.startOk = false;
                const bool started = s.start(*p, 0, {}, &error);
                if (fault <= 5) assert(!started); else {
                    assert(started);
                    if (fault == 6) b.pollOk = false;
                    if (fault == 7) b.releaseOk = false;
                    bool running = false;
                    assert(!s.poll(*p, 0, running, {}, &error));
                }
            }
        }
        assert(s.state() == GtnSessionState::Faulted && b.released == 1);
        const int starts = b.started;
        assert(!s.start(*p, 0, {}, &error));
        assert(!s.prepare(*p, 0, {}, &error));
        s.abort(&error);
        assert(b.started == starts && b.released == 1);
        assert(s.ownsGroup() == (fault == 7));
    }
    // Stop between bounded fill slices; the remaining list is never submitted.
    auto f = fixture(cam::ControllerMotionMode::PhysicalAxes);
    auto& block = f.snapshot.motionPlan.blocks[0];
    const auto node = block.physicalKnots.back();
    while (block.physicalKnots.size() < 100) block.physicalKnots.append(node);
    block.fences.back().knotIndex = 99;
    auto p = f.prepare(); Backend b; GtnExactSession s(b);
    assert(s.prepare(*p, 0, {}, &error));
    bool complete = false;
    assert(s.fill(*p, 0, complete, {}, &error) && !complete && b.appended == 32);
    assert(!s.fill(*p, 0, complete, [] { return true; }, &error));
    assert(b.appended == 32 && b.started == 0 && b.released == 1);
    // A second section cannot acquire until the first is drained and released.
    auto multiple = fixture(cam::ControllerMotionMode::PhysicalAxes);
    auto second = multiple.snapshot.motionPlan.blocks.front();
    second.blockId = 22; second.contourId = 10;
    second.hasEntryBoundary = true;
    second.entryBoundary = multiple.snapshot.motionPlan.blocks.front().physicalKnots.back();
    for (auto& knot : second.physicalKnots) knot.contourId = 10;
    second.sourceSpans = {{10, -1, 3, 0, 1, 4}};
    second.fences.front().knotIndex = -1;
    multiple.snapshot.motionPlan.blocks.append(second);
    multiple.recipe.toolsByContour.insert(10, multiple.recipe.toolsByContour.value(9));
    auto mp = multiple.prepare(); Backend mb; GtnExactSession ms(mb);
    assert(mp->sections().size() == 2);
    for (int ordinal = 0; ordinal < 2; ++ordinal) {
        assert(mb.acquired == mb.released);
        assert(ms.prepare(*mp, ordinal, {}, &error));
        assert(ms.fill(*mp, ordinal, complete, {}, &error) && complete);
        assert(ms.start(*mp, ordinal, {}, &error));
        bool running = false;
        assert(ms.poll(*mp, ordinal, running, {}, &error) && !running);
    }
    assert(mb.acquired == 2 && mb.released == 2 && mb.started == 2);
    // DataEnd back-pressure is bounded and cannot trigger Start.
    Backend pending; pending.pending = 300; GtnExactSession waiting(pending);
    assert(waiting.prepare(*p, 0, {}, &error));
    bool filled = true;
    for (int attempt = 0; attempt < 300 && filled; ++attempt) filled = waiting.fill(*p, 0, complete, {}, &error);
    assert(!filled && pending.sealed == 256 && pending.started == 0 && pending.released == 1);
    // Actual orchestration + coordinator wrapper + existing Stop priority lane.
    for (bool stop : {false, true}) {
        DeviceCommandQueue queue;
        assert(queue.start());
        ProcessInterruptContext interrupt;
        ProcessDeviceCoordinator coordinator;
        auto inner = std::make_unique<Sink>(queue, interrupt);
        auto* trace = inner.get();
        trace->stopDuringFill = stop;
        CoordinatedMotionCommandSink wrapped(std::move(inner), coordinator);
        assert(executePreparedSections(*p, wrapped, queue, interrupt, "s3", [] { return DeviceCommandResult{}; }, &error) == !stop);
        assert(trace->backend.released == 1);
        assert(trace->backend.started == (stop ? 0 : 1));
        assert(stop ? trace->fills == 1 : trace->fills > 1);
        assert(queue.shutdown());
    }
}
} // namespace lcnc::process::gtn_session_test
