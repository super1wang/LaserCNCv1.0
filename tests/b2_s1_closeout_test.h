#pragma once

#include "modules/process/runtime/exact_section_execution.h"
#include "modules/process/runtime/process_interrupt_context.h"
#include "modules/process/tool/tool.h"

#include <QSemaphore>
#include <QElapsedTimer>
#include <QThread>
#include <atomic>
#include <cassert>
#include <limits>
#include <thread>
#include <type_traits>

namespace lcnc::process::b2_s1_test {

template<class T>
void checkField(T FrozenToolExecutionRecipe::*field,
    const lcnc::cam::ToolpathExportSnapshot& snapshot, const DeviceRunRecipe& recipe)
{
    auto changed = recipe;
    auto& value = changed.toolsByContour[9].*field;
    if constexpr (std::is_same_v<T, bool>) value = !value;
    else value += 1;
    assert(deviceRunRecipeHash(changed) != recipe.revision);
    if constexpr (std::is_floating_point_v<T>) {
        for (const T bad : {std::numeric_limits<T>::quiet_NaN(), std::numeric_limits<T>::infinity()}) {
            value = bad;
            changed.revision = deviceRunRecipeHash(changed);
            QString error;
            assert(!PreparedDeviceProgram::prepare(snapshot, changed, 42, false, &error));
            assert(error.contains("Non-finite"));
        }
    }
}

inline bool await(const std::function<bool()>& condition)
{
    QElapsedTimer timer;
    timer.start();
    while (!condition() && timer.elapsed() < 3000) QThread::msleep(1);
    return condition();
}

inline void verify(lcnc::cam::ToolpathExportSnapshot snapshot, DeviceRunRecipe recipe)
{
    using lcnc::cam::CamMotionBlock;
    using lcnc::cam::MotionProcessFence;
    using lcnc::cam::finalizeMotionPlan;
#define LCNC_EXECUTION_FIELD(type, name, legacy) checkField(&FrozenToolExecutionRecipe::name, snapshot, recipe);
#include "modules/process/runtime/frozen_tool_execution_fields.inc"
#undef LCNC_EXECUTION_FIELD
    for (auto field : {&FrozenToolExecutionRecipe::toolName, &FrozenToolExecutionRecipe::sourceId,
                      &FrozenToolExecutionRecipe::profileSchema}) {
        auto changed = recipe;
        changed.toolsByContour[9].*field += QStringLiteral("-changed");
        assert(deviceRunRecipeHash(changed) != recipe.revision);
    }
    QString error;
    auto fallback = recipe;
    fallback.toolsByContour[9].toolName = QStringLiteral("__fallback__");
    fallback.revision = deviceRunRecipeHash(fallback);
    assert(!PreparedDeviceProgram::prepare(snapshot, fallback, 42, false, &error));

    Tool legacy;
    legacy.m_strName = "explicit";
    for (auto field : {&Tool::m_dXPosition, &Tool::m_dX1Position, &Tool::m_dYPosition, &Tool::m_dY1Position,
        &Tool::m_dAPosition, &Tool::m_dA1Position, &Tool::m_dAPos, &Tool::m_dA1Pos,
        &Tool::m_dExtend, &Tool::m_dExtend_End, &Tool::m_dOffsetDistance, &Tool::m_dOffsetDiameter,
        &Tool::m_dOffsetIgnoreLength, &Tool::m_dRadius, &Tool::m_dFocusOffset,
        &Tool::m_dServoCuttingHeight, &Tool::m_dLinkageParameterA, &Tool::m_dLinkageParameterB}) {
        for (const double bad : {1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
            auto changed = legacy; changed.*field = bad;
            assert(!freezeToolExecutionRecipe(changed, QStringLiteral("explicit"), &error));
        }
    }
    for (auto field : {&Tool::m_bXIsMove, &Tool::m_bX1IsMove, &Tool::m_bYIsMove, &Tool::m_bY1IsMove,
        &Tool::m_bAIsMove, &Tool::m_bA1IsMove, &Tool::m_bAZero, &Tool::m_bA1Zero,
        &Tool::m_bAxisZLinkage, &Tool::m_bCuttingHead, &Tool::m_bCrossBridge, &Tool::m_bFlightCutting}) {
        auto changed = legacy; changed.*field = true;
        assert(!freezeToolExecutionRecipe(changed, QStringLiteral("explicit"), &error));
    }

    auto& plan = snapshot.motionPlan;
    auto next = plan.blocks.back();
    next.blockId = 14;
    next.contourId = 10;
    next.entryBoundary = next.physicalKnots.back();
    next.physicalKnots[0].axes[0] = 36;
    next.physicalKnots[0].contourId = 10;
    next.sourceSpans = {{10, -1, 0, 0, 1, -1}};
    plan.blocks.append(next);
    assert(finalizeMotionPlan(&plan, &error));
    recipe.toolsByContour.insert(10, recipe.toolsByContour[9]);
    recipe.planHash = plan.planHash;
    recipe.contextHash = plan.contextHash;
    recipe.revision = deviceRunRecipeHash(recipe);
    const auto program = PreparedDeviceProgram::prepare(snapshot, recipe, 42, false, &error);
    assert(program && program->sections().size() == 2);
    const auto& first = program->sections()[0];
    const auto& second = program->sections()[1];
    assert(first.ordinal == 0 && first.contourId == 9 && first.firstBlockId == 11 && first.lastBlockId == 13);
    assert(second.ordinal == 1 && second.contourId == 10 && second.firstBlock == 3 && second.lastBlock == 3);
    assert(first.planHash == plan.planHash && second.runEpoch == 42);
    assert(first.lastBlockHash == plan.blocks[2].blockHash && second.firstBlockHash == plan.blocks[3].blockHash);
    struct SectionTrace final : ExactPlanConsumer {
        QVector<std::uint64_t> blocks;
        QVector<double> knots;
        bool begin(const PreparedDeviceProgram&, QString*) override { return true; }
        bool block(const CamMotionBlock& b, const FrozenToolExecutionRecipe&, QString*) override {
            blocks.append(b.blockId); return true;
        }
        bool knot(const CamMotionBlock& b, int index, QString*) override {
            knots.append(b.physicalKnots[index].axes[0]); return true;
        }
        bool fence(const CamMotionBlock&, const MotionProcessFence&, QString*) override { return true; }
        bool seal(QString*) override { return true; }
        void discard() noexcept override {}
    } sectionTrace;
    assert(consumeExactSection(*program, 1, sectionTrace, {}, &error));
    assert(sectionTrace.blocks == QVector<std::uint64_t>{14});
    assert(sectionTrace.knots == QVector<double>{36}); // predecessor 24 is not re-emitted
    auto unsafe = snapshot;
    unsafe.motionPlan.blocks[2].fences.back().laserEnabledAfterFence = true;
    unsafe.motionPlan.blocks[2].fences.back().changesLaserState = false;
    unsafe.motionPlan.blocks[3].fences.front().changesLaserState = false;
    assert(finalizeMotionPlan(&unsafe.motionPlan, &error));
    auto unsafeRecipe = recipe;
    unsafeRecipe.planHash = unsafe.motionPlan.planHash;
    unsafeRecipe.contextHash = unsafe.motionPlan.contextHash;
    unsafeRecipe.revision = deviceRunRecipeHash(unsafeRecipe);
    assert(!PreparedDeviceProgram::prepare(unsafe, unsafeRecipe, 42, false, &error));
    assert(error.contains("pause-safe"));

    // Scenario 0: live pause/resume; 1: pause then Stop; 2: paused before
    // initial Start; 3: pause during preparation; 4: pause in queued health;
    // 5: Start failure must never be replayed.
    for (int scenario = 0; scenario < 6; ++scenario) {
        ProcessInterruptContext interrupt;
        DeviceCommandQueue queue;
        assert(queue.start());
        struct FakeSink final : IExactSectionSink {
            const PreparedDeviceProgram* expected{};
            ProcessInterruptContext* interrupt{};
            DeviceCommandQueue* queue{};
            int scenario{};
            std::atomic<int> starts{0};
            std::atomic<int> preparations{0};
            std::atomic<int> completed{0};
            QSemaphore secondStarted;
            bool check(const PreparedDeviceProgram& p) {
                return &p == expected && p.runEpoch() == 42 && queue->isWorkerThread()
                    && p.recipe().revision == expected->recipe().revision;
            }
            bool prepareExactSection(const PreparedDeviceProgram& p, int index, QString*) override {
                assert(check(p));
                assert(index == preparations.fetch_add(1));
                if (scenario == 3 && index == 0) interrupt->requestPause();
                return true;
            }
            bool startExactSection(const PreparedDeviceProgram& p, int index, QString* error) override {
                assert(check(p));
                assert(!interrupt->isPaused() && !interrupt->isStopping());
                assert(index == starts.fetch_add(1));
                if (index == 1) secondStarted.release();
                if (scenario == 5) { if (error) *error = QStringLiteral("Start failed"); return false; }
                if ((scenario == 0 || scenario == 1) && index == 0) interrupt->requestPause();
                return true;
            }
            bool isExactSectionRunning(const PreparedDeviceProgram& p, int index, QString*) override {
                assert(check(p));
                assert(index == completed.fetch_add(1));
                return false;
            }
        } sink;
        sink.expected = program.get(); sink.interrupt = &interrupt; sink.queue = &queue; sink.scenario = scenario;
        if (scenario == 2) interrupt.requestPause();
        QSemaphore finished;
        bool ok = false;
        QString executionError;
        std::atomic<int> healthCalls{0};
        std::atomic<int> safeStops{0};
        const auto health = [&] {
            assert(queue.isWorkerThread());
            if (scenario == 4 && healthCalls.fetch_add(1) == 0) interrupt.requestPause();
            return DeviceCommandResult{};
        };
        std::thread workflow([&] {
            ok = executePreparedSections(*program, sink, queue, interrupt, QStringLiteral("exact"), health, &executionError);
            if (!ok) queue.executeAndWait([&] { ++safeStops; return DeviceCommandResult{}; }, TaskPriority::Stop, -1);
            finished.release();
        });
        if (scenario < 2) {
            assert(await([&] { return interrupt.resumePoint(QStringLiteral("exact")).env.value(QStringLiteral("nextSection"), -1).toInt() == 1; }));
            assert(sink.completed == 1 && sink.starts == 1);
            assert(!sink.secondStarted.tryAcquire(1, 50));
            const auto point = interrupt.resumePoint(QStringLiteral("exact"));
            assert(point.env.value(QStringLiteral("runEpoch")).toULongLong() == 42);
            assert(point.env.value(QStringLiteral("planHash")).toByteArray() == program->plan().planHash);
            assert(point.env.value(QStringLiteral("recipeRevision")).toByteArray() == program->recipe().revision);
            if (scenario == 1) interrupt.requestStop();
            else interrupt.requestResume();
        } else if (scenario < 5) {
            assert(await([&] { return interrupt.isPaused() && interrupt.hasResumePoint(QStringLiteral("exact")); }));
            assert(sink.starts == 0);
            assert(!finished.tryAcquire(1, 50));
            if (scenario == 2) interrupt.requestStop();
            else interrupt.requestResume();
        }
        const bool done = finished.tryAcquire(1, 3000);
        if (!done) interrupt.requestStop();
        workflow.join();
        assert(done);
        if (scenario == 1 || scenario == 2 || scenario == 5) {
            assert(!ok && safeStops == 1);
            const int starts = sink.starts;
            // Re-entering a completed/aborted workflow invocation cannot use
            // its saved checkpoint as an automatic replay authorization.
            interrupt.requestResume();
            assert(!executePreparedSections(*program, sink, queue, interrupt, QStringLiteral("exact"), health, &error));
            assert(sink.starts == starts);
        } else {
            assert(ok && sink.starts == 2 && sink.completed == 2 && safeStops == 0);
            assert(!interrupt.hasResumePoint(QStringLiteral("exact")));
        }
        assert(queue.shutdown());
    }
}
} // namespace lcnc::process::b2_s1_test
