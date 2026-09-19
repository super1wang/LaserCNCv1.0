#include "modules/process/runtime/exact_section_execution.h"

#include "modules/process/runtime/process_interrupt_context.h"
#include "modules/process/runtime/queued_motion_wait.h"

namespace lcnc::process {
bool executePreparedSections(const PreparedDeviceProgram& program, IExactSectionSink& sink,
    DeviceCommandQueue& queue, ProcessInterruptContext& interrupt, const QString& nodeId,
    const std::function<DeviceCommandResult()>& boundaryHealth, QString* error)
{
    if (error) error->clear();
    const auto fail = [&](const QString& reason) {
        if (error) *error = reason;
        return false;
    };
    if (queue.isWorkerThread() || program.sections().isEmpty() || !boundaryHealth)
        return fail(QStringLiteral("Invalid exact-section execution context"));
    if (interrupt.hasResumePoint(nodeId))
        return fail(QStringLiteral("Exact-plan replay is forbidden; fresh run required"));
    const auto checkpoint = [&](int nextSection) {
        return interrupt.checkpoint(nodeId, QStringLiteral("exactSectionBoundary"),
            {{QStringLiteral("nextSection"), nextSection},
             {QStringLiteral("planHash"), program.plan().planHash},
             {QStringLiteral("contextHash"), program.plan().contextHash},
             {QStringLiteral("recipeRevision"), program.recipe().revision},
             {QStringLiteral("runEpoch"), qulonglong(program.runEpoch())}});
    };
    for (const auto& section : program.sections()) {
        if (!checkpoint(section.ordinal)) return fail(QStringLiteral("Exact-plan run stopped"));
        const auto prepared = queue.executeAndWait([&] {
            if (interrupt.isStopping()) return DeviceCommandResult{false, QStringLiteral("Run stopped")};
            QString reason;
            const bool ok = sink.prepareExactSection(program, section.ordinal, &reason);
            return DeviceCommandResult{ok, reason};
        }, TaskPriority::Workflow, -1);
        if (!prepared.success) return fail(prepared.error);
        bool sealed = false;
        while (!sealed) {
            const auto filled = queue.executeAndWait([&] {
                if (interrupt.isStopping()) return DeviceCommandResult{false, QStringLiteral("Run stopped during fill")};
                QString reason;
                const bool ok = sink.continueExactPreparation(program, section.ordinal, sealed, &reason);
                return DeviceCommandResult{ok, reason};
            }, TaskPriority::Workflow, -1);
            if (!filled.success) return fail(filled.error);
        }
        // Pause can arrive during encode or while Start is queued. Recheck at
        // the device-side admission point; defer ONLY an unstarted section.
        for (;;) {
            if (!checkpoint(section.ordinal)) return fail(QStringLiteral("Exact-plan run stopped"));
            bool deferred = false;
            const auto started = queue.executeAndWait([&] {
                if (interrupt.isStopping()) return DeviceCommandResult{false, QStringLiteral("Run stopped")};
                const auto health = boundaryHealth();
                if (!health.success) return health;
                if (interrupt.isStopping()) return DeviceCommandResult{false, QStringLiteral("Run stopped")};
                if (interrupt.isPaused()) { deferred = true; return DeviceCommandResult{}; }
                QString reason;
                const bool ok = sink.startExactSection(program, section.ordinal, &reason);
                return DeviceCommandResult{ok, reason};
            }, TaskPriority::Workflow, -1);
            if (!started.success) return fail(started.error); // never retry Start
            if (!deferred) break;
        }
        const auto completed = waitForQueuedMotion(queue, [&](bool& running) {
            QString reason;
            running = sink.isExactSectionRunning(program, section.ordinal, &reason);
            return DeviceCommandResult{reason.isEmpty(), reason};
        }, [&] { return interrupt.isStopping(); });
        if (!completed.success) return fail(completed.error);
    }
    if (!checkpoint(int(program.sections().size()))) return fail(QStringLiteral("Exact-plan run stopped"));
    interrupt.clearResumePoint(nodeId);
    return true;
}
} // namespace lcnc::process
