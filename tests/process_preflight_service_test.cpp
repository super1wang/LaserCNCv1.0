#include "modules/process/runtime/process_preflight_service.h"

#include <QSemaphore>
#include <QThread>

#include <atomic>
#include <cassert>
#include <memory>

using namespace lcnc::process;

int main()
{
    DeviceCommandQueue queue;
    assert(queue.start());

    const Qt::HANDLE callerThread = QThread::currentThreadId();
    Qt::HANDLE runnerThread = nullptr;
    ProcessPreflightService service(
        queue,
        [&runnerThread](const ProcessPreflightRequest& request, ProcessPreflightReport* report) {
            runnerThread = QThread::currentThreadId();
            assert(request.axisNames == QStringList({QStringLiteral("X"), QStringLiteral("Y")}));
            report->axisPositions.insert(QStringLiteral("X"), 12.5);
            report->axisEnabled.insert(QStringLiteral("X"), true);
            return DeviceCommandResult{};
        });

    QSemaphore completed;
    std::uint64_t completedGeneration = 0;
    std::shared_ptr<const ProcessPreflightReport> completedReport;
    DeviceCommandResult completedResult;
    ProcessPreflightRequest request;
    request.axisNames = {QStringLiteral("X"), QStringLiteral("Y")};
    const DeviceCommandTicket ticket = service.request(
        41,
        request,
        [&](std::uint64_t generation,
            const DeviceCommandResult& result,
            std::shared_ptr<const ProcessPreflightReport> report) {
            completedGeneration = generation;
            completedResult = result;
            completedReport = std::move(report);
            completed.release();
        });
    assert(ticket.accepted);
    assert(ticket.id != 0);
    assert(completed.tryAcquire(1, 2000));
    assert(completedGeneration == 41);
    assert(completedResult.success);
    assert(completedResult.completion == DeviceCommandCompletion::Succeeded);
    assert(completedReport);
    assert(completedReport->axisPositions.value(QStringLiteral("X")) == 12.5);
    assert(completedReport->axisEnabled.value(QStringLiteral("X")));
    assert(runnerThread != nullptr && runnerThread != callerThread);

    QSemaphore blockerEntered;
    QSemaphore releaseBlocker;
    assert(queue.submit(
        [&] {
            blockerEntered.release();
            releaseBlocker.acquire();
        },
        TaskPriority::Stop));
    assert(blockerEntered.tryAcquire(1, 2000));

    QSemaphore cancelled;
    DeviceCommandResult cancelledResult;
    const auto cancelledTicket = service.request(
        42,
        request,
        [&](std::uint64_t generation,
            const DeviceCommandResult& result,
            std::shared_ptr<const ProcessPreflightReport>) {
            assert(generation == 42);
            cancelledResult = result;
            cancelled.release();
        });
    assert(cancelledTicket.accepted);
    assert(queue.cancel(cancelledTicket.id));
    assert(cancelled.tryAcquire(1, 2000));
    assert(!cancelledResult.success);
    assert(cancelledResult.completion == DeviceCommandCompletion::Cancelled);
    releaseBlocker.release();

    assert(queue.shutdown(2000));
    return 0;
}
