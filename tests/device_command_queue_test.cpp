#include "modules/process/runtime/device_command_queue.h"

#include <QCoreApplication>
#include <QMutex>
#include <QSemaphore>
#include <QTextStream>
#include <QThread>

#include <algorithm>
#include <stdexcept>
#include <thread>

namespace {

int fail(const QString& message)
{
    QTextStream(stderr) << message << '\n';
    return 1;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    lcnc::process::DeviceCommandQueue queue;
    if (!queue.start())
        return fail(QStringLiteral("Device command queue did not start"));

    const auto guiNormalWait = queue.executeAndWait(
        [] { return lcnc::process::DeviceCommandResult{}; }, TaskPriority::Normal, 100);
    if (guiNormalWait.completion != lcnc::process::DeviceCommandCompletion::Failed)
        return fail(QStringLiteral("GUI thread was allowed to block a normal device command"));
    const auto guiStopWait = queue.executeAndWait(
        [] { return lcnc::process::DeviceCommandResult{}; }, TaskPriority::Stop, 1000);
    if (!guiStopWait.success)
        return fail(QStringLiteral("GUI-thread shutdown Stop command did not complete"));

    QMutex orderMutex;
    QStringList order;
    QSemaphore normalStarted;
    QSemaphore allowNormalFinish;
    QSemaphore completed;
    bool allCommandsUsedWorkerThread = true;

    if (!queue.submit([&] {
            allCommandsUsedWorkerThread = allCommandsUsedWorkerThread && queue.isWorkerThread();
            {
                QMutexLocker locker(&orderMutex);
                order.append(QStringLiteral("normal-active"));
            }
            normalStarted.release();
            allowNormalFinish.acquire();
            {
                QMutexLocker locker(&orderMutex);
                order.append(QStringLiteral("normal-finished"));
            }
            completed.release();
        })) {
        return fail(QStringLiteral("Could not queue active normal command"));
    }

    if (!normalStarted.tryAcquire(1, 1000))
        return fail(QStringLiteral("Normal command did not start"));

    if (!queue.submit([&] {
            allCommandsUsedWorkerThread = allCommandsUsedWorkerThread && queue.isWorkerThread();
            QMutexLocker locker(&orderMutex);
            order.append(QStringLiteral("normal-queued-1"));
            completed.release();
        })
        || !queue.submit([&] {
            allCommandsUsedWorkerThread = allCommandsUsedWorkerThread && queue.isWorkerThread();
            QMutexLocker locker(&orderMutex);
            order.append(QStringLiteral("normal-queued-2"));
            completed.release();
        })
        || !queue.submit([&] {
            allCommandsUsedWorkerThread = allCommandsUsedWorkerThread && queue.isWorkerThread();
            QMutexLocker locker(&orderMutex);
            order.append(QStringLiteral("interactive"));
            completed.release();
        }, TaskPriority::Interactive)
        || !queue.submit([&] {
            allCommandsUsedWorkerThread = allCommandsUsedWorkerThread && queue.isWorkerThread();
            QMutexLocker locker(&orderMutex);
            order.append(QStringLiteral("poll-old"));
            completed.release();
        }, TaskPriority::Polling, QStringLiteral("controller-status"))
        || !queue.submit([&] {
            allCommandsUsedWorkerThread = allCommandsUsedWorkerThread && queue.isWorkerThread();
            QMutexLocker locker(&orderMutex);
            order.append(QStringLiteral("poll-latest"));
            completed.release();
        }, TaskPriority::Polling, QStringLiteral("controller-status"))
        || !queue.submitWorkflow([&] {
            allCommandsUsedWorkerThread = allCommandsUsedWorkerThread && queue.isWorkerThread();
            QMutexLocker locker(&orderMutex);
            order.append(QStringLiteral("workflow"));
            completed.release();
        })
        || !queue.submitStop([&] {
            allCommandsUsedWorkerThread = allCommandsUsedWorkerThread && queue.isWorkerThread();
            QMutexLocker locker(&orderMutex);
            order.append(QStringLiteral("stop"));
            completed.release();
        })) {
        return fail(QStringLiteral("Could not queue priority commands"));
    }

    allowNormalFinish.release();
    if (!completed.tryAcquire(7, 2000))
        return fail(QStringLiteral("Queued commands did not finish"));
    if (!queue.shutdown(2000))
        return fail(QStringLiteral("Device command queue did not stop"));

    const QStringList expected = {
        QStringLiteral("normal-active"),
        QStringLiteral("normal-finished"),
        QStringLiteral("stop"),
        QStringLiteral("workflow"),
        QStringLiteral("interactive"),
        QStringLiteral("normal-queued-1"),
        QStringLiteral("normal-queued-2"),
        QStringLiteral("poll-latest"),
    };
    if (!allCommandsUsedWorkerThread || order != expected)
        return fail(QStringLiteral("Device queue did not preserve worker affinity or Stop priority"));

    lcnc::process::DeviceCommandQueue completionQueue;
    if (!completionQueue.start())
        return fail(QStringLiteral("Completion queue did not start"));
    QMutex completionMutex;
    QList<lcnc::process::DeviceCommandCompletion> completions;
    const auto record = [&](const lcnc::process::DeviceCommandResult& result) {
        QMutexLocker locker(&completionMutex);
        completions.append(result.completion);
    };
    QSemaphore activeStarted;
    QSemaphore releaseActive;
    if (!completionQueue.submit([&] { activeStarted.release(); releaseActive.acquire(); })
        || !activeStarted.tryAcquire(1, 1000))
        return fail(QStringLiteral("Completion queue active command did not start"));

    const auto oldTicket = completionQueue.submitWithTicket(
        [] { return lcnc::process::DeviceCommandResult{}; }, TaskPriority::Polling,
        record, QStringLiteral("status"));
    const auto newTicket = completionQueue.submitWithTicket(
        [] { return lcnc::process::DeviceCommandResult{}; }, TaskPriority::Polling,
        record, QStringLiteral("status"));
    const auto cancelledTicket = completionQueue.submitWithTicket(
        [] { return lcnc::process::DeviceCommandResult{}; }, TaskPriority::Normal, record);
    if (!oldTicket.accepted || !newTicket.accepted || oldTicket.id == newTicket.id
        || !cancelledTicket.accepted || !completionQueue.cancel(cancelledTicket.id))
        return fail(QStringLiteral("Device queue ticket cancellation/coalescing failed"));

    QSemaphore timeoutDone;
    lcnc::process::DeviceCommandCompletion timeoutStatus = lcnc::process::DeviceCommandCompletion::Succeeded;
    std::thread timeoutWaiter([&] {
        const auto timeout = completionQueue.executeAndWait(
            [] { return lcnc::process::DeviceCommandResult{}; }, TaskPriority::Normal, 20);
        timeoutStatus = timeout.completion;
        timeoutDone.release();
    });
    if (!timeoutDone.tryAcquire(1, 1000))
        return fail(QStringLiteral("Timed command wait did not return"));
    timeoutWaiter.join();
    if (timeoutStatus != lcnc::process::DeviceCommandCompletion::TimedOut)
        return fail(QStringLiteral("Timed command wait did not report TimedOut"));
    if (completionQueue.submit([] {}, TaskPriority::Normal))
        return fail(QStringLiteral("Queue accepted ordinary work behind a timed-out command"));
    if (!completionQueue.submitStop([] {}))
        return fail(QStringLiteral("Queue rejected Stop work behind a timed-out command"));

    releaseActive.release();
    QThread::msleep(50);
    if (!completionQueue.submit([] {}, TaskPriority::Normal))
        return fail(QStringLiteral("Queue did not recover after the timed-out command exited"));
    if (!completionQueue.shutdown(2000))
        return fail(QStringLiteral("Completion queue did not stop"));
    QMutexLocker completionLocker(&completionMutex);
    if (!completions.contains(lcnc::process::DeviceCommandCompletion::Superseded)
        || !completions.contains(lcnc::process::DeviceCommandCompletion::Cancelled)
        || !completions.contains(lcnc::process::DeviceCommandCompletion::Succeeded))
        return fail(QStringLiteral("Device queue did not report completion outcomes"));

    lcnc::process::DeviceCommandQueue shutdownQueue;
    if (!shutdownQueue.start())
        return fail(QStringLiteral("Shutdown queue did not start"));
    QSemaphore shutdownActive;
    QSemaphore releaseShutdownActive;
    QSemaphore shutdownCompletion;
    lcnc::process::DeviceCommandCompletion shutdownStatus = lcnc::process::DeviceCommandCompletion::Succeeded;
    if (!shutdownQueue.submit([&] { shutdownActive.release(); releaseShutdownActive.acquire(); })
        || !shutdownActive.tryAcquire(1, 1000)
        || !shutdownQueue.submitWithTicket(
            [] { return lcnc::process::DeviceCommandResult{}; }, TaskPriority::Normal,
            [&](const lcnc::process::DeviceCommandResult& result) {
                shutdownStatus = result.completion;
                shutdownCompletion.release();
            }).accepted)
        return fail(QStringLiteral("Shutdown completion setup failed"));
    std::thread shutdownWaiter([&] { shutdownQueue.shutdown(2000); });
    QThread::msleep(20);
    releaseShutdownActive.release();
    shutdownWaiter.join();
    if (!shutdownCompletion.tryAcquire(1, 1000)
        || shutdownStatus != lcnc::process::DeviceCommandCompletion::Shutdown)
        return fail(QStringLiteral("Shutdown did not complete queued command as Shutdown"));

    lcnc::process::DeviceCommandQueue exceptionQueue;
    if (!exceptionQueue.start())
        return fail(QStringLiteral("Exception queue did not start"));
    QSemaphore exceptionCompleted;
    lcnc::process::DeviceCommandCompletion exceptionStatus =
        lcnc::process::DeviceCommandCompletion::Succeeded;
    if (!exceptionQueue.submit(
            []() -> lcnc::process::DeviceCommandResult {
                throw std::runtime_error("expected device command failure");
            },
            TaskPriority::Normal,
            [&](const lcnc::process::DeviceCommandResult& result) {
                exceptionStatus = result.completion;
                exceptionCompleted.release();
            })
        || !exceptionCompleted.tryAcquire(1, 1000)
        || exceptionStatus != lcnc::process::DeviceCommandCompletion::Failed
        || !exceptionQueue.shutdown(2000)) {
        return fail(QStringLiteral("Exception did not complete as Failed"));
    }

    lcnc::process::DeviceCommandQueue completionThrowQueue;
    if (!completionThrowQueue.start())
        return fail(QStringLiteral("Completion-throw queue did not start"));
    QSemaphore completionThrowCommandRan;
    QSemaphore completionThrowFollowUpRan;
    int throwingCompletionCalls = 0;
    if (!completionThrowQueue.submit(
            [&] {
                completionThrowCommandRan.release();
                return lcnc::process::DeviceCommandResult{};
            },
            TaskPriority::Normal,
            [&](const lcnc::process::DeviceCommandResult&) {
                ++throwingCompletionCalls;
                throw std::runtime_error("expected completion failure");
            })
        || !completionThrowCommandRan.tryAcquire(1, 1000)
        || !completionThrowQueue.submit([&] { completionThrowFollowUpRan.release(); })
        || !completionThrowFollowUpRan.tryAcquire(1, 1000)
        || throwingCompletionCalls != 1
        || !completionThrowQueue.shutdown(2000)) {
        return fail(QStringLiteral("Throwing completion was retried or stopped the device thread"));
    }

    lcnc::process::DeviceCommandQueue stopOnlyQueue;
    if (!stopOnlyQueue.start())
        return fail(QStringLiteral("Stop-only queue did not start"));
    QSemaphore stopOnlyActiveStarted;
    QSemaphore releaseStopOnlyActive;
    QSemaphore stopOnlyCancelled;
    QSemaphore stopOnlyStopRan;
    QSemaphore stopOnlyRecoveredRan;
    QMutex stopOnlyMutex;
    QList<lcnc::process::DeviceCommandCompletion> stopOnlyCompletions;
    bool staleStopOnlyCommandRan = false;
    if (!stopOnlyQueue.submit([&] {
            stopOnlyActiveStarted.release();
            releaseStopOnlyActive.acquire();
        })
        || !stopOnlyActiveStarted.tryAcquire(1, 1000)) {
        return fail(QStringLiteral("Stop-only queue active command did not start"));
    }
    const auto submitStaleCommand = [&](TaskPriority priority) {
        return stopOnlyQueue.submit(
            [&] {
                staleStopOnlyCommandRan = true;
                return lcnc::process::DeviceCommandResult{};
            },
            priority,
            [&](const lcnc::process::DeviceCommandResult& result) {
                QMutexLocker locker(&stopOnlyMutex);
                stopOnlyCompletions.append(result.completion);
                stopOnlyCancelled.release();
            });
    };
    if (!submitStaleCommand(TaskPriority::Workflow)
        || !submitStaleCommand(TaskPriority::Interactive)
        || !submitStaleCommand(TaskPriority::Normal)
        || !submitStaleCommand(TaskPriority::Polling)) {
        releaseStopOnlyActive.release();
        return fail(QStringLiteral("Could not queue stale Stop-only commands"));
    }
    stopOnlyQueue.beginStopOnly();
    if (stopOnlyQueue.submit([] {}, TaskPriority::Polling)
        || stopOnlyQueue.submit([] {}, TaskPriority::Workflow)
        || !stopOnlyQueue.submitStop([&] { stopOnlyStopRan.release(); })
        || !stopOnlyCancelled.tryAcquire(4, 1000)) {
        releaseStopOnlyActive.release();
        return fail(QStringLiteral("Stop-only admission policy failed"));
    }
    releaseStopOnlyActive.release();
    if (!stopOnlyStopRan.tryAcquire(1, 1000)
        || staleStopOnlyCommandRan) {
        return fail(QStringLiteral("Stop-only queue executed stale commands after safety Stop"));
    }
    {
        QMutexLocker locker(&stopOnlyMutex);
        if (stopOnlyCompletions.size() != 4
            || std::any_of(stopOnlyCompletions.cbegin(), stopOnlyCompletions.cend(),
                           [](lcnc::process::DeviceCommandCompletion completion) {
                               return completion != lcnc::process::DeviceCommandCompletion::Cancelled;
                           })) {
            return fail(QStringLiteral("Stop-only queue did not cancel every pending non-Stop command"));
        }
    }
    stopOnlyQueue.endStopOnly();
    if (!stopOnlyQueue.submit([&] { stopOnlyRecoveredRan.release(); }, TaskPriority::Polling)
        || !stopOnlyRecoveredRan.tryAcquire(1, 1000)
        || !stopOnlyQueue.shutdown(2000)) {
        return fail(QStringLiteral("Stop-only recovery policy failed"));
    }

    lcnc::process::DeviceCommandQueue lifecycleQueue;
    if (lifecycleQueue.isWorkerThread())
        return fail(QStringLiteral("Caller thread was mistaken for the device executor"));
    for (int iteration = 0; iteration < 100; ++iteration) {
        QSemaphore iterationCompleted;
        bool ranOnDeviceThread = false;
        if (!lifecycleQueue.start()
            || !lifecycleQueue.submit([&] {
                ranOnDeviceThread = lifecycleQueue.isWorkerThread();
                iterationCompleted.release();
            })
            || !iterationCompleted.tryAcquire(1, 1000)
            || !ranOnDeviceThread
            || !lifecycleQueue.shutdown(2000)
            || !lifecycleQueue.shutdown(2000)) {
            return fail(QStringLiteral("Repeated queue lifecycle failed at iteration %1")
                            .arg(iteration));
        }
    }

    return 0;
}
