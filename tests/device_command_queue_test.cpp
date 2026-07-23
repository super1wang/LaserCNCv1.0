#include "modules/process/runtime/device_command_queue.h"

#include <QCoreApplication>
#include <QMutex>
#include <QSemaphore>
#include <QTextStream>
#include <QThread>

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
            order.append(QStringLiteral("normal-queued"));
            completed.release();
        })
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
        || !queue.submitEmergency([&] {
            allCommandsUsedWorkerThread = allCommandsUsedWorkerThread && queue.isWorkerThread();
            QMutexLocker locker(&orderMutex);
            order.append(QStringLiteral("emergency"));
            completed.release();
        })) {
        return fail(QStringLiteral("Could not queue priority commands"));
    }

    allowNormalFinish.release();
    if (!completed.tryAcquire(5, 2000))
        return fail(QStringLiteral("Queued commands did not finish"));
    if (!queue.shutdown(2000))
        return fail(QStringLiteral("Device command queue did not stop"));

    const QStringList expected = {
        QStringLiteral("normal-active"),
        QStringLiteral("normal-finished"),
        QStringLiteral("emergency"),
        QStringLiteral("workflow"),
        QStringLiteral("normal-queued"),
        QStringLiteral("poll-latest"),
    };
    if (!allCommandsUsedWorkerThread || order != expected)
        return fail(QStringLiteral("Device queue did not preserve worker affinity or emergency priority"));

    return 0;
}
