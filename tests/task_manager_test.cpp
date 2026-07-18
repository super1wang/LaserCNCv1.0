#include "core/task/task_manager.h"

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QTextStream>
#include <QThread>

#include <stdexcept>

namespace {

int fail(const QString& message)
{
    QTextStream(stderr) << message << '\n';
    return 1;
}

bool pumpUntil(const std::function<bool()>& predicate, int timeoutMs)
{
    QElapsedTimer timer;
    timer.start();
    while (!predicate() && timer.elapsed() < timeoutMs) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        QThread::msleep(1);
    }
    return predicate();
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    TaskManager manager;

    bool abortObserved = false;
    bool abortFinished = false;
    bool abortSuccess = false;
    TaskId abortTask = manager.run(QStringLiteral("cooperative abort"), [&abortObserved](TaskProgress* progress) {
        while (!progress->isAbortRequested()) {
            progress->setValue(25);
            progress->setStepName(QStringLiteral("running"));
            QThread::msleep(1);
        }
        abortObserved = true;
    });
    QObject::connect(&manager, &TaskManager::taskFinished, &app,
        [&abortFinished, &abortSuccess, abortTask](TaskId id, bool success) {
            if (id == abortTask) {
                abortFinished = true;
                abortSuccess = success;
            }
        });

    if (!pumpUntil([&manager, abortTask] { return manager.isRunning(abortTask); }, 500))
        return fail(QStringLiteral("Task did not start"));
    if (manager.waitForDone(abortTask, 1))
        return fail(QStringLiteral("Timed wait unexpectedly completed a running task"));
    manager.requestAbort(abortTask);
    if (!manager.waitForDone(abortTask, 2000)
        || !pumpUntil([&abortFinished] { return abortFinished; }, 2000))
        return fail(QStringLiteral("Cooperative task did not finish after abort"));
    if (!abortObserved || !abortSuccess || manager.isRunning(abortTask))
        return fail(QStringLiteral("Cooperative abort task produced an invalid terminal state"));

    bool exceptionFinished = false;
    bool exceptionSuccess = true;
    TaskId exceptionTask = manager.run(QStringLiteral("throwing task"), [](TaskProgress*) {
        throw std::runtime_error("intentional test failure");
    });
    QObject::connect(&manager, &TaskManager::taskFinished, &app,
        [&exceptionFinished, &exceptionSuccess, exceptionTask](TaskId id, bool success) {
            if (id == exceptionTask) {
                exceptionFinished = true;
                exceptionSuccess = success;
            }
        });
    if (!manager.waitForDone(exceptionTask, 2000)
        || !pumpUntil([&exceptionFinished] { return exceptionFinished; }, 2000))
        return fail(QStringLiteral("Throwing task did not finish"));
    if (exceptionSuccess)
        return fail(QStringLiteral("Throwing task was reported as successful"));

    return 0;
}
