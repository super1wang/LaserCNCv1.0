#include "core/task/task_manager.h"

#include <QFuture>
#include <QElapsedTimer>
#include <QThread>
#include <QtConcurrent/QtConcurrent>

#include <utility>

#include "core/logging/logger.h"

// ── Singleton accessor ────────────────────────────────────────────────────────
//   Lifecycle owned by lcnc::Kernel.
//   Use lcnc::Kernel::current().taskManager() to access this object.
TaskManager* TaskManager::s_instance = nullptr;

TaskManager::TaskManager(QObject* parent)
    : QObject(parent)
{
    Q_ASSERT_X(!s_instance, "TaskManager",
               "second TaskManager instance — must be Kernel-owned only");
    s_instance = this;
}

TaskManager::~TaskManager()
{
    // Jobs may retain borrowed module/document pointers.  Keep their progress
    // objects and watcher entities alive until every worker has left its
    // callable; otherwise shutdown can race into use-after-free.
    for (Entity* entity : std::as_const(m_tasks)) {
        if (entity && entity->progress)
            entity->progress->requestAbort();
    }
    for (Entity* entity : std::as_const(m_tasks)) {
        if (entity && entity->watcher)
            entity->watcher->waitForFinished();
    }

    const auto remaining = m_tasks;
    m_tasks.clear();
    for (Entity* entity : remaining) {
        if (!entity)
            continue;
        if (entity->watcher) {
            disconnect(entity->watcher, nullptr, this, nullptr);
            delete entity->watcher;
        }
        delete entity->progress;
        delete entity;
    }

    if (s_instance == this) s_instance = nullptr;
}

// ── Public interface ───────────────────────────────────────────────────────────
TaskId TaskManager::run(const QString& label, TaskJob job)
{
    const TaskId id = m_nextId++;

    auto* entity          = new Entity;
    entity->id            = id;
    entity->label         = label;
    entity->progress      = new TaskProgress();
    entity->watcher       = new QFutureWatcher<void>(this);
    entity->success       = false;

    // Wire progress callbacks to Qt signals (invoked from worker thread via
    // Qt::QueuedConnection when dispatched through the watcher)
    TaskProgress* prog = entity->progress;
    prog->setCallback([this, id](int pct, const QString& step) {
        // These are invoked from the worker thread; use Qt::QueuedConnection.
        QMetaObject::invokeMethod(this, [this, id, pct, step]() {
            emit taskProgressChanged(id, pct);
            if (!step.isEmpty())
                emit taskStepChanged(id, step);
        }, Qt::QueuedConnection);
    });

    m_tasks.insert(id, entity);

    // Connect watcher finished signal
    connect(entity->watcher, &QFutureWatcher<void>::finished,
            this, [this, id]() { onTaskFinished(id); });

    // Launch async
    emit taskStarted(id, label);
    QFuture<void> future = QtConcurrent::run([job, prog, entity, label]() {
        try {
            job(prog);
            entity->success = true;
        } catch (const std::exception& e) {
            entity->success = false;
            LCNC_ERR(lcnc::LogCode::TaskUnhandled,
                     "TaskManager: task '{}' threw std::exception: {}",
                     label.toStdString(), e.what());
        } catch (...) {
            entity->success = false;
            LCNC_ERR(lcnc::LogCode::TaskUnhandled,
                     "TaskManager: task '{}' threw unknown exception",
                     label.toStdString());
        }
    });
    entity->watcher->setFuture(future);

    return id;
}

void TaskManager::requestAbort(TaskId id)
{
    if (auto* e = m_tasks.value(id, nullptr))
        e->progress->requestAbort();
}

bool TaskManager::isRunning(TaskId id) const
{
    if (auto* e = m_tasks.value(id, nullptr))
        return e->watcher->isRunning();
    return false;
}

int TaskManager::percent(TaskId id) const
{
    if (auto* e = m_tasks.value(id, nullptr))
        return e->progress->percent();
    return 0;
}

bool TaskManager::waitForDone(TaskId id, int timeoutMs)
{
    if (auto* e = m_tasks.value(id, nullptr)) {
        if (timeoutMs < 0) {
            e->watcher->waitForFinished();
            return true;
        }

        QElapsedTimer timer;
        timer.start();
        while (e->watcher->isRunning() && timer.elapsed() < timeoutMs)
            QThread::msleep(1);
        return !e->watcher->isRunning();
    }
    return false;
}

void TaskManager::onTaskFinished(TaskId id)
{
    auto* e = m_tasks.value(id, nullptr);
    if (!e) return;

    const bool success = e->success;
    emit taskFinished(id, success);

    // Clean up
    delete e->progress;
    e->watcher->deleteLater();
    delete e;
    m_tasks.remove(id);
}
