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
    TaskSpec spec;
    spec.label = label;
    return run(std::move(spec), std::move(job));
}

TaskId TaskManager::run(TaskSpec spec, TaskJob job)
{
    const TaskId id = m_nextId++;

    auto* entity          = new Entity;
    entity->id            = id;
    entity->spec          = std::move(spec);
    if (entity->spec.label.isEmpty())
        entity->spec.label = tr("未命名任务");
    entity->progress      = new TaskProgress();
    entity->watcher       = new QFutureWatcher<void>(this);

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
    if (entity->spec.userVisible)
        emit taskStarted(id, entity->spec.label);
    emit taskStatusChanged(id, TaskExecutionStatus::Queued);
    const QString label = entity->spec.label;
    QFuture<void> future = QtConcurrent::run([this, job, prog, entity, label]() {
        setStatus(entity, TaskExecutionStatus::Running);
        try {
            job(prog);
            setStatus(entity, prog->isAbortRequested()
                                  ? TaskExecutionStatus::Cancelled
                                  : TaskExecutionStatus::Succeeded);
        } catch (const std::exception& e) {
            if (prog->isAbortRequested()) {
                setStatus(entity, TaskExecutionStatus::Cancelled);
            } else {
                {
                    std::lock_guard<std::mutex> lock(entity->stateMutex);
                    entity->error = QString::fromUtf8(e.what());
                }
                setStatus(entity, TaskExecutionStatus::Failed);
                LCNC_ERR(lcnc::LogCode::TaskUnhandled,
                         "TaskManager: task '{}' threw std::exception: {}",
                         label.toStdString(), e.what());
            }
        } catch (...) {
            if (prog->isAbortRequested()) {
                setStatus(entity, TaskExecutionStatus::Cancelled);
            } else {
                {
                    std::lock_guard<std::mutex> lock(entity->stateMutex);
                    entity->error = tr("未知异常");
                }
                setStatus(entity, TaskExecutionStatus::Failed);
                LCNC_ERR(lcnc::LogCode::TaskUnhandled,
                         "TaskManager: task '{}' threw unknown exception",
                         label.toStdString());
            }
        }
    });
    entity->watcher->setFuture(future);

    return id;
}

void TaskManager::requestAbort(TaskId id)
{
    if (auto* e = m_tasks.value(id, nullptr)) {
        if (!e->spec.cancellable)
            return;
        e->progress->requestAbort();
        setStatus(e, TaskExecutionStatus::CancelRequested);
    }
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

TaskExecutionStatus TaskManager::status(TaskId id) const
{
    if (auto* e = m_tasks.value(id, nullptr))
        return e->status.load();
    return TaskExecutionStatus::Stale;
}

TaskSnapshot TaskManager::snapshot(TaskId id) const
{
    TaskSnapshot result;
    if (auto* e = m_tasks.value(id, nullptr)) {
        result.taskId = e->id;
        result.label = e->spec.label;
        result.scope = e->spec.scope;
        result.priority = e->spec.priority;
        result.status = e->status.load();
        result.percent = e->progress ? e->progress->percent() : 0;
        result.step = e->progress ? e->progress->stepName() : QString();
        result.userVisible = e->spec.userVisible;
        result.cancellable = e->spec.cancellable;
        std::lock_guard<std::mutex> lock(e->stateMutex);
        result.error = e->error;
    }
    return result;
}

QList<TaskSnapshot> TaskManager::activeTasks() const
{
    QList<TaskSnapshot> result;
    result.reserve(m_tasks.size());
    for (auto it = m_tasks.cbegin(); it != m_tasks.cend(); ++it)
        result.append(snapshot(it.key()));
    return result;
}

void TaskManager::onTaskFinished(TaskId id)
{
    auto* e = m_tasks.value(id, nullptr);
    if (!e) return;

    const TaskExecutionStatus finalStatus = e->status.load();
    QString error;
    {
        std::lock_guard<std::mutex> lock(e->stateMutex);
        error = e->error;
    }
    const bool success = finalStatus == TaskExecutionStatus::Succeeded;
    emit taskFinishedDetailed(id, finalStatus, error);
    emit taskFinished(id, success);

    // Clean up
    delete e->progress;
    e->watcher->deleteLater();
    delete e;
    m_tasks.remove(id);
}

void TaskManager::setStatus(Entity* entity, TaskExecutionStatus status)
{
    if (!entity)
        return;
    entity->status.store(status);
    const TaskId id = entity->id;
    QMetaObject::invokeMethod(this, [this, id, status] {
        if (m_tasks.contains(id))
            emit taskStatusChanged(id, status);
    }, Qt::QueuedConnection);
}
