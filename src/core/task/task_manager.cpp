#include "core/task/task_manager.h"

#include <QFuture>
#include <QtConcurrent/QtConcurrent>

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
        e->watcher->future().waitForFinished();
        return true;
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
