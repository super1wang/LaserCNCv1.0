#pragma once

#include <QObject>
#include <QString>
#include <QMap>
#include <QList>
#include <QFutureWatcher>
#include <functional>
#include <atomic>
#include <mutex>

#include "core/task/task_progress.h"
#include "core/task/task_types.h"

/**
 * @brief Manages asynchronous tasks with progress reporting.
 *
 * Each task is a callable (TaskJob) that receives a TaskProgress*.
 * Tasks run via QtConcurrent::run() so they are non-blocking for the UI.
 * Progress updates are delivered to the GUI thread through Qt queued signals.
 */
class TaskManager : public QObject
{
    Q_OBJECT
public:
    using TaskJob = std::function<void(TaskProgress*)>;

    /// Start an async task; returns a handle for later monitoring/abort.
    TaskId run(const QString& label, TaskJob job);
    TaskId run(TaskSpec spec, TaskJob job);

    /// Request cooperative abort; job must check progress->isAbortRequested().
    void requestAbort(TaskId id);

    bool isRunning(TaskId id) const;
    int  percent(TaskId id)   const;
    TaskExecutionStatus status(TaskId id) const;
    TaskSnapshot snapshot(TaskId id) const;
    QList<TaskSnapshot> activeTasks() const;

    /// Block until the task completes (max @a timeoutMs ms; -1 = forever).
    bool waitForDone(TaskId id, int timeoutMs = -1);

signals:
    void taskStarted(TaskId id, const QString& label);
    void taskProgressChanged(TaskId id, int percent);
    void taskStepChanged(TaskId id, const QString& step);
    void taskFinished(TaskId id, bool success);
    void taskStatusChanged(TaskId id, TaskExecutionStatus status);
    void taskFinishedDetailed(TaskId id, TaskExecutionStatus status, const QString& error);

public:
    /// Constructed once by lcnc::Kernel during registerCoreServices.
    explicit TaskManager(QObject* parent = nullptr);
    ~TaskManager() override;

private:
    static TaskManager* s_instance;

    struct Entity {
        TaskId               id;
        TaskSpec             spec;
        TaskProgress*        progress{nullptr};
        QFutureWatcher<void>* watcher{nullptr};
        std::atomic<TaskExecutionStatus> status{TaskExecutionStatus::Queued};
        mutable std::mutex      stateMutex;
        QString              error;
    };

    void onTaskFinished(TaskId id);
    void setStatus(Entity* entity, TaskExecutionStatus status);

    QMap<TaskId, Entity*> m_tasks;
    TaskId                m_nextId{0};
};
