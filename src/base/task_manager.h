#pragma once

#include <QObject>
#include <QString>
#include <QMap>
#include <QFutureWatcher>
#include <functional>

#include "base/task_progress.h"

using TaskId = int;
constexpr TaskId kInvalidTaskId = -1;

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

    static TaskManager* instance();

    /// Start an async task; returns a handle for later monitoring/abort.
    TaskId run(const QString& label, TaskJob job);

    /// Request cooperative abort; job must check progress->isAbortRequested().
    void requestAbort(TaskId id);

    bool isRunning(TaskId id) const;
    int  percent(TaskId id)   const;

    /// Block until the task completes (max @a timeoutMs ms; -1 = forever).
    bool waitForDone(TaskId id, int timeoutMs = -1);

signals:
    void taskStarted(TaskId id, const QString& label);
    void taskProgressChanged(TaskId id, int percent);
    void taskStepChanged(TaskId id, const QString& step);
    void taskFinished(TaskId id, bool success);

private:
    explicit TaskManager(QObject* parent = nullptr);
    static TaskManager* s_instance;

    struct Entity {
        TaskId               id;
        QString              label;
        TaskProgress*        progress{nullptr};
        QFutureWatcher<void>* watcher{nullptr};
        bool                 success{false};
    };

    void onTaskFinished(TaskId id);

    QMap<TaskId, Entity*> m_tasks;
    TaskId                m_nextId{0};
};
