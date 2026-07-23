#pragma once

#include "core/task/task_types.h"

#include <QMutex>
#include <QThread>
#include <QWaitCondition>
#include <QString>

#include <array>
#include <deque>
#include <functional>
#include <memory>

namespace lcnc::process {

struct DeviceCommandResult {
    bool success{true};
    QString error;
};

/**
 * @brief Single-thread command queue for Process device SDK access.
 *
 * Commands are FIFO inside each priority lane.  The lane order is shared with
 * TaskManager: Stop > Workflow > Interactive > Normal > Polling.  A running
 * vendor call cannot be forcibly preempted, so callers must keep every command
 * bounded and must not retain device pointers across submit().
 */
class DeviceCommandQueue final
{
public:
    using Command = std::function<void()>;
    using ResultCommand = std::function<DeviceCommandResult()>;
    using Completion = std::function<void(const DeviceCommandResult&)>;

    DeviceCommandQueue();
    ~DeviceCommandQueue();

    bool start();
    bool submit(Command command,
                TaskPriority priority = TaskPriority::Normal,
                const QString& coalesceKey = {});
    bool submit(ResultCommand command,
                TaskPriority priority,
                Completion completion = {},
                const QString& coalesceKey = {});
    /**
     * @brief Submit a bounded command and wait outside the GUI thread.
     *
     * This is intended for the workflow orchestration thread. A timeout does
     * not cancel an already-running vendor call; individual device commands
     * must therefore remain short and cancellation-aware.
     */
    DeviceCommandResult executeAndWait(ResultCommand command,
                                       TaskPriority priority,
                                       int timeoutMs = 5000);
    bool submitEmergency(Command command);
    bool submitStop(Command command);
    bool submitWorkflow(Command command);
    bool shutdown(int timeoutMs = 5000);

    bool isWorkerThread() const;
    bool isRunning() const;

private:
    class WorkerThread final : public QThread
    {
    public:
        explicit WorkerThread(DeviceCommandQueue& owner)
            : m_owner(owner) {}

    protected:
        void run() override;

    private:
        DeviceCommandQueue& m_owner;
    };

    struct QueuedCommand {
        Command command;
        QString coalesceKey;
    };

    static constexpr int priorityIndex(TaskPriority priority)
    {
        return static_cast<int>(priority);
    }

    bool enqueue(Command command, TaskPriority priority, const QString& coalesceKey);
    void runWorker();

    mutable QMutex m_mutex;
    QWaitCondition m_workAvailable;
    std::array<std::deque<QueuedCommand>, 5> m_commands;
    Qt::HANDLE m_workerThreadId{nullptr};
    bool m_accepting{false};
    bool m_shutdownRequested{false};
    WorkerThread m_thread;
};

} // namespace lcnc::process
