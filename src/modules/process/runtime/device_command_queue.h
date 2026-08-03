#pragma once

#include "core/task/task_types.h"

#include <QMutex>
#include <QThread>
#include <QWaitCondition>
#include <QString>

#include <array>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>

namespace lcnc::process {

using DeviceCommandId = std::uint64_t;

enum class DeviceCommandCompletion {
    Succeeded,
    Failed,
    Superseded,
    Cancelled,
    Shutdown,
    TimedOut,
};

struct DeviceCommandResult {
    bool success{true};
    QString error;
    DeviceCommandCompletion completion{DeviceCommandCompletion::Succeeded};
};

struct DeviceCommandTicket {
    DeviceCommandId id{0};
    bool accepted{false};
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
    DeviceCommandTicket submitWithTicket(ResultCommand command,
                                         TaskPriority priority,
                                         Completion completion = {},
                                         const QString& coalesceKey = {});
    /// Cancels a pending command. Running vendor calls are never interrupted.
    bool cancel(DeviceCommandId id);
    /**
     * @brief Submit a bounded command and wait outside the GUI thread.
     *
     * This is intended for the workflow orchestration thread.  The Stop lane
     * is additionally permitted during GUI-thread module shutdown, where the
     * application must synchronously retain or release device ownership. A timeout does
     * not cancel an already-running vendor call; individual device commands
     * must therefore remain short and cancellation-aware.
     */
    DeviceCommandResult executeAndWait(ResultCommand command,
                                       TaskPriority priority,
                                       int timeoutMs = 5000);
    bool submitStop(Command command);
    bool submitWorkflow(Command command);
    /// Cancels pending non-Stop work and rejects new non-Stop work while preserving the safety lane.
    void beginStopOnly();
    /// Reopens non-Stop lanes after a successful safe-stop transaction.
    void endStopOnly();
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
        ResultCommand command;
        Completion completion;
        DeviceCommandId id{0};
        QString coalesceKey;
    };

    static constexpr int priorityIndex(TaskPriority priority)
    {
        return static_cast<int>(priority);
    }

    DeviceCommandTicket enqueue(ResultCommand command,
                                TaskPriority priority,
                                Completion completion,
                                const QString& coalesceKey);
    void markTimedOut(DeviceCommandId id);
    static DeviceCommandResult completionResult(DeviceCommandCompletion completion,
                                                QString error = {});
    static void notifyCompletion(Completion completion, const DeviceCommandResult& result);
    void runWorker();

    mutable QMutex m_mutex;
    QWaitCondition m_workAvailable;
    std::array<std::deque<QueuedCommand>, 5> m_commands;
    Qt::HANDLE m_workerThreadId{nullptr};
    bool m_accepting{false};
    bool m_stopOnly{false};
    bool m_shutdownRequested{false};
    DeviceCommandId m_activeCommandId{0};
    DeviceCommandId m_timeoutBarrierId{0};
    DeviceCommandId m_nextCommandId{1};
    WorkerThread m_thread;
};

} // namespace lcnc::process
