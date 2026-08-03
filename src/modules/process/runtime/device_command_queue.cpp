#include "modules/process/runtime/device_command_queue.h"

#include "core/logging/logger.h"

#include <QCoreApplication>
#include <QSemaphore>

#include <climits>
#include <utility>
#include <vector>

namespace lcnc::process {

DeviceCommandQueue::DeviceCommandQueue()
    : m_thread(*this)
{
    m_thread.setObjectName(QStringLiteral("ProcessDeviceExecutor"));
}

DeviceCommandQueue::~DeviceCommandQueue()
{
    (void)shutdown();
}

bool DeviceCommandQueue::start()
{
    QMutexLocker locker(&m_mutex);
    if (m_thread.isRunning())
        return true;

    for (auto& commands : m_commands)
        commands.clear();
    m_workerThreadId = nullptr;
    m_shutdownRequested = false;
    m_activeCommandId = 0;
    m_timeoutBarrierId = 0;
    m_accepting = true;
    m_stopOnly = false;
    m_thread.start();
    return true;
}

bool DeviceCommandQueue::submit(Command command,
                                TaskPriority priority,
                                const QString& coalesceKey)
{
    if (!command)
        return false;
    return enqueue([command = std::move(command)] {
        command();
        return DeviceCommandResult{};
    }, priority, {}, coalesceKey).accepted;
}

bool DeviceCommandQueue::submit(ResultCommand command,
                                TaskPriority priority,
                                Completion completion,
                                const QString& coalesceKey)
{
    return submitWithTicket(std::move(command), priority,
                            std::move(completion), coalesceKey).accepted;
}

DeviceCommandTicket DeviceCommandQueue::submitWithTicket(ResultCommand command,
                                                          TaskPriority priority,
                                                          Completion completion,
                                                          const QString& coalesceKey)
{
    return enqueue(std::move(command), priority, std::move(completion), coalesceKey);
}

DeviceCommandResult DeviceCommandQueue::completionResult(DeviceCommandCompletion completion,
                                                           QString error)
{
    return {completion == DeviceCommandCompletion::Succeeded,
            std::move(error), completion};
}

void DeviceCommandQueue::notifyCompletion(Completion completion,
                                          const DeviceCommandResult& result)
{
    if (!completion)
        return;
    try {
        completion(result);
    } catch (const std::exception& exception) {
        LCNC_ERR(lcnc::LogCode::TaskUnhandled,
                 "Process device command completion threw std::exception: {}", exception.what());
    } catch (...) {
        LCNC_ERR(lcnc::LogCode::TaskUnhandled,
                 "Process device command completion threw an unknown exception");
    }
}

bool DeviceCommandQueue::cancel(DeviceCommandId id)
{
    Completion completion;
    bool cancelled = false;
    {
        QMutexLocker locker(&m_mutex);
        for (auto& commands : m_commands) {
            for (auto it = commands.begin(); it != commands.end(); ++it) {
                if (it->id != id)
                    continue;
                completion = std::move(it->completion);
                commands.erase(it);
                cancelled = true;
                break;
            }
            if (cancelled)
                break;
        }
    }
    notifyCompletion(std::move(completion),
                     completionResult(DeviceCommandCompletion::Cancelled,
                                      QStringLiteral("Device command cancelled before execution")));
    return cancelled;
}

DeviceCommandResult DeviceCommandQueue::executeAndWait(ResultCommand command,
                                                        TaskPriority priority,
                                                        int timeoutMs)
{
    if (!command)
        return completionResult(DeviceCommandCompletion::Failed,
                                QStringLiteral("Missing device command"));

    if (auto* app = QCoreApplication::instance(); app && app->thread() == QThread::currentThread()
        && priority != TaskPriority::Stop) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "DeviceCommandQueue::executeAndWait must not block the GUI thread");
        return completionResult(DeviceCommandCompletion::Failed,
                                QStringLiteral("Device command must not block the GUI thread"));
    }

    if (isWorkerThread()) {
        try {
            return command();
        } catch (const std::exception& exception) {
            LCNC_ERR(lcnc::LogCode::TaskUnhandled,
                     "Process device result command threw std::exception: {}", exception.what());
            return completionResult(DeviceCommandCompletion::Failed,
                                    QString::fromUtf8(exception.what()));
        } catch (...) {
            LCNC_ERR(lcnc::LogCode::TaskUnhandled,
                     "Process device result command threw an unknown exception");
            return completionResult(DeviceCommandCompletion::Failed,
                                    QStringLiteral("Unknown device command exception"));
        }
    }

    struct WaitState {
        QSemaphore finished;
        DeviceCommandResult result = completionResult(
            DeviceCommandCompletion::Failed, QStringLiteral("Device command was not executed"));
    };
    const auto state = std::make_shared<WaitState>();
    const DeviceCommandTicket ticket = submitWithTicket(
        std::move(command), priority,
        [state](const DeviceCommandResult& result) {
            state->result = result;
            state->finished.release();
        });
    if (!ticket.accepted) {
        return completionResult(DeviceCommandCompletion::Shutdown,
                                QStringLiteral("Device command queue is not accepting work"));
    }

    if (!state->finished.tryAcquire(1, timeoutMs < 0 ? INT_MAX : timeoutMs)) {
        markTimedOut(ticket.id);
        return completionResult(DeviceCommandCompletion::TimedOut,
                                QStringLiteral("Timed out waiting for device command"));
    }
    return state->result;
}

bool DeviceCommandQueue::submitStop(Command command)
{
    return submit(std::move(command), TaskPriority::Stop, {});
}

bool DeviceCommandQueue::submitWorkflow(Command command)
{
    return submit(std::move(command), TaskPriority::Workflow, {});
}

void DeviceCommandQueue::beginStopOnly()
{
    std::vector<Completion> dropped;
    {
        QMutexLocker locker(&m_mutex);
        m_stopOnly = true;
        // A command accepted before Stop must not run after the safety
        // transaction. A currently executing vendor call remains bounded but
        // cannot be preempted by this queue.
        for (int index = priorityIndex(TaskPriority::Workflow);
             index < static_cast<int>(m_commands.size()); ++index) {
            for (QueuedCommand& command : m_commands[static_cast<std::size_t>(index)])
                if (command.completion)
                    dropped.push_back(std::move(command.completion));
            m_commands[static_cast<std::size_t>(index)].clear();
        }
        m_workAvailable.wakeOne();
    }

    for (Completion& completion : dropped)
        notifyCompletion(std::move(completion),
                         completionResult(DeviceCommandCompletion::Cancelled,
                                          QStringLiteral("Device command discarded by safety stop")));
}

void DeviceCommandQueue::endStopOnly()
{
    QMutexLocker locker(&m_mutex);
    if (m_accepting && !m_shutdownRequested && m_timeoutBarrierId == 0)
        m_stopOnly = false;
}

bool DeviceCommandQueue::shutdown(int timeoutMs)
{
    std::vector<Completion> dropped;
    {
        QMutexLocker locker(&m_mutex);
        if (!m_thread.isRunning()) {
            m_accepting = false;
            return true;
        }

        // Do not let work issued before shutdown run after the safe-stop
        // request. Stop work remains queued and is drained.
        m_accepting = false;
        m_shutdownRequested = true;
        for (int index = priorityIndex(TaskPriority::Workflow);
             index < static_cast<int>(m_commands.size()); ++index) {
            for (QueuedCommand& command : m_commands[static_cast<std::size_t>(index)])
                if (command.completion)
                    dropped.push_back(std::move(command.completion));
            m_commands[static_cast<std::size_t>(index)].clear();
        }
        m_workAvailable.wakeOne();
    }

    for (Completion& completion : dropped)
        notifyCompletion(std::move(completion),
                         completionResult(DeviceCommandCompletion::Shutdown,
                                          QStringLiteral("Device command discarded during shutdown")));

    return m_thread.wait(timeoutMs < 0 ? ULONG_MAX : static_cast<unsigned long>(timeoutMs));
}

bool DeviceCommandQueue::isWorkerThread() const
{
    QMutexLocker locker(&m_mutex);
    return m_workerThreadId != nullptr && m_workerThreadId == QThread::currentThreadId();
}

bool DeviceCommandQueue::isRunning() const
{
    return m_thread.isRunning();
}

DeviceCommandTicket DeviceCommandQueue::enqueue(ResultCommand command,
                                                 TaskPriority priority,
                                                 Completion completion,
                                                 const QString& coalesceKey)
{
    if (!command)
        return {};

    Completion superseded;
    DeviceCommandTicket ticket;
    QMutexLocker locker(&m_mutex);
    if (!m_accepting || m_shutdownRequested
        || (m_stopOnly && priority != TaskPriority::Stop)
        || (m_timeoutBarrierId != 0 && priority != TaskPriority::Stop))
        return {};

    auto& commands = m_commands[static_cast<std::size_t>(priorityIndex(priority))];
    if (!coalesceKey.isEmpty()) {
        for (QueuedCommand& pending : commands) {
            if (pending.coalesceKey == coalesceKey) {
                superseded = std::move(pending.completion);
                ticket = {m_nextCommandId++, true};
                pending.command = std::move(command);
                pending.completion = std::move(completion);
                pending.id = ticket.id;
                m_workAvailable.wakeOne();
                locker.unlock();
                notifyCompletion(std::move(superseded),
                                 completionResult(DeviceCommandCompletion::Superseded,
                                                  QStringLiteral("Device command superseded by a newer coalesced command")));
                return ticket;
            }
        }
    }
    ticket = {m_nextCommandId++, true};
    commands.push_back({std::move(command), std::move(completion), ticket.id, coalesceKey});
    m_workAvailable.wakeOne();
    return ticket;
}

void DeviceCommandQueue::markTimedOut(DeviceCommandId id)
{
    QMutexLocker locker(&m_mutex);
    if (m_activeCommandId == id) {
        m_timeoutBarrierId = id;
        return;
    }
    for (const auto& commands : m_commands) {
        for (const QueuedCommand& command : commands) {
            if (command.id == id) {
                m_timeoutBarrierId = id;
                return;
            }
        }
    }
}

void DeviceCommandQueue::WorkerThread::run()
{
    m_owner.runWorker();
}

void DeviceCommandQueue::runWorker()
{
    {
        QMutexLocker locker(&m_mutex);
        m_workerThreadId = QThread::currentThreadId();
    }

    while (true) {
        QueuedCommand command;
        bool hasCommand = false;
        {
            QMutexLocker locker(&m_mutex);
            const auto hasCommands = [this] {
                for (const auto& commands : m_commands)
                    if (!commands.empty())
                        return true;
                return false;
            };
            while (!hasCommands() && !m_shutdownRequested)
                m_workAvailable.wait(&m_mutex);

            for (auto& commands : m_commands) {
                if (commands.empty())
                    continue;
                command = std::move(commands.front());
                commands.pop_front();
                m_activeCommandId = command.id;
                hasCommand = true;
                break;
            }
            if (!hasCommand && m_shutdownRequested) {
                break;
            }
        }

        DeviceCommandResult result;
        try {
            result = command.command();
            if (result.completion == DeviceCommandCompletion::Succeeded && !result.success)
                result.completion = DeviceCommandCompletion::Failed;
        } catch (const std::exception& exception) {
            LCNC_ERR(lcnc::LogCode::TaskUnhandled,
                      "Process device command threw std::exception: {}", exception.what());
            result = completionResult(DeviceCommandCompletion::Failed,
                                      QString::fromUtf8(exception.what()));
        } catch (...) {
            LCNC_ERR(lcnc::LogCode::TaskUnhandled,
                      "Process device command threw an unknown exception");
            result = completionResult(DeviceCommandCompletion::Failed,
                                      QStringLiteral("Unknown device command exception"));
        }

        // Completion is an observer notification, not part of the device command.
        // It is invoked exactly once even when user code throws.
        notifyCompletion(std::move(command.completion), result);
        {
            QMutexLocker locker(&m_mutex);
            if (m_timeoutBarrierId == command.id)
                m_timeoutBarrierId = 0;
            m_activeCommandId = 0;
        }
    }

    QMutexLocker locker(&m_mutex);
    m_workerThreadId = nullptr;
    m_accepting = false;
    m_activeCommandId = 0;
    m_timeoutBarrierId = 0;
}

} // namespace lcnc::process
