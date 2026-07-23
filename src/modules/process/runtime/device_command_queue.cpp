#include "modules/process/runtime/device_command_queue.h"

#include "core/logging/logger.h"

#include <QCoreApplication>
#include <QSemaphore>

#include <climits>
#include <utility>

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
    m_accepting = true;
    m_thread.start();
    return true;
}

bool DeviceCommandQueue::submit(Command command,
                                TaskPriority priority,
                                const QString& coalesceKey)
{
    return enqueue(std::move(command), priority, coalesceKey);
}

bool DeviceCommandQueue::submit(ResultCommand command,
                                TaskPriority priority,
                                Completion completion,
                                const QString& coalesceKey)
{
    if (!command)
        return false;

    return enqueue([command = std::move(command), completion = std::move(completion)] {
        DeviceCommandResult result;
        try {
            result = command();
        } catch (const std::exception& exception) {
            result.success = false;
            result.error = QString::fromUtf8(exception.what());
            LCNC_ERR(lcnc::LogCode::TaskUnhandled,
                     "Process device result command threw std::exception: {}", exception.what());
        } catch (...) {
            result.success = false;
            result.error = QStringLiteral("Unknown device command exception");
            LCNC_ERR(lcnc::LogCode::TaskUnhandled,
                     "Process device result command threw an unknown exception");
        }
        if (completion)
            completion(result);
    }, priority, coalesceKey);
}

DeviceCommandResult DeviceCommandQueue::executeAndWait(ResultCommand command,
                                                        TaskPriority priority,
                                                        int timeoutMs)
{
    if (!command)
        return {false, QStringLiteral("Missing device command")};

    if (auto* app = QCoreApplication::instance(); app && app->thread() == QThread::currentThread()) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "DeviceCommandQueue::executeAndWait must not block the GUI thread");
        return {false, QStringLiteral("Device command must not block the GUI thread")};
    }

    if (isWorkerThread()) {
        try {
            return command();
        } catch (const std::exception& exception) {
            LCNC_ERR(lcnc::LogCode::TaskUnhandled,
                     "Process device result command threw std::exception: {}", exception.what());
            return {false, QString::fromUtf8(exception.what())};
        } catch (...) {
            LCNC_ERR(lcnc::LogCode::TaskUnhandled,
                     "Process device result command threw an unknown exception");
            return {false, QStringLiteral("Unknown device command exception")};
        }
    }

    struct WaitState {
        QSemaphore finished;
        DeviceCommandResult result{false, QStringLiteral("Device command was not executed")};
    };
    const auto state = std::make_shared<WaitState>();
    if (!submit(std::move(command), priority,
                [state](const DeviceCommandResult& result) {
                    state->result = result;
                    state->finished.release();
                })) {
        return {false, QStringLiteral("Device command queue is not accepting work")};
    }

    if (!state->finished.tryAcquire(1, timeoutMs < 0 ? INT_MAX : timeoutMs))
        return {false, QStringLiteral("Timed out waiting for device command")};
    return state->result;
}

bool DeviceCommandQueue::submitEmergency(Command command)
{
    return submitStop(std::move(command));
}

bool DeviceCommandQueue::submitStop(Command command)
{
    return enqueue(std::move(command), TaskPriority::Stop, {});
}

bool DeviceCommandQueue::submitWorkflow(Command command)
{
    return enqueue(std::move(command), TaskPriority::Workflow, {});
}

bool DeviceCommandQueue::shutdown(int timeoutMs)
{
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
            m_commands[static_cast<std::size_t>(index)].clear();
        }
        m_workAvailable.wakeOne();
    }

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

bool DeviceCommandQueue::enqueue(Command command,
                                 TaskPriority priority,
                                 const QString& coalesceKey)
{
    if (!command)
        return false;

    QMutexLocker locker(&m_mutex);
    if (!m_accepting || m_shutdownRequested)
        return false;

    auto& commands = m_commands[static_cast<std::size_t>(priorityIndex(priority))];
    if (!coalesceKey.isEmpty()) {
        for (QueuedCommand& pending : commands) {
            if (pending.coalesceKey == coalesceKey) {
                pending.command = std::move(command);
                m_workAvailable.wakeOne();
                return true;
            }
        }
    }
    commands.push_back({std::move(command), coalesceKey});
    m_workAvailable.wakeOne();
    return true;
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
        Command command;
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
                command = std::move(commands.front().command);
                commands.pop_front();
                break;
            }
            if (!command && m_shutdownRequested) {
                break;
            }
        }

        try {
            command();
        } catch (const std::exception& exception) {
            LCNC_ERR(lcnc::LogCode::TaskUnhandled,
                     "Process device command threw std::exception: {}", exception.what());
        } catch (...) {
            LCNC_ERR(lcnc::LogCode::TaskUnhandled,
                     "Process device command threw an unknown exception");
        }
    }

    QMutexLocker locker(&m_mutex);
    m_workerThreadId = nullptr;
    m_accepting = false;
}

} // namespace lcnc::process
