#include "core/task/module_task_scope.h"

#include "core/task/task_manager.h"

#include <QElapsedTimer>

#include <algorithm>

namespace lcnc {

void ModuleTaskScope::track(TaskId taskId)
{
    if (taskId != kInvalidTaskId)
        m_taskIds.insert(taskId);
}

void ModuleTaskScope::release(TaskId taskId)
{
    m_taskIds.remove(taskId);
}

bool ModuleTaskScope::cancelAndWait(TaskManager& manager, int timeoutMs)
{
    const QSet<TaskId> taskIds = m_taskIds;
    for (TaskId taskId : taskIds) {
        if (manager.isRunning(taskId))
            manager.requestAbort(taskId);
    }

    QElapsedTimer elapsed;
    elapsed.start();
    bool allFinished = true;
    for (TaskId taskId : taskIds) {
        if (!manager.isRunning(taskId)) {
            release(taskId);
            continue;
        }
        const int remainingMs = std::max(0, timeoutMs - static_cast<int>(elapsed.elapsed()));
        if (!manager.waitForDone(taskId, remainingMs) && manager.isRunning(taskId)) {
            allFinished = false;
            continue;
        }
        release(taskId);
    }
    return allFinished;
}

} // namespace lcnc
