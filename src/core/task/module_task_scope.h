#pragma once

#include "core/task/task_types.h"

#include <QSet>

class TaskManager;

namespace lcnc {

/**
 * @brief Owns the cancellable TaskManager handles belonging to one module service.
 *
 * Module facades must not duplicate task cancellation bookkeeping.  A service
 * creates one scope, tracks every task it starts, and drains it during stop().
 */
class ModuleTaskScope
{
public:
    void track(TaskId taskId);
    void release(TaskId taskId);
    [[nodiscard]] bool empty() const noexcept { return m_taskIds.isEmpty(); }
    [[nodiscard]] bool cancelAndWait(TaskManager& manager, int timeoutMs);

private:
    QSet<TaskId> m_taskIds;
};

} // namespace lcnc
