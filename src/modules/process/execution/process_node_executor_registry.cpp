#include "modules/process/execution/process_node_executor_registry.h"

#include "core/logging/logger.h"

namespace lcnc::process {

void ProcessNodeExecutorRegistry::registerExecutor(std::shared_ptr<IProcessNodeExecutor> executor)
{
    if (!executor) {
        LCNC_WARN(lcnc::LogCode::Generic, "process.nodeExecutor: skip null executor");
        return;
    }

    const QString key = executor->executorKey().trimmed();
    if (key.isEmpty()) {
        LCNC_WARN(lcnc::LogCode::Generic, "process.nodeExecutor: skip executor with empty key");
        return;
    }

    m_executors.insert(key, std::move(executor));
    LCNC_INFO(lcnc::LogCode::Generic,
              "process.nodeExecutor: registered key='{}' total={}",
              key.toStdString(),
              m_executors.size());
}

std::shared_ptr<IProcessNodeExecutor> ProcessNodeExecutorRegistry::executor(const QString& executorKey) const
{
    return m_executors.value(executorKey.trimmed());
}

QStringList ProcessNodeExecutorRegistry::executorKeys() const
{
    return m_executors.keys();
}

bool ProcessNodeExecutorRegistry::contains(const QString& executorKey) const
{
    return m_executors.contains(executorKey.trimmed());
}

} // namespace lcnc::process