#pragma once

#include "modules/process/runtime/process_execution_context.h"

#include <QMap>
#include <QString>
#include <QStringList>
#include <QVariantMap>
#include <memory>

namespace lcnc::process {

/**
 * @brief Runtime request passed to a Process node executor.
 */
struct ProcessNodeExecutionRequest
{
    QString nodeId;
    QString executorKey;
    QString displayName;
    QVariantMap parameters;
};

/**
 * @brief Executes one workflow node type without owning UI or document state.
 */
class IProcessNodeExecutor
{
public:
    virtual ~IProcessNodeExecutor() = default;
    virtual QString executorKey() const = 0;
    virtual bool execute(const ProcessNodeExecutionRequest& request,
                         ProcessExecutionContext& context,
                         QString* errorMessage = nullptr) = 0;
};

/**
 * @brief Registry for Process node executors keyed by ProcessNodeRegistry metadata.
 */
class ProcessNodeExecutorRegistry
{
public:
    void registerExecutor(std::shared_ptr<IProcessNodeExecutor> executor);
    std::shared_ptr<IProcessNodeExecutor> executor(const QString& executorKey) const;
    QStringList executorKeys() const;
    bool contains(const QString& executorKey) const;
    int size() const { return m_executors.size(); }

private:
    QMap<QString, std::shared_ptr<IProcessNodeExecutor>> m_executors;
};

} // namespace lcnc::process