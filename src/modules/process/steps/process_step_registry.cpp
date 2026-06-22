#include "modules/process/steps/process_step_registry.h"

namespace lcnc::process {

ProcessStepRegistry& ProcessStepRegistry::instance()
{
    static ProcessStepRegistry registry;
    return registry;
}

void ProcessStepRegistry::registerStep(std::shared_ptr<IProcessWorkflowStep> step)
{
    if (!step)
        return;
    const ProcessNodeDescriptor descriptor = step->descriptor();
    m_stepsByType.insert(descriptor.type, step);
    if (!descriptor.executorKey.trimmed().isEmpty())
        m_stepsByExecutorKey.insert(descriptor.executorKey, std::move(step));
}

std::shared_ptr<IProcessWorkflowStep> ProcessStepRegistry::step(ProcessNodeType type) const
{
    auto it = m_stepsByType.constFind(type);
    if (it == m_stepsByType.cend())
        return {};
    const auto descriptor = it.value()->descriptor();
    if (!isPluginEnabled(descriptor.executorKey))
        return {};
    return it.value();
}

std::shared_ptr<IProcessWorkflowStep> ProcessStepRegistry::stepByExecutorKey(const QString& key) const
{
    auto it = m_stepsByExecutorKey.constFind(key);
    if (it == m_stepsByExecutorKey.cend())
        return {};
    if (!isPluginEnabled(key))
        return {};
    return it.value();
}

QVector<ProcessNodeDescriptor> ProcessStepRegistry::descriptors() const
{
    QVector<ProcessNodeDescriptor> result;
    for (auto it = m_stepsByType.cbegin(); it != m_stepsByType.cend(); ++it) {
        ProcessNodeDescriptor descriptor = it.value()->descriptor();
        descriptor.addable = descriptor.addable && isPluginEnabled(descriptor.executorKey);
        result.append(std::move(descriptor));
    }
    return result;
}

QVector<ProcessNodeDescriptor> ProcessStepRegistry::descriptorsAll() const
{
    QVector<ProcessNodeDescriptor> result;
    for (auto it = m_stepsByType.cbegin(); it != m_stepsByType.cend(); ++it) {
        ProcessNodeDescriptor descriptor = it.value()->descriptor();
        descriptor.pluginEnabled = isPluginEnabled(descriptor.executorKey);
        result.append(std::move(descriptor));
    }
    return result;
}

bool ProcessStepRegistry::setPluginEnabled(const QString& pluginKey, bool enabled)
{
    const QString key = pluginKey.trimmed();
    if (key.isEmpty())
        return false;
    const auto step = stepByExecutorKey(key);
    if (step && step->descriptor().required && !enabled)
        return false;
    if (enabled)
        m_disabledPluginKeys.remove(key);
    else
        m_disabledPluginKeys.insert(key);
    return true;
}

bool ProcessStepRegistry::isPluginEnabled(const QString& pluginKey) const
{
    return pluginKey.trimmed().isEmpty() || !m_disabledPluginKeys.contains(pluginKey.trimmed());
}

void ProcessStepRegistry::clear()
{
    m_stepsByType.clear();
    m_stepsByExecutorKey.clear();
    m_disabledPluginKeys.clear();
}

} // namespace lcnc::process
