#pragma once

#include "modules/process/steps/process_step_interfaces.h"

#include <QMap>
#include <QSet>
#include <memory>

namespace lcnc::process {

class ProcessStepRegistry
{
public:
    static ProcessStepRegistry& instance();

    void registerStep(std::shared_ptr<IProcessWorkflowStep> step);
    std::shared_ptr<IProcessWorkflowStep> step(ProcessNodeType type) const;
    std::shared_ptr<IProcessWorkflowStep> stepByExecutorKey(const QString& key) const;

    QVector<ProcessNodeDescriptor> descriptors() const;
    QVector<ProcessNodeDescriptor> descriptorsAll() const;
    bool setPluginEnabled(const QString& pluginKey, bool enabled);
    bool isPluginEnabled(const QString& pluginKey) const;
    void clear();

private:
    QMap<ProcessNodeType, std::shared_ptr<IProcessWorkflowStep>> m_stepsByType;
    QMap<QString, std::shared_ptr<IProcessWorkflowStep>> m_stepsByExecutorKey;
    QSet<QString> m_disabledPluginKeys;
};

} // namespace lcnc::process
