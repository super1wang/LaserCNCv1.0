#pragma once

#include "modules/process/execution/process_node_executor_registry.h"
#include "modules/process/steps/process_step_context.h"
#include "modules/process/workflow/process_node_registry.h"

class QWidget;

namespace lcnc::process {

class IProcessWorkflowStep
{
public:
    virtual ~IProcessWorkflowStep() = default;

    virtual ProcessNodeDescriptor descriptor() const = 0;
    virtual QString summary(const ProcessNode& node) const = 0;

    virtual QWidget* createParameterEditor(const ProcessNode& node, QWidget* parent) const = 0;
    virtual bool applyParameterEditor(QWidget* editor,
                                      ProcessNode& node,
                                      QString* errorMessage = nullptr) const = 0;

    virtual bool execute(const ProcessNodeExecutionRequest& request,
                         ProcessStepContext& context,
                         QString* errorMessage = nullptr) = 0;

    virtual int completionDelayMs(const ProcessNodeExecutionRequest& request) const
    {
        Q_UNUSED(request);
        return 1;
    }
};

} // namespace lcnc::process
