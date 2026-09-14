#pragma once
#include "modules/process/steps/process_step_interfaces.h"
namespace lcnc::process {
class OutputSignalStep final : public IProcessWorkflowStep {
public:
    ProcessNodeDescriptor descriptor() const override;
    QString summary(const ProcessNode& node) const override;
    QWidget* createParameterEditor(const ProcessNode& node, QWidget* parent) const override;
    bool applyParameterEditor(QWidget* editor, ProcessNode& node, QString* errorMessage = nullptr) const override;
    bool execute(const ProcessNodeExecutionRequest& request, ProcessStepContext& context, QString* errorMessage = nullptr) override;
};
}
