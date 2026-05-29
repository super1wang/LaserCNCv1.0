#pragma once

#include "modules/process/instructions/process_instruction_planner.h"
#include "modules/process/toolpath/process_toolpath_service.h"

#include <QObject>

namespace lcnc { class ProcessSettings; }

namespace lcnc::process {

/**
 * @brief Prepares Process workflow artifacts from CAM toolpath and settings snapshots.
 */
class ProcessWorkflowService : public QObject
{
    Q_OBJECT
public:
    explicit ProcessWorkflowService(ProcessToolpathService& toolpaths, QObject* parent = nullptr);

    bool prepare(const lcnc::ProcessSettings& settings, QString* errorMessage = nullptr);
    const ProcessJobPlan& jobPlan() const { return m_jobPlan; }
    const ProcessCommandBuffer& commandBuffer() const { return m_commandBuffer; }

signals:
    void workflowPrepared(int contourCount, int commandCount);
    void workflowRejected(const QString& message);

private:
    ProcessToolpathService& m_toolpaths;
    ProcessInstructionPlanner m_planner;
    ProcessJobPlan m_jobPlan;
    ProcessCommandBuffer m_commandBuffer;
};

} // namespace lcnc::process