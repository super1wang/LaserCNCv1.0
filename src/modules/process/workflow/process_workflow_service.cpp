#include "modules/process/workflow/process_workflow_service.h"

#include "core/logging/logger.h"

namespace lcnc::process {

ProcessWorkflowService::ProcessWorkflowService(ProcessToolpathService& toolpaths, QObject* parent)
    : QObject(parent)
    , m_toolpaths(toolpaths)
{
}

bool ProcessWorkflowService::prepare(const lcnc::ProcessSettings& settings, QString* errorMessage)
{
    m_toolpaths.refreshSnapshot();
    m_jobPlan = m_toolpaths.buildJobPlan();
    if (!m_jobPlan.valid) {
        const QString message = m_jobPlan.warnings.isEmpty()
            ? tr("Process 工作流准备失败")
            : m_jobPlan.warnings.join(QStringLiteral("; "));
        if (errorMessage)
            *errorMessage = message;
        emit workflowRejected(message);
        return false;
    }

    m_commandBuffer = m_planner.planJob(m_jobPlan);
    LCNC_INFO(lcnc::LogCode::Generic,
              "process.workflow: prepared contours={} commands={} warnings={}",
              m_jobPlan.contours.size(),
              m_commandBuffer.size(),
              m_jobPlan.warnings.size());
    emit workflowPrepared(m_jobPlan.contours.size(), m_commandBuffer.size());
    return !m_commandBuffer.isEmpty();
}

} // namespace lcnc::process