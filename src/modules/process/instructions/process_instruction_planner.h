#pragma once

#include "modules/process/instructions/process_command.h"
#include "modules/process/toolpath/process_toolpath_service.h"

namespace lcnc::process {

/**
 * @brief Converts Process job plans to controller-neutral command buffers.
 */
class ProcessInstructionPlanner
{
public:
    ProcessCommandBuffer planJob(const ProcessJobPlan& jobPlan) const;
};

} // namespace lcnc::process