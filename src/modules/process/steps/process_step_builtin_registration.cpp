#include "modules/process/steps/process_step_builtin_registration.h"

#include "modules/process/steps/process_step_registry.h"
#include "modules/process/steps/input_signal_wait/input_signal_wait_step.h"
#include "modules/process/steps/multi_axis_move/multi_axis_move_step.h"
#include "modules/process/steps/normal_cutting/normal_cutting_step.h"
#include "modules/process/steps/output_signal/output_signal_step.h"
#include "modules/process/steps/single_axis_move/single_axis_move_step.h"
#include "modules/process/steps/start/start_step.h"
#include "modules/process/steps/stop/stop_step.h"

#include <memory>

namespace lcnc::process {

void registerBuiltinProcessSteps(ProcessStepRegistry& registry)
{
    registry.registerStep(std::make_shared<StartStep>());
    registry.registerStep(std::make_shared<StopStep>());
    registry.registerStep(std::make_shared<SingleAxisMoveStep>());
    registry.registerStep(std::make_shared<MultiAxisMoveStep>());
    registry.registerStep(std::make_shared<OutputSignalStep>());
    registry.registerStep(std::make_shared<InputSignalWaitStep>());
    registry.registerStep(std::make_shared<NormalCuttingStep>());
}

} // namespace lcnc::process
