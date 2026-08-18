#pragma once

#include "core/project/cam/travel_plan_contracts.h"
#include "modules/process/runtime/machine_pose5.h"

namespace lcnc::process {

/// Converts a CAM-certified rapid target without changing any solved axis.
MachinePose5 solvedRapidPose(const lcnc::cam::RapidMoveSegment& segment);

} // namespace lcnc::process
