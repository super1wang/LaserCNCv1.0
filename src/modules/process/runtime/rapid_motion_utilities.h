#pragma once

#include "core/project/cam/travel_plan_contracts.h"
#include "modules/process/runtime/machine_pose5.h"

namespace lcnc::process {

/// Converts a CAM-certified rapid target without changing any solved axis.
MachinePose5 solvedRapidPose(const lcnc::cam::RapidMoveSegment& segment);

/// A committed travel transition is valid only between two contours in the
/// same new run. The first selected contour always needs a fresh APOS-based
/// initial approach, even when a stale transition happens to use source id 0.
bool canReuseCommittedTransition(
    std::uint64_t previousSelectedContourId,
    const lcnc::cam::RapidTransition& transition);

} // namespace lcnc::process
