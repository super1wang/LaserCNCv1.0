#include "modules/process/runtime/rapid_motion_utilities.h"

namespace lcnc::process {

MachinePose5 solvedRapidPose(const lcnc::cam::RapidMoveSegment& segment)
{
    MachinePose5 pose;
    pose.x = segment.target.axes[0];
    pose.y = segment.target.axes[1];
    pose.z = segment.target.axes[2];
    pose.r1 = segment.target.axes[3];
    pose.r2 = segment.target.axes[4];
    pose.tcpMcsX = segment.target.tcpMcsX;
    pose.tcpMcsY = segment.target.tcpMcsY;
    pose.tcpMcsZ = segment.target.tcpMcsZ;
    pose.tcpMcsValid = segment.target.tcpMcsValid;
    pose.r1Name = segment.target.rotaryAxis1Name;
    pose.r2Name = segment.target.rotaryAxis2Name;
    pose.mask = segment.target.activeMask;
    return pose;
}

bool canReuseCommittedTransition(
    std::uint64_t previousSelectedContourId,
    const lcnc::cam::RapidTransition& transition)
{
    return previousSelectedContourId != 0
        && transition.fromContourId == previousSelectedContourId
        && transition.isValid();
}

} // namespace lcnc::process
