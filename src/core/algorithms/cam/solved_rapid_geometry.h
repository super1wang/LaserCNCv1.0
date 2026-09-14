#pragma once

#include "core/project/cam/travel_plan_contracts.h"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

namespace lcnc::cam_algo {

// Input geometry belongs to the workpiece-local frame used by continuous IK.
// This is assignment from local geometry, never an incremental transform of
// the previously published world point. Solved axes are not modified.
// 中文翻译：从连续求解使用的局部几何重建世界 TCP/法线，不累乘旧世界点，不改变轴值。
inline void setSolvedRapidWorldGeometry(lcnc::cam::RapidPose& pose,
                                        const gp_Pnt& localTcp,
                                        const gp_Dir& localNormal,
                                        const gp_Trsf& solvedWorkpieceToWorld)
{
    const gp_Pnt world = localTcp.Transformed(solvedWorkpieceToWorld);
    const gp_Dir normal = localNormal.Transformed(solvedWorkpieceToWorld);
    pose.tcpX = world.X();
    pose.tcpY = world.Y();
    pose.tcpZ = world.Z();
    pose.surfaceNormalX = normal.X();
    pose.surfaceNormalY = normal.Y();
    pose.surfaceNormalZ = normal.Z();
    pose.tcpMcsValid = false;
}

} // namespace lcnc::cam_algo
