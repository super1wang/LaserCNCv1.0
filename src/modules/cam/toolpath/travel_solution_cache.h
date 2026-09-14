#pragma once

#include "modules/cam/contracts/toolpath_export_dto.h"

namespace lcnc::cam {

// A transition solve also changes the rotary branch of successor contours.
// Restore its complete geometric solution, but never restore old collision
// proof or editable contour metadata from this companion cache.
// 中文翻译：空程连续求解同时改写后继轮廓，必须整体恢复几何；碰撞证明和可编辑元数据不从该缓存恢复。
inline bool restoreTravelSolution(const ToolpathExportSnapshot& cached,
                                  const TravelPlanKey& key,
                                  ToolpathExportSnapshot* target)
{
    if (!target || !(cached.travelPlan.key == key)
        || cached.revision != target->revision
        || cached.machineConfigurationFingerprint != target->machineConfigurationFingerprint
        || cached.machiningMode != target->machiningMode
        || cached.solverId != target->solverId || cached.solverVersion != target->solverVersion
        || cached.machineAxisLayout.count != target->machineAxisLayout.count
        || cached.contours.size() != target->contours.size()
        || cached.pointsByContourId.size() != target->pointsByContourId.size())
        return false;
    for (int i = 0; i < cached.machineAxisLayout.count; ++i) {
        if (cached.machineAxisLayout.axes[i].name != target->machineAxisLayout.axes[i].name
            || cached.machineAxisLayout.axes[i].role != target->machineAxisLayout.axes[i].role)
            return false;
    }
    for (int i = 0; i < cached.contours.size(); ++i) {
        const auto& source = cached.contours[i];
        const auto& destination = target->contours[i];
        if (source.contourId != destination.contourId
            || source.workpieceEntry != destination.workpieceEntry
            || source.hasLeadIn != destination.hasLeadIn
            || source.cuttingOffsetMm != destination.cuttingOffsetMm
            || source.rapidOffsetMm != destination.rapidOffsetMm)
            return false;
        const auto sourcePoints = cached.pointsByContourId.constFind(source.contourId);
        const auto targetPoints = target->pointsByContourId.constFind(destination.contourId);
        if ((sourcePoints == cached.pointsByContourId.cend())
                != (targetPoints == target->pointsByContourId.cend())
            || (sourcePoints != cached.pointsByContourId.cend()
                && sourcePoints->size() != targetPoints->size()))
            return false;
    }
    for (int i = 0; i < cached.contours.size(); ++i)
        target->contours[i].leadInPoint = cached.contours[i].leadInPoint;
    target->pointsByContourId = cached.pointsByContourId;
    return true;
}

} // namespace lcnc::cam
