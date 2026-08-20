#pragma once

#include "core/algorithms/cam/contour_order_planner.h"
#include "core/project/cam/cam_data_contracts.h"
#include "core/project/cam/layer_contracts.h"
#include "modules/cam/contracts/toolpath_export_dto.h"
#include "modules/cam/contracts/i_cam_contour_sequence_provider.h"

class LaserToolpath;
class MachineKinematics;
class QString;

namespace lcnc::cam {
class LayerContainer;

/// Pure ordering/revision projection for the CAM toolpath model.
///
/// Keeping this logic outside CamModule prevents UI/lifecycle concerns from
/// becoming a second source of contour-order truth.
class ToolpathSequenceService final
{
public:
    static std::uint64_t computeToolpathRevision(
        const LaserToolpath& toolpath,
        bool generationParamsDirty,
        bool pipelineComplete);

    static ContourSequenceSnapshot buildSequenceSnapshot(
        const ToolpathExportSnapshot& snapshot,
        const LayerContainer* layers);

    static QVector<ContourId> planAutomaticOrder(
        const LaserToolpath& toolpath,
        const LayerContainer& layers,
        const MachineKinematics* kinematics,
        AutoSortAxis axis,
        QString* errorMessage);
};

} // namespace lcnc::cam
