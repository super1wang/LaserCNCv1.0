#pragma once

#include "core/kernel/i_service.h"
#include "core/project/cam/cam_data_manager.h"

#include <TopoDS_Face.hxx>

#include <QString>

#include <cstdint>
#include <memory>
#include <vector>

class GuiDocument;
class MachineKinematics;

namespace lcnc::cam {

/**
 * @brief Immutable input for machining-face presentation.
 *
 * This remains module-internal because it contains OCC geometry.  Consumers
 * outside CAM receive only their existing OCC-free DTOs.
 */
struct MachiningFaceDisplaySnapshot {
    std::uint64_t faceId{0};
    TopoDS_Face face;
    QString workpieceEntry;
    bool manual{false};
    MachiningFaceRole role{MachiningFaceRole::MachiningSurface};
};

/**
 * @brief GUI-thread projection of machining-face snapshots into AIS objects.
 *
 * Owns presentation-only AIS handles.  It neither changes CAM data nor
 * decides face roles, so the module can keep lifecycle/event orchestration
 * without retaining renderer state.
 */
class CamDisplayProjectionService final : public lcnc::IService
{
public:
    CamDisplayProjectionService();
    ~CamDisplayProjectionService() override;

    void refreshMachiningFaces(GuiDocument* document,
                               const std::vector<MachiningFaceDisplaySnapshot>& faces,
                               bool visible);
    void clearMachiningFaces(GuiDocument* document);

    // Re-apply the current WPC transform to each machining-face AIS so the
    // highlights track the workpiece as axes move.  No-op when nothing is shown.
    void updateMachiningFaceTransforms(GuiDocument* document, const MachineKinematics* kinematics);

private:
    class State;
    std::unique_ptr<State> m_state;
};

} // namespace lcnc::cam
