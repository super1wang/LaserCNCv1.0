#pragma once

#include "core/kernel/i_service.h"
#include "core/project/cam/cam_data_manager.h"
#include "core/project/project_types.h"

#include <QColor>
#include <QList>
#include <QString>

#include <cstdint>

namespace lcnc::cam {

struct ProjectExplorerContour {
    ContourId contourId{0};
    std::uint64_t layerId{0};
    QString name;
    QString sourceInfo;
    int pointCount{0};
    int contourIndex{-1};
    bool enabled{true};
};

struct ProjectExplorerLayer {
    std::uint64_t layerId{0};
    QString name;
    QString toolName;
    QColor color;
    QList<ContourId> contourIds;
    bool enabled{true};
};

struct ProjectExplorerFace {
    std::uint64_t faceId{0};
    QString displayName;
    bool manual{false};
    MachiningFaceRole role{MachiningFaceRole::MachiningSurface};
};

struct ProjectExplorerSnapshot {
    DocumentId documentId{kInvalidDocumentId};
    QList<ProjectExplorerContour> contours;
    QList<ProjectExplorerLayer> layers;
    QList<ProjectExplorerFace> faces;
    bool facesVisible{true};
    CamPipelineStageState stages[static_cast<int>(CamPipelineStage::Count)];
};

/** Read-only CAM projection consumed by the project explorer. */
class ICamProjectExplorerProjection : public lcnc::IService
{
public:
    ~ICamProjectExplorerProjection() override = default;
    virtual ProjectExplorerSnapshot projectExplorerSnapshot() const = 0;
};

} // namespace lcnc::cam
