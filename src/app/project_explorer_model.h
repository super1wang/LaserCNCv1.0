#pragma once

#include "core/project/project_types.h"
#include "core/project/cam/cam_data_contracts.h"

#include <QList>
#include <QColor>
#include <QString>
#include <QStringList>

#include <cstdint>

class CadModule;
class CamModule;

namespace lcnc::app {

enum class ProjectExplorerNodeKind {
    WorkpieceRoot = 0,
    CadDocument,
    CadGroup,
    CadShape,
    CadSketch,
    CadSketchElement,
    CadTemporarySketch,
    MachineRoot,
    MachineAxis,
    MachineShape,
    MachineUnassignedGroup,
    ToolpathRoot,
    ToolpathLayer,
    ToolpathContour
};

struct ProjectExplorerNode {
    ProjectExplorerNodeKind kind{ProjectExplorerNodeKind::WorkpieceRoot};
    DocumentId documentId{kInvalidDocumentId};
    QString nodeKey;
    QString displayName;
    QString infoText;
    QString entry;
    QStringList leafEntries;
    QString axisName;
    std::uint64_t layerId{0};
    QColor layerColor;
    QString toolName;
    int contourIndex{-1};
    lcnc::cam::ContourId contourId{0};
    bool checkable{true};
    bool checked{true};
    bool selectable{true};
    bool draggable{false};
    bool droppable{false};
    bool muted{false};
    QString toolTip;
    QList<ProjectExplorerNode> children;
};

struct ProjectExplorerSnapshot {
    QList<ProjectExplorerNode> roots;
};

class ProjectExplorerModel
{
public:
    static ProjectExplorerSnapshot build(CadModule* cad, CamModule* cam);
};

bool isCadProjectNode(ProjectExplorerNodeKind kind);
bool isMachineProjectNode(ProjectExplorerNodeKind kind);
bool isToolpathProjectNode(ProjectExplorerNodeKind kind);

} // namespace lcnc::app