#pragma once

#include "modules/cad/selection/cad_selection.h"

#include <QString>
#include <QVector>

namespace lcnc::cad::task {

/// Top-level grouping used by the CAD TaskPanel home page.
enum class CadToolCategory {
    Document = 0,
    BaseModeling,
    SketchFeature,
    Selection,
    Boolean,
    Measure,
    Delete,
    FaceTool,
    EdgeTool,
    SketchEdit
};

/// How a TaskPanel tool should be activated.
enum class CadToolActivation {
    Command = 0,
    SketchPage,
    PrimitivePage,
    FeaturePage,
    TransformPage
};

/// Declarative metadata for one CAD modeling or document command.
struct CadToolDescriptor {
    QString toolId;
    QString commandId;
    QString title;
    QString icon;
    CadToolCategory category{CadToolCategory::BaseModeling};
    CadToolActivation activation{CadToolActivation::Command};
    QVector<lcnc::cad::selection::CadSelectionDomain> acceptedDomains;
    int targetIndex{-1};
    int minSelectedShapes{0};
    int maxSelectedShapes{-1};
    bool showWhenNoDocument{true};
    bool showWhenHasDocument{true};
    bool requiresSelectedSketch{false};
    bool requiresSketchEditing{false};
    int order{0};
};

} // namespace lcnc::cad::task