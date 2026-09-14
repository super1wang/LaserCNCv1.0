#pragma once

#include "core/project/project_types.h"

#include <QList>
#include <QString>

namespace lcnc::cad::selection {

/// Domain of an item currently selected in the CAD workspace.
enum class CadSelectionDomain {
    None = 0,
    DocumentShape,
    SubShapeFace,
    SubShapeEdge,
    SubShapeVertex,
    Sketch,
    SketchElement,
    SketchConstraint,
    Feature
};

/// One normalized selection item shared by model tree, view, and TaskPanel.
struct CadSelectionItem {
    DocumentId docId{kInvalidDocumentId};
    CadSelectionDomain domain{CadSelectionDomain::None};
    QString entry;
    int sketchId{0};
    int sketchElementId{0};
    int sketchHandleIndex{-1};
    int subShapeIndex{-1};
};

/// Snapshot of the current CAD selection and editing state.
struct CadSelectionContext {
    DocumentId docId{kInvalidDocumentId};
    QList<CadSelectionItem> items;
    bool hasDocument{false};
    bool sketchEditing{false};
    bool hasSelectedSketch{false};
    int selectedShapeCount{0};
    int selectedSketchId{0};
};

} // namespace lcnc::cad::selection