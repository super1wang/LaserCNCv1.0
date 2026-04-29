#pragma once

#include "modules/cad/selection/cad_selection.h"

#include <QStringList>

namespace lcnc::cad::selection {

/// Converts legacy view/tree selection data into normalized CAD selection contexts.
class CadSelectionResolver
{
public:
    /// Build a context from XCAF document shape entries.
    static CadSelectionContext fromShapeEntries(DocumentId docId,
                                                const QStringList& entries,
                                                bool hasDocument = true,
                                                bool sketchEditing = false,
                                                bool hasSelectedSketch = false,
                                                int selectedSketchId = 0);

    /// Build a context from a model-tree node payload.
    static CadSelectionContext fromDocumentTreeNode(DocumentId docId,
                                                    const QString& nodeKey,
                                                    const QString& entry,
                                                    const QStringList& leafEntries,
                                                    bool hasDocument = true,
                                                    bool sketchEditing = false);

    /// Build a context from a view overlay key emitted by SketchOverlayRenderer.
    static CadSelectionContext fromOverlayKey(DocumentId docId,
                                              const QString& overlayKey,
                                              bool hasDocument = true,
                                              bool sketchEditing = false);

    /// True when the node key points to a finished sketch node.
    static bool isFinishedSketchNode(const QString& nodeKey);

    /// Parse a finished sketch id from the node key; returns 0 if absent.
    static int sketchIdFromNodeKey(const QString& nodeKey);

private:
    static int sketchElementIdFromNodeKey(const QString& nodeKey);
    static int activeSketchElementIdFromNodeKey(const QString& nodeKey);
};

} // namespace lcnc::cad::selection