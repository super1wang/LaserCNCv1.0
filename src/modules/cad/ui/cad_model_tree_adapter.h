#pragma once

#include "core/document/lcnc_application.h"
#include "modules/cad/cad_module.h"

#include <QList>
#include <QString>
#include <QStringList>

namespace lcnc::cad::ui {

/// Semantic kind for a document-tree node rendered by MainWindow.
enum class CadTreeNodeKind {
    Document = 0,
    Group,
    Shape,
    Sketch,
    SketchElement,
    TemporarySketch
};

/// Tree node DTO used to render CAD document, shape, and sketch nodes.
struct CadTreeNode {
    CadTreeNodeKind kind{CadTreeNodeKind::Group};
    DocumentId documentId{kInvalidDocumentId};
    QString nodeKey;
    QString displayName;
    QString entry;
    QStringList leafEntries;
    bool checkable{true};
    bool checked{true};
    bool selectable{true};
    bool muted{false};
    QList<CadTreeNode> children;
};

/// Builds a UI-neutral CAD document tree snapshot from CadModule data.
class CadModelTreeAdapter
{
public:
    /// Build all workpiece document nodes, including active and finished sketches.
    static QList<CadTreeNode> build(CadModule* cad);

private:
    static CadTreeNode fromShapeNode(DocumentId docId,
                                     const CadModule::DocumentTreeNode& source);
};

} // namespace lcnc::cad::ui