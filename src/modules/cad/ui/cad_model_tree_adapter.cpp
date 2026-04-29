#include "modules/cad/ui/cad_model_tree_adapter.h"

#include <QObject>
#include <utility>

namespace lcnc::cad::ui {

CadTreeNode CadModelTreeAdapter::fromShapeNode(DocumentId docId,
                                               const CadModule::DocumentTreeNode& source)
{
    CadTreeNode node;
    node.documentId = docId;
    node.nodeKey = source.nodeKey;
    node.displayName = source.displayName;
    node.entry = source.entry;
    node.leafEntries = source.leafEntries;
    node.kind = source.entry.isEmpty() ? CadTreeNodeKind::Group : CadTreeNodeKind::Shape;
    for (const auto& childSource : source.children)
        node.children.append(fromShapeNode(docId, childSource));
    return node;
}

QList<CadTreeNode> CadModelTreeAdapter::build(CadModule* cad)
{
    QList<CadTreeNode> result;
    if (!cad)
        return result;

    const auto documents = cad->documentTreeDocuments();
    result.reserve(documents.size());
    for (const auto& document : documents) {
        CadTreeNode documentNode;
        documentNode.kind = CadTreeNodeKind::Document;
        documentNode.documentId = document.documentId;
        documentNode.nodeKey = document.nodeKey;
        documentNode.displayName = document.displayName;
        documentNode.leafEntries = document.leafEntries;

        for (const auto& childSource : document.children)
            documentNode.children.append(fromShapeNode(document.documentId, childSource));

        if (cad->isSketchEditing() && document.documentId == cad->activeDocumentId()) {
            CadTreeNode sketchNode;
            sketchNode.kind = CadTreeNodeKind::TemporarySketch;
            sketchNode.documentId = document.documentId;
            sketchNode.nodeKey = QStringLiteral("__sketch_temp__");
            sketchNode.displayName = QObject::tr("[新草图]");
            sketchNode.checkable = false;
            for (const auto& element : cad->sketchElementSnapshots()) {
                CadTreeNode elementNode;
                elementNode.kind = CadTreeNodeKind::SketchElement;
                elementNode.documentId = document.documentId;
                elementNode.nodeKey = QStringLiteral("__sketch_element_%1__").arg(element.id);
                elementNode.displayName = element.label;
                elementNode.checkable = false;
                sketchNode.children.append(elementNode);
            }
            documentNode.children.append(sketchNode);
        }

        const auto finishedSketches = cad->finishedSketchSnapshots(document.documentId);
        for (const auto& sketch : finishedSketches) {
            CadTreeNode sketchNode;
            sketchNode.kind = CadTreeNodeKind::Sketch;
            sketchNode.documentId = document.documentId;
            sketchNode.nodeKey = QStringLiteral("__sketch_finished_%1__").arg(sketch.sketchId);
            sketchNode.displayName = sketch.usedByFeature
                ? QObject::tr("%1（已用）").arg(sketch.name)
                : sketch.name;
            sketchNode.checked = sketch.visible;
            sketchNode.muted = sketch.usedByFeature;
            for (const auto& element : sketch.elements) {
                CadTreeNode elementNode;
                elementNode.kind = CadTreeNodeKind::SketchElement;
                elementNode.documentId = document.documentId;
                elementNode.nodeKey = QStringLiteral("__sketch_finished_element_%1_%2__")
                    .arg(sketch.sketchId)
                    .arg(element.id);
                elementNode.displayName = element.label;
                elementNode.checkable = false;
                sketchNode.children.append(elementNode);
            }
            documentNode.children.append(sketchNode);
        }

        result.append(std::move(documentNode));
    }
    return result;
}

} // namespace lcnc::cad::ui