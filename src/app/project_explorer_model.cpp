#include "app/project_explorer_model.h"

#include "core/algorithms/cam/laser_toolpath.h"
#include "core/document/lcnc_document.h"
#include "core/document/xcaf_utils.h"
#include "core/kernel/kernel.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/project/lcnc_project_manager.h"
#include "modules/cad/cad_module.h"
#include "modules/cam/cam_module.h"

#include <QObject>
#include <QMap>
#include <QSet>
#include <QHash>

#include <TDF_LabelSequence.hxx>

namespace lcnc::app {
namespace {

void collectLeafEntries(ProjectExplorerNode& node)
{
    QStringList leaves;
    if (!node.entry.isEmpty())
        leaves.append(node.entry);

    for (ProjectExplorerNode& child : node.children) {
        collectLeafEntries(child);
        leaves.append(child.leafEntries);
    }
    node.leafEntries = leaves;
}

ProjectExplorerNode workpieceShapeNode(const LcncDocument::ShapeTreeNode& sourceNode,
                                       DocumentId docId,
                                       const QString& keyPrefix,
                                       int childIndex)
{
    ProjectExplorerNode node;
    node.kind = sourceNode.entry.isEmpty()
        ? ProjectExplorerNodeKind::CadGroup
        : ProjectExplorerNodeKind::CadShape;
    node.documentId = docId;
    node.nodeKey = sourceNode.entry.isEmpty()
        ? QStringLiteral("group:%1/%2").arg(keyPrefix).arg(childIndex)
        : QStringLiteral("entry:%1:%2").arg(docId).arg(sourceNode.entry);
    node.entry = sourceNode.entry;
    node.displayName = sourceNode.displayName.isEmpty()
        ? sourceNode.entry
        : sourceNode.displayName;
    node.checkable = true;
    node.checked = true;

    for (int index = 0; index < sourceNode.children.size(); ++index)
        node.children.append(workpieceShapeNode(sourceNode.children.at(index), docId, node.nodeKey, index));

    collectLeafEntries(node);
    return node;
}

void appendFallbackWorkpieceShapes(ProjectExplorerNode& documentNode, LcncDocument* doc)
{
    if (!doc)
        return;

    TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
    for (int index = 1; index <= labels.Length(); ++index) {
        const TDF_Label label = labels.Value(index);
        const QString entry = XcafUtils::entry(label);
        QString displayName = XcafUtils::name(label);
        if (displayName.isEmpty())
            displayName = entry;

        ProjectExplorerNode node;
        node.kind = ProjectExplorerNodeKind::CadShape;
        node.documentId = doc->id();
        node.nodeKey = QStringLiteral("entry:%1:%2").arg(doc->id()).arg(entry);
        node.displayName = displayName;
        node.entry = entry;
        node.checkable = true;
        node.checked = true;
        collectLeafEntries(node);
        documentNode.children.append(node);
    }
}

void appendSketchNodes(ProjectExplorerNode& documentNode, CadModule* cad, DocumentId docId)
{
    if (!cad)
        return;

    if (cad->isSketchEditing() && docId == cad->workpieceDocumentId()) {
        ProjectExplorerNode sketchNode;
        sketchNode.kind = ProjectExplorerNodeKind::CadTemporarySketch;
        sketchNode.documentId = docId;
        sketchNode.nodeKey = QStringLiteral("__sketch_temp__");
        sketchNode.displayName = QObject::tr("[新草图]");
        for (const auto& element : cad->sketchElementSnapshots()) {
            ProjectExplorerNode elementNode;
            elementNode.kind = ProjectExplorerNodeKind::CadSketchElement;
            elementNode.documentId = docId;
            elementNode.nodeKey = QStringLiteral("__sketch_element_%1__").arg(element.id);
            elementNode.displayName = element.label;
            sketchNode.children.append(elementNode);
        }
        documentNode.children.append(sketchNode);
    }

    const auto finishedSketches = cad->finishedSketchSnapshots(docId);
    for (const auto& sketch : finishedSketches) {
        ProjectExplorerNode sketchNode;
        sketchNode.kind = ProjectExplorerNodeKind::CadSketch;
        sketchNode.documentId = docId;
        sketchNode.nodeKey = QStringLiteral("__sketch_finished_%1__").arg(sketch.sketchId);
        sketchNode.displayName = sketch.usedByFeature
            ? QObject::tr("%1（已用）").arg(sketch.name)
            : sketch.name;
        sketchNode.checked = sketch.visible;
        sketchNode.muted = sketch.usedByFeature;
        for (const auto& element : sketch.elements) {
            ProjectExplorerNode elementNode;
            elementNode.kind = ProjectExplorerNodeKind::CadSketchElement;
            elementNode.documentId = docId;
            elementNode.nodeKey = QStringLiteral("__sketch_finished_element_%1_%2__")
                .arg(sketch.sketchId)
                .arg(element.id);
            elementNode.displayName = element.label;
            sketchNode.children.append(elementNode);
        }
        documentNode.children.append(sketchNode);
    }
}

void appendWorkpieceSection(ProjectExplorerSnapshot& snapshot, CadModule* cad)
{
    LcncDocument* doc = cad ? cad->workpieceDocument() : nullptr;
    const int workpieceCount = doc
        ? doc->entityLabels(LcncDocument::EntityKind::Workpiece).Length()
        : 0;
    const QString stateDisplayName = lcnc::Kernel::current()
        .projectManager()
        ->session()
        .workpiece()
        .displayName
        .trimmed();

    ProjectExplorerNode root;
    root.kind = ProjectExplorerNodeKind::WorkpieceRoot;
    root.nodeKey = QStringLiteral("project.workpiece");
    root.displayName = QObject::tr("工件");
    root.infoText = (!doc || workpieceCount <= 0) ? QObject::tr("未加载") : QString();
    root.selectable = true;
    root.checked = true;

    if (doc && workpieceCount > 0) {
        ProjectExplorerNode documentNode;
        documentNode.kind = ProjectExplorerNodeKind::CadDocument;
        documentNode.documentId = doc->id();
        documentNode.nodeKey = QStringLiteral("workpiece:%1").arg(doc->id());
        documentNode.displayName = stateDisplayName.isEmpty()
            ? (doc->name().trimmed().isEmpty() ? QObject::tr("工件模型") : doc->name().trimmed())
            : stateDisplayName;

        const auto& hierarchy = doc->entityTree(LcncDocument::EntityKind::Workpiece);
        if (!hierarchy.isEmpty()) {
            for (int index = 0; index < hierarchy.size(); ++index) {
                documentNode.children.append(
                    workpieceShapeNode(hierarchy.at(index), doc->id(), documentNode.nodeKey, index));
            }
        } else {
            appendFallbackWorkpieceShapes(documentNode, doc);
        }

        appendSketchNodes(documentNode, cad, doc->id());
        collectLeafEntries(documentNode);
        root.children.append(documentNode);
    }

    collectLeafEntries(root);
    snapshot.roots.append(std::move(root));
}

void appendToolpathSection(ProjectExplorerSnapshot& snapshot, CamModule* cam)
{
    ProjectExplorerNode root;
    root.kind = ProjectExplorerNodeKind::ToolpathRoot;
    root.documentId = cam ? cam->camDocumentId() : kInvalidDocumentId;
    root.nodeKey = QStringLiteral("project.cam");
    root.displayName = QObject::tr("CAM 数据");
    root.selectable = true;
    root.droppable = true;
    root.checked = true;

    if (!cam) {
        root.infoText = QObject::tr("未生成");
        snapshot.roots.append(std::move(root));
        return;
    }

    const LaserToolpath& toolpath = cam->toolpath();
    const auto& layers = cam->toolpathLayers();
    root.infoText = toolpath.contourCount() > 0
        ? QObject::tr("%1 图层 / %2 条轮廓")
            .arg(static_cast<int>(layers.size()))
            .arg(toolpath.contourCount())
        : QObject::tr("未生成");

    auto contourNode = [&](int index) {
        const LaserContour& contour = toolpath.contour(index);
        ProjectExplorerNode node;
        node.kind = ProjectExplorerNodeKind::ToolpathContour;
        node.documentId = root.documentId;
        node.layerId = contour.layerId;
        node.contourId = static_cast<lcnc::cam::ContourId>(contour.contourId);
        node.nodeKey = QStringLiteral("project.toolpath.contour.%1").arg(
            node.contourId != 0
                ? QString::number(static_cast<qulonglong>(node.contourId))
                : QString::number(index));
        node.displayName = contour.name;
        node.infoText = QObject::tr("%1 点").arg(contour.points.size());
        node.contourIndex = index;
        node.checkable = true;
        node.checked = contour.enabled;
        node.draggable = true;
        node.toolTip = contour.sourceInfo;
        return node;
    };

    QSet<int> placedContourIndexes;
    for (const ToolpathLayer& layer : layers) {
        ProjectExplorerNode layerNode;
        layerNode.kind = ProjectExplorerNodeKind::ToolpathLayer;
        layerNode.documentId = root.documentId;
        layerNode.layerId = layer.layerId;
        layerNode.layerColor = layer.color;
        layerNode.toolName = layer.toolName;
        layerNode.nodeKey = QStringLiteral("project.toolpath.layer.%1").arg(
            QString::number(static_cast<qulonglong>(layer.layerId)));
        layerNode.displayName = layer.name.trimmed().isEmpty()
            ? QObject::tr("图层 %1").arg(root.children.size() + 1)
            : layer.name;
        layerNode.checkable = true;
        layerNode.checked = layer.enabled;
        layerNode.selectable = true;
        layerNode.droppable = true;
        layerNode.infoText = layer.toolName.trimmed().isEmpty()
            ? QObject::tr("%1 条轮廓").arg(static_cast<int>(layer.contourIds.size()))
            : QObject::tr("%1 条 / %2")
                .arg(static_cast<int>(layer.contourIds.size()))
                .arg(layer.toolName.trimmed());

        for (std::uint64_t contourId : layer.contourIds) {
            const int index = cam->contourIndexById(static_cast<lcnc::cam::ContourId>(contourId));
            if (index < 0 || index >= toolpath.contourCount())
                continue;
            layerNode.children.append(contourNode(index));
            placedContourIndexes.insert(index);
        }

        root.children.append(layerNode);
    }

    for (int index = 0; index < toolpath.contourCount(); ++index) {
        if (placedContourIndexes.contains(index))
            continue;
        root.children.append(contourNode(index));
    }

    // Phase C：删除"无 toolpath 时按 CAM XCAF 还原轮廓节点"的兜底分支 ——
    // CAM 已不再镜像 wire 到 XCAF；toolpath 缓存通过 cam_toolpath.toml 还原。

    snapshot.roots.append(std::move(root));
}

} // namespace

ProjectExplorerSnapshot ProjectExplorerModel::build(CadModule* cad, CamModule* cam)
{
    ProjectExplorerSnapshot snapshot;
    appendWorkpieceSection(snapshot, cad);
    appendToolpathSection(snapshot, cam);
    return snapshot;
}

bool isCadProjectNode(ProjectExplorerNodeKind kind)
{
    switch (kind) {
    case ProjectExplorerNodeKind::WorkpieceRoot:
    case ProjectExplorerNodeKind::CadDocument:
    case ProjectExplorerNodeKind::CadGroup:
    case ProjectExplorerNodeKind::CadShape:
    case ProjectExplorerNodeKind::CadSketch:
    case ProjectExplorerNodeKind::CadSketchElement:
    case ProjectExplorerNodeKind::CadTemporarySketch:
        return true;
    default:
        return false;
    }
}

// Phase D：机台节点已从工程树删除，该函数保留以兼容 main_window 调用方；
// 后续 Phase F 可改为返回 false 并删掉调用方 switch 分支。
bool isMachineProjectNode(ProjectExplorerNodeKind kind)
{
    (void)kind;
    return false;
}

bool isToolpathProjectNode(ProjectExplorerNodeKind kind)
{
    return kind == ProjectExplorerNodeKind::ToolpathRoot
    || kind == ProjectExplorerNodeKind::ToolpathLayer
        || kind == ProjectExplorerNodeKind::ToolpathContour;
}

} // namespace lcnc::app