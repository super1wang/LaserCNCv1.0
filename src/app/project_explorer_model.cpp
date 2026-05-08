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

#include <TDF_LabelSequence.hxx>

namespace lcnc::app {
namespace {

QMap<QString, QString> labelNameMap(LcncDocument* doc, LcncDocument::EntityKind kind)
{
    QMap<QString, QString> result;
    if (!doc)
        return result;

    const TDF_LabelSequence labels = doc->entityLabels(kind);
    for (int index = 1; index <= labels.Length(); ++index) {
        const TDF_Label label = labels.Value(index);
        result.insert(XcafUtils::entry(label), XcafUtils::name(label));
    }
    return result;
}

QString axisDisplayName(const MachineAxisDef& axis)
{
    if (axis.name == QStringLiteral("BASE"))
        return QObject::tr("BASE（固定基座）");

    if (axis.motionType == MachineAxisDef::Linear)
        return QObject::tr("%1 轴  (线性  ±%2 mm)").arg(axis.name).arg(axis.maxVal, 0, 'f', 0);

    const QString range = axis.maxVal >= 9000.0
        ? QObject::tr("连续旋转")
        : QObject::tr("±%1°").arg(axis.maxVal, 0, 'f', 0);
    return QObject::tr("%1 轴  (旋转  %2)").arg(axis.name, range);
}

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

void appendMachineSection(ProjectExplorerSnapshot& snapshot, CamModule* cam)
{
    LcncDocument* doc = cam ? cam->machineDocument() : nullptr;
    const DocumentId docId = cam ? cam->machineDocumentId() : kInvalidDocumentId;
    MachineKinematics* kin = doc ? doc->machineKinematics() : nullptr;

    ProjectExplorerNode root;
    root.kind = ProjectExplorerNodeKind::MachineRoot;
    root.documentId = docId;
    root.nodeKey = QStringLiteral("project.machine");
    root.displayName = QObject::tr("机台模型");
    root.infoText = (!doc || !kin || kin->axes().isEmpty()) ? QObject::tr("未配置") : QString();
    root.checkable = true;
    root.checked = true;

    if (!doc || !kin || kin->axes().isEmpty()) {
        snapshot.roots.append(std::move(root));
        return;
    }

    const QMap<QString, QString> machineNames = labelNameMap(doc, LcncDocument::EntityKind::Machine);
    QSet<QString> placed;

    for (const MachineAxisDef& axis : kin->axes()) {
        ProjectExplorerNode axisNode;
        axisNode.kind = ProjectExplorerNodeKind::MachineAxis;
        axisNode.documentId = docId;
        axisNode.nodeKey = QStringLiteral("project.machine.axis.%1").arg(axis.name);
        axisNode.displayName = axisDisplayName(axis);
        axisNode.axisName = axis.name;
        axisNode.checkable = true;
        axisNode.checked = true;
        axisNode.selectable = true;

        const QStringList machineEntries = kin->shapesForAxis(axis.name);
        for (const QString& entry : machineEntries) {
            ProjectExplorerNode shapeNode;
            shapeNode.kind = ProjectExplorerNodeKind::MachineShape;
            shapeNode.documentId = docId;
            shapeNode.nodeKey = QStringLiteral("project.machine.shape.%1").arg(entry);
            shapeNode.displayName = machineNames.value(entry, entry);
            shapeNode.entry = entry;
            shapeNode.axisName = axis.name;
            shapeNode.checkable = true;
            shapeNode.checked = true;
            axisNode.children.append(shapeNode);
            placed.insert(entry);
        }

        collectLeafEntries(axisNode);
        root.children.append(axisNode);
    }

    ProjectExplorerNode unassignedNode;
    unassignedNode.kind = ProjectExplorerNodeKind::MachineUnassignedGroup;
    unassignedNode.documentId = docId;
    unassignedNode.nodeKey = QStringLiteral("project.machine.unassigned");
    unassignedNode.displayName = QObject::tr("未分配");
    unassignedNode.checkable = true;
    unassignedNode.checked = true;
    unassignedNode.selectable = false;

    for (auto it = machineNames.cbegin(); it != machineNames.cend(); ++it) {
        if (placed.contains(it.key()))
            continue;

        ProjectExplorerNode shapeNode;
        shapeNode.kind = ProjectExplorerNodeKind::MachineShape;
        shapeNode.documentId = docId;
        shapeNode.nodeKey = QStringLiteral("project.machine.unassigned.%1").arg(it.key());
        shapeNode.displayName = it.value();
        shapeNode.entry = it.key();
        shapeNode.checkable = true;
        shapeNode.checked = true;
        unassignedNode.children.append(shapeNode);
    }

    if (!unassignedNode.children.isEmpty()) {
        collectLeafEntries(unassignedNode);
        root.children.append(unassignedNode);
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
    root.infoText = toolpath.contourCount() > 0
        ? QObject::tr("%1 条轮廓").arg(toolpath.contourCount())
        : QObject::tr("未生成");

    for (int index = 0; index < toolpath.contourCount(); ++index) {
        const LaserContour& contour = toolpath.contour(index);
        ProjectExplorerNode node;
        node.kind = ProjectExplorerNodeKind::ToolpathContour;
        node.documentId = root.documentId;
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
        root.children.append(node);
    }

    if (toolpath.contourCount() == 0) {
        LcncDocument* camDoc = cam->camDocument();
        const QMap<QString, QString> contourNames = labelNameMap(camDoc, LcncDocument::EntityKind::Cam);
        int index = 0;
        for (auto it = contourNames.cbegin(); it != contourNames.cend(); ++it, ++index) {
            ProjectExplorerNode node;
            node.kind = ProjectExplorerNodeKind::ToolpathContour;
            node.documentId = camDoc ? camDoc->id() : kInvalidDocumentId;
            node.nodeKey = QStringLiteral("project.cam.geometry.%1").arg(index);
            node.displayName = it.value().isEmpty()
                ? QObject::tr("轮廓 %1").arg(index + 1)
                : it.value();
            node.entry = it.key();
            node.checkable = true;
            node.checked = true;
            root.children.append(node);
        }
        if (!root.children.isEmpty())
            root.infoText = QObject::tr("%1 条轮廓").arg(root.children.size());
    }

    snapshot.roots.append(std::move(root));
}

} // namespace

ProjectExplorerSnapshot ProjectExplorerModel::build(CadModule* cad, CamModule* cam)
{
    ProjectExplorerSnapshot snapshot;
    appendWorkpieceSection(snapshot, cad);
    appendMachineSection(snapshot, cam);
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

bool isMachineProjectNode(ProjectExplorerNodeKind kind)
{
    switch (kind) {
    case ProjectExplorerNodeKind::MachineRoot:
    case ProjectExplorerNodeKind::MachineAxis:
    case ProjectExplorerNodeKind::MachineShape:
    case ProjectExplorerNodeKind::MachineUnassignedGroup:
        return true;
    default:
        return false;
    }
}

bool isToolpathProjectNode(ProjectExplorerNodeKind kind)
{
    return kind == ProjectExplorerNodeKind::ToolpathRoot
        || kind == ProjectExplorerNodeKind::ToolpathContour;
}

} // namespace lcnc::app