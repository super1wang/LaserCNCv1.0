#include "app/project_explorer_model.h"

#include "core/algorithms/cam/laser_toolpath.h"
#include "core/document/lcnc_document.h"
#include "core/document/xcaf_utils.h"
#include "core/kernel/kernel.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/project/lcnc_project_manager.h"
#include "modules/cad/contracts/i_cad_project_explorer_projection.h"
#include "modules/cam/contracts/i_cam_project_explorer_projection.h"

#include <NCollection_Sequence.hxx>
#include <QHash>
#include <QMap>
#include <QObject>
#include <QRegularExpression>
#include <QSet>
#include <TDF_Label.hxx>
#include <utility>

namespace lcnc::app {
namespace {

QString localizedGeneratedToolpathName(const QString& value)
{
    // Generated names are persisted in English so project data remains stable
    // across language changes. Translate only the recognised generated forms at
    // the presentation boundary; user-provided names remain untouched.
    const auto numberedName = [&value](const QString& prefix, const char* source) {
        const QRegularExpression expression(
            QStringLiteral("^%1 (\\d+)$").arg(QRegularExpression::escape(prefix)));
        const QRegularExpressionMatch match = expression.match(value);
        return match.hasMatch() ? QObject::tr(source).arg(match.captured(1)) : QString();
    };

    // 中文翻译：加工轮廓 %1；外轮廓 %1；孔 %1；边缘 %1
    for (const auto& item : {std::pair{QStringLiteral("Machining contour"), "Machining contour %1"},
                             std::pair{QStringLiteral("Outer contour"), "Outer contour %1"},
                             std::pair{QStringLiteral("Hole"), "Hole %1"},
                             std::pair{QStringLiteral("Edge"), "Edge %1"}}) {
        const QString translated = numberedName(item.first, item.second);
        if (!translated.isEmpty())
            return translated;
    }

    // 中文翻译：外表面(%1面) ∩ 截面(%2面)
    static const QRegularExpression tubeLayerExpression(
        QStringLiteral("^Outer surface \\((\\d+) surface\\) ∩ Cross section \\((\\d+) surface\\)$"));
    const QRegularExpressionMatch tubeLayerMatch = tubeLayerExpression.match(value);
    if (tubeLayerMatch.hasMatch()) {
        return QObject::tr("Outer surface (%1 surface) ∩ Cross section (%2 surface)")
            .arg(tubeLayerMatch.captured(1), tubeLayerMatch.captured(2));
    }

    // 中文翻译：未分组；类型 %1；手动加工面组边界；加工面组外边界；加工面组孔边界
    if (value == QStringLiteral("Not grouped"))
        return QObject::tr("Not grouped");
    if (value == QStringLiteral("Manually process quilt boundaries"))
        return QObject::tr("Manually process quilt boundaries");
    if (value == QStringLiteral("Processing outer boundary of dough group"))
        return QObject::tr("Processing outer boundary of dough group");
    if (value == QStringLiteral("Machining quilt hole boundaries"))
        return QObject::tr("Machining quilt hole boundaries");

    static const QRegularExpression typeExpression(QStringLiteral("^Type (\\d+)$"));
    const QRegularExpressionMatch typeMatch = typeExpression.match(value);
    if (typeMatch.hasMatch())
        return QObject::tr("Type %1").arg(typeMatch.captured(1));
    return value;
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

    NCollection_Sequence<TDF_Label> labels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
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

void appendSketchNodes(ProjectExplorerNode& documentNode,
                       const lcnc::cad::ICadProjectExplorerProjection* cad,
                       DocumentId docId)
{
    if (!cad)
        return;

    if (cad->projectExplorerIsSketchEditing()
        && docId == cad->projectExplorerWorkpieceDocument()->id()) {
        ProjectExplorerNode sketchNode;
        sketchNode.kind = ProjectExplorerNodeKind::CadTemporarySketch;
        sketchNode.documentId = docId;
        sketchNode.nodeKey = QStringLiteral("__sketch_temp__");
        // 中文翻译：[新草图]
        sketchNode.displayName = QObject::tr("[new sketch]");
        for (const auto& element : cad->projectExplorerActiveSketchElements()) {
            ProjectExplorerNode elementNode;
            elementNode.kind = ProjectExplorerNodeKind::CadSketchElement;
            elementNode.documentId = docId;
            elementNode.nodeKey = QStringLiteral("__sketch_element_%1__").arg(element.id);
            elementNode.displayName = element.label;
            sketchNode.children.append(elementNode);
        }
        documentNode.children.append(sketchNode);
    }

    const auto finishedSketches = cad->projectExplorerFinishedSketches(docId);
    for (const auto& sketch : finishedSketches) {
        ProjectExplorerNode sketchNode;
        sketchNode.kind = ProjectExplorerNodeKind::CadSketch;
        sketchNode.documentId = docId;
        sketchNode.nodeKey = QStringLiteral("__sketch_finished_%1__").arg(sketch.sketchId);
        sketchNode.displayName = sketch.usedByFeature
            // 中文翻译：%1（已用）
            ? QObject::tr("%1 (used)").arg(sketch.name)
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

void appendWorkpieceSection(ProjectExplorerSnapshot& snapshot,
                            const lcnc::cad::ICadProjectExplorerProjection* cad)
{
    LcncDocument* doc = cad ? cad->projectExplorerWorkpieceDocument() : nullptr;
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
    // 中文翻译：工件
    root.displayName = QObject::tr("workpiece");
    // 中文翻译：未加载
    root.infoText = (!doc || workpieceCount <= 0) ? QObject::tr("not loaded") : QString();
    root.selectable = true;
    root.checked = true;

    if (doc && workpieceCount > 0) {
        ProjectExplorerNode documentNode;
        documentNode.kind = ProjectExplorerNodeKind::CadDocument;
        documentNode.documentId = doc->id();
        documentNode.nodeKey = QStringLiteral("workpiece:%1").arg(doc->id());
        documentNode.displayName = stateDisplayName.isEmpty()
            // 中文翻译：工件模型
            ? (doc->name().trimmed().isEmpty() ? QObject::tr("workpiece model") : doc->name().trimmed())
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

void appendToolpathSection(ProjectExplorerSnapshot& snapshot,
                           const lcnc::cam::ICamProjectExplorerProjection* cam)
{
    ProjectExplorerNode root;
    root.kind = ProjectExplorerNodeKind::ToolpathRoot;
    const lcnc::cam::ProjectExplorerSnapshot camSnapshot = cam
        ? cam->projectExplorerSnapshot() : lcnc::cam::ProjectExplorerSnapshot{};
    root.documentId = camSnapshot.documentId;
    root.nodeKey = QStringLiteral("project.cam");
    // The tree represents the user-editable machining result, not an opaque
    // implementation cache.  Keep the label aligned with the staged CAM flow.
    // 中文翻译：加工轮廓
    root.displayName = QObject::tr("Machining contour");
    root.selectable = true;
    root.droppable = true;
    root.checked = true;

    if (!cam) {
        // 中文翻译：未生成
        root.infoText = QObject::tr("Not generated");
        snapshot.roots.append(std::move(root));
        return;
    }

    root.infoText = !camSnapshot.contours.isEmpty()
        // 中文翻译：%1 图层 / %2 条轮廓
        ? QObject::tr("%1 layer / %2 outlines")
            .arg(static_cast<int>(camSnapshot.layers.size()))
            .arg(camSnapshot.contours.size())
        // 中文翻译：未生成
        : QObject::tr("Not generated");
    const struct { lcnc::cam::CamPipelineStage stage; const char* name; } stages[] = {
        // 中文翻译：分离面
        {lcnc::cam::CamPipelineStage::FaceSeparation, "separation surface"},
        // 中文翻译：提取轮廓
        {lcnc::cam::CamPipelineStage::ContourExtraction, "Extract contours"},
        // 中文翻译：离散点
        {lcnc::cam::CamPipelineStage::PointDiscretization, "discrete points"},
        // 中文翻译：几何刀路
        {lcnc::cam::CamPipelineStage::GeometricToolpath, "Geometric tool path"},
        // 中文翻译：机床求解
        {lcnc::cam::CamPipelineStage::MachineSolve, "Machine tool solution"},
    };
    QStringList stageSummary;
    for (const auto& item : stages) {
        const auto state = camSnapshot.stages[static_cast<int>(item.stage)];
        // 中文翻译：未执行
        const QString status = !state.available ? QObject::tr("Not executed")
            // 中文翻译：过期；完成
            : state.dirty ? QObject::tr("Expired") : QObject::tr("Complete");
        stageSummary << QObject::tr("%1:%2").arg(QString::fromUtf8(item.name), status);
    }
    root.toolTip = stageSummary.join(QStringLiteral(" · "));

    QHash<lcnc::cam::ContourId, lcnc::cam::ProjectExplorerContour> contoursById;
    for (const auto& contour : camSnapshot.contours)
        contoursById.insert(contour.contourId, contour);
    auto contourNode = [&](const lcnc::cam::ProjectExplorerContour& contour) {
        ProjectExplorerNode node;
        node.kind = ProjectExplorerNodeKind::ToolpathContour;
        node.documentId = root.documentId;
        node.layerId = contour.layerId;
        node.contourId = contour.contourId;
        node.nodeKey = QStringLiteral("project.toolpath.contour.%1").arg(
            node.contourId != 0
                ? QString::number(static_cast<qulonglong>(node.contourId))
                : QString::number(contour.contourIndex));
        node.displayName = localizedGeneratedToolpathName(contour.name);
        // 中文翻译：%1 点
        node.infoText = QObject::tr("%1 points").arg(contour.pointCount);
        node.contourIndex = contour.contourIndex;
        node.checkable = true;
        node.checked = contour.enabled;
        node.draggable = true;
        node.toolTip = localizedGeneratedToolpathName(contour.sourceInfo);
        return node;
    };

    QSet<int> placedContourIndexes;
    for (const auto& layer : camSnapshot.layers) {
        ProjectExplorerNode layerNode;
        layerNode.kind = ProjectExplorerNodeKind::ToolpathLayer;
        layerNode.documentId = root.documentId;
        layerNode.layerId = layer.layerId;
        layerNode.layerColor = layer.color;
        layerNode.toolName = layer.toolName;
        layerNode.nodeKey = QStringLiteral("project.toolpath.layer.%1").arg(
            QString::number(static_cast<qulonglong>(layer.layerId)));
        layerNode.displayName = layer.name.trimmed().isEmpty()
            // 中文翻译：图层 %1
            ? QObject::tr("Layer %1").arg(root.children.size() + 1)
            : localizedGeneratedToolpathName(layer.name);
        layerNode.checkable = true;
        layerNode.checked = layer.enabled;
        layerNode.selectable = true;
        layerNode.droppable = true;
        layerNode.infoText = layer.toolName.trimmed().isEmpty()
            // 中文翻译：工具: 未指定
            ? QObject::tr("Tools: unspecified")
            // 中文翻译：工具: %1
            : QObject::tr("Tool: %1").arg(layer.toolName.trimmed());

        for (const lcnc::cam::ContourId contourId : layer.contourIds) {
            const auto found = contoursById.constFind(contourId);
            if (found == contoursById.cend())
                continue;
            layerNode.children.append(contourNode(found.value()));
            placedContourIndexes.insert(found->contourIndex);
        }

        root.children.append(layerNode);
    }

    for (const auto& contour : camSnapshot.contours) {
        if (placedContourIndexes.contains(contour.contourIndex))
            continue;
        root.children.append(contourNode(contour));
    }

    // Phase C：删除"无 toolpath 时按 CAM XCAF 还原轮廓节点"的兜底分支 ——
    // CAM 已不再镜像 wire 到 XCAF；toolpath 缓存通过 cam_toolpath.toml 还原。

    snapshot.roots.append(std::move(root));
}

void appendMachiningFaceSection(ProjectExplorerSnapshot& snapshot,
                                const lcnc::cam::ICamProjectExplorerProjection* cam)
{
    const lcnc::cam::ProjectExplorerSnapshot camSnapshot = cam
        ? cam->projectExplorerSnapshot() : lcnc::cam::ProjectExplorerSnapshot{};
    ProjectExplorerNode root;
    root.kind = ProjectExplorerNodeKind::MachiningFaceRoot;
    root.nodeKey = QStringLiteral("project.machiningfaces");
    // 中文翻译：加工面
    root.displayName = QObject::tr("Processing surface");
    root.selectable = true;
    root.checkable = true;
    root.checked = cam && camSnapshot.facesVisible;

    // 横截面是内部工艺数据，不在“加工面”树节点中显示。
    const QList<lcnc::cam::ProjectExplorerFace>& faces = camSnapshot.faces;
    if (faces.isEmpty()) {
        // 中文翻译：未选择
        root.infoText = QObject::tr("Not selected");
        collectLeafEntries(root);
        snapshot.roots.append(std::move(root));
        return;
    }

    // 中文翻译：%1 个
    root.infoText = QObject::tr("%1").arg(faces.size());
    for (const lcnc::cam::ProjectExplorerFace& info : faces) {
        ProjectExplorerNode node;
        node.kind = ProjectExplorerNodeKind::MachiningFace;
        node.nodeKey = QStringLiteral("project.machiningface.%1")
            .arg(static_cast<qulonglong>(info.faceId));
        node.displayName = info.displayName;
        QString role;
        switch (info.role) {
        case lcnc::cam::MachiningFaceRole::CrossSection:
            // 中文翻译：横截面
            role = QObject::tr("cross section");
            break;
        case lcnc::cam::MachiningFaceRole::MachiningSurface:
            // 中文翻译：加工面
            role = QObject::tr("Processing surface");
            break;
        }
        node.infoText = QObject::tr("%1 · %2")
            // 中文翻译：手动；自动
            .arg(info.manual ? QObject::tr("Manual") : QObject::tr("automatic"), role);
        node.machiningFaceId = info.faceId;
        node.selectable = true;
        node.checkable = false;
        root.children.append(node);
    }
    collectLeafEntries(root);
    snapshot.roots.append(std::move(root));
}

} // namespace

ProjectExplorerSnapshot ProjectExplorerModel::build(
    const lcnc::cad::ICadProjectExplorerProjection* cad,
    const lcnc::cam::ICamProjectExplorerProjection* cam)
{
    ProjectExplorerSnapshot snapshot;
    appendWorkpieceSection(snapshot, cad);
    appendMachiningFaceSection(snapshot, cam);
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

bool isToolpathProjectNode(ProjectExplorerNodeKind kind)
{
    return kind == ProjectExplorerNodeKind::ToolpathRoot
    || kind == ProjectExplorerNodeKind::ToolpathLayer
        || kind == ProjectExplorerNodeKind::ToolpathContour;
}

bool isMachiningFaceProjectNode(ProjectExplorerNodeKind kind)
{
    return kind == ProjectExplorerNodeKind::MachiningFaceRoot
        || kind == ProjectExplorerNodeKind::MachiningFace;
}

} // namespace lcnc::app
