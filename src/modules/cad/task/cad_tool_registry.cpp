#include "modules/cad/task/cad_tool_registry.h"

#include <algorithm>
#include <utility>

namespace lcnc::cad::task {
namespace {

bool toolLess(const CadToolDescriptor& lhs, const CadToolDescriptor& rhs)
{
    if (lhs.category != rhs.category)
        return static_cast<int>(lhs.category) < static_cast<int>(rhs.category);
    if (lhs.order != rhs.order)
        return lhs.order < rhs.order;
    return lhs.toolId < rhs.toolId;
}

CadToolDescriptor commandTool(QString toolId,
                              QString commandId,
                              QString title,
                              CadToolCategory category,
                              int order)
{
    CadToolDescriptor descriptor;
    descriptor.toolId = std::move(toolId);
    descriptor.commandId = std::move(commandId);
    descriptor.title = std::move(title);
    descriptor.category = category;
    descriptor.activation = CadToolActivation::Command;
    descriptor.order = order;
    return descriptor;
}

void acceptShapes(CadToolDescriptor* descriptor)
{
    if (!descriptor)
        return;
    descriptor->acceptedDomains = {
        lcnc::cad::selection::CadSelectionDomain::DocumentShape
    };
}

CadToolDescriptor pageTool(QString toolId,
                           QString title,
                           CadToolCategory category,
                           CadToolActivation activation,
                           int targetIndex,
                           int order)
{
    CadToolDescriptor descriptor;
    descriptor.toolId = std::move(toolId);
    descriptor.title = std::move(title);
    descriptor.category = category;
    descriptor.activation = activation;
    descriptor.targetIndex = targetIndex;
    descriptor.showWhenNoDocument = false;
    descriptor.order = order;
    return descriptor;
}

} // namespace

void CadToolRegistry::registerTool(const CadToolDescriptor& descriptor)
{
    for (auto& existing : m_tools) {
        if (existing.toolId == descriptor.toolId) {
            existing = descriptor;
            return;
        }
    }
    m_tools.append(descriptor);
}

QVector<CadToolDescriptor> CadToolRegistry::tools() const
{
    QVector<CadToolDescriptor> result = m_tools;
    std::sort(result.begin(), result.end(), toolLess);
    return result;
}

QVector<CadToolDescriptor> CadToolRegistry::toolsByCategory(CadToolCategory category) const
{
    QVector<CadToolDescriptor> result;
    for (const auto& tool : m_tools) {
        if (tool.category == category)
            result.append(tool);
    }
    std::sort(result.begin(), result.end(), toolLess);
    return result;
}

CadToolRegistry CadToolRegistry::createDefault()
{
    CadToolRegistry registry;

    auto newFile = commandTool(QStringLiteral("cad.document.new"),
                               QStringLiteral("file.new"),
                               QStringLiteral("新建文件"),
                               CadToolCategory::Document,
                               10);
    newFile.showWhenHasDocument = false;
    registry.registerTool(newFile);

    auto openFile = commandTool(QStringLiteral("cad.document.open"),
                                QStringLiteral("file.open"),
                                QStringLiteral("打开文件"),
                                CadToolCategory::Document,
                                20);
    openFile.showWhenHasDocument = false;
    registry.registerTool(openFile);

    registry.registerTool(pageTool(QStringLiteral("cad.sketch.begin"),
                                   QStringLiteral("新建草图"),
                                   CadToolCategory::BaseModeling,
                                   CadToolActivation::SketchPage,
                                   -1,
                                   10));
    registry.registerTool(pageTool(QStringLiteral("cad.primitive.box"),
                                   QStringLiteral("长方体"),
                                   CadToolCategory::BaseModeling,
                                   CadToolActivation::PrimitivePage,
                                   0,
                                   20));
    registry.registerTool(pageTool(QStringLiteral("cad.primitive.cylinder"),
                                   QStringLiteral("圆柱体"),
                                   CadToolCategory::BaseModeling,
                                   CadToolActivation::PrimitivePage,
                                   1,
                                   30));
    registry.registerTool(pageTool(QStringLiteral("cad.primitive.sphere"),
                                   QStringLiteral("球体"),
                                   CadToolCategory::BaseModeling,
                                   CadToolActivation::PrimitivePage,
                                   2,
                                   40));
    registry.registerTool(pageTool(QStringLiteral("cad.primitive.cone"),
                                   QStringLiteral("圆锥体"),
                                   CadToolCategory::BaseModeling,
                                   CadToolActivation::PrimitivePage,
                                   3,
                                   50));
    registry.registerTool(pageTool(QStringLiteral("cad.primitive.torus"),
                                   QStringLiteral("圆环体"),
                                   CadToolCategory::BaseModeling,
                                   CadToolActivation::PrimitivePage,
                                   4,
                                   60));

    auto extrude = pageTool(QStringLiteral("cad.feature.extrude"),
                            QStringLiteral("拉伸凸台"),
                            CadToolCategory::SketchFeature,
                            CadToolActivation::FeaturePage,
                            0,
                            10);
    extrude.requiresSelectedSketch = true;
    registry.registerTool(extrude);

    auto revolve = pageTool(QStringLiteral("cad.feature.revolve"),
                            QStringLiteral("旋转凸台"),
                            CadToolCategory::SketchFeature,
                            CadToolActivation::FeaturePage,
                            1,
                            20);
    revolve.requiresSelectedSketch = true;
    registry.registerTool(revolve);

    auto transform = pageTool(QStringLiteral("cad.transform"),
                              QStringLiteral("变换"),
                              CadToolCategory::Selection,
                              CadToolActivation::TransformPage,
                              0,
                              10);
    transform.showWhenNoDocument = false;
    transform.minSelectedShapes = 1;
    acceptShapes(&transform);
    registry.registerTool(transform);

    auto deleteShape = commandTool(QStringLiteral("cad.shape.delete"),
                                   QStringLiteral("cad.deleteShape"),
                                   QStringLiteral("删除"),
                                   CadToolCategory::Delete,
                                   30);
    deleteShape.showWhenNoDocument = false;
    deleteShape.minSelectedShapes = 1;
    acceptShapes(&deleteShape);
    registry.registerTool(deleteShape);

    auto explode = commandTool(QStringLiteral("cad.shape.explode"),
                               QStringLiteral("cad.explode"),
                               QStringLiteral("拆解"),
                               CadToolCategory::Selection,
                               40);
    explode.showWhenNoDocument = false;
    explode.minSelectedShapes = 1;
    acceptShapes(&explode);
    registry.registerTool(explode);

    auto measure = commandTool(QStringLiteral("cad.measure.distance"),
                               QStringLiteral("cad.measureDist"),
                               QStringLiteral("测距"),
                               CadToolCategory::Measure,
                               50);
    measure.showWhenNoDocument = false;
    measure.minSelectedShapes = 1;
    acceptShapes(&measure);
    registry.registerTool(measure);

    auto boolUnion = commandTool(QStringLiteral("cad.boolean.union"),
                                 QStringLiteral("cad.boolUnion"),
                                 QStringLiteral("并集"),
                                 CadToolCategory::Boolean,
                                 10);
    boolUnion.showWhenNoDocument = false;
    boolUnion.minSelectedShapes = 2;
    acceptShapes(&boolUnion);
    registry.registerTool(boolUnion);

    auto boolCut = commandTool(QStringLiteral("cad.boolean.cut"),
                               QStringLiteral("cad.boolCut"),
                               QStringLiteral("差集"),
                               CadToolCategory::Boolean,
                               20);
    boolCut.showWhenNoDocument = false;
    boolCut.minSelectedShapes = 2;
    acceptShapes(&boolCut);
    registry.registerTool(boolCut);

    auto boolCommon = commandTool(QStringLiteral("cad.boolean.common"),
                                  QStringLiteral("cad.boolCommon"),
                                  QStringLiteral("交集"),
                                  CadToolCategory::Boolean,
                                  30);
    boolCommon.showWhenNoDocument = false;
    boolCommon.minSelectedShapes = 2;
    acceptShapes(&boolCommon);
    registry.registerTool(boolCommon);

    return registry;
}

} // namespace lcnc::cad::task