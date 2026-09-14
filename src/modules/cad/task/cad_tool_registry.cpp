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
                               // 中文翻译：新建文件
                               QStringLiteral("Create new file"),
                               CadToolCategory::Document,
                               10);
    newFile.showWhenHasDocument = false;
    registry.registerTool(newFile);

    auto openFile = commandTool(QStringLiteral("cad.document.open"),
                                QStringLiteral("file.open"),
                                // 中文翻译：打开文件
                                QStringLiteral("open file"),
                                CadToolCategory::Document,
                                20);
    openFile.showWhenHasDocument = false;
    registry.registerTool(openFile);

    registry.registerTool(pageTool(QStringLiteral("cad.sketch.begin"),
                                   // 中文翻译：新建草图
                                   QStringLiteral("Create a new sketch"),
                                   CadToolCategory::BaseModeling,
                                   CadToolActivation::SketchPage,
                                   -1,
                                   10));
    registry.registerTool(pageTool(QStringLiteral("cad.primitive.box"),
                                   // 中文翻译：长方体
                                   QStringLiteral("cuboid"),
                                   CadToolCategory::BaseModeling,
                                   CadToolActivation::PrimitivePage,
                                   0,
                                   20));
    registry.registerTool(pageTool(QStringLiteral("cad.primitive.cylinder"),
                                   // 中文翻译：圆柱体
                                   QStringLiteral("cylinder"),
                                   CadToolCategory::BaseModeling,
                                   CadToolActivation::PrimitivePage,
                                   1,
                                   30));
    registry.registerTool(pageTool(QStringLiteral("cad.primitive.sphere"),
                                   // 中文翻译：球体
                                   QStringLiteral("sphere"),
                                   CadToolCategory::BaseModeling,
                                   CadToolActivation::PrimitivePage,
                                   2,
                                   40));
    registry.registerTool(pageTool(QStringLiteral("cad.primitive.cone"),
                                   // 中文翻译：圆锥体
                                   QStringLiteral("cone"),
                                   CadToolCategory::BaseModeling,
                                   CadToolActivation::PrimitivePage,
                                   3,
                                   50));
    registry.registerTool(pageTool(QStringLiteral("cad.primitive.torus"),
                                   // 中文翻译：圆环体
                                   QStringLiteral("torus"),
                                   CadToolCategory::BaseModeling,
                                   CadToolActivation::PrimitivePage,
                                   4,
                                   60));

    auto extrude = pageTool(QStringLiteral("cad.feature.extrude"),
                            // 中文翻译：拉伸凸台
                            QStringLiteral("extrude boss"),
                            CadToolCategory::SketchFeature,
                            CadToolActivation::FeaturePage,
                            0,
                            10);
    extrude.requiresSelectedSketch = true;
    registry.registerTool(extrude);

    auto revolve = pageTool(QStringLiteral("cad.feature.revolve"),
                            // 中文翻译：旋转凸台
                            QStringLiteral("rotating boss"),
                            CadToolCategory::SketchFeature,
                            CadToolActivation::FeaturePage,
                            1,
                            20);
    revolve.requiresSelectedSketch = true;
    registry.registerTool(revolve);

    auto transform = pageTool(QStringLiteral("cad.transform"),
                              // 中文翻译：变换
                              QStringLiteral("transform"),
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
                                   // 中文翻译：删除
                                   QStringLiteral("Delete"),
                                   CadToolCategory::Delete,
                                   30);
    deleteShape.showWhenNoDocument = false;
    deleteShape.minSelectedShapes = 1;
    acceptShapes(&deleteShape);
    registry.registerTool(deleteShape);

    auto explode = commandTool(QStringLiteral("cad.shape.explode"),
                               QStringLiteral("cad.explode"),
                               // 中文翻译：拆解
                               QStringLiteral("Explode"),
                               CadToolCategory::Selection,
                               40);
    explode.showWhenNoDocument = false;
    explode.minSelectedShapes = 1;
    acceptShapes(&explode);
    registry.registerTool(explode);

    auto measure = commandTool(QStringLiteral("cad.measure.distance"),
                               QStringLiteral("cad.measureDist"),
                               // 中文翻译：测距
                               QStringLiteral("Ranging"),
                               CadToolCategory::Measure,
                               50);
    measure.showWhenNoDocument = false;
    measure.minSelectedShapes = 1;
    acceptShapes(&measure);
    registry.registerTool(measure);

    auto boolUnion = commandTool(QStringLiteral("cad.boolean.union"),
                                 QStringLiteral("cad.boolUnion"),
                                 // 中文翻译：并集
                                 QStringLiteral("union"),
                                 CadToolCategory::Boolean,
                                 10);
    boolUnion.showWhenNoDocument = false;
    boolUnion.minSelectedShapes = 2;
    acceptShapes(&boolUnion);
    registry.registerTool(boolUnion);

    auto boolCut = commandTool(QStringLiteral("cad.boolean.cut"),
                               QStringLiteral("cad.boolCut"),
                               // 中文翻译：差集
                               QStringLiteral("difference set"),
                               CadToolCategory::Boolean,
                               20);
    boolCut.showWhenNoDocument = false;
    boolCut.minSelectedShapes = 2;
    acceptShapes(&boolCut);
    registry.registerTool(boolCut);

    auto boolCommon = commandTool(QStringLiteral("cad.boolean.common"),
                                  QStringLiteral("cad.boolCommon"),
                                  // 中文翻译：交集
                                  QStringLiteral("intersection"),
                                  CadToolCategory::Boolean,
                                  30);
    boolCommon.showWhenNoDocument = false;
    boolCommon.minSelectedShapes = 2;
    acceptShapes(&boolCommon);
    registry.registerTool(boolCommon);

    return registry;
}

} // namespace lcnc::cad::task