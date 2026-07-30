#include "modules/cad/task/cad_command_dispatcher.h"

#include "modules/cad/cad_module.h"

#include <TopoDS_Shape.hxx>

#include <utility>

namespace lcnc::cad::task {
namespace {

CadModule::PrimitiveParameters primitiveParams(const QVariantMap& params)
{
    CadModule::PrimitiveParameters result;
    result.sizeX = params.value(QStringLiteral("sizeX"), result.sizeX).toDouble();
    result.sizeY = params.value(QStringLiteral("sizeY"), result.sizeY).toDouble();
    result.sizeZ = params.value(QStringLiteral("sizeZ"), result.sizeZ).toDouble();
    result.radius1 = params.value(QStringLiteral("radius1"), result.radius1).toDouble();
    result.radius2 = params.value(QStringLiteral("radius2"), result.radius2).toDouble();
    return result;
}

CadToolDescriptor makeDescriptor(const QString& toolId,
                                 const QString& title,
                                 CadToolCategory category,
                                 CadToolActivation activation,
                                 int targetIndex)
{
    CadToolDescriptor descriptor;
    descriptor.toolId = toolId;
    descriptor.title = title;
    descriptor.category = category;
    descriptor.activation = activation;
    descriptor.targetIndex = targetIndex;
    return descriptor;
}

class PrimitiveToolCommand final : public ICadToolCommand
{
public:
    PrimitiveToolCommand(CadModule* cadModule, QString toolId, QString title, int primitiveIndex)
        : m_cadModule(cadModule)
        , m_descriptor(makeDescriptor(std::move(toolId), std::move(title),
                                      CadToolCategory::BaseModeling,
                                      CadToolActivation::PrimitivePage,
                                      primitiveIndex))
    {
    }

    CadToolDescriptor descriptor() const override { return m_descriptor; }

    bool preview(const CadCommandRequest& request, TopoDS_Shape* outShape, QString* errMsg) override
    {
        return m_cadModule && m_cadModule->buildPrimitivePreview(
            m_descriptor.targetIndex, primitiveParams(request.params), outShape, errMsg);
    }

    bool execute(const CadCommandRequest& request, QString* errMsg) override
    {
        if (!m_cadModule) {
            if (errMsg)
                // 中文翻译：CAD 模块不可用
                *errMsg = QStringLiteral("CAD module is not available");
            return false;
        }
        const bool ok = m_cadModule->createPrimitive(m_descriptor.targetIndex,
                                                     primitiveParams(request.params));
        if (!ok && errMsg && errMsg->isEmpty())
            // 中文翻译：创建基础体失败
            *errMsg = QStringLiteral("Failed to create base body");
        return ok;
    }

private:
    CadModule* m_cadModule{nullptr};
    CadToolDescriptor m_descriptor;
};

class FeatureToolCommand final : public ICadToolCommand
{
public:
    FeatureToolCommand(CadModule* cadModule, QString toolId, QString title, int featureIndex)
        : m_cadModule(cadModule)
        , m_descriptor(makeDescriptor(std::move(toolId), std::move(title),
                                      CadToolCategory::SketchFeature,
                                      CadToolActivation::FeaturePage,
                                      featureIndex))
    {
        m_descriptor.requiresSelectedSketch = true;
    }

    CadToolDescriptor descriptor() const override { return m_descriptor; }

    bool preview(const CadCommandRequest& request, TopoDS_Shape* outShape, QString* errMsg) override
    {
        return m_cadModule && m_cadModule->buildFeaturePreview(
            m_descriptor.targetIndex,
            request.params.value(QStringLiteral("length"), 10.0).toDouble(),
            request.params.value(QStringLiteral("angle"), 360.0).toDouble(),
            outShape,
            errMsg);
    }

    bool execute(const CadCommandRequest& request, QString* errMsg) override
    {
        if (!m_cadModule) {
            if (errMsg)
                // 中文翻译：CAD 模块不可用
                *errMsg = QStringLiteral("CAD module is not available");
            return false;
        }
        const bool ok = m_cadModule->applyFeature(
            m_descriptor.targetIndex,
            request.params.value(QStringLiteral("length"), 10.0).toDouble(),
            request.params.value(QStringLiteral("angle"), 360.0).toDouble());
        if (!ok && errMsg && errMsg->isEmpty())
            // 中文翻译：应用特征失败
            *errMsg = QStringLiteral("Apply feature failed");
        return ok;
    }

private:
    CadModule* m_cadModule{nullptr};
    CadToolDescriptor m_descriptor;
};

} // namespace

CadCommandDispatcher::CadCommandDispatcher(CadModule* cadModule)
    : m_cadModule(cadModule)
{
}

void CadCommandDispatcher::registerDefaultTools()
{
    registerCommand(std::make_unique<PrimitiveToolCommand>(m_cadModule,
                                                           QStringLiteral("cad.primitive.box"),
                                                           // 中文翻译：长方体
                                                           QStringLiteral("cuboid"),
                                                           0));
    registerCommand(std::make_unique<PrimitiveToolCommand>(m_cadModule,
                                                           QStringLiteral("cad.primitive.cylinder"),
                                                           // 中文翻译：圆柱体
                                                           QStringLiteral("cylinder"),
                                                           1));
    registerCommand(std::make_unique<PrimitiveToolCommand>(m_cadModule,
                                                           QStringLiteral("cad.primitive.sphere"),
                                                           // 中文翻译：球体
                                                           QStringLiteral("sphere"),
                                                           2));
    registerCommand(std::make_unique<PrimitiveToolCommand>(m_cadModule,
                                                           QStringLiteral("cad.primitive.cone"),
                                                           // 中文翻译：圆锥体
                                                           QStringLiteral("cone"),
                                                           3));
    registerCommand(std::make_unique<PrimitiveToolCommand>(m_cadModule,
                                                           QStringLiteral("cad.primitive.torus"),
                                                           // 中文翻译：圆环体
                                                           QStringLiteral("torus"),
                                                           4));
    registerCommand(std::make_unique<FeatureToolCommand>(m_cadModule,
                                                         QStringLiteral("cad.feature.extrude"),
                                                         // 中文翻译：拉伸凸台
                                                         QStringLiteral("extrude boss"),
                                                         0));
    registerCommand(std::make_unique<FeatureToolCommand>(m_cadModule,
                                                         QStringLiteral("cad.feature.revolve"),
                                                         // 中文翻译：旋转凸台
                                                         QStringLiteral("rotating boss"),
                                                         1));
    registerCommand(std::make_unique<FeatureToolCommand>(m_cadModule,
                                                         QStringLiteral("cad.feature.sweep"),
                                                         // 中文翻译：扫掠
                                                         QStringLiteral("sweep"),
                                                         2));
}

void CadCommandDispatcher::registerCommand(std::unique_ptr<ICadToolCommand> command)
{
    if (!command)
        return;
    const QString toolId = command->descriptor().toolId;
    if (toolId.isEmpty())
        return;
    m_commands.insert(toolId, std::shared_ptr<ICadToolCommand>(std::move(command)));
}

bool CadCommandDispatcher::preview(const QString& toolId,
                                   const CadCommandRequest& request,
                                   TopoDS_Shape* outShape,
                                   QString* errMsg)
{
    auto it = m_commands.find(toolId);
    if (it == m_commands.end()) {
        if (errMsg)
            // 中文翻译：未注册 CAD 工具: %1
            *errMsg = QStringLiteral("Unregistered CAD tool: %1").arg(toolId);
        return false;
    }
    return it.value()->preview(request, outShape, errMsg);
}

bool CadCommandDispatcher::execute(const QString& toolId,
                                   const CadCommandRequest& request,
                                   QString* errMsg)
{
    auto it = m_commands.find(toolId);
    if (it == m_commands.end()) {
        if (errMsg)
            // 中文翻译：未注册 CAD 工具: %1
            *errMsg = QStringLiteral("Unregistered CAD tool: %1").arg(toolId);
        return false;
    }
    return it.value()->execute(request, errMsg);
}

} // namespace lcnc::cad::task