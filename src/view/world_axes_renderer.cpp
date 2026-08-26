#include "view/world_axes_renderer.h"

#include "view/graphics_scene.h"
#include "core/logging/logger.h"

#include <QCoreApplication>

#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <Quantity_Color.hxx>
#include <Standard_Failure.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

namespace lcnc::view {

namespace {

/// 创建一段彩色线段 AIS_Shape；线宽 2.5，无穷状态关闭。
Handle(AIS_Shape) makeAxisAis(const TopoDS_Shape& edge,
                              double r, double g, double b)
{
    Handle(AIS_Shape) ais = new AIS_Shape(edge);
    ais->SetColor(Quantity_Color(r, g, b, Quantity_TOC_RGB));
    ais->SetWidth(2.5);
    ais->SetInfiniteState(false);
    return ais;
}

} // namespace

// ───────────────────────── singleton ──────────────────────────────────────────

WorldAxesRenderer& WorldAxesRenderer::instance()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "WorldAxesRenderer::instance");
    static WorldAxesRenderer* s_inst = []{
        // 父对象绑定到 QCoreApplication，确保进程退出时与 Qt 一同释放。
        return new WorldAxesRenderer(QCoreApplication::instance());
    }();
    return *s_inst;
}

WorldAxesRenderer::WorldAxesRenderer(QObject* parent) : QObject(parent)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "WorldAxesRenderer ctor");
}

WorldAxesRenderer::~WorldAxesRenderer()
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "WorldAxesRenderer dtor scenes={}", m_perScene.size());
    // 不主动 erase；进程退出时 OCC context 会随 GraphicsScene 一起销毁。
}

// ───────────────────────── public API ─────────────────────────────────────────

void WorldAxesRenderer::setAxisLength(double mm)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "WorldAxesRenderer::setAxisLength {:.1f}", mm);
    if (mm <= 0.0) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "WorldAxesRenderer::setAxisLength rejected non-positive: {}", mm);
        return;
    }
    if (qFuzzyCompare(mm, m_axisLength))
        return;
    m_axisLength = mm;
    m_shapesBuilt = false;
    // 已挂载的场景全部清空 AIS 对象（下次 attach/setGloballyVisible 会重建）
    for (auto& [scene, entry] : m_perScene) {
        if (!scene)
            continue;
        try {
            const auto& ctx = scene->context();
            for (auto& obj : entry.objects) {
                if (!obj.IsNull() && ctx->IsDisplayed(obj))
                    ctx->Erase(obj, false);
            }
            scene->viewer()->Redraw();
        } catch (...) {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "WorldAxesRenderer::setAxisLength erase failed");
        }
        entry.objects.clear();
    }
    if (m_globallyVisible) {
        for (auto& [scene, _] : m_perScene)
            applyVisibilityForScene(scene, true);
    }
}

void WorldAxesRenderer::setMachineAxisDirections(const QList<MachineAxisDef>& axes)
{
    auto linearAxis = [&axes](const QString& name, gp_Dir* result) {
        for (const MachineAxisDef& axis : axes) {
            if (axis.motionType == MachineAxisDef::Linear
                && axis.name.compare(name, Qt::CaseInsensitive) == 0) {
                *result = axis.direction;
                return true;
            }
        }
        return false;
    };

    gp_Dir nextX = m_axisX;
    gp_Dir nextY = m_axisY;
    gp_Dir nextZ = m_axisZ;
    if (!linearAxis(QStringLiteral("X"), &nextX)
        || !linearAxis(QStringLiteral("Y"), &nextY)
        || !linearAxis(QStringLiteral("Z"), &nextZ)) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "WorldAxesRenderer: machine coordinate axes require linear X, Y and Z definitions");
        return;
    }

    if (m_axisX.IsEqual(nextX, 1e-9) && m_axisY.IsEqual(nextY, 1e-9)
        && m_axisZ.IsEqual(nextZ, 1e-9)) {
        return;
    }

    m_axisX = nextX;
    m_axisY = nextY;
    m_axisZ = nextZ;
    m_shapesBuilt = false;
    for (auto& [scene, entry] : m_perScene) {
        if (!scene)
            continue;
        const auto& ctx = scene->context();
        for (auto& object : entry.objects) {
            if (!object.IsNull() && ctx->IsDisplayed(object))
                ctx->Erase(object, false);
        }
        entry.objects.clear();
        scene->viewer()->Redraw();
    }
    if (m_globallyVisible) {
        for (auto& [scene, _] : m_perScene)
            applyVisibilityForScene(scene, true);
    }
}

void WorldAxesRenderer::setGloballyVisible(bool visible)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "WorldAxesRenderer::setGloballyVisible {}", visible);
    if (m_globallyVisible == visible)
        return;
    m_globallyVisible = visible;
    LCNC_INFO(lcnc::LogCode::Generic,
              "World axes global visibility -> {} (scenes={})",
              visible, m_perScene.size());
    for (auto& [scene, _] : m_perScene)
        applyVisibilityForScene(scene, visible);
}

void WorldAxesRenderer::attach(GraphicsScene* scene)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "WorldAxesRenderer::attach scene={}",
               static_cast<const void*>(scene));
    if (!scene) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "WorldAxesRenderer::attach: null scene");
        return;
    }
    auto it = m_perScene.find(scene);
    if (it != m_perScene.end()) {
        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "WorldAxesRenderer::attach: already attached, skip");
        return;
    }
    m_perScene.emplace(scene, SceneEntry{});
    // 监听 scene 析构以自动清理映射
    connect(scene, &QObject::destroyed,
            this, &WorldAxesRenderer::onSceneDestroyed);
    if (m_globallyVisible)
        applyVisibilityForScene(scene, true);
}

void WorldAxesRenderer::detach(GraphicsScene* scene)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "WorldAxesRenderer::detach scene={}",
               static_cast<const void*>(scene));
    if (!scene)
        return;
    auto it = m_perScene.find(scene);
    if (it == m_perScene.end())
        return;
    try {
        const auto& ctx = scene->context();
        for (auto& obj : it->second.objects) {
            if (!obj.IsNull() && ctx->IsDisplayed(obj))
                ctx->Erase(obj, false);
        }
        scene->viewer()->Redraw();
    } catch (const Standard_Failure& f) {
        LCNC_ERR(lcnc::LogCode::Generic, "WorldAxesRenderer::detach OCC failure: {}", f.what());
    } catch (const std::exception& e) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "WorldAxesRenderer::detach std::exception: {}", e.what());
    } catch (...) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "WorldAxesRenderer::detach unknown exception");
    }
    disconnect(scene, &QObject::destroyed,
               this, &WorldAxesRenderer::onSceneDestroyed);
    m_perScene.erase(it);
}

// ───────────────────────── private ────────────────────────────────────────────

void WorldAxesRenderer::rebuildShapes()
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "WorldAxesRenderer::rebuildShapes len={:.1f}", m_axisLength);
    try {
        const gp_Pnt o(0.0, 0.0, 0.0);
        m_shapes.edgeX = BRepBuilderAPI_MakeEdge(
            o, o.Translated(gp_Vec(m_axisX) * m_axisLength)).Edge();
        m_shapes.edgeY = BRepBuilderAPI_MakeEdge(
            o, o.Translated(gp_Vec(m_axisY) * m_axisLength)).Edge();
        m_shapes.edgeZ = BRepBuilderAPI_MakeEdge(
            o, o.Translated(gp_Vec(m_axisZ) * m_axisLength)).Edge();
        const double r = m_axisLength * 0.012;
        m_shapes.sphere = BRepPrimAPI_MakeSphere(o, r).Shape();
        m_shapesBuilt = true;
    } catch (const Standard_Failure& f) {
        LCNC_ERR(lcnc::LogCode::Generic, "WorldAxesRenderer::rebuildShapes OCC failure: {}",
                 f.what());
    } catch (const std::exception& e) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "WorldAxesRenderer::rebuildShapes std::exception: {}", e.what());
    }
}

void WorldAxesRenderer::ensureSceneObjects(GraphicsScene* scene)
{
    auto it = m_perScene.find(scene);
    if (it == m_perScene.end())
        return;
    if (!it->second.objects.empty())
        return;
    if (!m_shapesBuilt)
        rebuildShapes();
    if (!m_shapesBuilt)
        return;
    try {
        // 每个场景独立创建 AIS_Shape 实例，避免 OCC 在多 context 间共享对象时的冲突。
        Handle(AIS_Shape) ax = makeAxisAis(m_shapes.edgeX, 1.0, 0.15, 0.15);
        Handle(AIS_Shape) ay = makeAxisAis(m_shapes.edgeY, 0.15, 0.85, 0.15);
        Handle(AIS_Shape) az = makeAxisAis(m_shapes.edgeZ, 0.20, 0.40, 1.0);
        Handle(AIS_Shape) sp = new AIS_Shape(m_shapes.sphere);
        sp->SetColor(Quantity_Color(1.0, 1.0, 0.2, Quantity_TOC_RGB));
        it->second.objects = {ax, ay, az, sp};
    } catch (const Standard_Failure& f) {
        LCNC_ERR(lcnc::LogCode::Generic, "WorldAxesRenderer::ensureSceneObjects OCC failure: {}",
                 f.what());
    } catch (const std::exception& e) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "WorldAxesRenderer::ensureSceneObjects std::exception: {}",
                 e.what());
    }
}

void WorldAxesRenderer::applyVisibilityForScene(GraphicsScene* scene, bool visible)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "WorldAxesRenderer::applyVisibilityForScene scene={} visible={}",
               static_cast<const void*>(scene), visible);
    if (!scene)
        return;
    auto it = m_perScene.find(scene);
    if (it == m_perScene.end())
        return;
    if (visible)
        ensureSceneObjects(scene);

    try {
        const auto& ctx = scene->context();
        for (auto& obj : it->second.objects) {
            if (obj.IsNull())
                continue;
            if (visible) {
                if (!ctx->IsDisplayed(obj))
                    ctx->Display(obj, false);
                ctx->Deactivate(obj);  // 不可拾取
            } else {
                if (ctx->IsDisplayed(obj))
                    ctx->Erase(obj, false);
            }
        }
        scene->viewer()->Redraw();
    } catch (const Standard_Failure& f) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "WorldAxesRenderer::applyVisibilityForScene OCC failure: {}", f.what());
    } catch (const std::exception& e) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "WorldAxesRenderer::applyVisibilityForScene std::exception: {}",
                 e.what());
    } catch (...) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "WorldAxesRenderer::applyVisibilityForScene unknown exception");
    }
}

void WorldAxesRenderer::onSceneDestroyed(QObject* obj)
{
    auto* scene = static_cast<GraphicsScene*>(obj);
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "WorldAxesRenderer::onSceneDestroyed scene={}",
               static_cast<const void*>(scene));
    m_perScene.erase(scene);
}

} // namespace lcnc::view
