#pragma once

#include <QObject>

#include "core/kinematics/machine_kinematics.h"

#include <AIS_InteractiveObject.hxx>
#include <AIS_Shape.hxx>
#include <TopoDS_Shape.hxx>

#include <unordered_map>
#include <vector>

class GraphicsScene;

namespace lcnc::view {

/**
 * @brief 持久世界坐标系指示器（红 X / 绿 Y / 蓝 Z 三轴 + 黄色原点小球）。
 *
 * v2 设计：
 *   - 单一全局可见性开关（@ref isGloballyVisible）；
 *   - 可同时挂载到任意多个 GraphicsScene（机台 scene + 各文档 scene）；
 *   - 每个 scene 拥有独立的一组 AIS 对象（共享同一份只读 TopoDS_Shape）；
 *   - 监听 GraphicsScene 的 QObject::destroyed 自动清理避免悬空。
 *
 * 用法：
 // 中文翻译：坐标系
 *   - 用户拨动 ribbon "coordinate system" 按钮 → 调 setGloballyVisible(b)；
 *   - MainWindow 监听 guiDocumentAdded → 调 attach(scene)；
 *   - MainWindow 监听 guiDocumentClosed → 调 detach(scene)；
 *   - 同一 scene 重复 attach 安全（去重）。
 */
class WorldAxesRenderer : public QObject
{
    Q_OBJECT
public:
    /// 取进程内唯一实例（绑定 QApplication 父对象，线程安全的延迟初始化）。
    static WorldAxesRenderer& instance();

    /// 全局可见性开关；状态会同步广播到所有已 attach 的场景。
    void setGloballyVisible(bool visible);

    /// 当前全局可见性。
    bool isGloballyVisible() const { return m_globallyVisible; }

    /// 把渲染器挂到指定场景。重复 attach 同一场景安全。
    void attach(GraphicsScene* scene);

    /// 从指定场景卸载（同时擦除并销毁该场景的 AIS 对象）。
    void detach(GraphicsScene* scene);

    /// 三轴长度（毫米）。变更后会重建所有 AIS。默认 200。
    void setAxisLength(double mm);

    /// Use the configured linear machine axes for the displayed world axes.
    /// This is display-only; machine motion continues to use the same source
    /// definitions in MachineKinematics.
    void setMachineAxisDirections(const QList<MachineAxisDef>& axes);

private:
    explicit WorldAxesRenderer(QObject* parent = nullptr);
    ~WorldAxesRenderer() override;

    /// 构建/重建全局共享的 TopoDS_Shape（轴线 + 小球）。
    void rebuildShapes();

    /// 为指定场景按需创建 AIS 对象（不显示，仅创建）。
    void ensureSceneObjects(GraphicsScene* scene);

    /// 在指定场景上 Display/Erase 已创建对象。
    void applyVisibilityForScene(GraphicsScene* scene, bool visible);

private slots:
    /// 场景析构时的清理槽。
    void onSceneDestroyed(QObject* obj);

private:
    struct AxisShapes {
        TopoDS_Shape edgeX;
        TopoDS_Shape edgeY;
        TopoDS_Shape edgeZ;
        TopoDS_Shape sphere;
    };

    struct SceneEntry {
        std::vector<Handle(AIS_InteractiveObject)> objects; // 4 个 AIS
    };

    bool   m_globallyVisible{false};
    double m_axisLength{200.0};
    gp_Dir m_axisX{1.0, 0.0, 0.0};
    gp_Dir m_axisY{0.0, 1.0, 0.0};
    gp_Dir m_axisZ{0.0, 0.0, 1.0};
    AxisShapes m_shapes;
    bool   m_shapesBuilt{false};
    std::unordered_map<GraphicsScene*, SceneEntry> m_perScene;
};

} // namespace lcnc::view
