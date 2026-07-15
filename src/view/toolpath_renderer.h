#pragma once

#include <QList>
#include <AIS_InteractiveObject.hxx>
#include <AIS_Shape.hxx>
#include <gp_Pnt.hxx>

class GuiDocument;
class LaserToolpath;
class MachineKinematics;

namespace lcnc::view {

/**
 * @brief 刀路 AIS 渲染器（v2.2 从 CamModule 抽出）。
 *
 * 拥有刀路派生覆盖 AIS 对象（引入线 / 法线 / 预览）以及可见性 / 法线显示
 * 状态。轮廓主体由 CAM document 通过 GuiDocument registry 唯一显示。
 *
 * 算法上零业务依赖：仅根据传入的 LaserToolpath + 参数 + 引入线预览生成 AIS。
 */
class ToolpathRenderer
{
public:
    /// 引入线预览（与 picking 服务共享的瞬态状态，由 CamModule 在调用前填好）。
    struct LeadInPreview {
        int    contourIndex{-1};
        int    pointIndex{-1};
        gp_Pnt entryPoint;
        double entryParam{0.0};
        bool   valid{false};
    };

    ToolpathRenderer();
    ~ToolpathRenderer();

    /// 擦除并按当前 visible/normal 标志重绘刀路派生覆盖 AIS。
    void refresh(GuiDocument* gd,
                 const LaserToolpath& toolpath,
                 MachineKinematics* kin,
                 const LeadInPreview& preview = {});

    /// 只刷新引入线和引入线预览，不重建轮廓 AIS。
    void refreshLeadIns(GuiDocument* gd,
                        const LaserToolpath& toolpath,
                        MachineKinematics* kin,
                        const LeadInPreview& preview = {});

    /// 只刷新法线 AIS，不重建轮廓或引入线。
    void refreshNormals(GuiDocument* gd,
                        const LaserToolpath& toolpath,
                        MachineKinematics* kin);

    /// 只刷新单条轮廓对应的引入线 / 法线 AIS。
    void refreshContour(GuiDocument* gd,
                        const LaserToolpath& toolpath,
                        MachineKinematics* kin,
                        int contourIndex,
                        const LeadInPreview& preview = {});

    /// Apply the current workpiece transform to cached derived overlay AIS.
    void updateTransforms(GuiDocument* gd,
                          const LaserToolpath& toolpath,
                          MachineKinematics* kin);

    /// 仅擦除 AIS（不修改可见性标志）。
    void erase(GuiDocument* gd);

    /// 切换刀路总可见性（Display/Erase 已存在的 AIS）。
    void setVisible(GuiDocument* gd, bool visible);
    bool isVisible() const { return m_visible; }

    /// 是否绘制法线指示线。
    void setShowNormals(bool on) { m_showNormals = on; }
    bool showNormals() const { return m_showNormals; }

    /// 法线采样步长（毫米）。<= 0 忽略。
    bool   setNormalSampleStep(double mm);
    double normalSampleStep() const { return m_normalSampleStep; }

    const QList<Handle(AIS_Shape)>& contourAis() const { return m_contourAis; }
    int contourIndexForAis(const Handle(AIS_InteractiveObject)& object) const;
    QList<int> selectedContourIndexes(GuiDocument* gd) const;
    int selectedContourIndex(GuiDocument* gd) const;

private:
    struct ContourAisBundle {
        Handle(AIS_Shape) leadIn;
        Handle(AIS_Shape) normal;
    };

    void ensureBundleCount(GuiDocument* gd, int count);
    void clearAis(GuiDocument* gd, bool updateView);
    void eraseAis(GuiDocument* gd, Handle(AIS_Shape)& ais);
    void eraseBundle(GuiDocument* gd, ContourAisBundle& bundle);
    void rebuildLeadInAis(GuiDocument* gd, const LaserToolpath& tp, int contourIndex);
    void rebuildNormalAis(GuiDocument* gd, const LaserToolpath& tp, int contourIndex);
    void rebuildPreviewAis(GuiDocument* gd, const LaserToolpath& tp, const LeadInPreview& preview);
    void rebuildContourMirror();
    void redraw(GuiDocument* gd);

    QList<ContourAisBundle> m_bundles;
    QList<Handle(AIS_Shape)> m_contourAis;
    Handle(AIS_Shape) m_previewLeadInAis;
    int m_previewContourIndex{-1};
    bool   m_visible{true};
    bool   m_showNormals{false};
    double m_normalSampleStep{2.0};
};

} // namespace lcnc::view
