#pragma once

#include <QList>
#include <AIS_Shape.hxx>
#include <gp_Pnt.hxx>

class GuiDocument;
class LaserToolpath;

namespace lcnc::view {

/**
 * @brief 刀路 AIS 渲染器（v2.2 从 CamModule 抽出）。
 *
 * 拥有所有刀路相关 AIS 对象（轮廓 / 引入线 / 法线）以及可见性 / 法线显示
 * 状态。CamModule 持有 unique_ptr 并在需要时传入 GuiDocument 调用 refresh()。
 *
 * 算法上零业务依赖：仅根据传入的 LaserToolpath + 参数 + 引入线预览生成 AIS。
 */
class ToolpathRenderer
{
public:
    /// 引入线预览（与 picking 服务共享的瞬态状态，由 CamModule 在调用前填好）。
    struct LeadInPreview {
        int    contourIndex{-1};
        gp_Pnt entryPoint;
        double entryParam{0.0};
        bool   valid{false};
    };

    ToolpathRenderer();
    ~ToolpathRenderer();

    /// 擦除并按当前 visible/normal 标志重绘所有刀路 AIS。
    void refresh(GuiDocument* gd,
                 const LaserToolpath& toolpath,
                 const LeadInPreview& preview = {});

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

private:
    void displayContours(GuiDocument* gd, const LaserToolpath& tp);
    void displayLeadIns(GuiDocument* gd, const LaserToolpath& tp, const LeadInPreview& preview);
    void displayNormals(GuiDocument* gd, const LaserToolpath& tp);

    QList<Handle(AIS_Shape)> m_contourAis;
    QList<Handle(AIS_Shape)> m_leadInAis;
    QList<Handle(AIS_Shape)> m_normalAis;
    bool   m_visible{true};
    bool   m_showNormals{false};
    double m_normalSampleStep{2.0};
};

} // namespace lcnc::view
