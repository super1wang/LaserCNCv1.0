#pragma once

#include <QHash>
#include <QMap>
#include <QString>
#include <AIS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <TopoDS_Shape.hxx>
#include <QColor>

class GuiDocument;
class MachineKinematics;

namespace lcnc::view {

/**
 * @brief 刀头锥外观参数（颜色 / 透明度 / 缩放）。
 * 由 CamModule 从 AppSettings::ColorSettings 注入，渲染器自身不读设置。
 */
struct CutterHeadAppearance {
    QColor color{255, 0, 0};
    double transparency = 0.0; ///< 0.0 = opaque, 1.0 = fully transparent
    double scale = 1.0;         ///< 圆锥整体缩放（1.0 = 默认尺寸）
};

/**
 * @brief 机台坐标轴/刀头辅助 AIS 渲染器（v2.2 从 CamModule 抽出）。
 *
 * 仅负责 A/C 旋转轴指示线 + 刀头线 + 锥形指示器的显示与变换更新。
 */
class MachineGuideRenderer
{
public:
    MachineGuideRenderer();
    ~MachineGuideRenderer();

    /// 设置刀头锥外观（颜色/透明度/缩放），在下一次 refresh() 生效。
    void setCutterHeadAppearance(const CutterHeadAppearance& appearance);
    /// Set a presentation-only local +Z nozzle/cone. A null shape keeps the
    /// legacy visual cone fallback. It is never collision geometry.
    void setCutterDisplayProxy(const TopoDS_Shape& shape);

    /// 重新创建所有引导 AIS（先 erase 再 display）。
    void refresh(GuiDocument* gd,
                 MachineKinematics* kin,
                 const gp_Pnt& cutterHeadWorldTip);

    /// 擦除所有引导 AIS。
    void erase(GuiDocument* gd);

    /// 仅刷新已存在 AIS 的局部变换（轴角/刀头位置变化时调用）。
    void updateTransforms(GuiDocument* gd,
                          MachineKinematics* kin,
                          const gp_Pnt& cutterHeadWorldTip);

    void setRotaryAxisVisible(GuiDocument* gd, bool visible);
    void setCutterHeadVisible(GuiDocument* gd, bool visible);
    bool rotaryAxisVisible() const { return m_rotaryAxisVisible; }
    bool cutterHeadVisible() const { return m_cutterHeadVisible; }

private:
    void applyVisibility(GuiDocument* gd);
    gp_Pnt rotationCenter(MachineKinematics* kin) const;
    QMap<QString, Handle(AIS_Shape)>& guideMap(GuiDocument* gd);
    const QMap<QString, Handle(AIS_Shape)>* guideMap(GuiDocument* gd) const;

private:
    QHash<GuiDocument*, QMap<QString, Handle(AIS_Shape)>> m_axisGuideAisByDocument;
    CutterHeadAppearance m_cutterHeadAppearance;
    TopoDS_Shape m_cutterDisplayProxy;
    bool m_rotaryAxisVisible{true};
    bool m_cutterHeadVisible{true};
};

} // namespace lcnc::view
