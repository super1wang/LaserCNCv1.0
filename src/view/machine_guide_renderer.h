#pragma once

#include <QMap>
#include <QString>
#include <AIS_Shape.hxx>
#include <gp_Pnt.hxx>

class GuiDocument;
class MachineKinematics;

namespace lcnc::view {

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

    /// 重新创建所有引导 AIS（先 erase 再 display）。
    void refresh(GuiDocument* gd,
                 MachineKinematics* kin,
                 const gp_Pnt& cutterHeadModelPos);

    /// 擦除所有引导 AIS。
    void erase(GuiDocument* gd);

    /// 仅刷新已存在 AIS 的局部变换（轴角/刀头位置变化时调用）。
    void updateTransforms(GuiDocument* gd, MachineKinematics* kin);

private:
    QMap<QString, Handle(AIS_Shape)> m_axisGuideAis;
};

} // namespace lcnc::view
