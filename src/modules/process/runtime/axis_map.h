#pragma once

#include <QString>
#include <QVector>
#include <array>
#include <cstdint>

namespace lcnc { class MachineConfigurationService; }

namespace lcnc::process {

/**
 * @brief 语义轴（X/Y/Z/R1/R2）到控制器物理轴索引的映射。
 *
 * 5 轴坐标系下：[0]=X→idx, [1]=Y→idx, [2]=Z→idx, [3]=R1(A)→idx, [4]=R2(B/C)→idx。
 * idx < 0 表示该语义轴在当前构型不存在（典型：3 轴机型 R1/R2 缺位）。
 *
 * AxisMap 同时携带每轴的默认速度/加减速/Jerk，用于工具参数无效时回退（与 Fix #2 同源，
 * 但来自构型而非每次重读 TOML）。
 */
class AxisMap
{
public:
    enum SemanticAxis { X = 0, Y, Z, R1, R2, Count };

    struct PerAxis
    {
        QString name;
        int    controllerIndex{-1};
        double velocity{0.0};
        double acceleration{0.0};
        double jerk{0.0};
    };

    /// 从 MachineConfigurationService 构造；缺失轴的 controllerIndex 留 -1。
    /// machineConfig==nullptr 时返回一个保守默认（X=0,Y=1,Z=2,R1=3,R2=4）。
    static AxisMap from(lcnc::MachineConfigurationService* machineConfig);

    /// 是否 5 轴有效（X/Y/Z/R1/R2 全部存在）。
    bool isFiveAxis() const;
    /// 当前激活的语义轴数（>=2 才能切割）。
    int activeCount() const;

    /// 查询。
    int  controllerIndex(SemanticAxis a) const { return m_axes[a].controllerIndex; }
    QString axisName(SemanticAxis a) const      { return m_axes[a].name; }
    const PerAxis& axis(SemanticAxis a) const  { return m_axes[a]; }
    bool isPresent(SemanticAxis a) const       { return m_axes[a].controllerIndex >= 0; }

    /// 按 mask 把语义轴展开为 "(0, 1, 2)" 这样的 ACS 轴元组。
    QString axisTupleText(std::uint8_t mask) const;
    // 中文翻译：控制器索引
    /// 按 mask 输出语义轴对应的"Controller index"列表。
    QVector<int> activeControllerIndices(std::uint8_t mask) const;

private:
    std::array<PerAxis, Count> m_axes{};
};

} // namespace lcnc::process
