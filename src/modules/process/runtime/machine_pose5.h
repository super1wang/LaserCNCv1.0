#pragma once

#include <QString>

#include <cstdint>

namespace lcnc::process {

/**
 * @brief 5 轴目标位姿。哪些轴参与本次运动由 mask 决定。
 *
 * 语义轴顺序固定：bit0=X, bit1=Y, bit2=Z, bit3=R1, bit4=R2。
 * 控制器索引（0..N）由 sink 内部的 AxisMap 映射。
 */
struct MachinePose5
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double r1{0.0};
    double r2{0.0};
    QString r1Name;
    QString r2Name;

    /// 默认参与 X+Y，编排层可显式覆盖：
    ///   pose.mask = MachinePose5::X | MachinePose5::Y | MachinePose5::Z;
    std::uint8_t mask{0x03};

    static constexpr std::uint8_t Bx  = 0x01;
    static constexpr std::uint8_t By  = 0x02;
    static constexpr std::uint8_t Bz  = 0x04;
    static constexpr std::uint8_t Br1 = 0x08;
    static constexpr std::uint8_t Br2 = 0x10;
};

} // namespace lcnc::process
