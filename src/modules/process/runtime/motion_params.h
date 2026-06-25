#pragma once

namespace lcnc::process {

/**
 * @brief 单段运动的速度/加减速/Jerk/拐角参数。
 *
 * 来源：从 Tool 反序列化（Tool::SetFromTable），无效时由 MachineConfigurationService
 * 的每轴默认填充（见 MotionSinkFactory）。
 */
struct MotionParams
{
    double feed{0.0};            // 段速度
    double acc{0.0};             // 加速度
    double dec{0.0};             // 减速度
    double jerk{0.0};            // 加加速度

    // ACS XSEG 专用拐角参数
    double junctionVel{0.0};     // 拐角速度
    double junctionAngle{0.0};   // 拐角角度（度）
    double xsegEndVel{0.0};      // XSEG 末速度
};

} // namespace lcnc::process
