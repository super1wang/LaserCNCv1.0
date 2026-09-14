#pragma once

#include <utility>

namespace lcnc::process {

enum class DeviceShutdownFailure
{
    None,
    SafeStop,
    LaserDisconnect,
    MotionDisconnect
};

// Safety actions are ordered but independent: a failed laser command must not
// prevent a motion stop or another safe-output attempt.
// 中文翻译：安全动作按序且独立执行；激光关闭失败不能跳过运动停止或其他安全输出。
template<typename... Actions>
bool attemptAllSafetyActions(Actions&&... actions)
{
    bool success = true;
    ((success = std::forward<Actions>(actions)() && success), ...);
    return success;
}

// Disconnect only after the complete safe-stop transaction is confirmed. Keep
// motion communication available if the independent laser channel fails to close.
// 中文翻译：完整安全停止成功后才断开；激光通道断开失败时保留运动控制器通信。
template<typename Stop, typename LaserDisconnect, typename MotionDisconnect>
DeviceShutdownFailure runDeviceShutdownSequence(
    Stop&& stop, LaserDisconnect&& laserDisconnect, MotionDisconnect&& motionDisconnect)
{
    if (!std::forward<Stop>(stop)())
        return DeviceShutdownFailure::SafeStop;
    if (!std::forward<LaserDisconnect>(laserDisconnect)())
        return DeviceShutdownFailure::LaserDisconnect;
    if (!std::forward<MotionDisconnect>(motionDisconnect)())
        return DeviceShutdownFailure::MotionDisconnect;
    return DeviceShutdownFailure::None;
}

} // namespace lcnc::process
