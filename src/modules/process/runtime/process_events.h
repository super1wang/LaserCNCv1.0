#pragma once

/**
 * @file process_events.h
 * @brief Process 模块向 EventBus 广播的事件 POD（OCC-free，CAM/UI 可订阅）。
 */

#include <cstdint>

namespace lcnc::process::events {

/// 切割链表内容发生变化（图层工具映射或 CAM 已确认顺序发生变化）。
/// 订阅方典型动作：从 CAM contour sequence 和 Process 工艺绑定重新拉数据。
struct CuttingPlanChanged
{
    std::uint64_t revision{0};
};

/// 「切割路径显示」开关被切换。
struct TravelPathVisibilityToggled
{
    bool visible{false};
};

/// 「切割链表序号显示」开关被切换。
struct ContourOrderLabelVisibilityToggled
{
    bool visible{false};
};

} // namespace lcnc::process::events
