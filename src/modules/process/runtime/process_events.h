#pragma once

/**
 * @file process_events.h
 * @brief Process 模块向 EventBus 广播的事件 POD（OCC-free，CAM/UI 可订阅）。
 */

#include <cstdint>

namespace lcnc::process::events {

/// 切割链表内容发生变化（增删图层/工具映射、手动顺序、排序策略、同步 CAM）。
/// 订阅方典型动作：从 IProcessCuttingPlanProvider 重新拉数据并 redraw。
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
