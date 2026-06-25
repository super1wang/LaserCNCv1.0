#pragma once

/**
 * @file i_process_cutting_plan_provider.h
 * @brief Process 切割链表的"只读视图"接口（供 CAM 等外部模块消费）。
 *
 * 设计动机：CAM 模块的 TravelPathRenderer 需要知道"当前生效的切割顺序"
 * 才能画出虚线空程，但绝不能反向 include 任何 process 内部头文件以保持
 * 层依赖单向（process→cam）。本接口暴露最小子集（有序 contourId + revision），
 * 在 Kernel ServiceRegistry 中以共享指针形式注册。
 */

#include "core/kernel/i_service.h"
#include "modules/cam/contracts/cam_data_contracts.h"

#include <QVector>
#include <cstdint>

namespace lcnc::process {

class IProcessCuttingPlanProvider : public lcnc::IService
{
public:
    ~IProcessCuttingPlanProvider() override = default;

    /// 当前按 sortStrategy/manual 顺序排好、过滤了 enabled 的轮廓 ID 列表。
    /// 顺序即为加工顺序；空表示无可加工轮廓。
    virtual QVector<lcnc::cam::ContourId> orderedContourIds() const = 0;

    /// 一个单调递增的版本号，发生任何变化时增加；调用方可缓存上一次值做判脏。
    virtual std::uint64_t planRevision() const = 0;
};

} // namespace lcnc::process
