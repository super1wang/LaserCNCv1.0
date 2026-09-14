#pragma once

#include <mutex>

namespace lcnc::process {

/**
 * @brief Process runtime 的唯一设备访问租约。
 *
 * ACS、GTN 和激光/IO SDK 都不能假定可被多个 Qt worker 并发调用。所有
 * 设备读取、写入、连接和断开均须持有本租约；递归属性允许安全停机路径在
 * 已持有租约的流程调用栈中再次关闭输出。
 *
 * 该对象只负责串行化，不拥有供应商对象。供应商对象仍由 Process 的
 * ProcessDeviceRuntime 生命周期管理，因此不会在后台读取期间被提前销毁。
 */
class ProcessDeviceCoordinator final
{
public:
    using Lease = std::unique_lock<std::recursive_mutex>;

    Lease acquire() { return Lease(m_mutex); }

private:
    std::recursive_mutex m_mutex;
};

} // namespace lcnc::process
