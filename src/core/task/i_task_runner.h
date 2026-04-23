#pragma once

#include <QString>
#include <functional>

#include "core/kernel/i_service.h"
#include "core/task/task_progress.h"

class TaskManager;

namespace lcnc {

/**
 * @brief 异步任务调度服务接口。
 *
 * 包装现有 @ref TaskManager 单例，提供面向接口的访问方式。
 * 设计动机：让 AsyncCommandBase 等用户依赖 ITaskRunner 而非具体
 * TaskManager，便于测试中注入同步 stub（直接在当前线程跑回调）。
 */
class ITaskRunner : public IService
{
public:
    using TaskJob = std::function<void(TaskProgress*)>;

    /**
     * @brief 提交一个异步任务，返回任务 id（无效时返回负值）。
     * @param label  UI 进度面板显示的中文标题。
     * @param job    实际作业；通过 @c TaskProgress* 上报进度并响应 abort。
     */
    virtual int run(const QString& label, TaskJob job) = 0;

    /// 请求协作中止；任务必须自检 @c progress->isAbortRequested()。
    virtual void requestAbort(int taskId) = 0;
};

} // namespace lcnc
