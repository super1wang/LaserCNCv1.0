#pragma once

#include <memory>
#include <vector>

#include <QString>
#include <QStringList>

#include "core/kernel/i_module.h"

namespace lcnc {

class IKernel;

/**
 * @brief 模块容器 + 启动调度器。
 *
 * 职责：
 *   - 收集 @ref IModule 实例（@c addModule）；
 *   - 按 @ref ModuleInfo::dependencies 做拓扑排序；
 *   - 顺序调用 @c init → @c start ；
 *   - 反向调用 @c stop 并释放。
 *
 * 失败处理：
 *   - 任一模块 @c init 返回 false ：已 init 的模块按反向调用 @c stop ，
 *     整体启动失败；
 *   - @c start 失败：仅停掉已 start 模块；保留 init 状态以便诊断；
 *   - 检测到循环依赖或缺失依赖时打 ERR 日志并失败。
 */
class ModuleRegistry
{
public:
    /// 增加模块；实际所有权由本注册表持有。
    void addModule(std::unique_ptr<IModule> module);

    /**
     * @brief 启动所有模块（拓扑排序后逐个 init + start）。
     * @return true 表示全部成功。
     */
    bool startAll(IKernel& kernel);

    /**
     * @brief 反向停止所有模块（与启动相反顺序）。
     */
    void stopAll();

    /// 已注册模块数。
    std::size_t size() const noexcept { return m_modules.size(); }

    /// 按 id 查找；不存在返回 nullptr。
    IModule* find(const QString& id) const;

private:
    /// 内部：把 m_modules 拓扑排序写到 m_startupOrder；失败返回 false。
    bool computeStartupOrder();

    std::vector<std::unique_ptr<IModule>> m_modules;
    std::vector<IModule*>                 m_startupOrder;  ///< 已拓扑排序
    std::vector<IModule*>                 m_started;       ///< 成功启动的，反向 stop 用
    bool                                  m_initialized = false;
};

} // namespace lcnc
