#pragma once

#include <memory>
#include <typeindex>
#include <unordered_map>

#include "core/kernel/i_service.h"
#include "core/logging/logger.h"

namespace lcnc {

/**
 * @brief 按类型 (typeid(T)) 注册与查找服务实例的容器。
 *
 * 这是微内核中"服务定位器"的最简实现：
 *   - 注册：@ref registerService<T>(ptr) — 同一接口 T 重复注册会覆盖并打 WARN；
 *   - 查找：@ref getService<T>() — 无注册时返回空 @c shared_ptr ，调用方需做空检查；
 *   - 清空：@ref clear() — 由 Kernel 在 shutdown 反序释放服务时调用。
 *
 * 设计原则：
 *   - **不抛异常**：失败一律返回 nullptr / false，并写日志；
 *   - **不持有所有权语义之外的状态**：纯容器，没有依赖图、没有生命周期管理；
 *   - **线程安全**：当前实现假定注册阶段在主线程、查询阶段也以主线程为主；
 *     若未来需多线程查询再视情况引入读写锁。
 */
class ServiceRegistry
{
public:
    /**
     * @brief 注册一个服务实例，类型由模板参数 T 唯一索引。
     * @tparam T  必须是 @ref IService 的派生接口类型。
     * @param svc 共享指针；不可为空。
     * @return    true 表示首次注册；false 表示参数无效或为空指针。
     */
    template <class T>
    bool registerService(std::shared_ptr<T> svc);

    /**
     * @brief 取消注册类型 T 的服务。
     * @return true 表示存在并已移除。
     */
    template <class T>
    bool unregisterService();

    /**
     * @brief 取得类型 T 的服务实例。
     * @return 若未注册返回空 shared_ptr，调用方需自行空判。
     */
    template <class T>
    std::shared_ptr<T> getService() const;

    /**
     * @brief 判断类型 T 的服务是否已注册。
     */
    template <class T>
    bool has() const;

    /**
     * @brief 清空所有服务。@warning 需保证此时无任何模块仍持有这些服务。
     */
    void clear();

    /// 当前已注册服务数量（调试/检视用）。
    std::size_t size() const noexcept { return m_map.size(); }

private:
    std::unordered_map<std::type_index, std::shared_ptr<IService>> m_map;
};

// ─── 模板实现 ────────────────────────────────────────────────────────────────

template <class T>
bool ServiceRegistry::registerService(std::shared_ptr<T> svc)
{
    static_assert(std::is_base_of_v<IService, T>,
                  "ServiceRegistry::registerService<T>: T 必须继承自 lcnc::IService");

    if (!svc) {
        LCNC_WARN(LogCode::InternalUnexpectedState,
                  "ServiceRegistry: registerService<{}>() got null pointer",
                  typeid(T).name());
        return false;
    }

    const auto key = std::type_index(typeid(T));
    if (m_map.count(key) != 0) {
        LCNC_WARN(LogCode::InternalUnexpectedState,
                  "ServiceRegistry: service '{}' already registered, overwriting",
                  typeid(T).name());
    }

    m_map[key] = std::static_pointer_cast<IService>(svc);
    LCNC_DEBUG(LogCode::Generic,
               "ServiceRegistry: registered '{}' (count={})",
               typeid(T).name(), m_map.size());
    return true;
}

template <class T>
bool ServiceRegistry::unregisterService()
{
    const auto key = std::type_index(typeid(T));
    auto it = m_map.find(key);
    if (it == m_map.end()) {
        LCNC_DEBUG(LogCode::Generic,
                   "ServiceRegistry: unregisterService<{}>() not found",
                   typeid(T).name());
        return false;
    }
    m_map.erase(it);
    LCNC_DEBUG(LogCode::Generic,
               "ServiceRegistry: unregistered '{}' (count={})",
               typeid(T).name(), m_map.size());
    return true;
}

template <class T>
std::shared_ptr<T> ServiceRegistry::getService() const
{
    const auto key = std::type_index(typeid(T));
    auto it = m_map.find(key);
    if (it == m_map.end()) {
        LCNC_DEBUG(LogCode::Generic,
                   "ServiceRegistry: getService<{}>() returned null",
                   typeid(T).name());
        return {};
    }
    return std::static_pointer_cast<T>(it->second);
}

template <class T>
bool ServiceRegistry::has() const
{
    return m_map.count(std::type_index(typeid(T))) != 0;
}

} // namespace lcnc
