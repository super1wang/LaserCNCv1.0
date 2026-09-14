#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <typeindex>
#include <unordered_map>
#include <vector>

#include "core/logging/logger.h"

namespace lcnc {

/**
 * @brief 订阅句柄，用于稍后取消订阅。
 *
 * @c kInvalidSubscription 表示无效；@ref EventBus::unsubscribe 应在销毁
 * 时由订阅方主动调用，否则有悬挂回调风险。
 */
using SubscriptionId = std::uint64_t;
constexpr SubscriptionId kInvalidSubscription = 0;

/**
 * @brief 类型化发布/订阅总线。
 *
 * 用法：
 * @code
 *   struct MachineLoadedEvent { QString path; };
 *
 *   auto id = bus.subscribe<MachineLoadedEvent>(
 *       [](const MachineLoadedEvent& e){ ... });
 *
 *   bus.publish(MachineLoadedEvent{ "machine.step" });
 *   bus.unsubscribe(id);
 * @endcode
 *
 * 设计要点：
 *   - 事件类型 E 应为**轻量值类型**（拷贝 / move 廉价）；
 *   - 回调在 @ref publish 调用线程同步执行；UI 模块如需切回 GUI 线程，
 *     请在订阅者内部用 @c QMetaObject::invokeMethod(QueuedConnection) ；
 *   - 订阅期间持有的 lambda 捕获对象生命周期由订阅方保证。
 *
 * 线程安全：注册/发布/取消都加同一把互斥锁。回调期间锁已释放。
 */
class EventBus
{
public:
    template <class E>
    using Handler = std::function<void(const E&)>;

    /**
     * @brief 订阅类型 E 的事件，返回订阅句柄。
     * @return @c kInvalidSubscription 表示 handler 为空。
     */
    template <class E>
    SubscriptionId subscribe(Handler<E> handler);

    /**
     * @brief 发布事件 e 给所有订阅者，按订阅顺序同步回调。
     */
    template <class E>
    void publish(const E& e);

    /**
     * @brief 取消订阅。无效或已取消的句柄被静默忽略。
     */
    void unsubscribe(SubscriptionId id);

    /// 当前总订阅条目数（调试/检视用）。
    std::size_t subscriberCount() const;

private:
    struct EntryBase {
        SubscriptionId id = kInvalidSubscription;
    };

    template <class E>
    struct Entry : EntryBase {
        Handler<E> fn;
    };

    using EntryPtr   = std::shared_ptr<EntryBase>;
    using BucketList = std::vector<EntryPtr>;

    mutable std::mutex                              m_mutex;
    std::unordered_map<std::type_index, BucketList> m_buckets;
    std::atomic<SubscriptionId>                     m_nextId{1};
};

// ─── 模板实现 ────────────────────────────────────────────────────────────────

template <class E>
SubscriptionId EventBus::subscribe(Handler<E> handler)
{
    if (!handler) {
        LCNC_WARN(LogCode::InternalUnexpectedState,
                  "EventBus::subscribe<{}>() got empty handler",
                  typeid(E).name());
        return kInvalidSubscription;
    }

    auto entry = std::make_shared<Entry<E>>();
    entry->id  = m_nextId.fetch_add(1, std::memory_order_relaxed);
    entry->fn  = std::move(handler);

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_buckets[std::type_index(typeid(E))].push_back(entry);
    }

    LCNC_DEBUG(LogCode::Generic,
               "EventBus: subscribed id={} type='{}'",
               entry->id, typeid(E).name());
    return entry->id;
}

template <class E>
void EventBus::publish(const E& e)
{
    BucketList snapshot;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_buckets.find(std::type_index(typeid(E)));
        if (it == m_buckets.end() || it->second.empty()) {
            LCNC_DEBUG(LogCode::Generic,
                       "EventBus: publish '{}' had no subscribers",
                       typeid(E).name());
            return;
        }
        // 复制一份快照，避免回调中订阅/取消导致的迭代器失效
        snapshot = it->second;
    }

    LCNC_DEBUG(LogCode::Generic,
               "EventBus: publish '{}' to {} subscriber(s)",
               typeid(E).name(), snapshot.size());

    for (const auto& base : snapshot) {
        auto* typed = static_cast<Entry<E>*>(base.get());
        if (typed && typed->fn) {
            try {
                typed->fn(e);
            } catch (const std::exception& ex) {
                LCNC_ERR(LogCode::InternalUnexpectedState,
                         "EventBus: subscriber id={} threw on '{}': {}",
                         typed->id, typeid(E).name(), ex.what());
            } catch (...) {
                LCNC_ERR(LogCode::InternalUnexpectedState,
                         "EventBus: subscriber id={} threw unknown on '{}'",
                         typed->id, typeid(E).name());
            }
        }
    }
}

} // namespace lcnc
