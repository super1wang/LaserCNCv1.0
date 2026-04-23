#include "core/kernel/event_bus.h"

namespace lcnc {

void EventBus::unsubscribe(SubscriptionId id)
{
    if (id == kInvalidSubscription) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    for (auto& [type, bucket] : m_buckets) {
        auto it = std::find_if(bucket.begin(), bucket.end(),
                               [id](const EntryPtr& e) { return e && e->id == id; });
        if (it != bucket.end()) {
            bucket.erase(it);
            LCNC_DEBUG(LogCode::Generic,
                       "EventBus: unsubscribed id={} from type='{}'",
                       id, type.name());
            return;
        }
    }
    LCNC_DEBUG(LogCode::Generic, "EventBus: unsubscribe id={} not found", id);
}

std::size_t EventBus::subscriberCount() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::size_t total = 0;
    for (const auto& [type, bucket] : m_buckets) {
        (void)type;
        total += bucket.size();
    }
    return total;
}

} // namespace lcnc
