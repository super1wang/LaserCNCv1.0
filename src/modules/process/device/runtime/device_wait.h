#pragma once

#include <algorithm>
#include <chrono>
#include <thread>
#include <utility>

namespace lcnc::process {

enum class DeviceWaitStatus {
    Completed,
    TimedOut,
    Cancelled,
};

/**
 * Wait for a device condition without permitting an unbounded SDK polling loop.
 * Predicates are evaluated on the calling device thread; no ownership crosses
 * the call boundary.
 */
template <class CompletedPredicate, class CancelledPredicate>
DeviceWaitStatus
waitForDeviceCondition(CompletedPredicate&& completed, CancelledPredicate&& cancelled,
                       std::chrono::milliseconds timeout, std::chrono::milliseconds pollInterval) {
    using Clock = std::chrono::steady_clock;
    if (completed())
        return DeviceWaitStatus::Completed;
    if (cancelled())
        return DeviceWaitStatus::Cancelled;
    if (timeout <= std::chrono::milliseconds::zero())
        return DeviceWaitStatus::TimedOut;

    const auto deadline = Clock::now() + timeout;
    const auto interval = std::max(pollInterval, std::chrono::milliseconds(1));
    for (;;) {
        const auto now = Clock::now();
        if (now >= deadline)
            return DeviceWaitStatus::TimedOut;
        std::this_thread::sleep_for(std::min(
            interval, std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now)));
        if (completed())
            return DeviceWaitStatus::Completed;
        if (cancelled())
            return DeviceWaitStatus::Cancelled;
    }
}

template <class CompletedPredicate>
DeviceWaitStatus waitForDeviceCondition(CompletedPredicate&& completed,
                                        std::chrono::milliseconds timeout,
                                        std::chrono::milliseconds pollInterval) {
    return waitForDeviceCondition(
        std::forward<CompletedPredicate>(completed), [] { return false; }, timeout, pollInterval);
}

} // namespace lcnc::process
