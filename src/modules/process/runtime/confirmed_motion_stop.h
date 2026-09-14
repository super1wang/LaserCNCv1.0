#pragma once

#include <algorithm>
#include <chrono>
#include <thread>
#include <utility>

namespace lcnc::process {

enum class MotionStopObservation { Moving, Stopped, ReadFailed };
enum class MotionStopConfirmation { Confirmed, ReadFailed, TimedOut };

// A failed status read is terminal and never means stopped. Clock and pause
// are injectable so timeout/recovery behavior can be verified without an SDK.
template <class Sample, class Now, class Pause>
MotionStopConfirmation waitForConfirmedMotionStop(
    Sample&& sample, std::chrono::milliseconds timeout,
    std::chrono::milliseconds pollInterval, Now&& now, Pause&& pause)
{
    const auto deadline = now() + std::max(timeout, std::chrono::milliseconds::zero());
    const auto interval = std::max(pollInterval, std::chrono::milliseconds(1));
    for (;;) {
        switch (sample()) {
        case MotionStopObservation::Stopped:
            return MotionStopConfirmation::Confirmed;
        case MotionStopObservation::ReadFailed:
            return MotionStopConfirmation::ReadFailed;
        case MotionStopObservation::Moving:
            break;
        }
        const auto remaining = deadline - now();
        if (remaining <= decltype(remaining)::zero())
            return MotionStopConfirmation::TimedOut;
        pause(std::min(interval, std::chrono::duration_cast<std::chrono::milliseconds>(remaining)));
    }
}

template <class Sample>
MotionStopConfirmation waitForConfirmedMotionStop(
    Sample&& sample, std::chrono::milliseconds timeout,
    std::chrono::milliseconds pollInterval)
{
    return waitForConfirmedMotionStop(std::forward<Sample>(sample), timeout, pollInterval,
        [] { return std::chrono::steady_clock::now(); },
        [](std::chrono::milliseconds interval) { std::this_thread::sleep_for(interval); });
}

} // namespace lcnc::process
