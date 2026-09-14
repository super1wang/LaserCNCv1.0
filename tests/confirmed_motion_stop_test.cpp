#include "modules/process/runtime/confirmed_motion_stop.h"

#include <chrono>
#include <iostream>
#include <vector>

using namespace std::chrono_literals;
using lcnc::process::MotionStopConfirmation;
using lcnc::process::MotionStopObservation;

int main()
{
    auto verify = [](const std::vector<MotionStopObservation>& samples,
                     MotionStopConfirmation expected, int expectedSamples,
                     std::chrono::milliseconds expectedTime) {
        auto time = std::chrono::steady_clock::time_point{};
        int reads = 0;
        bool released = false;
        const auto result = lcnc::process::waitForConfirmedMotionStop(
            [&] { return samples[std::min(static_cast<std::size_t>(reads++), samples.size() - 1)]; },
            30ms, 10ms, [&] { return time; },
            [&](std::chrono::milliseconds interval) { time += interval; });
        if (result == MotionStopConfirmation::Confirmed)
            released = true;
        return result == expected && reads == expectedSamples
            && time.time_since_epoch() == expectedTime
            && released == (expected == MotionStopConfirmation::Confirmed);
    };

    if (!verify({MotionStopObservation::Stopped}, MotionStopConfirmation::Confirmed, 1, 0ms)
        || !verify({MotionStopObservation::Moving, MotionStopObservation::Moving,
                    MotionStopObservation::Stopped}, MotionStopConfirmation::Confirmed, 3, 20ms)
        || !verify({MotionStopObservation::ReadFailed, MotionStopObservation::Stopped},
                   MotionStopConfirmation::ReadFailed, 1, 0ms)
        || !verify({MotionStopObservation::Moving, MotionStopObservation::ReadFailed,
                    MotionStopObservation::Stopped}, MotionStopConfirmation::ReadFailed, 2, 10ms)
        || !verify({MotionStopObservation::Moving}, MotionStopConfirmation::TimedOut, 4, 30ms)) {
        std::cerr << "stop confirmation must retain ownership on read failure or timeout\n";
        return 1;
    }
    return 0;
}
