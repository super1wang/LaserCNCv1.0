#include "modules/process/device/runtime/device_wait.h"

#include <atomic>
#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

int main() {
    int polls = 0;
    const auto completed =
        lcnc::process::waitForDeviceCondition([&polls] { return ++polls >= 3; }, 100ms, 1ms);
    if (completed != lcnc::process::DeviceWaitStatus::Completed) {
        std::cerr << "completion condition was not observed\n";
        return 1;
    }

    const auto start = std::chrono::steady_clock::now();
    const auto timedOut = lcnc::process::waitForDeviceCondition([] { return false; }, 20ms, 2ms);
    const auto elapsed = std::chrono::steady_clock::now() - start;
    if (timedOut != lcnc::process::DeviceWaitStatus::TimedOut || elapsed > 250ms) {
        std::cerr << "timeout was not bounded\n";
        return 2;
    }

    std::atomic_bool cancelled{false};
    polls = 0;
    const auto cancelledResult = lcnc::process::waitForDeviceCondition([] { return false; },
                                                                       [&] {
                                                                           cancelled = ++polls >= 2;
                                                                           return cancelled.load();
                                                                       },
                                                                       100ms, 1ms);
    if (cancelledResult != lcnc::process::DeviceWaitStatus::Cancelled) {
        std::cerr << "cancellation was not observed\n";
        return 3;
    }

    return 0;
}
