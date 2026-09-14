#include "modules/process/runtime/controller_recovery_sequence.h"
#include "modules/process/runtime/gtn_axis_status_policy.h"

#include <vector>

using namespace lcnc::process;

int main()
{
    const auto checkReadback = [](const std::vector<ControllerHealthSample>& samples,
                                 bool expected, int expectedReads) {
        auto clock = std::chrono::steady_clock::time_point{};
        int reads = 0;
        const bool healthy = verifyRecoveredControllerHealth(
            [&] { return samples[std::min(static_cast<std::size_t>(reads++), samples.size() - 1)]; },
            [&] { return clock; },
            [&](std::chrono::milliseconds duration) { clock += duration; });
        return healthy == expected && reads == expectedReads;
    };
    using Sample = ControllerHealthSample;
    if (!checkReadback({Sample::Clean}, true, 2)
        || !checkReadback({Sample::FaultActive, Sample::Clean, Sample::Clean}, true, 3)
        || !checkReadback({Sample::Clean, Sample::FaultActive, Sample::Clean, Sample::Clean}, true, 4)
        || !checkReadback({Sample::Clean, Sample::ReadFailed, Sample::Clean}, false, 2)
        || !checkReadback({Sample::FaultActive}, false, 26)) return 6;
    // Both the latched raw status and current soft/hardware limit inputs block
    // machining. An enabled, fault-free axis is not blocked by its old history.
    if (gtnMachiningFault(0x220, false, false, true, false) != 0x20
        || gtnMachiningFault(0x240, false, false, false, true) != 0x40
        || gtnMachiningFault(0x220, false, false, false, false) != 0x20
        || gtnMachiningFault(0x200, true, false, false, false) != 0x20
        || gtnMachiningFault(0x200, false, false, true, false) != 0x20
        || gtnMachiningFault(0x202, false, false, false, false) != 0x2
        || gtnMachiningFault(0x200, false, false, false, false) != 0)
        return 1;
    for (int failingStage = 0; failingStage <= 3; ++failingStage) {
        std::vector<int> calls;
        bool historicalError = true;
        const auto result = recoverStoppedController(
            [&] { calls.push_back(0); return failingStage != 0; },
            [&] { calls.push_back(1); return failingStage != 1; },
            [&] { calls.push_back(2); return failingStage != 2; },
            [&] { calls.push_back(3); historicalError = false; });
        if (calls.size() != static_cast<std::size_t>(failingStage + 1)) return 2;
        for (int i = 0; i <= failingStage; ++i)
            if (calls[i] != i) return 3;
        if ((result == ControllerRecoveryResult::Recovered) != (failingStage == 3)
            || historicalError != (failingStage != 3)) return 4;
    }
    // A limit that remains/reappears after ClrSts must not commit recovery.
    for (bool physicalLimit : {false, true}) {
        long status = 0x220;
        bool historicalError = true;
        const auto result = recoverStoppedController(
            [] { return true; },
            [&] { status = 0x200; return true; },
            [&] { return gtnMachiningFault(status, false, false, physicalLimit, false) == 0; },
            [&] { historicalError = false; });
        if (historicalError != physicalLimit
            || (result == ControllerRecoveryResult::Recovered) == physicalLimit) return 5;
    }
    return 0;
}
