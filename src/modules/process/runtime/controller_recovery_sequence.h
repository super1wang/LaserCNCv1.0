#pragma once

#include <algorithm>
#include <chrono>
#include <thread>
#include <utility>

namespace lcnc::process {

enum class ControllerRecoveryResult { StopFailed, ClearFailed, VerifyFailed, Recovered };
enum class ControllerHealthSample { Clean, FaultActive, ReadFailed };

// Give the controller a bounded opportunity to publish its post-clear state.
// Require two clean observations; a single transient clear is not recovery.
// Any unreadable sample fails immediately rather than being interpreted as 0.
template<class Sample, class Now, class Pause>
bool verifyRecoveredControllerHealth(Sample sample, Now now, Pause pause)
{
    const auto deadline = now() + std::chrono::milliseconds(250);
    int cleanSamples = 0;
    for (;;) {
        switch (sample()) {
        case ControllerHealthSample::ReadFailed: return false;
        case ControllerHealthSample::FaultActive: cleanSamples = 0; break;
        case ControllerHealthSample::Clean:
            if (++cleanSamples == 2) return true;
            break;
        }
        const auto remaining = deadline - now();
        if (remaining <= decltype(remaining)::zero()) return false;
        pause(std::min(std::chrono::milliseconds(10),
            std::chrono::duration_cast<std::chrono::milliseconds>(remaining)));
    }
}

template<class Sample>
bool verifyRecoveredControllerHealth(Sample sample)
{
    return verifyRecoveredControllerHealth(std::move(sample),
        [] { return std::chrono::steady_clock::now(); },
        [](std::chrono::milliseconds interval) { std::this_thread::sleep_for(interval); });
}

// This operation is invoked only by explicit Stop Reset, under the device
// lease. A successful clear request alone is not proof of a healthy device.
template<class StopAndRelease, class ClearFaults, class VerifyCurrent, class Commit>
ControllerRecoveryResult recoverStoppedController(
    StopAndRelease stopAndRelease, ClearFaults clearFaults,
    VerifyCurrent verifyCurrent, Commit commit)
{
    if (!stopAndRelease()) return ControllerRecoveryResult::StopFailed;
    if (!clearFaults()) return ControllerRecoveryResult::ClearFailed;
    if (!verifyCurrent()) return ControllerRecoveryResult::VerifyFailed;
    commit();
    return ControllerRecoveryResult::Recovered;
}

} // namespace lcnc::process
