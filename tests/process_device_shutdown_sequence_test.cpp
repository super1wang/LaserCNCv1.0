#include "modules/process/runtime/device_shutdown_sequence.h"

#include <iostream>
#include <string>
#include <vector>

namespace {

int fail(const char* message)
{
    std::cerr << message << '\n';
    return 1;
}

} // namespace

int main()
{
    using lcnc::process::DeviceShutdownFailure;
    using lcnc::process::attemptAllSafetyActions;
    using lcnc::process::runDeviceShutdownSequence;

    // Every safety failure retains both communication channels, but must not
    // suppress any of the later laser/motion/output safety attempts.
    const std::vector<std::string> safetyOrder{
        "laser_stop", "aiming_stop", "motion_stop", "buffer_stop",
        "laser_do_off", "blow_do_off", "blow2_do_off"};
    for (int failingAction = 0; failingAction < 7; ++failingAction) {
        std::vector<std::string> calls;
        const auto attempt = [&](int action) {
            calls.push_back(safetyOrder[action]);
            return action != failingAction;
        };
        const auto failure = runDeviceShutdownSequence(
            [&] {
                return attemptAllSafetyActions(
                    [&] { return attempt(0); }, [&] { return attempt(1); },
                    [&] { return attempt(2); }, [&] { return attempt(3); },
                    [&] { return attempt(4); }, [&] { return attempt(5); },
                    [&] { return attempt(6); });
            },
            [&] { calls.push_back("laser_disconnect"); return true; },
            [&] { calls.push_back("motion_disconnect"); return true; });
        if (failure != DeviceShutdownFailure::SafeStop || calls != safetyOrder)
            return fail("A safety failure skipped an action or disconnected a device");
    }

    for (const auto expected : {DeviceShutdownFailure::None,
                               DeviceShutdownFailure::LaserDisconnect,
                               DeviceShutdownFailure::MotionDisconnect}) {
        std::vector<std::string> calls;
        const auto failure = runDeviceShutdownSequence(
            [&] { calls.push_back("safe_stop"); return true; },
            [&] {
                calls.push_back("laser_disconnect");
                return expected != DeviceShutdownFailure::LaserDisconnect;
            },
            [&] {
                calls.push_back("motion_disconnect");
                return expected != DeviceShutdownFailure::MotionDisconnect;
            });
        const std::vector<std::string> expectedCalls =
            expected == DeviceShutdownFailure::LaserDisconnect
            ? std::vector<std::string>{"safe_stop", "laser_disconnect"}
            : std::vector<std::string>{"safe_stop", "laser_disconnect", "motion_disconnect"};
        if (failure != expected || calls != expectedCalls)
            return fail("Disconnect failure or laser-first ownership order was lost");
    }

    // A failed safe-stop attempt is never silently retried by the shutdown
    // policy even if a second attempt would succeed.
    int stopAttempts = 0;
    int disconnectAttempts = 0;
    const auto failure = runDeviceShutdownSequence(
        [&] { return ++stopAttempts > 1; },
        [&] { ++disconnectAttempts; return true; },
        [&] { ++disconnectAttempts; return true; });
    if (failure != DeviceShutdownFailure::SafeStop
        || stopAttempts != 1 || disconnectAttempts != 0) {
        return fail("The initial stop failure was masked by an automatic retry");
    }
    return 0;
}
