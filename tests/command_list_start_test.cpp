#include "modules/process/runtime/command_list_start.h"

#include <iostream>

namespace {

struct Fixture {
    bool cancelled{false};
    bool cancelOnFinalize{false};
    bool cancelOnPrepare{false};
    bool cancelOnWait{false};
    bool cancelOnStart{false};
    bool retryAllowed{true};
    short finalizeCode{0};
    short startCode{0};
    int pendingCount{0};
    int finalizations{0};
    int preparations{0};
    int starts{0};
    int waits{0};

    lcnc::process::CommandListStartResult run()
    {
        return lcnc::process::startCancellableCommandList(
            [&] { return cancelled; },
            [&]() -> short {
                ++finalizations;
                if (cancelOnFinalize) cancelled = true;
                if (pendingCount-- > 0) return 10700;
                return finalizeCode;
            },
            [&] { ++preparations; if (cancelOnPrepare) cancelled = true; },
            [&] { ++starts; if (cancelOnStart) cancelled = true; return startCode; },
            [&] { return retryAllowed; },
            [&] { ++waits; if (cancelOnWait) cancelled = true; },
            10700);
    }
};

bool check(bool condition, const char* name)
{
    if (!condition) std::cerr << name << '\n';
    return condition;
}

} // namespace

int main()
{
    using State = lcnc::process::CommandListStartState;
    Fixture normal;
    if (!check(normal.run().state == State::Started && normal.starts == 1,
               "A valid batch must start once")) return 1;

    Fixture stopped;
    stopped.cancelled = true;
    if (!check(stopped.run().state == State::Cancelled
               && stopped.finalizations == 0 && stopped.starts == 0,
               "An already stopped batch must not call the SDK")) return 1;

    Fixture finalizeStop;
    finalizeStop.cancelOnFinalize = true;
    const auto finalizeStopResult = finalizeStop.run();
    if (!check(finalizeStopResult.state == State::Cancelled
               && finalizeStopResult.apiResult == 0 && finalizeStop.starts == 0,
               "Stop during successful DataEnd must prevent Start")) return 1;

    Fixture evidenceStop;
    evidenceStop.cancelOnPrepare = true;
    if (!check(evidenceStop.run().state == State::Cancelled && evidenceStop.starts == 0,
               "Stop during pre-start evidence reads must prevent Start")) return 1;

    Fixture retryStop;
    retryStop.pendingCount = 2;
    retryStop.cancelOnWait = true;
    if (!check(retryStop.run().state == State::Cancelled
               && retryStop.finalizations == 1 && retryStop.starts == 0,
               "Stop during pending DataEnd must prevent retry and Start")) return 1;

    Fixture retry;
    retry.pendingCount = 2;
    const auto retryResult = retry.run();
    if (!check(retryResult.state == State::Started && retryResult.finalizeAttempts == 3
               && retry.waits == 2 && retry.starts == 1,
               "Only pending DataEnd must be retried")) return 1;

    Fixture timedOut;
    timedOut.finalizeCode = 10700;
    timedOut.retryAllowed = false;
    if (!check(timedOut.run().state == State::FinalizeFailed && timedOut.starts == 0,
               "A DataEnd timeout must not start motion")) return 1;

    Fixture invalid;
    invalid.finalizeCode = 11700;
    const auto invalidResult = invalid.run();
    if (!check(invalidResult.state == State::FinalizeFailed && invalidResult.apiResult == 11700
               && invalid.waits == 0 && invalid.starts == 0,
               "A terminal command error must not retry or start")) return 1;

    Fixture invalidAndStopped;
    invalidAndStopped.finalizeCode = 11700;
    invalidAndStopped.cancelOnFinalize = true;
    const auto invalidStopResult = invalidAndStopped.run();
    if (!check(invalidStopResult.state == State::FinalizeFailed
               && invalidStopResult.apiResult == 11700 && invalidAndStopped.starts == 0,
               "Concurrent Stop must not erase an actual SDK validation error")) return 1;

    Fixture startFailure;
    startFailure.startCode = 1;
    const auto startResult = startFailure.run();
    if (!check(startResult.state == State::StartFailed && startResult.apiResult == 1,
               "The actual Start error must be preserved")) return 1;

    Fixture inFlightStop;
    inFlightStop.cancelOnStart = true;
    if (!check(inFlightStop.run().state == State::Started
               && inFlightStop.starts == 1 && inFlightStop.cancelled,
               "An in-flight Start must remain recorded and preserve the pending Stop")) return 1;
    return 0;
}
