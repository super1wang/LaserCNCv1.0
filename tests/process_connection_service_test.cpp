#include "modules/process/runtime/process_connection_service.h"

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QThread>

#include <atomic>
#include <iostream>
#include <stdexcept>

namespace {

bool spinUntil(const std::function<bool()>& predicate, int timeoutMs = 2000)
{
    QElapsedTimer timer;
    timer.start();
    while (!predicate() && timer.elapsed() < timeoutMs) {
        QCoreApplication::processEvents();
        QThread::msleep(1);
    }
    return predicate();
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    lcnc::process::DeviceCommandQueue queue;
    if (!queue.start())
        return 1;

    bool connected = false;
    bool disconnected = false;
    lcnc::process::ProcessConnectionService service(
        queue,
        [&connected](bool pureSimulation, lcnc::process::ProcessConnectionService::Progress progress) {
            if (pureSimulation)
                return lcnc::process::DeviceCommandResult{false, QStringLiteral("Unexpected simulation mode")};
            progress(50, QStringLiteral("connecting"));
            connected = true;
            return lcnc::process::DeviceCommandResult{};
        },
        [&disconnected] {
            disconnected = true;
            return lcnc::process::DeviceCommandResult{};
        });

    int progress = 0;
    bool connectCompleted = false;
    bool initialConnectSucceeded = false;
    const auto connectTicket = service.connect(false,
        [&progress](int percent, const QString&) { progress = percent; },
        [&connectCompleted, &initialConnectSucceeded](const lcnc::process::DeviceCommandResult& result) {
            initialConnectSucceeded = result.success;
            connectCompleted = true;
        });
    if (!connectTicket.accepted || !spinUntil([&] { return connectCompleted; })
        || !initialConnectSucceeded || !connected || progress != 50)
        return 1;

    bool disconnectCompleted = false;
    bool initialDisconnectSucceeded = false;
    const auto disconnectTicket = service.disconnect(
        [&disconnectCompleted, &initialDisconnectSucceeded](const lcnc::process::DeviceCommandResult& result) {
            initialDisconnectSucceeded = result.success;
            disconnectCompleted = true;
        });
    if (!disconnectTicket.accepted || !spinUntil([&] { return disconnectCompleted; })
        || !initialDisconnectSucceeded || !disconnected)
        return 1;
    // A failed operation does not imply the controller session closed. The
    // probe is executed on the device worker, including after exceptions.
    std::atomic<bool> connectionOpen{true};
    std::atomic<int> scenario{0};
    std::atomic<bool> probeOnWorker{true};
    lcnc::process::ProcessConnectionService probedService(
        queue,
        [&](bool, lcnc::process::ProcessConnectionService::Progress) {
            if (scenario.load() == 3)
                throw std::runtime_error("connect runner failure");
            return lcnc::process::DeviceCommandResult{false, QStringLiteral("connect failed but retained")};
        },
        [&] {
            if (scenario.load() == 4)
                throw std::runtime_error("disconnect runner failure");
            if (scenario.load() == 1)
                return lcnc::process::DeviceCommandResult{false, QStringLiteral("stop failed but retained")};
            connectionOpen.store(false);
            return lcnc::process::DeviceCommandResult{};
        }, nullptr,
        [&] {
            probeOnWorker.store(probeOnWorker.load() && queue.isWorkerThread());
            if (scenario.load() == 5)
                throw std::runtime_error("probe failed");
            return connectionOpen.load();
        });
    if (probedService.lastMotionConnectionOpen().has_value()) {
        std::cerr << "Connection state should be unknown before the first worker probe\n";
        return 1;
    }
    for (int testCase = 0; testCase < 6; ++testCase) {
        scenario.store(testCase);
        connectionOpen.store(true);
        bool completed = false;
        lcnc::process::DeviceCommandResult result;
        const auto completion = [&](const lcnc::process::DeviceCommandResult& value) {
            result = value;
            completed = true;
        };
        const auto ticket = (testCase == 0 || testCase == 3)
            ? probedService.connect(false, {}, completion)
            : probedService.disconnect(completion);
        if (!ticket.accepted || !spinUntil([&] { return completed; })) {
            std::cerr << "Connection probe scenario did not complete\n";
            return 1;
        }
        const auto state = probedService.lastMotionConnectionOpen();
        const bool expectedSuccess = testCase == 2;
        const bool expectedOpen = testCase == 0 || testCase == 1
            || testCase == 3 || testCase == 4;
        if (result.success != expectedSuccess || !probeOnWorker.load()
            || (testCase == 5 ? state.has_value()
                              : (!state.has_value() || *state != expectedOpen))) {
            std::cerr << "A connection failure was masked or its retained session state was lost\n";
            return 1;
        }
    }
    if (!queue.shutdown(2000))
        return 1;
    return 0;
}
