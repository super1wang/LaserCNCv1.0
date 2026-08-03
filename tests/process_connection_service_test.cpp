#include "modules/process/runtime/process_connection_service.h"

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QThread>

#include <cassert>

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
    assert(queue.start());

    bool connected = false;
    bool disconnected = false;
    lcnc::process::ProcessConnectionService service(
        queue,
        [&connected](bool pureSimulation, lcnc::process::ProcessConnectionService::Progress progress) {
            assert(!pureSimulation);
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
    const auto connectTicket = service.connect(false,
        [&progress](int percent, const QString&) { progress = percent; },
        [&connectCompleted](const lcnc::process::DeviceCommandResult& result) {
            assert(result.success);
            connectCompleted = true;
        });
    assert(connectTicket.accepted);
    assert(spinUntil([&] { return connectCompleted; }));
    assert(connected && progress == 50);

    bool disconnectCompleted = false;
    const auto disconnectTicket = service.disconnect(
        [&disconnectCompleted](const lcnc::process::DeviceCommandResult& result) {
            assert(result.success);
            disconnectCompleted = true;
        });
    assert(disconnectTicket.accepted);
    assert(spinUntil([&] { return disconnectCompleted; }));
    assert(disconnected);
    assert(queue.shutdown(2000));
    return 0;
}
