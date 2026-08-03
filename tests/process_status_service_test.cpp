#include "modules/process/runtime/process_status_service.h"

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QSemaphore>
#include <QThread>

#include <atomic>
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

    std::atomic_int hardwareCalls{0};
    std::atomic_int peripheralCalls{0};
    lcnc::process::ProcessStatusService service(
        queue,
        [&hardwareCalls](const QStringList& axes, const QVector<QPair<QString, QString>>&) {
            ++hardwareCalls;
            lcnc::process::DeviceStatusSnapshot snapshot;
            snapshot.connected = true;
            for (const QString& axis : axes)
                snapshot.axes.push_back({axis, 7.5, true, true});
            return snapshot;
        },
        [&peripheralCalls] {
            ++peripheralCalls;
            return lcnc::process::DevicePeripheralSnapshot{
                QStringLiteral("test"), true, true, {}, true};
        });
    service.setRequestProvider([] {
        lcnc::process::ProcessStatusRequest request;
        request.connected = true;
        request.simulationMode = false;
        request.axisNames = {QStringLiteral("X")};
        return request;
    });

    int hardwareDelivered = 0;
    int peripheralDelivered = 0;
    bool safetyActive = false;
    service.setHardwareHandler([&](const lcnc::process::DeviceCommandResult& result,
                                   const lcnc::process::DeviceStatusSnapshot& snapshot) {
        assert(result.success);
        assert(snapshot.connected && snapshot.axes.size() == 1 && snapshot.axes.front().valid);
        ++hardwareDelivered;
    });
    service.setPeripheralHandler([&](const lcnc::process::DeviceCommandResult& result,
                                     const lcnc::process::DevicePeripheralSnapshot& snapshot) {
        assert(result.success && snapshot.valid);
        ++peripheralDelivered;
    });
    service.setSafetyMonitoringHandler([&](bool active) { safetyActive = active; });

    service.start();
    service.start();
    assert(safetyActive);
    assert(spinUntil([&] { return hardwareDelivered == 1 && peripheralDelivered == 1; }));
    assert(hardwareCalls == 1 && peripheralCalls == 1);

    service.stop();
    service.stop();
    assert(!safetyActive);
    service.requestHardwarePoll();
    service.requestPeripheralPoll();
    QCoreApplication::processEvents();
    assert(hardwareCalls == 1 && peripheralCalls == 1);

    QSemaphore firstPollEntered;
    QSemaphore releaseFirstPoll;
    std::atomic_int generationPollCalls{0};
    lcnc::process::ProcessStatusService generationService(
        queue,
        [&firstPollEntered, &releaseFirstPoll, &generationPollCalls](const QStringList&, const QVector<QPair<QString, QString>>&) {
            const int call = ++generationPollCalls;
            if (call == 1) {
                firstPollEntered.release();
                releaseFirstPoll.acquire();
            }
            lcnc::process::DeviceStatusSnapshot snapshot;
            snapshot.connected = true;
            snapshot.axes.push_back({QStringLiteral("X"), static_cast<double>(call), true, true});
            return snapshot;
        },
        [] { return lcnc::process::DevicePeripheralSnapshot{}; });
    generationService.setRequestProvider([] {
        lcnc::process::ProcessStatusRequest request;
        request.connected = true;
        request.simulationMode = true;
        request.acsSimulator = true;
        request.axisNames = {QStringLiteral("X")};
        return request;
    });
    int generationDeliveries = 0;
    double deliveredPosition = 0.0;
    generationService.setHardwareHandler([&](const lcnc::process::DeviceCommandResult& result,
                                              const lcnc::process::DeviceStatusSnapshot& snapshot) {
        assert(result.success);
        ++generationDeliveries;
        deliveredPosition = snapshot.axes.front().pos;
    });
    generationService.start();
    assert(firstPollEntered.tryAcquire(1, 2000));
    generationService.stop();
    generationService.start();
    releaseFirstPoll.release();
    assert(spinUntil([&] { return generationDeliveries == 1; }));
    assert(generationPollCalls == 2);
    assert(deliveredPosition == 2.0);
    generationService.stop();

    assert(queue.shutdown(2000));
    return 0;
}
