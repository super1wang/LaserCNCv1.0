#include "modules/process/runtime/process_manual_motion_service.h"
#include "modules/process/runtime/process_interactive_io_service.h"

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

    int relativeCalls = 0;
    int absoluteCalls = 0;
    int continuousCalls = 0;
    int stopCalls = 0;
    lcnc::process::ProcessManualMotionService service(
        queue,
        [&relativeCalls](Axis axis, double distance, double velocity) {
            assert(axis == Axis::X && distance == 2.0 && velocity > 0.0);
            ++relativeCalls;
            return lcnc::process::DeviceCommandResult{};
        },
        [&absoluteCalls](Axis axis, double position, double velocity) {
            assert(axis == Axis::Y && position == 4.0 && velocity > 0.0);
            ++absoluteCalls;
            return lcnc::process::DeviceCommandResult{};
        },
        [&continuousCalls](Axis axis, bool positive, double velocity) {
            assert(axis == Axis::Z && positive && velocity > 0.0);
            ++continuousCalls;
            return lcnc::process::DeviceCommandResult{};
        },
        [&stopCalls](Axis axis) {
            assert(axis == Axis::Z);
            ++stopCalls;
            return lcnc::process::DeviceCommandResult{};
        });

    bool relativeCompleted = false;
    assert(service.moveRelative(QStringLiteral("x"), 2.0, 10.0, true, false,
        [&relativeCompleted](const QString& message) {
            assert(message.contains(QStringLiteral("X")));
            relativeCompleted = true;
        }).accepted);
    assert(spinUntil([&] { return relativeCompleted; }));
    assert(relativeCalls == 1);

    bool rejected = false;
    const auto disabled = service.moveAbsolute(QStringLiteral("Y"), 4.0, 10.0, false, false,
        [&rejected](const QString& message) {
            assert(message.contains(QStringLiteral("not enabled")));
            rejected = true;
        });
    assert(!disabled.accepted);
    assert(spinUntil([&] { return rejected; }));
    assert(absoluteCalls == 0);

    bool continuousCompleted = false;
    assert(service.startContinuous(QStringLiteral("Z"), true, 10.0, true, false,
        [&continuousCompleted](const QString&) { continuousCompleted = true; }).accepted);
    assert(spinUntil([&] { return continuousCompleted; }));
    bool stopCompleted = false;
    assert(service.stopContinuous(QStringLiteral("Z"),
        [&stopCompleted](const QString&) { stopCompleted = true; }).accepted);
    assert(spinUntil([&] { return stopCompleted; }));
    assert(continuousCalls == 1 && stopCalls == 1);

    int axisEnableCalls = 0;
    int outputCalls = 0;
    lcnc::process::ProcessInteractiveIoService ioService(
        queue,
        [&axisEnableCalls](Axis axis, bool enabled) {
            assert(axis == Axis::A && enabled);
            ++axisEnableCalls;
            return lcnc::process::DeviceCommandResult{};
        },
        [&outputCalls](const QString& channel, bool value) {
            assert(channel == QStringLiteral("aLaser") && value);
            ++outputCalls;
            return lcnc::process::DeviceCommandResult{};
        });
    bool axisEnabled = false;
    assert(ioService.setAxisEnabled(QStringLiteral("A"), true,
        [&axisEnabled](const lcnc::process::DeviceCommandResult& result) {
            assert(result.success);
            axisEnabled = true;
        }).accepted);
    assert(spinUntil([&] { return axisEnabled; }));
    bool outputSet = false;
    assert(ioService.setDigitalOutput(QStringLiteral("aLaser"), true,
        [&outputSet](const lcnc::process::DeviceCommandResult& result) {
            assert(result.success);
            outputSet = true;
        }).accepted);
    assert(spinUntil([&] { return outputSet; }));
    assert(axisEnableCalls == 1 && outputCalls == 1);

    bool lockedCompletion = false;
    lcnc::process::ProcessInteractiveIoService lockedIoService(
        queue,
        [](Axis, bool) { return lcnc::process::DeviceCommandResult{}; },
        [](const QString&, bool) { return lcnc::process::DeviceCommandResult{}; },
        nullptr,
        [] { return false; });
    assert(!lockedIoService.setDigitalOutput(QStringLiteral("aLaser"), true,
        [&lockedCompletion](const lcnc::process::DeviceCommandResult& result) {
            assert(!result.success);
            assert(result.error.contains(QStringLiteral("locked")));
            lockedCompletion = true;
        }).accepted);
    assert(spinUntil([&] { return lockedCompletion; }));

    assert(queue.shutdown(2000));
    return 0;
}
