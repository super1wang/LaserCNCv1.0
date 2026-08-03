#include "modules/process/runtime/process_manual_motion_service.h"

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

    assert(queue.shutdown(2000));
    return 0;
}
