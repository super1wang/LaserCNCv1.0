#include "modules/process/runtime/process_interactive_io_service.h"

#include "modules/process/runtime/process_device_runtime.h"

#include <QMetaObject>

namespace lcnc::process {

ProcessInteractiveIoService::ProcessInteractiveIoService(ProcessDeviceRuntime& runtime,
                                                         DeviceCommandQueue& queue,
                                                         QObject* parent)
    : ProcessInteractiveIoService(queue,
        [&runtime](Axis axis, bool enabled) { return runtime.setAxisEnabled(axis, enabled); },
        [&runtime](const QString& channel, bool value) { return runtime.setDigitalOutput(channel, value); },
        parent)
{
}

ProcessInteractiveIoService::ProcessInteractiveIoService(DeviceCommandQueue& queue,
                                                         AxisRunner axisRunner,
                                                         OutputRunner outputRunner,
                                                         QObject* parent)
    : QObject(parent)
    , m_queue(queue)
    , m_axisRunner(std::move(axisRunner))
    , m_outputRunner(std::move(outputRunner))
{
}

void ProcessInteractiveIoService::complete(Completion completion, DeviceCommandResult result)
{
    if (!completion)
        return;
    QMetaObject::invokeMethod(this, [completion = std::move(completion), result] { completion(result); },
                              Qt::QueuedConnection);
}

DeviceCommandTicket ProcessInteractiveIoService::setAxisEnabled(const QString& axisName, bool enabled,
                                                                 Completion completion)
{
    const QString axisNameNormalized = axisName.trimmed().toUpper();
    const auto axis = enum_cast<Axis>(axisNameNormalized.toStdString());
    if (!axis || !m_axisRunner) {
        complete(std::move(completion), {false, tr("Axis is not registered or the device queue is unavailable")});
        return {};
    }
    const auto ticket = m_queue.submitWithTicket(
        [runner = m_axisRunner, axis = *axis, enabled] { return runner(axis, enabled); },
        TaskPriority::Interactive,
        [this, completion](const DeviceCommandResult& result) { complete(completion, result); });
    if (!ticket.accepted)
        complete(std::move(completion), {false, tr("Axis enable command failed to queue")});
    return ticket;
}

DeviceCommandTicket ProcessInteractiveIoService::setDigitalOutput(const QString& channel, bool value,
                                                                   Completion completion)
{
    const QString normalizedChannel = channel.trimmed();
    if (normalizedChannel.isEmpty() || !m_outputRunner) {
        complete(std::move(completion), {false, tr("Digital-output runner unavailable")});
        return {};
    }
    const auto ticket = m_queue.submitWithTicket(
        [runner = m_outputRunner, normalizedChannel, value] { return runner(normalizedChannel, value); },
        TaskPriority::Interactive,
        [this, completion](const DeviceCommandResult& result) { complete(completion, result); });
    if (!ticket.accepted)
        complete(std::move(completion), {false, tr("IO output command failed to queue")});
    return ticket;
}

} // namespace lcnc::process
