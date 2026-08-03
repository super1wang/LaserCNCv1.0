#include "modules/process/runtime/process_manual_motion_service.h"

#include "modules/process/runtime/process_device_runtime.h"

#include <QMetaObject>

namespace lcnc::process {

ProcessManualMotionService::ProcessManualMotionService(ProcessDeviceRuntime& runtime,
                                                       DeviceCommandQueue& queue,
                                                       QObject* parent)
    : ProcessManualMotionService(
          queue,
          [&runtime](Axis axis, double distance, double velocity) {
              return runtime.moveRelative(axis, distance, velocity);
          },
          [&runtime](Axis axis, double position, double velocity) {
              return runtime.moveAbsolute(axis, position, velocity);
          },
          [&runtime](Axis axis, bool positive, double velocity) {
              return runtime.jog(axis, positive, velocity);
          },
          [&runtime](Axis axis) { return runtime.stopAxis(axis); },
          parent)
{
}

ProcessManualMotionService::ProcessManualMotionService(DeviceCommandQueue& queue,
                                                       RelativeRunner relativeRunner,
                                                       AbsoluteRunner absoluteRunner,
                                                       JogRunner jogRunner,
                                                       StopRunner stopRunner,
                                                       QObject* parent)
    : QObject(parent)
    , m_queue(queue)
    , m_relativeRunner(std::move(relativeRunner))
    , m_absoluteRunner(std::move(absoluteRunner))
    , m_jogRunner(std::move(jogRunner))
    , m_stopRunner(std::move(stopRunner))
{
}

std::optional<Axis> ProcessManualMotionService::axisFor(const QString& axisName)
{
    return enum_cast<Axis>(axisName.trimmed().toUpper().toStdString());
}

DeviceCommandTicket ProcessManualMotionService::reject(const QString& message, Completion completion)
{
    if (completion)
        QMetaObject::invokeMethod(this, [completion = std::move(completion), message] { completion(message); },
                                  Qt::QueuedConnection);
    return {};
}

void ProcessManualMotionService::complete(Completion completion,
                                          const DeviceCommandResult& result,
                                          const QString& successMessage)
{
    if (!completion)
        return;
    const QString message = result.success ? successMessage : result.error;
    QMetaObject::invokeMethod(this, [completion = std::move(completion), message] { completion(message); },
                              Qt::QueuedConnection);
}

DeviceCommandTicket ProcessManualMotionService::moveRelative(const QString& axisName,
                                                              double distance,
                                                              double velocity,
                                                              bool axisEnabled,
                                                               bool stopRecoveryRequired,
                                                              Completion completion)
{
    const QString axis = axisName.trimmed().toUpper();
    if (axis.isEmpty() || distance == 0.0 || stopRecoveryRequired)
        return reject(tr("Manual motion request is unavailable"), std::move(completion));
    if (!axisEnabled)
        return reject(tr("%1 axis is not enabled, jog has been ignored").arg(axis), std::move(completion));
    const auto parsed = axisFor(axis);
    if (!parsed)
        return reject(tr("%1 axis is not registered and cannot be jogged").arg(axis), std::move(completion));
    if (!m_relativeRunner)
        return reject(tr("Manual-motion runner unavailable"), std::move(completion));
    const QString message = tr("Jog %1 axis %2").arg(axis, distance > 0.0 ? tr("forward") : tr("Negative"));
    const auto ticket = m_queue.submitWithTicket(
        [runner = m_relativeRunner, axis = *parsed, distance, velocity] {
            return runner(axis, distance, velocity);
        },
        TaskPriority::Interactive,
        [this, completion, message](const DeviceCommandResult& result) {
            complete(completion, result, message);
        });
    if (!ticket.accepted)
        complete(std::move(completion),
                 DeviceCommandResult{false, tr("Jog command failed to queue")}, {});
    return ticket;
}

DeviceCommandTicket ProcessManualMotionService::moveAbsolute(const QString& axisName,
                                                              double position,
                                                              double velocity,
                                                              bool axisEnabled,
                                                               bool stopRecoveryRequired,
                                                              Completion completion)
{
    const QString axis = axisName.trimmed().toUpper();
    if (axis.isEmpty() || stopRecoveryRequired)
        return reject(tr("Manual motion request is unavailable"), std::move(completion));
    if (!axisEnabled)
        return reject(tr("%1 axis is not enabled, absolute motion has been ignored").arg(axis), std::move(completion));
    const auto parsed = axisFor(axis);
    if (!parsed)
        return reject(tr("%1 axis is not registered and cannot move absolutely").arg(axis), std::move(completion));
    if (!m_absoluteRunner)
        return reject(tr("Manual-motion runner unavailable"), std::move(completion));
    const QString message = tr("%1 axis moved to %2").arg(axis).arg(position, 0, 'f', 3);
    const auto ticket = m_queue.submitWithTicket(
        [runner = m_absoluteRunner, axis = *parsed, position, velocity] {
            return runner(axis, position, velocity);
        },
        TaskPriority::Interactive,
        [this, completion, message](const DeviceCommandResult& result) {
            complete(completion, result, message);
        });
    if (!ticket.accepted)
        complete(std::move(completion),
                 DeviceCommandResult{false, tr("Absolute motion command failed to queue")}, {});
    return ticket;
}

DeviceCommandTicket ProcessManualMotionService::startContinuous(const QString& axisName,
                                                                 bool positive,
                                                                 double velocity,
                                                                 bool axisEnabled,
                                                                  bool stopRecoveryRequired,
                                                                 Completion completion)
{
    const QString axis = axisName.trimmed().toUpper();
    if (axis.isEmpty() || stopRecoveryRequired)
        return reject(tr("Manual motion request is unavailable"), std::move(completion));
    if (!axisEnabled)
        return reject(tr("%1 axis is not enabled, continuous motion is ignored").arg(axis), std::move(completion));
    const auto parsed = axisFor(axis);
    if (!parsed)
        return reject(tr("%1 axis is not registered and cannot move continuously.").arg(axis), std::move(completion));
    if (!m_jogRunner)
        return reject(tr("Manual-motion runner unavailable"), std::move(completion));
    const QString message = tr("Continuously jog %1 axis %2").arg(
        axis, positive ? tr("forward") : tr("Negative"));
    const auto ticket = m_queue.submitWithTicket(
        [runner = m_jogRunner, axis = *parsed, positive, velocity] { return runner(axis, positive, velocity); },
        TaskPriority::Interactive,
        [this, completion, message](const DeviceCommandResult& result) {
            complete(completion, result, message);
        });
    if (!ticket.accepted)
        complete(std::move(completion),
                 DeviceCommandResult{false, tr("Continuous jog commands failed to be queued")}, {});
    return ticket;
}

DeviceCommandTicket ProcessManualMotionService::stopContinuous(const QString& axisName, Completion completion)
{
    const QString axis = axisName.trimmed().toUpper();
    const auto parsed = axisFor(axis);
    if (axis.isEmpty() || !parsed)
        return reject(tr("Manual motion request is unavailable"), std::move(completion));
    if (!m_stopRunner)
        return reject(tr("Manual-motion runner unavailable"), std::move(completion));
    const QString message = tr("%1 axis continuous motion has stopped").arg(axis);
    const auto ticket = m_queue.submitWithTicket(
        [runner = m_stopRunner, axis = *parsed] { return runner(axis); },
        TaskPriority::Stop,
        [this, completion, message](const DeviceCommandResult& result) {
            complete(completion, result, message);
        });
    if (!ticket.accepted)
        complete(std::move(completion),
                 DeviceCommandResult{false, tr("%1 axis stop command failed to be queued").arg(axis)}, {});
    return ticket;
}

} // namespace lcnc::process
