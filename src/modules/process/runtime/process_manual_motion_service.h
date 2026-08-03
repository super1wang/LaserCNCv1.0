#pragma once

#include "core/kernel/i_service.h"
#include "modules/process/runtime/device_command_queue.h"
#include "modules/process/system/data_type.h"

#include <QObject>

#include <functional>
#include <optional>

class ProcessDeviceRuntime;

namespace lcnc::process {

/**
 * @brief Typed interactive manual-motion boundary for Process UI actions.
 *
 * It validates the UI request before it reaches the device queue and owns the
 * interactive/stop priority choice.  The caller supplies only state snapshots
 * and a GUI-thread status sink; vendor access remains in ProcessDeviceRuntime.
 */
class ProcessManualMotionService final : public QObject, public lcnc::IService
{
public:
    using Completion = std::function<void(const QString&)>;
    using RelativeRunner = std::function<DeviceCommandResult(Axis, double, double)>;
    using AbsoluteRunner = std::function<DeviceCommandResult(Axis, double, double)>;
    using JogRunner = std::function<DeviceCommandResult(Axis, bool, double)>;
    using StopRunner = std::function<DeviceCommandResult(Axis)>;

    ProcessManualMotionService(ProcessDeviceRuntime& runtime,
                               DeviceCommandQueue& queue,
                               QObject* parent = nullptr);
    ProcessManualMotionService(DeviceCommandQueue& queue,
                               RelativeRunner relativeRunner,
                               AbsoluteRunner absoluteRunner,
                               JogRunner jogRunner,
                               StopRunner stopRunner,
                               QObject* parent = nullptr);

    DeviceCommandTicket moveRelative(const QString& axisName, double distance, double velocity,
                                      bool axisEnabled, bool stopRecoveryRequired, Completion completion);
    DeviceCommandTicket moveAbsolute(const QString& axisName, double position, double velocity,
                                      bool axisEnabled, bool stopRecoveryRequired, Completion completion);
    DeviceCommandTicket startContinuous(const QString& axisName, bool positive, double velocity,
                                         bool axisEnabled, bool stopRecoveryRequired, Completion completion);
    DeviceCommandTicket stopContinuous(const QString& axisName, Completion completion);

private:
    static std::optional<Axis> axisFor(const QString& axisName);
    DeviceCommandTicket reject(const QString& message, Completion completion);
    void complete(Completion completion, const DeviceCommandResult& result,
                  const QString& successMessage);

    DeviceCommandQueue& m_queue;
    RelativeRunner m_relativeRunner;
    AbsoluteRunner m_absoluteRunner;
    JogRunner m_jogRunner;
    StopRunner m_stopRunner;
};

} // namespace lcnc::process
