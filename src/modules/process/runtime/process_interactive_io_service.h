#pragma once

#include "core/kernel/i_service.h"
#include "modules/process/runtime/device_command_queue.h"
#include "modules/process/runtime/process_axis_types.h"

#include <QObject>

#include <functional>

class ProcessDeviceRuntime;

namespace lcnc::process {

class ProcessInteractiveIoService final : public QObject, public lcnc::IService
{
public:
    using Completion = std::function<void(const DeviceCommandResult&)>;
    using AxisRunner = std::function<DeviceCommandResult(Axis, bool)>;
    using OutputRunner = std::function<DeviceCommandResult(const QString&, bool)>;
    using InteractionAllowed = std::function<bool()>;

    ProcessInteractiveIoService(ProcessDeviceRuntime& runtime, DeviceCommandQueue& queue,
                                 QObject* parent = nullptr,
                                 InteractionAllowed interactionAllowed = {});
    ProcessInteractiveIoService(DeviceCommandQueue& queue, AxisRunner axisRunner,
                                 OutputRunner outputRunner, QObject* parent = nullptr,
                                 InteractionAllowed interactionAllowed = {});

    DeviceCommandTicket setAxisEnabled(const QString& axisName, bool enabled, Completion completion);
    DeviceCommandTicket setDigitalOutput(const QString& channel, bool value, Completion completion);

private:
    void complete(Completion completion, DeviceCommandResult result);
    DeviceCommandQueue& m_queue;
    AxisRunner m_axisRunner;
    OutputRunner m_outputRunner;
    InteractionAllowed m_interactionAllowed;
};

} // namespace lcnc::process
