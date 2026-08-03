#pragma once

#include "core/kernel/i_service.h"
#include "modules/process/runtime/device_command_queue.h"

#include <QObject>

#include <functional>

class ProcessDeviceRuntime;

namespace lcnc::process {

class ProcessConnectionService final : public QObject, public lcnc::IService
{
public:
    using Progress = std::function<void(int, const QString&)>;
    using Completion = std::function<void(const DeviceCommandResult&)>;
    using ConnectRunner = std::function<DeviceCommandResult(bool, Progress)>;
    using DisconnectRunner = std::function<DeviceCommandResult()>;

    ProcessConnectionService(ProcessDeviceRuntime& runtime,
                             DeviceCommandQueue& queue,
                             QObject* parent = nullptr);
    ProcessConnectionService(DeviceCommandQueue& queue,
                             ConnectRunner connectRunner,
                             DisconnectRunner disconnectRunner,
                             QObject* parent = nullptr);

    DeviceCommandTicket connect(bool pureSimulation, Progress progress, Completion completion);
    DeviceCommandTicket disconnect(Completion completion);

private:
    DeviceCommandQueue& m_queue;
    ConnectRunner m_connectRunner;
    DisconnectRunner m_disconnectRunner;
};

} // namespace lcnc::process
