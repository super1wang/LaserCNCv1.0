#pragma once

#include "core/kernel/i_service.h"
#include "modules/process/runtime/device_command_queue.h"

#include <QObject>

#include <functional>
#include <atomic>
#include <optional>

class ProcessDeviceRuntime;

namespace lcnc::process {

class ProcessConnectionService final : public QObject, public lcnc::IService
{
public:
    using Progress = std::function<void(int, const QString&)>;
    using Completion = std::function<void(const DeviceCommandResult&)>;
    using ConnectRunner = std::function<DeviceCommandResult(bool, Progress)>;
    using DisconnectRunner = std::function<DeviceCommandResult()>;
    using ConnectionProbe = std::function<bool()>;

    ProcessConnectionService(ProcessDeviceRuntime& runtime,
                             DeviceCommandQueue& queue,
                             QObject* parent = nullptr);
    ProcessConnectionService(DeviceCommandQueue& queue,
                             ConnectRunner connectRunner,
                             DisconnectRunner disconnectRunner,
                             QObject* parent = nullptr,
                             ConnectionProbe connectionProbe = {});

    DeviceCommandTicket connect(bool pureSimulation, Progress progress, Completion completion);
    DeviceCommandTicket disconnect(Completion completion);
    /// Worker-captured session state, never a GUI-thread SDK read. Unknown is
    /// distinct from disconnected when no probe exists or the probe failed.
    std::optional<bool> lastMotionConnectionOpen() const noexcept;

private:
    DeviceCommandResult runConnectionOperation(
        const char* operation, const std::function<DeviceCommandResult()>& runner);
    bool captureMotionConnectionState();
    DeviceCommandQueue& m_queue;
    ConnectRunner m_connectRunner;
    DisconnectRunner m_disconnectRunner;
    ConnectionProbe m_connectionProbe;
    std::atomic<int> m_lastMotionConnectionOpen{-1};
};

} // namespace lcnc::process
