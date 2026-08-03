#pragma once

#include "core/kernel/i_service.h"
#include "modules/process/runtime/device_command_queue.h"
#include "modules/process/runtime/process_device_runtime.h"

#include <QObject>
#include <QTimer>

#include <functional>

class ProcessDeviceRuntime;

namespace lcnc::process {

struct ProcessStatusRequest {
    bool connected{false};
    bool simulationMode{true};
    bool acsSimulator{false};
    QStringList axisNames;
    QVector<QPair<QString, QString>> digitalOutputs;
};

class ProcessStatusService final : public QObject, public lcnc::IService
{
public:
    using RequestProvider = std::function<ProcessStatusRequest()>;
    using HardwarePoller = std::function<DeviceStatusSnapshot(
        const QStringList&, const QVector<QPair<QString, QString>>&)>;
    using PeripheralPoller = std::function<DevicePeripheralSnapshot()>;
    using HardwareHandler = std::function<void(const DeviceCommandResult&, const DeviceStatusSnapshot&)>;
    using PeripheralHandler = std::function<void(const DeviceCommandResult&, const DevicePeripheralSnapshot&)>;
    using SafetyMonitoringHandler = std::function<void(bool)>;

    ProcessStatusService(ProcessDeviceRuntime& runtime,
                         DeviceCommandQueue& queue,
                         QObject* parent = nullptr);
    ProcessStatusService(DeviceCommandQueue& queue,
                         HardwarePoller hardwarePoller,
                         PeripheralPoller peripheralPoller,
                         QObject* parent = nullptr);

    void setRequestProvider(RequestProvider provider);
    void setHardwareHandler(HardwareHandler handler);
    void setPeripheralHandler(PeripheralHandler handler);
    void setSafetyMonitoringHandler(SafetyMonitoringHandler handler);

    void start();
    void stop();
    void requestHardwarePoll();
    void requestPeripheralPoll();
    bool isActive() const { return m_active; }

private:
    void onHardwareTimer();
    void onPeripheralTimer();

    DeviceCommandQueue& m_queue;
    HardwarePoller m_hardwarePoller;
    PeripheralPoller m_peripheralPoller;
    QTimer m_hardwareTimer;
    QTimer m_peripheralTimer;
    RequestProvider m_requestProvider;
    HardwareHandler m_hardwareHandler;
    PeripheralHandler m_peripheralHandler;
    SafetyMonitoringHandler m_safetyMonitoringHandler;
    bool m_active{false};
    bool m_hardwareInFlight{false};
    bool m_peripheralInFlight{false};
};

} // namespace lcnc::process
