#include "modules/process/runtime/process_status_service.h"

#include <QMetaObject>

namespace lcnc::process {

ProcessStatusService::ProcessStatusService(ProcessDeviceRuntime& runtime,
                                           DeviceCommandQueue& queue,
                                           QObject* parent)
    : ProcessStatusService(
          queue,
          [&runtime](const QStringList& axes, const QVector<QPair<QString, QString>>& outputs) {
              return runtime.pollStatus(axes, outputs);
          },
          [&runtime] { return runtime.pollPeripheralStatus(); },
          parent)
{
}

ProcessStatusService::ProcessStatusService(DeviceCommandQueue& queue,
                                           HardwarePoller hardwarePoller,
                                           PeripheralPoller peripheralPoller,
                                           QObject* parent)
    : QObject(parent)
    , m_queue(queue)
    , m_hardwarePoller(std::move(hardwarePoller))
    , m_peripheralPoller(std::move(peripheralPoller))
{
    m_hardwareTimer.setInterval(150);
    m_peripheralTimer.setInterval(2000);
    connect(&m_hardwareTimer, &QTimer::timeout, this, &ProcessStatusService::onHardwareTimer);
    connect(&m_peripheralTimer, &QTimer::timeout, this, &ProcessStatusService::onPeripheralTimer);
}

void ProcessStatusService::setRequestProvider(RequestProvider provider)
{
    m_requestProvider = std::move(provider);
}

void ProcessStatusService::setHardwareHandler(HardwareHandler handler)
{
    m_hardwareHandler = std::move(handler);
}

void ProcessStatusService::setPeripheralHandler(PeripheralHandler handler)
{
    m_peripheralHandler = std::move(handler);
}

void ProcessStatusService::setSafetyMonitoringHandler(SafetyMonitoringHandler handler)
{
    m_safetyMonitoringHandler = std::move(handler);
}

void ProcessStatusService::start()
{
    if (m_active)
        return;
    const ProcessStatusRequest request = m_requestProvider ? m_requestProvider() : ProcessStatusRequest{};
    if (!request.connected || (request.simulationMode && !request.acsSimulator))
        return;
    m_active = true;
    m_hardwareTimer.start();
    if (!request.simulationMode)
        m_peripheralTimer.start();
    requestHardwarePoll();
    requestPeripheralPoll();
    if (m_safetyMonitoringHandler)
        m_safetyMonitoringHandler(!request.acsSimulator);
}

void ProcessStatusService::stop()
{
    m_active = false;
    m_hardwareTimer.stop();
    m_peripheralTimer.stop();
    m_hardwareInFlight = false;
    m_peripheralInFlight = false;
    if (m_safetyMonitoringHandler)
        m_safetyMonitoringHandler(false);
}

void ProcessStatusService::onHardwareTimer()
{
    requestHardwarePoll();
}

void ProcessStatusService::onPeripheralTimer()
{
    requestPeripheralPoll();
}

void ProcessStatusService::requestHardwarePoll()
{
    if (!m_active || m_hardwareInFlight || !m_requestProvider)
        return;
    const ProcessStatusRequest request = m_requestProvider();
    if (!request.connected || (request.simulationMode && !request.acsSimulator)
        || (request.axisNames.isEmpty() && request.digitalOutputs.isEmpty())) {
        return;
    }
    m_hardwareInFlight = true;
    auto snapshot = std::make_shared<DeviceStatusSnapshot>();
    const auto ticket = m_queue.submitWithTicket(
        [this, request, snapshot] {
            if (m_hardwarePoller)
                *snapshot = m_hardwarePoller(request.axisNames, request.digitalOutputs);
            return DeviceCommandResult{};
        },
        TaskPriority::Polling,
        [this, snapshot](const DeviceCommandResult& result) {
            QMetaObject::invokeMethod(this, [this, result, snapshot] {
                m_hardwareInFlight = false;
                if (m_active && m_hardwareHandler)
                    m_hardwareHandler(result, *snapshot);
            }, Qt::QueuedConnection);
        },
        QStringLiteral("controller-status"));
    if (!ticket.accepted)
        m_hardwareInFlight = false;
}

void ProcessStatusService::requestPeripheralPoll()
{
    if (!m_active || m_peripheralInFlight || !m_requestProvider)
        return;
    const ProcessStatusRequest request = m_requestProvider();
    if (!request.connected || request.simulationMode)
        return;
    m_peripheralInFlight = true;
    auto snapshot = std::make_shared<DevicePeripheralSnapshot>();
    const auto ticket = m_queue.submitWithTicket(
        [this, snapshot] {
            if (m_peripheralPoller)
                *snapshot = m_peripheralPoller();
            return DeviceCommandResult{};
        },
        TaskPriority::Polling,
        [this, snapshot](const DeviceCommandResult& result) {
            QMetaObject::invokeMethod(this, [this, result, snapshot] {
                m_peripheralInFlight = false;
                if (m_active && m_peripheralHandler)
                    m_peripheralHandler(result, *snapshot);
            }, Qt::QueuedConnection);
        },
        QStringLiteral("peripheral-status"));
    if (!ticket.accepted)
        m_peripheralInFlight = false;
}

} // namespace lcnc::process
