#include "modules/process/runtime/process_status_service.h"

#include "core/logging/logger.h"

#include <QMetaObject>

namespace lcnc::process {

ProcessStatusService::ProcessStatusService(ProcessDeviceRuntime& runtime,
                                           QObject* parent)
    : ProcessStatusService(
          [&runtime](const QStringList& axes, const QVector<QPair<QString, QString>>& outputs) {
              return runtime.pollStatus(axes, outputs);
          },
          [&runtime] { return runtime.pollPeripheralStatus(); },
          parent)
{
}

ProcessStatusService::ProcessStatusService(HardwarePoller hardwarePoller,
                                           PeripheralPoller peripheralPoller,
                                           QObject* parent)
    : QObject(parent)
    , m_hardwarePoller(std::move(hardwarePoller))
    , m_peripheralPoller(std::move(peripheralPoller))
{
    m_hardwareTimer.setInterval(150);
    m_peripheralTimer.setInterval(2000);
    connect(&m_hardwareTimer, &QTimer::timeout, this, &ProcessStatusService::onHardwareTimer);
    connect(&m_peripheralTimer, &QTimer::timeout, this, &ProcessStatusService::onPeripheralTimer);
}

ProcessStatusService::~ProcessStatusService()
{
    // ProcessModule owns this service and its safety monitor as separate
    // members. During C++ member teardown the monitor is already gone, so the
    // destructor must not invoke the external safety callback again.
    // 中文翻译：析构阶段外部安全监控对象可能已销毁，只停止自身轮询，禁止再次回调。
    ++m_generation;
    m_active = false;
    m_hardwareTimer.stop();
    m_peripheralTimer.stop();
    if (m_hardwareTicket != 0)
        (void)m_hardwareQueue.cancel(m_hardwareTicket);
    if (m_peripheralTicket != 0)
        (void)m_peripheralQueue.cancel(m_peripheralTicket);
    m_hardwareTicket = 0;
    m_peripheralTicket = 0;
    const bool hardwareStopped = m_hardwareQueue.shutdown(5000);
    const bool peripheralStopped = m_peripheralQueue.shutdown(5000);
    if (!hardwareStopped || !peripheralStopped) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "process.status: polling executor shutdown timed out hardware={} peripheral={}",
                 hardwareStopped, peripheralStopped);
    }
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
    if (!m_hardwareQueue.start() || !m_peripheralQueue.start()) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "process.status: failed to start independent polling executors");
        return;
    }
    ++m_generation;
    m_active = true;
    m_hardwareTimer.start();
    if (!request.simulationMode)
        m_peripheralTimer.start();
    requestHardwarePoll();
    requestPeripheralPoll();
    if (m_safetyMonitoringHandler)
        m_safetyMonitoringHandler(!request.acsSimulator);
    LCNC_INFO(lcnc::LogCode::Generic,
              "process.status: independent polling started interval_ms=150 peripheral_interval_ms=2000");
}

void ProcessStatusService::stop()
{
    ++m_generation;
    m_active = false;
    m_hardwareTimer.stop();
    m_peripheralTimer.stop();
    if (m_hardwareTicket != 0)
        (void)m_hardwareQueue.cancel(m_hardwareTicket);
    if (m_peripheralTicket != 0)
        (void)m_peripheralQueue.cancel(m_peripheralTicket);
    m_hardwareTicket = 0;
    m_peripheralTicket = 0;
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
    const std::uint64_t generation = m_generation;
    auto snapshot = std::make_shared<DeviceStatusSnapshot>();
    const auto ticket = m_hardwareQueue.submitWithTicket(
        [this, request, snapshot] {
            if (m_hardwarePoller)
                *snapshot = m_hardwarePoller(request.axisNames, request.digitalOutputs);
            return DeviceCommandResult{};
        },
        TaskPriority::Polling,
        [this, generation, snapshot](const DeviceCommandResult& result) {
            QMetaObject::invokeMethod(this, [this, generation, result, snapshot] {
                if (generation != m_generation)
                    return;
                m_hardwareInFlight = false;
                m_hardwareTicket = 0;
                if (m_active && m_hardwareHandler)
                    m_hardwareHandler(result, *snapshot);
            }, Qt::QueuedConnection);
        },
        QStringLiteral("controller-status"));
    if (!ticket.accepted)
        m_hardwareInFlight = false;
    else
        m_hardwareTicket = ticket.id;
}

void ProcessStatusService::requestPeripheralPoll()
{
    if (!m_active || m_peripheralInFlight || !m_requestProvider)
        return;
    const ProcessStatusRequest request = m_requestProvider();
    if (!request.connected || request.simulationMode)
        return;
    m_peripheralInFlight = true;
    const std::uint64_t generation = m_generation;
    auto snapshot = std::make_shared<DevicePeripheralSnapshot>();
    const auto ticket = m_peripheralQueue.submitWithTicket(
        [this, snapshot] {
            if (m_peripheralPoller)
                *snapshot = m_peripheralPoller();
            return DeviceCommandResult{};
        },
        TaskPriority::Polling,
        [this, generation, snapshot](const DeviceCommandResult& result) {
            QMetaObject::invokeMethod(this, [this, generation, result, snapshot] {
                if (generation != m_generation)
                    return;
                m_peripheralInFlight = false;
                m_peripheralTicket = 0;
                if (m_active && m_peripheralHandler)
                    m_peripheralHandler(result, *snapshot);
            }, Qt::QueuedConnection);
        },
        QStringLiteral("peripheral-status"));
    if (!ticket.accepted)
        m_peripheralInFlight = false;
    else
        m_peripheralTicket = ticket.id;
}

} // namespace lcnc::process
