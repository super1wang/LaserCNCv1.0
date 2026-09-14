#include "modules/process/runtime/process_connection_service.h"

#include "core/logging/logger.h"
#include "modules/process/runtime/process_device_runtime.h"

#include <QMetaObject>

#include <exception>

namespace lcnc::process {

ProcessConnectionService::ProcessConnectionService(ProcessDeviceRuntime& runtime,
                                                   DeviceCommandQueue& queue,
                                                   QObject* parent)
    : ProcessConnectionService(
          queue,
          [&runtime](bool pureSimulation, Progress progress) {
              return runtime.connectDevices(pureSimulation, progress);
          },
          [&runtime] { return runtime.disconnectDevices(); },
          parent,
          [&runtime] { return runtime.motionConnectionOpen(); })
{
}

ProcessConnectionService::ProcessConnectionService(DeviceCommandQueue& queue,
                                                   ConnectRunner connectRunner,
                                                   DisconnectRunner disconnectRunner,
                                                   QObject* parent,
                                                   ConnectionProbe connectionProbe)
    : QObject(parent)
    , m_queue(queue)
    , m_connectRunner(std::move(connectRunner))
    , m_disconnectRunner(std::move(disconnectRunner))
    , m_connectionProbe(std::move(connectionProbe))
{
}

std::optional<bool> ProcessConnectionService::lastMotionConnectionOpen() const noexcept
{
    const int state = m_lastMotionConnectionOpen.load(std::memory_order_acquire);
    if (state < 0)
        return std::nullopt;
    return state != 0;
}

bool ProcessConnectionService::captureMotionConnectionState()
{
    m_lastMotionConnectionOpen.store(-1, std::memory_order_release);
    if (!m_connectionProbe)
        return true;
    try {
        const bool open = m_connectionProbe();
        m_lastMotionConnectionOpen.store(open ? 1 : 0, std::memory_order_release);
        return true;
    } catch (const std::exception& exception) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "ProcessConnectionService: motion connection probe failed: {}", exception.what());
    } catch (...) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "ProcessConnectionService: motion connection probe threw an unknown exception");
    }
    return false;
}

DeviceCommandResult ProcessConnectionService::runConnectionOperation(
    const char* operation, const std::function<DeviceCommandResult()>& runner)
{
    DeviceCommandResult result;
    try {
        result = runner();
    } catch (const std::exception& exception) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "ProcessConnectionService: {} failed: {}", operation, exception.what());
        result = {false, QString::fromUtf8(exception.what())};
    } catch (...) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "ProcessConnectionService: {} threw an unknown exception", operation);
        // 中文翻译：设备连接操作发生未知异常。
        result = {false, QObject::tr("An unknown exception occurred during the device connection operation")};
    }
    // Even a failed runner may leave a live controller session behind. Capture
    // that state before dispatching the GUI completion, without retrying Stop.
    // 中文翻译：失败操作仍可能保留连接；返回 GUI 前采集真实会话状态，不重试停机。
    if (!captureMotionConnectionState() && result.success) {
        // 中文翻译：无法确认运动控制器连接状态。
        result = {false, QObject::tr("Cannot confirm the motion controller connection state")};
    }
    return result;
}

DeviceCommandTicket ProcessConnectionService::connect(bool pureSimulation,
                                                       Progress progress,
                                                       Completion completion)
{
    return m_queue.submitWithTicket(
        [this, pureSimulation, progress = std::move(progress)] {
            const auto reportProgress = [this, progress](int percent, const QString& step) {
                if (!progress)
                    return;
                QMetaObject::invokeMethod(this, [progress, percent, step] {
                    progress(percent, step);
                }, Qt::QueuedConnection);
            };
            return runConnectionOperation("connect", [this, pureSimulation, reportProgress] {
                if (!m_connectRunner)
                    // 中文翻译：连接执行器不可用
                    return DeviceCommandResult{false, QObject::tr("Connection runner unavailable")};
                return m_connectRunner(pureSimulation, reportProgress);
            });
        },
        TaskPriority::Workflow,
        [this, completion = std::move(completion)](const DeviceCommandResult& result) {
            if (!completion)
                return;
            QMetaObject::invokeMethod(this, [completion, result] {
                completion(result);
            }, Qt::QueuedConnection);
        });
}

DeviceCommandTicket ProcessConnectionService::disconnect(Completion completion)
{
    return m_queue.submitWithTicket(
        [this] {
            return runConnectionOperation("disconnect", [this] {
                if (!m_disconnectRunner)
                    // 中文翻译：断开执行器不可用
                    return DeviceCommandResult{false, QObject::tr("Disconnection runner unavailable")};
                return m_disconnectRunner();
            });
        },
        TaskPriority::Stop,
        [this, completion = std::move(completion)](const DeviceCommandResult& result) {
            if (!completion)
                return;
            QMetaObject::invokeMethod(this, [completion, result] {
                completion(result);
            }, Qt::QueuedConnection);
        });
}

} // namespace lcnc::process
