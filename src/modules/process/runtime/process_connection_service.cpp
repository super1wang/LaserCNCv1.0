#include "modules/process/runtime/process_connection_service.h"

#include "modules/process/runtime/process_device_runtime.h"

#include <QMetaObject>

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
          parent)
{
}

ProcessConnectionService::ProcessConnectionService(DeviceCommandQueue& queue,
                                                   ConnectRunner connectRunner,
                                                   DisconnectRunner disconnectRunner,
                                                   QObject* parent)
    : QObject(parent)
    , m_queue(queue)
    , m_connectRunner(std::move(connectRunner))
    , m_disconnectRunner(std::move(disconnectRunner))
{
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
            if (!m_connectRunner)
                // 中文翻译：连接执行器不可用
                return DeviceCommandResult{false, QObject::tr("Connection runner unavailable")};
            return m_connectRunner(pureSimulation, reportProgress);
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
            if (!m_disconnectRunner)
                // 中文翻译：断开执行器不可用
                return DeviceCommandResult{false, QObject::tr("Disconnection runner unavailable")};
            return m_disconnectRunner();
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
