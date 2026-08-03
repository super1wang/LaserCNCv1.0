#include "modules/process/runtime/process_preflight_service.h"

#include "modules/process/runtime/process_device_runtime.h"

#include <QObject>
#include <utility>

namespace lcnc::process {

ProcessPreflightService::ProcessPreflightService(ProcessDeviceRuntime& runtime,
                                                 DeviceCommandQueue& queue)
    : ProcessPreflightService(queue, [&runtime](const ProcessPreflightRequest& request,
                                               ProcessPreflightReport* report) {
          return runtime.runPreflight(request, report);
      })
{
}

ProcessPreflightService::ProcessPreflightService(DeviceCommandQueue& queue, Runner runner)
    : m_queue(queue)
    , m_runner(std::move(runner))
{
}

DeviceCommandTicket ProcessPreflightService::request(std::uint64_t generation,
                                                     ProcessPreflightRequest request,
                                                     Completion completion)
{
    auto report = std::make_shared<ProcessPreflightReport>();
    return m_queue.submitWithTicket(
        [runner = m_runner, request = std::move(request), report] {
            if (!runner)
                // 中文翻译：预检执行器不可用
                return DeviceCommandResult{false, QObject::tr("Preflight runner unavailable")};
            return runner(request, report.get());
        },
        TaskPriority::Workflow,
        [generation, report, completion = std::move(completion)](
            const DeviceCommandResult& result) {
            if (completion)
                completion(generation, result, report);
        });
}

} // namespace lcnc::process
