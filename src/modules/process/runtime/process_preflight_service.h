#pragma once

#include "core/kernel/i_service.h"
#include "modules/process/runtime/device_command_queue.h"
#include "modules/process/runtime/process_preflight_types.h"

#include <cstdint>
#include <functional>
#include <memory>

class ProcessDeviceRuntime;

namespace lcnc::process {

class ProcessPreflightService final : public lcnc::IService
{
public:
    using Runner = std::function<DeviceCommandResult(
        const ProcessPreflightRequest&, ProcessPreflightReport*)>;
    using Completion = std::function<void(
        std::uint64_t,
        const DeviceCommandResult&,
        std::shared_ptr<const ProcessPreflightReport>)>;

    ProcessPreflightService(ProcessDeviceRuntime& runtime, DeviceCommandQueue& queue);
    ProcessPreflightService(DeviceCommandQueue& queue, Runner runner);

    DeviceCommandTicket request(std::uint64_t generation,
                                ProcessPreflightRequest request,
                                Completion completion);

private:
    DeviceCommandQueue& m_queue;
    Runner m_runner;
};

} // namespace lcnc::process
