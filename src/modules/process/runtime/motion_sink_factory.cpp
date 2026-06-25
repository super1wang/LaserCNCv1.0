#include "modules/process/runtime/i_motion_command_sink.h"

#include "core/kernel/kernel.h"
#include "core/kinematics/machine_configuration_service.h"
#include "core/logging/logger.h"
#include "modules/process/cutting/pure_simulation_toolpath_ticker.h"
#include "modules/process/device/MotionControl/ACSMotionControl.h"
#if LCNC_PROCESS_HAS_GTN
#include "modules/process/device/MotionControl/GTNMotionControl.h"
#include "modules/process/runtime/gtn_buffered_command_sink.h"
#endif
#include "modules/process/device/MotionControl/MotionControl.h"
#include "modules/process/process_module.h"
#include "modules/process/runtime/acs_text_command_sink.h"
#include "modules/process/runtime/pure_simulation_sink.h"

namespace lcnc::process {

std::unique_ptr<IMotionCommandSink>
MotionSinkFactory::create(MotionControl* mc,
                          bool simulationMode,
                          PureSimulationToolpathTicker* simTicker,
                          ProcessModule* processModule)
{
    // 构型驱动的轴映射 —— 一次构造、整个 sink 生命周期复用。
    auto* kernel = lcnc::Kernel::tryCurrent();
    auto* machineConfig = kernel ? kernel->service<lcnc::MachineConfigurationService>() : nullptr;
    AxisMap axes = AxisMap::from(machineConfig);

    // 无硬件 / 仿真模式：仿真 sink。
    const bool hardwareLive = mc && mc->IsConnected();
    if (!hardwareLive || simulationMode) {
        if (!simTicker) {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "MotionSinkFactory: pure-simulation requested but no ticker provided");
            return nullptr;
        }
        return std::make_unique<PureSimulationSink>(simTicker, processModule, axes);
    }

    const std::string backend = mc ? mc->GetName() : std::string();
#if LCNC_PROCESS_HAS_GTN
    if (auto* gtn = dynamic_cast<GTNMotionControl*>(mc)) {
        return std::make_unique<GtnBufferedCommandSink>(gtn, axes);
    }
#endif
    if (auto* acs = dynamic_cast<ACSMotionControl*>(mc)) {
        return std::make_unique<AcsTextCommandSink>(acs, axes);
    }

    LCNC_ERR(lcnc::LogCode::Generic,
             "MotionSinkFactory: unknown backend '{}', falling back to PureSimulation", backend);
    if (!simTicker)
        return nullptr;
    return std::make_unique<PureSimulationSink>(simTicker, processModule, axes);
}

} // namespace lcnc::process
