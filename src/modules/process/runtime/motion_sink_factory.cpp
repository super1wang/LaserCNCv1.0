#include "modules/process/runtime/i_motion_command_sink.h"

#include "core/kernel/kernel.h"
#include "core/kinematics/machine_configuration_service.h"
#include "core/logging/logger.h"
#include "modules/process/cutting/pure_simulation_toolpath_ticker.h"
#if LCNC_PROCESS_HAS_ACS
#include "modules/process/device/MotionControl/ACSMotionControl.h"
#include "modules/process/device/MotionControl/SimulateCMHPMotionControl.h"
#include "modules/process/runtime/acs_text_command_sink.h"
#endif
#if LCNC_PROCESS_HAS_GTN
#include "modules/process/device/MotionControl/GTNMotionControl.h"
#include "modules/process/runtime/gtn_buffered_command_sink.h"
#endif
#include "modules/process/device/MotionControl/MotionControl.h"
#include "modules/process/process_module.h"
#include "modules/process/runtime/pure_simulation_sink.h"

#include <QPointer>

namespace lcnc::process {

namespace {

#if LCNC_PROCESS_HAS_ACS
AcsTextCommandSink::PositionObserver makePositionObserver(ProcessModule* processModule)
{
    const QPointer<ProcessModule> guardedModule(processModule);
    return [guardedModule](const QString& axisName, double position) {
        if (!guardedModule)
            return;
        QMetaObject::invokeMethod(guardedModule.data(),
                                  [guardedModule, axisName, position] {
                                      if (guardedModule)
                                          guardedModule->setAxisPosition(axisName, position);
                                  },
                                  Qt::QueuedConnection);
    };
}
#endif

} // namespace

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

    // Any selected ACS/GTN backend is authoritative.  A disconnected or
    // failed real backend must never be converted into PureSimulation.
#if LCNC_PROCESS_HAS_ACS
    if (auto* acs = dynamic_cast<ACSMotionControl*>(mc)) {
        if (!acs->IsConnected()) {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "MotionSinkFactory: selected ACS backend '{}' is disconnected",
                     acs->GetName());
            return nullptr;
        }
        return std::make_unique<AcsTextCommandSink>(acs, axes,
                                                    makePositionObserver(processModule));
    }
#endif
#if LCNC_PROCESS_HAS_GTN
    if (auto* gtn = dynamic_cast<GTNMotionControl*>(mc)) {
        if (!gtn->IsConnected()) {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "MotionSinkFactory: selected GTN backend '{}' is disconnected",
                     gtn->GetName());
            return nullptr;
        }
        return std::make_unique<GtnBufferedCommandSink>(gtn, axes);
    }
#endif

    // PureSimulation is entered only by an explicit pure-simulation mode.  It
    // is not a connection-failure fallback.
    if (simulationMode) {
        if (!simTicker) {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "MotionSinkFactory: pure-simulation requested but no ticker provided");
            return nullptr;
        }
        return std::make_unique<PureSimulationSink>(simTicker, processModule, axes);
    }

    const std::string backend = mc ? mc->GetName() : std::string("<none>");
    LCNC_ERR(lcnc::LogCode::Generic,
             "MotionSinkFactory: backend '{}' is unavailable; PureSimulation fallback is forbidden",
             backend);
    return nullptr;
}

} // namespace lcnc::process
