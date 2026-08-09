#include "core/kernel/kernel.h"
#include "core/kinematics/machine_configuration_service.h"
#include "modules/process/runtime/device_command_queue.h"
#include "modules/process/runtime/process_device_runtime.h"
#include "modules/process/runtime/process_runtime_configuration.h"
#include "modules/process/settings/process_settings_service.h"

#include <QCoreApplication>
#include <QSemaphore>
#include <QTemporaryDir>
#include <QTextStream>

#include <memory>

namespace {

int fail(const QString& message)
{
    QTextStream(stderr) << message << '\n';
    return 1;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    QTemporaryDir settingsDirectory;
    if (!settingsDirectory.isValid())
        return fail(QStringLiteral("Could not create temporary Process settings directory"));

    lcnc::Kernel kernel;
    auto machineConfiguration = std::make_shared<lcnc::MachineConfigurationService>();
    if (!kernel.services().registerService<lcnc::MachineConfigurationService>(machineConfiguration))
        return fail(QStringLiteral("Could not register the test machine configuration"));

    lcnc::process::ProcessSettingsService settings(settingsDirectory.path());
    if (!settings.initialize())
        return fail(QStringLiteral("Could not initialize temporary Process settings"));

    lcnc::process::ProcessRuntimeConfiguration configuration;
    configuration.setSimulationMode(false);
    configuration.setEnabledAxes({QStringLiteral("X"), QStringLiteral("Y"), QStringLiteral("Z")});

    lcnc::process::DeviceCommandQueue queue;
    if (!queue.start())
        return fail(QStringLiteral("Could not start Process device executor"));

    const auto runtimeHolder = std::make_shared<std::shared_ptr<ProcessDeviceRuntime>>(
        std::make_shared<ProcessDeviceRuntime>(settings, configuration));
    const auto releaseRuntimeOnExecutor = [&queue, runtimeHolder] {
        QSemaphore completed;
        lcnc::process::DeviceCommandResult result;
        if (!queue.submit([runtimeHolder] {
                lcnc::process::DeviceCommandResult disconnected;
                if (*runtimeHolder)
                    disconnected = (*runtimeHolder)->disconnectDevices();
                runtimeHolder->reset();
                return disconnected;
            }, TaskPriority::Stop,
            [&result, &completed](const lcnc::process::DeviceCommandResult& completion) {
                result = completion;
                completed.release();
            })) {
            return false;
        }
        return completed.tryAcquire(1, 30000) && result.success;
    };
    // Opening a real ACS Simulator validates acsc_OpenCommSimulator and the
    // deployed Simulator.prg.  The position-setting command exercises the ACS
    // setfpos path, while the remaining polls validate the active SDK session
    // without repeatedly relaunching the vendor RPC server, which has a
    // multi-second process-release interval.
    for (int iteration = 0; iteration < 100; ++iteration) {
        QSemaphore completed;
        lcnc::process::DeviceCommandResult result;
        if (!queue.submit([runtimeHolder, iteration] {
            const auto& runtime = *runtimeHolder;
            if (iteration == 0) {
                return runtime->connectDevices(false, [](int, const QString&) {});
            }
            if (iteration == 1) {
                return runtime->setAxisPositions({{Axis::X, 12.5}});
            }
            const bool connected = runtime->pollStatus({}, {}).connected;
            return lcnc::process::DeviceCommandResult{connected,
                connected ? QString()
                          : QObject::tr("SimulatorCMHP session disconnected unexpectedly")};
        }, TaskPriority::Stop,
        [&result, &completed](const lcnc::process::DeviceCommandResult& completion) {
            result = completion;
            completed.release();
        }) || !completed.tryAcquire(1, 30000)) {
            (void)releaseRuntimeOnExecutor();
            (void)queue.shutdown(30000);
            return fail(QStringLiteral("SimulatorCMHP iteration %1 timed out")
                            .arg(iteration + 1));
        }
        if (!result.success) {
            (void)releaseRuntimeOnExecutor();
            (void)queue.shutdown(30000);
            return fail(QStringLiteral("SimulatorCMHP iteration %1 failed: %2")
                            .arg(iteration + 1).arg(result.error));
        }
    }

    // Destroy ACS-owned controller objects on their sole executor thread.
    if (!releaseRuntimeOnExecutor() || !queue.shutdown(30000))
        return fail(QStringLiteral("SimulatorCMHP executor shutdown failed"));
    return 0;
}
