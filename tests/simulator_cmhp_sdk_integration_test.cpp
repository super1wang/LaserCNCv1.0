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
                bool disconnected = true;
                if (*runtimeHolder) {
                    if (MotionControl* motion = (*runtimeHolder)->motionControl();
                        motion && motion->IsConnected()) {
                        disconnected = motion->Disconnect();
                    }
                }
                runtimeHolder->reset();
                return lcnc::process::DeviceCommandResult{
                    disconnected,
                    disconnected ? QString() : QObject::tr("SimulatorCMHP disconnect failed")};
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
    // deployed Simulator.prg.  The 100 subsequent executor commands validate
    // the active SDK session without repeatedly relaunching the vendor RPC
    // server, which has a multi-second process-release interval.
    for (int iteration = 0; iteration < 100; ++iteration) {
        QSemaphore completed;
        lcnc::process::DeviceCommandResult result;
        if (!queue.submit([runtimeHolder, iteration] {
            const auto& runtime = *runtimeHolder;
            MotionControl* motion = runtime->motionControl();
            if (iteration == 0) {
                runtime->setMotionControl("SimulatorCMHP");
                motion = runtime->motionControl();
            }
            if (!motion)
                return lcnc::process::DeviceCommandResult{false,
                    QObject::tr("SimulatorCMHP controller was not created")};
            if (iteration == 0 && !motion->Connect())
                return lcnc::process::DeviceCommandResult{false,
                    QObject::tr("acsc_OpenCommSimulator or Simulator.prg initialization failed")};
            return lcnc::process::DeviceCommandResult{
                motion->IsConnected(),
                motion->IsConnected() ? QString()
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
