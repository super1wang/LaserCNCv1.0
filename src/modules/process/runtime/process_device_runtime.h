#pragma once

#include "modules/process/device/laser/ld_factory.h"
#include "modules/process/tool/tool_factory.h"
#include "modules/process/runtime/device_command_queue.h"
#include "modules/process/runtime/process_preflight_types.h"

#include <QTimer>
#include <QElapsedTimer>
#include <QStringList>
#include <QString>
#include <QMap>
#include <QPair>
#include <QVector>

#include <memory>
#include <functional>
#include "modules/process/device/motion_control/motion_control.h"
#include "modules/process/runtime/process_device_coordinator.h"

namespace lcnc::process {
class IMotionCommandSink;
class ProcessSettingsService;
class ProcessRuntimeConfiguration;
class PureSimulationToolpathTicker;
}
class ProcessModule;

namespace lcnc::process {
struct DeviceAxisStatusSample { QString name; double pos{0.0}; bool enabled{false}; bool valid{false}; };
struct DeviceDigitalOutputSample { QString channel; QString displayName; bool value{false}; bool valid{false}; };
struct DeviceStatusSnapshot { bool connected{false}; QVector<DeviceAxisStatusSample> axes; QVector<DeviceDigitalOutputSample> digitalOutputs; };
struct DevicePeripheralSnapshot { QString deviceName; bool connected{false}; bool initialized{false}; QString diagnostic; bool valid{false}; };
}

struct ButtonState
{
    QTimer*         pressTimer{nullptr};
    QElapsedTimer   elapsedTimer;
    bool            bLongPress{false};
    bool            bPressed{false};
};

class ProcessDeviceRuntime
{
public:
    ProcessDeviceRuntime(lcnc::process::ProcessSettingsService& settings,
            lcnc::process::ProcessRuntimeConfiguration& runtimeConfiguration);
    ~ProcessDeviceRuntime();

    string activeMotionControllerName() const;
    string configuredMotionControllerName() const;
    bool configuredControllerRequiresDevice() const;
    /// Stop motion buffers and force process outputs to their safe state.
    bool stopMotionAndSafeOutputs();
    /// Stops outputs and disconnects owned devices in the only safe ownership order.
    bool shutdownDevices();

    /// Creates, connects and configures the selected controller and laser on
    /// the device worker. Progress callbacks must marshal to the GUI thread.
    lcnc::process::DeviceCommandResult connectDevices(
        bool pureSimulation,
        const std::function<void(int, const QString&)>& reportProgress);
    /// Opens only the configured motion-controller session.  Used by the SDK
    /// integration gate before application-level axis/laser configuration.
    lcnc::process::DeviceCommandResult connectMotionControllerSession(bool pureSimulation);
    lcnc::process::DeviceCommandResult disconnectMotionControllerSession();
    /// Safe device-worker disconnect: stop outputs first, then laser, then motion.
    lcnc::process::DeviceCommandResult disconnectDevices();

    /// Typed motion and IO operations. These methods are called only from the
    /// device queue worker; they keep vendor pointers and the defensive lease
    /// inside the runtime boundary.
    lcnc::process::DeviceCommandResult moveRelative(Axis axis, double distance, double velocity);
    lcnc::process::DeviceCommandResult moveAbsolute(Axis axis, double position, double velocity);
    lcnc::process::DeviceCommandResult jog(Axis axis, bool positive, double velocity);
    lcnc::process::DeviceCommandResult stopAxis(Axis axis);
    lcnc::process::DeviceCommandResult stopAllMotion();
    lcnc::process::DeviceCommandResult moveAxes(
        const QVector<Axis>& axes, const QVector<double>& positions, double velocity, bool relative);
    lcnc::process::DeviceCommandResult setAxisEnabled(Axis axis, bool enabled);
    lcnc::process::DeviceCommandResult setDigitalOutput(DigitalOUT output, bool value);
    lcnc::process::DeviceCommandResult setDigitalOutput(const QString& outputName, bool value);
    lcnc::process::DeviceCommandResult setAnalogOutput(AnalogOUT output, double value,
                                                       const QString& outputName = {});
    lcnc::process::DeviceCommandResult homeAxes(const QStringList& axes, bool connected);
    lcnc::process::DeviceCommandResult moveToPreset(
        const QMap<QString, double>& targets, double velocity, const QString& positionName);
    lcnc::process::DeviceStatusSnapshot pollStatus(
        const QStringList& axisNames, const QVector<QPair<QString, QString>>& digitalOutputs);
    lcnc::process::DevicePeripheralSnapshot pollPeripheralStatus();

    /// Typed IO access used by monitoring jobs after they have entered the
    /// device executor.  Callers never need to inspect vendor channel maps.
    bool readDigitalChannel(const QString& channel, bool* value, QString* errorMessage);
    bool readAnalogChannel(const QString& channel, double* value, QString* errorMessage);
    lcnc::process::DeviceCommandResult runPreflight(
        const lcnc::process::ProcessPreflightRequest& request,
        lcnc::process::ProcessPreflightReport* report);
    lcnc::process::DeviceCommandResult validateContourBoundary();
    std::unique_ptr<lcnc::process::IMotionCommandSink> createMotionSink(
        bool simulationMode,
        lcnc::process::PureSimulationToolpathTicker* simTicker,
        ProcessModule* processModule);

    void setToolTable();
    void clearToolData();

    void setMotionControlTable(const table& table_MotionControl = {});
    void setDigitalTable(const table& table_Digital = {});
    void setAnalogTable(const table& table_Analog = {});
    void setLaserTable(const table& table_Laser = {});
    void setGasTable(const table& table_Gas = {});

private:
    using DeviceLock = lcnc::process::ProcessDeviceCoordinator::Lease;
    DeviceLock lockDeviceAccess() { return m_deviceCoordinator.acquire(); }
    void setMotionControl(string strName = "");
    void setLaserDevice(string strName = "");
    lcnc::process::ProcessSettingsService& m_settings;
    lcnc::process::ProcessRuntimeConfiguration& m_runtimeConfiguration;
    std::unique_ptr<MotionControl> m_motionControl;
    mutable lcnc::process::ProcessDeviceCoordinator m_deviceCoordinator;

    LDFactory       m_LDFactory;
    LaserDevice*    m_pLaserDevice{nullptr};

    ToolFactory     m_ToolFactory;

    string  m_strMotionControl;
    string  m_strLaserDevice;
};
