#pragma once

#include "modules/process/device/laser/ld_factory.h"
#include "modules/process/tool/tool_factory.h"

#include <QTimer>
#include <QElapsedTimer>
#include <QString>

#include <memory>
#include "modules/process/device/motion_control/motion_control.h"
#include "modules/process/runtime/process_device_coordinator.h"

namespace lcnc::process { class ProcessSettingsService; class ProcessRuntimeConfiguration; }

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
    using DeviceLock = lcnc::process::ProcessDeviceCoordinator::Lease;
    ProcessDeviceRuntime(lcnc::process::ProcessSettingsService& settings,
            lcnc::process::ProcessRuntimeConfiguration& runtimeConfiguration);
    ~ProcessDeviceRuntime();

    void setMotionControl(string strName = "");
    MotionControl* motionControl() { return m_motionControl.get(); };
    string activeMotionControllerName() const;
    string configuredMotionControllerName() const;
    bool configuredControllerRequiresDevice() const;
    /// Serializes all vendor SDK traffic for this Process runtime.
    DeviceLock lockDeviceAccess() { return m_deviceCoordinator.acquire(); }
    /// Stop motion buffers and force process outputs to their safe state.
    bool stopMotionAndSafeOutputs();
    /// Stops outputs and disconnects owned devices in the only safe ownership order.
    bool shutdownDevices();

    void setLaserDevice(string strName = "");
    LaserDevice* laserDevice() { return m_pLaserDevice; };

    /// Typed IO access used by monitoring jobs after they have entered the
    /// device executor.  Callers never need to inspect vendor channel maps.
    bool readDigitalChannel(const QString& channel, bool* value, QString* errorMessage);
    bool readAnalogChannel(const QString& channel, double* value, QString* errorMessage);

    void setToolTable();
    void clearToolData();

    void setMotionControlTable(const table& table_MotionControl = {});
    void setDigitalTable(const table& table_Digital = {});
    void setAnalogTable(const table& table_Analog = {});
    void setLaserTable(const table& table_Laser = {});
    void setGasTable(const table& table_Gas = {});

private:
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
