#pragma once

#include "LDFactory.h"
#include "ToolFactory.h"

#include <QTimer>
#include <QElapsedTimer>

#include <memory>
#include "modules/process/device/MotionControl/MotionControl.h"
#include "modules/process/runtime/process_device_coordinator.h"

namespace lcnc::process { class ProcessSettingsService; class ProcessRuntimeConfiguration; }

struct ButtonState
{
    QTimer*         pressTimer{nullptr};
    QElapsedTimer   elapsedTimer;
    bool            bLongPress{false};
    bool            bPressed{false};
};

class Service
{
public:
    using DeviceLock = lcnc::process::ProcessDeviceCoordinator::Lease;
    Service(lcnc::process::ProcessSettingsService& settings,
            lcnc::process::ProcessRuntimeConfiguration& runtimeConfiguration);
    ~Service();

    void SetMotionControl(string strName = "");
    MotionControl* GetMotionControl() { return m_motionControl.get(); };
    string configuredMotionControllerName() const;
    bool configuredControllerRequiresDevice() const;
    /// Serializes all vendor SDK traffic for this Process runtime.
    DeviceLock lockDeviceAccess() { return m_deviceCoordinator.acquire(); }
    /// Stops outputs and disconnects owned devices in the only safe ownership order.
    bool shutdownDevices();

    void SetLaserDevice(string strName = "");
    LaserDevice* GetLaserDevice() { return m_pLaserDevice; };

    void SetToolTable();
    void ClearToolDate();

    void SetMotionControlTable(const table& table_MotionControl = {});
    void SetDigitalTable(const table& table_Digital = {});
    void SetAnalogTable(const table& table_Analog = {});
    void SetLaserTable(const table& table_Laser = {});
    void SetGasTable(const table& table_Gas = {});

private:
    lcnc::process::ProcessSettingsService& m_settings;
    lcnc::process::ProcessRuntimeConfiguration& m_runtimeConfiguration;
    std::unique_ptr<MotionControl> m_motionControl;
    lcnc::process::ProcessDeviceCoordinator m_deviceCoordinator;

    LDFactory       m_LDFactory;
    LaserDevice*    m_pLaserDevice{nullptr};

    ToolFactory     m_ToolFactory;

    string  m_strMotionControl;
    string  m_strLaserDevice;
};
