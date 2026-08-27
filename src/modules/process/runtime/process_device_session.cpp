#include "modules/process/runtime/process_device_runtime.h"

#include "core/logging/logger.h"
#include "modules/process/device/process_io_types.h"
#include "modules/process/settings/process_settings_service.h"
#include "modules/process/runtime/process_runtime_configuration.h"
#include "modules/process/device/motion_control/simulate_cmhp_motion_control.h"

#if defined(LCNC_PROCESS_HAS_ACS) && LCNC_PROCESS_HAS_ACS
#include "modules/process/device/motion_control/acs_motion_control.h"
#endif
#if defined(LCNC_PROCESS_HAS_GTN) && LCNC_PROCESS_HAS_GTN
#include "modules/process/device/motion_control/gtn_motion_control.h"
#endif

#include <array>
#include <exception>
#include <string>

using lcnc::process::DigitalOUT;
using std::string;

ProcessDeviceRuntime::ProcessDeviceRuntime(lcnc::process::ProcessSettingsService& settings,
                                           lcnc::process::ProcessRuntimeConfiguration& runtimeConfiguration)
    : m_settings(settings)
    , m_runtimeConfiguration(runtimeConfiguration)
    , m_LDFactory(settings)
{
    setToolTable();
}

ProcessDeviceRuntime::~ProcessDeviceRuntime()
{
    (void)shutdownDevices();
    std::lock_guard<std::mutex> lifetimeLock(m_motionControlLifetimeMutex);
    m_motionControl.reset();
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::connectMotionControllerSession(
    bool pureSimulation)
{
    setMotionControl(pureSimulation ? "Simulator" : "");
    if (!m_motionControl)
        // 中文翻译：运动控制器实例化失败；当前构建未启用所选控制器
        return {false, QObject::tr("Motion controller instantiation failed; the selected controller is not enabled for the current build")};
    if (!m_motionControl->Connect())
        // 中文翻译：运动控制器连接失败；禁止回退到纯软件仿真
        return {false, QObject::tr("Motion controller connection failed; falling back to pure software simulation is prohibited")};
    return {};
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::disconnectMotionControllerSession()
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected())
        return {};
    if (!m_motionControl->Disconnect())
        // 中文翻译：运动控制器断开失败
        return {false, QObject::tr("Motion controller disconnection failed")};
    return {};
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::connectDevices(
    bool pureSimulation, const std::function<void(int, const QString&)>& reportProgress)
{
    try {
        const auto fail = [this](const QString& error) {
            (void)shutdownDevices();
            return lcnc::process::DeviceCommandResult{false, error};
        };
        // 中文翻译：正在创建运动控制器
        reportProgress(10, QObject::tr("Creating motion controller"));
        // 中文翻译：正在连接运动控制器
        reportProgress(25, QObject::tr("Connecting motion controller"));
        const auto motionConnection = connectMotionControllerSession(pureSimulation);
        if (!motionConnection.success)
            return fail(motionConnection.error);
        // 中文翻译：正在初始化运动轴
        reportProgress(50, QObject::tr("Initializing motion axes"));
        m_motionControl->rebuildAxes();

        // 中文翻译：正在创建激光器
        reportProgress(65, QObject::tr("Creating laser"));
        setLaserDevice();
        if (!m_pLaserDevice) {
            // 中文翻译：激光器连接失败；已取消本次设备连接
            return fail(QObject::tr("Laser connection failed; this device connection has been canceled"));
        }
        // 中文翻译：正在连接激光器
        reportProgress(75, QObject::tr("Connecting laser"));
        if (!m_pLaserDevice->Connect()) {
            // 中文翻译：激光器连接失败；已取消本次设备连接
            return fail(QObject::tr("Laser connection failed; this device connection has been canceled"));
        }

        // 中文翻译：正在加载设备参数
        reportProgress(88, QObject::tr("Loading device parameters"));
        if (!pureSimulation) {
            setMotionControlTable();
            setDigitalTable();
            setAnalogTable();
        }
        setLaserTable();
        // 中文翻译：设备初始化完成
        reportProgress(100, QObject::tr("Device initialization completed"));
        return {};
    } catch (const std::exception& exception) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "ProcessDeviceRuntime::connectDevices failed: {}", exception.what());
        (void)shutdownDevices();
        return {false, QString::fromUtf8(exception.what())};
    } catch (...) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "ProcessDeviceRuntime::connectDevices threw an unknown exception");
        (void)shutdownDevices();
        // 中文翻译：设备连接失败，请检查设置
        return {false, QObject::tr("Device connection failed, please check settings")};
    }
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::disconnectDevices()
{
    if (!stopMotionAndSafeOutputs()) {
        // 中文翻译：设备断开前安全输出复位失败
        return {false, QObject::tr("Safety output reset fails before device disconnection")};
    }

    const auto lock = lockDeviceAccess();
    if (m_pLaserDevice)
        m_pLaserDevice->Disconnect();
    if (m_motionControl)
        m_motionControl->Disconnect();
    return {};
}

bool ProcessDeviceRuntime::stopMotionAndSafeOutputs()
{
    const auto lock = lockDeviceAccess();
    bool success = true;

    // Stop the laser device first. Controller digital outputs alone are not a
    // sufficient guarantee for lasers with an independent control channel.
    if (m_pLaserDevice && m_pLaserDevice->IsConnected()) {
        success = m_pLaserDevice->StopLaser() && success;
        success = m_pLaserDevice->StopAimingBeam() && success;
    }
    if (!m_motionControl || !m_motionControl->IsConnected())
        return success;

    success = m_motionControl->StopMotion() && success;
    success = m_motionControl->StopAllBuffer() && success;
    // Only laser-emission and assist-gas outputs are forced off by Stop.  Other
    // process IO (chuck and cooling, for example) keeps its commanded state.
    const std::array<DigitalOUT, 3> outputs = {
        DigitalOUT::Laser, DigitalOUT::Blow, DigitalOUT::Blow2};
    for (const DigitalOUT output : outputs) {
        if (m_motionControl->m_mapDigitalOUT.count(output)
            && !m_motionControl->DigitalOutputSet(output, 0)) {
            success = false;
        }
    }
    return success;
}

bool ProcessDeviceRuntime::shutdownDevices()
{
    const auto lock = lockDeviceAccess();
    bool success = true;
    if (m_pLaserDevice && m_pLaserDevice->IsConnected()) {
        success = m_pLaserDevice->StopLaser() && success;
        success = m_pLaserDevice->StopAimingBeam() && success;
        m_pLaserDevice->Disconnect();
    }
    if (m_motionControl && m_motionControl->IsConnected()) {
        success = m_motionControl->StopMotion() && success;
        success = m_motionControl->StopAllBuffer() && success;
        success = m_motionControl->Disconnect() && success;
    }
    return success;
}

void ProcessDeviceRuntime::setMotionControl(string strName)
{
    const auto lock = lockDeviceAccess();
    std::lock_guard<std::mutex> lifetimeLock(m_motionControlLifetimeMutex);
    string strDevice = strName;
    if (strDevice.empty())
        strDevice = configuredMotionControllerName();

    // 控制器必须随 ProcessDeviceRuntime 生命期销毁：禁止函数内 static
    // 让 ACS/GTN SDK 句柄活过 Process 的停机顺序。切换类型时先断开旧实例，再创建新实例。
    if (m_motionControl && m_strMotionControl == strDevice)
        return;
    if (m_motionControl && m_motionControl->IsConnected())
        m_motionControl->Disconnect();
    m_motionControl.reset();

    if (strDevice == "Simulator") {
        m_motionControl = std::make_unique<SimulateCMHPMotionControl>(m_settings, m_runtimeConfiguration);
    }
#if defined(LCNC_PROCESS_HAS_ACS) && LCNC_PROCESS_HAS_ACS
    else if (strDevice == "SimulatorCMHP") {
        m_motionControl = std::make_unique<SimulateCMHPMotionControl>(m_settings, m_runtimeConfiguration);
    }
    else if (strDevice == "ACS") {
        m_motionControl = std::make_unique<ACSMotionControl>(m_settings, m_runtimeConfiguration);
    }
#endif
#if defined(LCNC_PROCESS_HAS_GTN) && LCNC_PROCESS_HAS_GTN
    else if (strDevice == "GTN") {
        m_motionControl = std::make_unique<GTNMotionControl>(m_settings, m_runtimeConfiguration);
    }
#endif
    else {
        LCNC_ERR(lcnc::LogCode::InternalUnexpectedState,
                 "Process ProcessDeviceRuntime: controller '{}' is unavailable in this build; fallback is forbidden",
                 strDevice);
        m_strMotionControl.clear();
        return;
    }
    m_strMotionControl = strDevice;
}

string ProcessDeviceRuntime::configuredMotionControllerName() const
{
#if defined(LCNC_PROCESS_HAS_ACS) && LCNC_PROCESS_HAS_ACS
    constexpr const char* fallback = "SimulatorCMHP";
#elif defined(LCNC_PROCESS_HAS_GTN) && LCNC_PROCESS_HAS_GTN
    constexpr const char* fallback = "GTN";
#else
    constexpr const char* fallback = "Simulator";
#endif
    return m_settings.rawValue(lcnc::process::ProcessConfigArea::Devices,
                               "MotionControl", "sType", fallback)
        .toString().trimmed().toStdString();
}

string ProcessDeviceRuntime::activeMotionControllerName() const
{
    const auto lock = m_deviceCoordinator.acquire();
    return m_strMotionControl;
}

bool ProcessDeviceRuntime::configuredControllerRequiresDevice() const
{
    return configuredMotionControllerName() != "Simulator";
}

void ProcessDeviceRuntime::setLaserDevice(string strName)
{
    const auto lock = lockDeviceAccess();
    string strDevice = strName;
    if (m_runtimeConfiguration.simulationMode())
    {
        strDevice = "Simulator";
    }

    if (strDevice.empty())
    {
        strDevice = m_settings.rawValue(lcnc::process::ProcessConfigArea::Devices, "Laser", "sType", "Simulator").toString().toStdString();
    }
    m_pLaserDevice = m_LDFactory.laserDevice(strDevice);
    if (!m_pLaserDevice) {
        LCNC_ERR(lcnc::LogCode::InternalUnexpectedState,
                 "Process ProcessDeviceRuntime: laser device '{}' is unavailable in this build; fallback is forbidden",
                 strDevice);
        m_strLaserDevice.clear();
        return;
    }
    m_strLaserDevice = strDevice;
}
