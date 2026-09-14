#include "modules/process/runtime/process_device_runtime.h"

#include "core/logging/logger.h"
#include "modules/process/device/process_io_types.h"
#include "modules/process/runtime/device_shutdown_sequence.h"
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

namespace {

template<typename Action>
bool attemptDeviceAction(const char* operation, Action&& action)
{
    try {
        const bool success = action();
        if (!success) {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "process.device: operation={} result=failed", operation);
        }
        return success;
    } catch (const std::exception& exception) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "process.device: operation={} result=exception error={}",
                 operation, exception.what());
    } catch (...) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "process.device: operation={} result=unknown_exception", operation);
    }
    return false;
}

} // namespace

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
    const auto lock = lockDeviceAccess();
    if (!setMotionControl(pureSimulation ? "Simulator" : "")) {
        // 中文翻译：无法安全断开旧控制器或创建所选控制器；已取消连接。
        return {false, QObject::tr("Cannot safely disconnect the previous controller or create the selected controller; connection canceled")};
    }
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
    if (!stopMotionAndSafeOutputs()) {
        // 中文翻译：无法确认安全停止；保留控制器连接。
        return {false, QObject::tr("Safe stop could not be confirmed; the controller connection is preserved")};
    }
    if (!attemptDeviceAction("DisconnectMotionSession", [this] {
            return m_motionControl->Disconnect() && !m_motionControl->IsConnected();
        }))
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
            // A rejected controller switch has already attempted safe shutdown.
            // Do not retry it here and disconnect after that first failure.
            // 中文翻译：控制器切换失败时已尝试安全停止；不能在此自动重试并断开。
            return motionConnection;
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
            if (!setMotionControlTable()) {
                // 中文翻译：控制器参数应用失败；保留当前设备状态。
                return {false, QObject::tr("Controller parameter application failed; the current device state is preserved")};
            }
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
    const auto lock = lockDeviceAccess();
    const auto failure = lcnc::process::runDeviceShutdownSequence(
        [this] { return stopMotionAndSafeOutputs(); },
        [this] {
            return attemptDeviceAction("DisconnectLaser", [this] {
                if (!m_pLaserDevice || !m_pLaserDevice->IsConnected())
                    return true;
                m_pLaserDevice->Disconnect();
                return !m_pLaserDevice->IsConnected();
            });
        },
        [this] {
            return attemptDeviceAction("DisconnectMotion", [this] {
                if (!m_motionControl || !m_motionControl->IsConnected())
                    return true;
                return m_motionControl->Disconnect() && !m_motionControl->IsConnected();
            });
        });
    if (failure == lcnc::process::DeviceShutdownFailure::SafeStop) {
        // 中文翻译：无法确认安全停止；保留控制器连接。
        return {false, QObject::tr("Safe stop could not be confirmed; the controller connection is preserved")};
    }
    if (failure == lcnc::process::DeviceShutdownFailure::LaserDisconnect) {
        // 中文翻译：激光器断开失败；保留运动控制器连接。
        return {false, QObject::tr("Laser disconnection failed; the motion controller connection is preserved")};
    }
    if (failure == lcnc::process::DeviceShutdownFailure::MotionDisconnect) {
        // 中文翻译：运动控制器断开失败
        return {false, QObject::tr("Motion controller disconnection failed")};
    }
    return {};
}

bool ProcessDeviceRuntime::stopMotionAndSafeOutputs()
{
    const auto lock = lockDeviceAccess();
    bool success = true;

    // Stop the laser device first. Controller digital outputs alone are not a
    // sufficient guarantee for lasers with an independent control channel.
    if (m_pLaserDevice && m_pLaserDevice->IsConnected()) {
        success = lcnc::process::attemptAllSafetyActions(
            [this] { return attemptDeviceAction("StopLaser", [this] {
                return m_pLaserDevice->StopLaser();
            }); },
            [this] { return attemptDeviceAction("StopAimingBeam", [this] {
                return m_pLaserDevice->StopAimingBeam();
            }); });
    }
    if (!m_motionControl || !m_motionControl->IsConnected())
        return success;

    success = lcnc::process::attemptAllSafetyActions(
        [this] { return attemptDeviceAction("StopMotion", [this] {
            return m_motionControl->StopMotion();
        }); },
        [this] { return attemptDeviceAction("StopAllBuffer", [this] {
            return m_motionControl->StopAllBuffer();
        }); }) && success;
    // Only laser-emission and assist-gas outputs are forced off by Stop.  Other
    // process IO (chuck and cooling, for example) keeps its commanded state.
    const std::array<DigitalOUT, 3> outputs = {
        DigitalOUT::Laser, DigitalOUT::Blow, DigitalOUT::Blow2};
    for (const DigitalOUT output : outputs) {
        if (m_motionControl->m_mapDigitalOUT.count(output)
            && !attemptDeviceAction("SafeDigitalOutput", [this, output] {
                return m_motionControl->DigitalOutputSet(output, 0);
            })) {
            success = false;
        }
    }
    return success;
}

bool ProcessDeviceRuntime::shutdownDevices()
{
    return disconnectDevices().success;
}

bool ProcessDeviceRuntime::setMotionControl(string strName)
{
    const auto lock = lockDeviceAccess();
    string strDevice = strName;
    if (strDevice.empty())
        strDevice = configuredMotionControllerName();

    // 控制器必须随 ProcessDeviceRuntime 生命期销毁：禁止函数内 static
    // 让 ACS/GTN SDK 句柄活过 Process 的停机顺序。切换类型时先断开旧实例，再创建新实例。
    if (m_motionControl && m_strMotionControl == strDevice)
        return true;
    if (m_motionControl && !shutdownDevices()) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "process.device: operation=SwitchController previous={} requested={} action=preserve_previous reason=safe_shutdown_failed",
                 m_strMotionControl, strDevice);
        return false;
    }
    std::lock_guard<std::mutex> lifetimeLock(m_motionControlLifetimeMutex);
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
        return false;
    }
    m_strMotionControl = strDevice;
    return true;
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

bool ProcessDeviceRuntime::motionConnectionOpen() const
{
    const auto lock = m_deviceCoordinator.acquire();
    return m_motionControl && m_motionControl->IsConnected();
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
