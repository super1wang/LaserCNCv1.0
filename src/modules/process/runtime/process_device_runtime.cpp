#include "modules/process/runtime/process_device_runtime.h"

#include "modules/process/system/data_type.h"
#include "modules/process/system/regex_patterns.h"
#include "core/logging/logger.h"
#include "modules/process/settings/process_settings_service.h"
#include "modules/process/runtime/process_runtime_configuration.h"
#include "modules/process/runtime/i_motion_command_sink.h"
#include "modules/process/runtime/process_cutting_safety.h"
#include "modules/process/device/motion_control/simulate_cmhp_motion_control.h"
#if defined(LCNC_PROCESS_HAS_ACS) && LCNC_PROCESS_HAS_ACS
#include "modules/process/device/motion_control/acs_motion_control.h"
#endif
#if defined(LCNC_PROCESS_HAS_GTN) && LCNC_PROCESS_HAS_GTN
#include "modules/process/device/motion_control/gtn_motion_control.h"
#endif

#include <array>

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

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::moveRelative(
    Axis axis, double distance, double velocity)
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected())
        // 中文翻译：未连接控制器，请先连接设备
        return {false, QObject::tr("The controller is not connected, please connect the device first")};
    if (!m_motionControl->IsMotorCreated(axis))
        // 中文翻译：轴未注册
        return {false, QObject::tr("Axis not registered")};
    const bool ok = m_motionControl->MoveRelative(axis, distance, velocity);
    // 中文翻译：相对运动命令失败
    return {ok, ok ? QString() : QObject::tr("Relative motion command failed")};
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::moveAbsolute(
    Axis axis, double position, double velocity)
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected())
        // 中文翻译：未连接控制器，请先连接设备
        return {false, QObject::tr("The controller is not connected, please connect the device first")};
    if (!m_motionControl->IsMotorCreated(axis))
        // 中文翻译：轴未注册
        return {false, QObject::tr("Axis not registered")};
    const bool ok = m_motionControl->MoveAbsolute(axis, position, velocity);
    // 中文翻译：绝对运动命令失败
    return {ok, ok ? QString() : QObject::tr("Absolute motion command failed")};
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::jog(
    Axis axis, bool positive, double velocity)
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected())
        // 中文翻译：未连接控制器，请先连接设备
        return {false, QObject::tr("The controller is not connected, please connect the device first")};
    if (!m_motionControl->IsMotorCreated(axis))
        // 中文翻译：轴未注册
        return {false, QObject::tr("Axis not registered")};
    const bool ok = m_motionControl->Jog(axis, positive, velocity);
    // 中文翻译：连续运动命令失败
    return {ok, ok ? QString() : QObject::tr("Continuous motion command failed")};
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::stopAxis(Axis axis)
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected()
        || !m_motionControl->IsMotorCreated(axis)) {
        // 中文翻译：轴停止条件不满足
        return {false, QObject::tr("Axis stop conditions are not met")};
    }
    const bool ok = m_motionControl->StopMotion(axis);
    // 中文翻译：轴停止命令失败
    return {ok, ok ? QString() : QObject::tr("Axis stop command failed")};
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::stopAllMotion()
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected())
        // 中文翻译：运动控制器未连接
        return {false, QObject::tr("Motion controller not connected")};
    const bool ok = m_motionControl->StopMotion() && m_motionControl->StopAllBuffer();
    // 中文翻译：停止运动失败
    return {ok, ok ? QString() : QObject::tr("Stop motion failed")};
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::moveAxes(
    const QVector<Axis>& axes, const QVector<double>& positions, double velocity, bool relative)
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected())
        // 中文翻译：运动控制器未连接
        return {false, QObject::tr("Motion controller not connected")};
    if (QString::fromStdString(m_motionControl->GetName()) == QStringLiteral("GTN"))
        // 中文翻译：GTN 控制器暂不支持同步多轴运动，请改为顺序执行
        return {false, QObject::tr("The GTN controller does not currently support synchronous multi-axis motion. Please execute it sequentially instead.")};
    if (axes.size() != positions.size() || axes.isEmpty())
        // 中文翻译：同步多轴运动失败
        return {false, QObject::tr("Synchronized multi-axis motion failed")};
    vector<Axis> nativeAxes;
    vector<double> nativePositions;
    nativeAxes.reserve(static_cast<std::size_t>(axes.size()));
    nativePositions.reserve(static_cast<std::size_t>(positions.size()));
    for (int index = 0; index < axes.size(); ++index) {
        if (!m_motionControl->IsMotorCreated(axes.at(index)))
            // 中文翻译：轴未注册
            return {false, QObject::tr("Axis not registered")};
        nativeAxes.push_back(axes.at(index));
        nativePositions.push_back(positions.at(index));
    }
    const bool ok = relative
        ? m_motionControl->MoveMRelative(nativeAxes, nativePositions, velocity)
        : m_motionControl->MoveMAbsolute(nativeAxes, nativePositions, velocity);
    // 中文翻译：同步多轴运动失败
    return {ok, ok ? QString() : QObject::tr("Synchronized multi-axis motion failed")};
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::setAxisEnabled(Axis axis, bool enabled)
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected())
        // 中文翻译：运动控制器未连接
        return {false, QObject::tr("Motion controller not connected")};
    if (!m_motionControl->IsMotorCreated(axis))
        // 中文翻译：轴未注册
        return {false, QObject::tr("Axis not registered")};
    const bool ok = m_motionControl->SetAxisEnable(axis, enabled);
    // 中文翻译：轴使能切换失败
    return {ok, ok ? QString() : QObject::tr("Axis enable switching failed")};
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::setDigitalOutput(
    DigitalOUT output, bool value)
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected())
        // 中文翻译：运动控制器未连接
        return {false, QObject::tr("Motion controller not connected")};
    if (!m_motionControl->m_mapDigitalOUT.count(output))
        // 中文翻译：IO 输出切换失败
        return {false, QObject::tr("IO output switching failed")};
    const bool ok = m_motionControl->DigitalOutputSet(output, value ? 1 : 0);
    // 中文翻译：IO 输出切换失败
    return {ok, ok ? QString() : QObject::tr("IO output switching failed")};
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::setDigitalOutput(
    const QString& outputName, bool value)
{
    QString enumName = outputName.trimmed();
    if (enumName.startsWith(QLatin1Char('a')) && enumName.size() >= 2)
        enumName = enumName.mid(1);
    const auto output = enum_cast<DigitalOUT>(enumName.toStdString());
    if (!output.has_value())
        // 中文翻译：IO 输出切换失败
        return {false, QObject::tr("IO output switching failed")};
    return setDigitalOutput(output.value(), value);
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::setAnalogOutput(
    AnalogOUT output, double value, const QString& outputName)
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected())
        // 中文翻译：运动控制器未连接
        return {false, QObject::tr("Motion controller not connected")};
    if (!m_motionControl->m_mapAnalogOUT.count(output))
        // 中文翻译：模拟量输出 %1 未注册
        return {false, QObject::tr("Analog output %1 is not registered").arg(outputName)};
    const bool ok = m_motionControl->AnalogOutputSet(output, value);
    // 中文翻译：输出信号 %1 设置失败
    return {ok, ok ? QString() : QObject::tr("Output signal %1 setup failed").arg(outputName)};
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::homeAxes(
    const QStringList& axes, bool connected)
{
    const auto lock = lockDeviceAccess();
    MotionControl* const motionControl = connected ? m_motionControl.get() : nullptr;
    for (const QString& axisName : axes) {
        if (!motionControl)
            continue;

        const auto axis = enum_cast<Axis>(axisName.toStdString());
        if (!axis.has_value() || !motionControl->IsMotorCreated(axis.value())) {
            LCNC_WARN(lcnc::LogCode::Generic,
                      "ProcessDeviceRuntime::homeAxes: skip hardware home for axis {} "
                      "(extension or not registered)", axisName.toStdString());
            continue;
        }
        if (!motionControl->Home(axis.value())) {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "ProcessDeviceRuntime::homeAxes: hardware Home failed for axis {}",
                     axisName.toStdString());
            // 中文翻译：回零失败，请检查日志
            return {false, QObject::tr("Return to zero failed, please check the log")};
        }
    }
    return {};
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::moveToPreset(
    const QMap<QString, double>& targets, double velocity, const QString& positionName)
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected())
        // 中文翻译：未连接控制器，请先连接设备
        return {false, QObject::tr("The controller is not connected, please connect the device first")};

    for (auto it = targets.cbegin(); it != targets.cend(); ++it) {
        const auto axis = enum_cast<Axis>(it.key().toStdString());
        if (!axis.has_value() || !m_motionControl->IsMotorCreated(axis.value()))
            // 中文翻译：%1 轴未在控制器中创建
            return {false, QObject::tr("%1 axis was not created in the controller").arg(it.key())};
        if (!m_motionControl->IsEnabled(axis.value()))
            // 中文翻译：%1 轴当前未使能
            return {false, QObject::tr("%1 axis is not currently enabled").arg(it.key())};
        if (!m_motionControl->IsHomed(axis.value()))
            // 中文翻译：%1 轴尚未回零
            return {false, QObject::tr("%1 axis has not returned to zero yet").arg(it.key())};
        if (m_motionControl->IsAxisMoving(axis.value()))
            // 中文翻译：%1 轴正在运动
            return {false, QObject::tr("%1 axis is moving").arg(it.key())};
    }
    for (auto it = targets.cbegin(); it != targets.cend(); ++it) {
        const auto axis = enum_cast<Axis>(it.key().toStdString());
        if (!m_motionControl->MoveAbsolute(axis.value(), it.value(), velocity)) {
            (void)m_motionControl->StopMotion();
            // 中文翻译：%1 轴移动至%2失败
            return {false, QObject::tr("%1 axis movement to %2 failed").arg(it.key(), positionName)};
        }
    }
    return {};
}

lcnc::process::DeviceStatusSnapshot ProcessDeviceRuntime::pollStatus(
    const QStringList& axisNames, const QVector<QPair<QString, QString>>& digitalOutputs)
{
    lcnc::process::DeviceStatusSnapshot snapshot;
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected())
        return snapshot;
    snapshot.connected = true;
    snapshot.axes.reserve(axisNames.size());
    for (const QString& name : axisNames) {
        lcnc::process::DeviceAxisStatusSample sample;
        sample.name = name;
        const auto axis = enum_cast<Axis>(name.toStdString());
        if (axis.has_value() && m_motionControl->IsMotorCreated(axis.value())) {
            double position = 0.0;
            if (m_motionControl->GetActualPos(axis.value(), position)) {
                sample.pos = position;
                sample.enabled = m_motionControl->IsEnabled(axis.value());
                sample.valid = true;
            }
        }
        snapshot.axes.push_back(sample);
    }
    snapshot.digitalOutputs.reserve(digitalOutputs.size());
    for (const auto& entry : digitalOutputs) {
        lcnc::process::DeviceDigitalOutputSample sample;
        sample.channel = entry.first;
        sample.displayName = entry.second;
        QString enumName = entry.first;
        if (enumName.startsWith(QLatin1Char('a')) && enumName.size() >= 2)
            enumName = enumName.mid(1);
        const auto output = enum_cast<DigitalOUT>(enumName.toStdString());
        int value = 0;
        if (output.has_value() && m_motionControl->m_mapDigitalOUT.count(output.value())
            && m_motionControl->DigitalOutputGet(output.value(), value)) {
            sample.value = value != 0;
            sample.valid = true;
        }
        snapshot.digitalOutputs.push_back(sample);
    }
    return snapshot;
}

lcnc::process::DevicePeripheralSnapshot ProcessDeviceRuntime::pollPeripheralStatus()
{
    lcnc::process::DevicePeripheralSnapshot snapshot;
    const auto lock = lockDeviceAccess();
    if (!m_pLaserDevice)
        return snapshot;
    snapshot.deviceName = QString::fromStdString(m_pLaserDevice->GetName());
    snapshot.connected = m_pLaserDevice->IsConnected();
    snapshot.initialized = m_pLaserDevice->IsInited();
    snapshot.valid = true;
    if (snapshot.connected && snapshot.deviceName != QStringLiteral("Simulator")
        && snapshot.deviceName != QStringLiteral("AnalogControl")) {
        snapshot.diagnostic = QString::fromStdString(m_pLaserDevice->GetTroubleshooting());
    }
    return snapshot;
}

bool ProcessDeviceRuntime::stopMotionAndSafeOutputs()
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected())
        return true;

    bool success = m_motionControl->StopMotion();
    success = m_motionControl->StopAllBuffer() && success;
    const std::array<DigitalOUT, 2> outputs = {DigitalOUT::Laser, DigitalOUT::Blow};
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

bool ProcessDeviceRuntime::readDigitalChannel(const QString& channel,
                                              bool* value,
                                              QString* errorMessage)
{
    if (!value)
        return false;
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected()) {
        // 中文翻译：运动控制器未连接
        if (errorMessage) *errorMessage = QObject::tr("Motion controller not connected");
        return false;
    }

    QString enumName = channel.trimmed();
    if (enumName.startsWith(QLatin1Char('a')) && enumName.size() >= 2)
        enumName = enumName.mid(1);
    int raw = 0;
    if (auto input = enum_cast<DigitalIN>(enumName.toStdString()); input.has_value()
        && m_motionControl->m_mapDigitalIN.count(input.value())
        && m_motionControl->DigitalInputGet(input.value(), raw)) {
        *value = raw != 0;
        return true;
    }
    if (auto output = enum_cast<DigitalOUT>(enumName.toStdString()); output.has_value()
        && m_motionControl->m_mapDigitalOUT.count(output.value())
        && m_motionControl->DigitalOutputGet(output.value(), raw)) {
        *value = raw != 0;
        return true;
    }
    // 中文翻译：通道未配置或读取失败: %1
    if (errorMessage) *errorMessage = QObject::tr("Channel not configured or read failed: %1").arg(channel);
    return false;
}

bool ProcessDeviceRuntime::readAnalogChannel(const QString& channel,
                                             double* value,
                                             QString* errorMessage)
{
    if (!value)
        return false;
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected()) {
        // 中文翻译：运动控制器未连接
        if (errorMessage) *errorMessage = QObject::tr("Motion controller not connected");
        return false;
    }

    QString enumName = channel.trimmed();
    if (enumName.startsWith(QLatin1Char('a')) && enumName.size() >= 2)
        enumName = enumName.mid(1);
    if (auto input = enum_cast<AnalogIN>(enumName.toStdString()); input.has_value()
        && m_motionControl->m_mapAnalogIN.count(input.value())
        && m_motionControl->AnalogInputGet(input.value(), *value)) {
        return true;
    }
    // 中文翻译：模拟量通道未配置或读取失败: %1
    if (errorMessage) *errorMessage = QObject::tr("Analog channel is not configured or failed to read: %1").arg(channel);
    return false;
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::runPreflight(
    const lcnc::process::ProcessPreflightRequest& request,
    lcnc::process::ProcessPreflightReport* report)
{
    const auto fail = [](const QString& error) {
        return lcnc::process::DeviceCommandResult{false, error};
    };
    if (!report)
        // 中文翻译：预检报告不可用
        return fail(QObject::tr("Preflight report unavailable"));

    const auto lock = lockDeviceAccess();
    MotionControl* const mc = m_motionControl.get();
    if (!mc || !mc->IsConnected())
        // 中文翻译：运动控制器未连接或连接已断开
        return fail(QObject::tr("Motion controller not connected or disconnected"));
    if (mc->ErrorOccurred())
        // 中文翻译：运动控制器存在异常，请清除故障后再加工
        return fail(QObject::tr("There is an abnormality in the motion controller. Please clear the fault before processing."));

    int fault = 0;
    if (!mc->IsAxisStatusNormal(fault))
        // 中文翻译：运动控制器状态读取失败，请检查控制器连接
        return fail(QObject::tr("Motion controller status reading failed, please check the controller connection"));
    if (fault != 0)
        // 中文翻译：运动控制器故障码: %1，请清除故障后再加工
        return fail(QObject::tr("Motion controller fault code: %1, please clear the fault before processing").arg(fault));

    QStringList disabledAxes;
    QStringList unregisteredAxes;
    for (const QString& axisName : request.axisNames) {
        const QString name = axisName.trimmed().toUpper();
        if (name.isEmpty() || name == QStringLiteral("BASE"))
            continue;
        const auto axis = enum_cast<Axis>(name.toStdString());
        if (!axis.has_value())
            continue;
        if (!mc->IsMotorCreated(axis.value())) {
            unregisteredAxes.append(name);
            continue;
        }
        double position = 0.0;
        if (mc->GetActualPos(axis.value(), position))
            report->axisPositions.insert(name, position);
        const bool enabled = mc->IsEnabled(axis.value());
        report->axisEnabled.insert(name, enabled);
        if (!enabled)
            disabledAxes.append(name);
    }
    if (!unregisteredAxes.isEmpty())
        // 中文翻译：运动控制器轴系未注册: %1
        return fail(QObject::tr("Motion controller axis is not registered: %1")
                        .arg(unregisteredAxes.join(QObject::tr("，"))));
    if (!disabledAxes.isEmpty())
        // 中文翻译：运动控制器轴系未使能: %1
        return fail(QObject::tr("Motion controller axis is not enabled: %1")
                        .arg(disabledAxes.join(QObject::tr("，"))));

    LaserDevice* const laser = m_pLaserDevice;
    if (!laser)
        // 中文翻译：激光器未创建，请先连接设备
        return fail(QObject::tr("The laser has not been created, please connect the device first"));
    const QString laserName = QString::fromStdString(laser->GetName());
    const bool analogLaser = laserName.compare(QStringLiteral("AnalogControl"), Qt::CaseInsensitive) == 0;
    if (!analogLaser && !laser->IsConnected())
        // 中文翻译：激光器未连接或连接已断开
        return fail(QObject::tr("Laser not connected or disconnected"));
    if (!laser->IsInited())
        // 中文翻译：激光器参数未初始化
        return fail(QObject::tr("Laser parameters not initialized"));
    if (!analogLaser) {
        const QString laserFault = QString::fromStdString(laser->GetTroubleshooting()).trimmed();
        if (!laserFault.isEmpty() && laserFault.compare(QStringLiteral("OK"), Qt::CaseInsensitive) != 0)
            // 中文翻译：激光器异常: %1
            return fail(QObject::tr("Laser exception: %1").arg(laserFault));
    }

    for (const auto& guard : request.digitalGuards) {
        if (!guard.enabled)
            continue;
        if (guard.channel.trimmed().isEmpty())
            // 中文翻译：%1监控通道未配置
            return fail(QObject::tr("%1 monitoring channel is not configured").arg(guard.title));
        QString enumName = guard.channel.trimmed();
        if (enumName.startsWith(QLatin1Char('a')) && enumName.size() >= 2)
            enumName = enumName.mid(1);
        const auto input = enum_cast<DigitalIN>(enumName.toStdString());
        if (!input.has_value() || !mc->m_mapDigitalIN.count(input.value()))
            // 中文翻译：%1状态获取失败: 通道未配置: %2
            return fail(QObject::tr("%1 status acquisition failed: Channel not configured: %2")
                            .arg(guard.title, guard.channel));
        int raw = 0;
        if (!mc->DigitalInputGet(input.value(), raw))
            // 中文翻译：%1状态获取失败: 通道读取失败: %2
            return fail(QObject::tr("%1 Status acquisition failed: Channel read failed: %2")
                            .arg(guard.title, guard.channel));
        if (raw != 0)
            // 中文翻译：%1异常
            return fail(QObject::tr("%1Exception").arg(guard.title));
    }

    for (const auto& guard : request.analogGuards) {
        if (!guard.enabled)
            continue;
        if (guard.channel.trimmed().isEmpty())
            // 中文翻译：%1通道未配置
            return fail(QObject::tr("%1 channel is not configured").arg(guard.title));
        QString enumName = guard.channel.trimmed();
        if (enumName.startsWith(QLatin1Char('a')) && enumName.size() >= 2)
            enumName = enumName.mid(1);
        const auto input = enum_cast<AnalogIN>(enumName.toStdString());
        if (!input.has_value() || !mc->m_mapAnalogIN.count(input.value()))
            // 中文翻译：%1通道未配置: %2
            return fail(QObject::tr("Channel %1 is not configured: %2").arg(guard.title, guard.channel));
        double value = 0.0;
        if (!mc->AnalogInputGet(input.value(), value))
            // 中文翻译：%1状态获取失败: %2
            return fail(QObject::tr("%1 status acquisition failed: %2").arg(guard.title, guard.channel));
        if (value < guard.threshold)
            // 中文翻译：%1异常: 当前值 %2 %3，阈值 %4 %3
            return fail(QObject::tr("%1Exception: Current value %2 %3, threshold %4 %3")
                            .arg(guard.title,
                                 QString::number(value, 'f', 3),
                                 guard.unit,
                                 QString::number(guard.threshold, 'f', 3)));
    }
    return {};
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::validateContourBoundary()
{
    const auto lock = lockDeviceAccess();
    MotionControl* const mc = m_motionControl.get();
    lcnc::process::ContourBoundaryHealth health;
    health.connected = mc && mc->IsConnected();
    if (!health.connected)
        return lcnc::process::evaluateContourBoundaryHealth(health);
    int fault = 0;
    health.statusReadable = mc->IsAxisStatusNormal(fault);
    health.faultCode = fault;
    if (!health.statusReadable || health.faultCode != 0)
        return lcnc::process::evaluateContourBoundaryHealth(health);
    for (Axis axis : mc->m_vecMotors) {
        if (!mc->IsMotorCreated(axis))
            health.missingAxes.append(QString::fromLatin1(enum_name(axis).data()));
        else if (!mc->IsEnabled(axis))
            health.disabledAxes.append(QString::fromLatin1(enum_name(axis).data()));
    }
    return lcnc::process::evaluateContourBoundaryHealth(health);
}

std::unique_ptr<lcnc::process::IMotionCommandSink> ProcessDeviceRuntime::createMotionSink(
    bool simulationMode,
    lcnc::process::PureSimulationToolpathTicker* simTicker,
    ProcessModule* processModule)
{
    const auto lock = lockDeviceAccess();
    return lcnc::process::MotionSinkFactory::create(
        simulationMode ? nullptr : m_motionControl.get(), simulationMode, simTicker, processModule);
}


void ProcessDeviceRuntime::setToolTable()
{
    clearToolData();

    // 从 TOOL 的 "ToolIndex" 子表中按 sTool_0, sTool_1, ... 键依次读取工具名，
    // 然后加载对应子表。规避 GetTable 返回表的大小受 sToolIndex / 其他杂键干扰。
    const table tabToolIndex = m_settings.rawTable(lcnc::process::ProcessConfigArea::Tools, "ToolIndex");
    int i = 0;
    while (true)
    {
        const std::string key = "sTool_" + std::to_string(i);
        auto it = tabToolIndex.find(key);
        if (it == tabToolIndex.end())
            break;

        const std::string strToolName = it->second.as_string();
        ++i;
        if (strToolName.empty())
            continue;

        const table tabTool = m_settings.rawTable(lcnc::process::ProcessConfigArea::Tools, QString::fromStdString(strToolName));
        Tool curtool;
        curtool.m_strName = strToolName;
        curtool.SetFromTable(tabTool);
        m_ToolFactory.SetTool(i - 1, curtool);
    }

    // 首次启动时 Settings UI 尚未打开，TOML 里一个工具都没有。
    // 以最保守的参数创建一个内置 Default，确保 ToolFactory 非空。
    if (i == 0)
    {
        Tool fallback;
        fallback.m_strName       = "Default";
        fallback.m_dLineVelocity  = 600.0;   // 10 mm/s — 与 sanitizedDefaultTool 一致
        fallback.m_dLineAcc       = 100.0;
        fallback.m_dLineJerk      = 1000.0;
        fallback.m_dIdleXYAccDec  = 100.0;
        fallback.m_dIdleXYJerk    = 1000.0;
        fallback.m_dLaserEnergy   = 20.0;
        fallback.m_dJunctionVelocity = 1.0;
        fallback.m_dJunctionAngle    = 1.0;
        fallback.m_dXsegEndVelocity  = 1.0;
        fallback.m_dIdleZHeight   = 0.0;
        fallback.m_dCuttingHeight = 0.0;
        fallback.m_strDirectionX  = "X";
        fallback.m_strDirectionY  = "Y";
        m_ToolFactory.SetTool(0, fallback);
    }
}

void ProcessDeviceRuntime::clearToolData()
{
    m_ToolFactory.ToolClear();
}

void ProcessDeviceRuntime::setMotionControlTable(const table& table_MotionControl)
{
    const auto lock = lockDeviceAccess();
    if (!table_MotionControl.size())
    {
        string strMotionControl;
        strMotionControl = m_settings.rawValue(lcnc::process::ProcessConfigArea::Devices, "MotionControl", "sType", "SimulatorCMHP").toString().toStdString();
        if (m_strMotionControl != strMotionControl)
        {
            if (m_motionControl)
                m_motionControl->Disconnect();
            setMotionControl(strMotionControl);
        }
        if (!m_motionControl) {
            LCNC_ERR(lcnc::LogCode::InternalUnexpectedState,
                     "Process ProcessDeviceRuntime: cannot apply motion table without a controller");
            return;
        }
        m_motionControl->setMotionControlTable();
    }
    else
    {
        if (!m_motionControl) {
            LCNC_ERR(lcnc::LogCode::InternalUnexpectedState,
                     "Process ProcessDeviceRuntime: cannot apply explicit motion table without a controller");
            return;
        }
        m_motionControl->setMotionControlTable(table_MotionControl);
    }
}

void ProcessDeviceRuntime::setDigitalTable(const table& table_Digital)
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl) {
        LCNC_WARN(lcnc::LogCode::InternalUnexpectedState,
                  "Process ProcessDeviceRuntime: ignored digital IO table because no controller is available");
        return;
    }
    if (!table_Digital.size())
        m_motionControl->setDigitalTable();
    else
        m_motionControl->setDigitalTable(table_Digital);
}

void ProcessDeviceRuntime::setAnalogTable(const table& table_Analog)
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl) {
        LCNC_WARN(lcnc::LogCode::InternalUnexpectedState,
                  "Process ProcessDeviceRuntime: ignored analog IO table because no controller is available");
        return;
    }
    if (!table_Analog.size())
        m_motionControl->setAnalogTable();
    else
        m_motionControl->setAnalogTable(table_Analog);
}

void ProcessDeviceRuntime::setLaserTable(const table& table_Laser)
{
    const auto lock = lockDeviceAccess();
    if (!table_Laser.size())
    {
        string strLaserDevice;
        strLaserDevice = m_settings.rawValue(lcnc::process::ProcessConfigArea::Devices, "Laser", "sType", "Simulator").toString().toStdString();
        if (m_strLaserDevice != strLaserDevice)
        {
            if (m_pLaserDevice)
                m_pLaserDevice->Disconnect();
            setLaserDevice(strLaserDevice);
        }

        if (!m_pLaserDevice) {
            LCNC_ERR(lcnc::LogCode::InternalUnexpectedState,
                     "Process ProcessDeviceRuntime: cannot apply laser table without a laser device");
            return;
        }

        if (m_pLaserDevice->GetName() == "AnalogControl")
        {
            double dResolution;
            dResolution = m_settings.rawValue(lcnc::process::ProcessConfigArea::Devices, "Laser", "fResolution", 0.0).toDouble();
            if (dResolution > 0)
            {
                double dEnergy = 0;
                dEnergy = m_settings.rawValue(lcnc::process::ProcessConfigArea::Devices, "Laser", "fEnergy", 0.0).toDouble();
                double dValue = dEnergy / 100.0 * dResolution;
                if (m_motionControl)
                    m_motionControl->AnalogOutputSet(AnalogOUT::Laser, dValue, true);
            }
        }
        else
            m_pLaserDevice->setLaserTable();
    }
    else
    {
        if (!m_pLaserDevice) {
            LCNC_ERR(lcnc::LogCode::InternalUnexpectedState,
                     "Process ProcessDeviceRuntime: ignored explicit laser table because no laser device is available");
            return;
        }
        m_pLaserDevice->setLaserTable(table_Laser);
    }
}

void ProcessDeviceRuntime::setGasTable(const table& table_Gas)
{
    const auto lock = lockDeviceAccess();
    table tGas = table_Gas;
    if (!table_Gas.size())
        tGas = m_settings.rawTable(lcnc::process::ProcessConfigArea::Operations);

    if (tGas.count("Gas") || tGas.count("GasSetting"))
    {
        double dPressure;
        int iConversions;
        dPressure = m_settings.rawValue(lcnc::process::ProcessConfigArea::Operations, "Gas", "fPressure", 0.0).toDouble();
        iConversions = m_settings.rawValue(lcnc::process::ProcessConfigArea::Operations, "GasSetting", "iConversions", 0).toInt();
        if (m_motionControl)
            m_motionControl->AnalogOutputSet(AnalogOUT::Pressure, iConversions / 2.0 * dPressure);
    }
}
