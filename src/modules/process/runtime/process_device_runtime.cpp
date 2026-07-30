#include "modules/process/runtime/process_device_runtime.h"

#include "modules/process/system/data_type.h"
#include "modules/process/system/regex_patterns.h"
#include "core/logging/logger.h"
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
