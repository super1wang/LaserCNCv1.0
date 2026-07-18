#include "RegexPatterns.h"
#include "Service.h"
#include "DataType.h"
#include "core/logging/logger.h"
#include "modules/process/settings/process_settings_service.h"
#include "modules/process/runtime/process_runtime_configuration.h"
#include "modules/process/device/MotionControl/SimulateCMHPMotionControl.h"
#if defined(LCNC_PROCESS_HAS_ACS) && LCNC_PROCESS_HAS_ACS
#include "modules/process/device/MotionControl/ACSMotionControl.h"
#endif
#if defined(LCNC_PROCESS_HAS_GTN) && LCNC_PROCESS_HAS_GTN
#include "modules/process/device/MotionControl/GTNMotionControl.h"
#endif

Service::Service(lcnc::process::ProcessSettingsService& settings,
                 lcnc::process::ProcessRuntimeConfiguration& runtimeConfiguration)
    : m_settings(settings)
	, m_runtimeConfiguration(runtimeConfiguration)
	, m_LDFactory(settings)
{
    SetToolTable();
}

Service::~Service()
{
    (void)shutdownDevices();
}

bool Service::shutdownDevices()
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

void Service::SetMotionControl(string strName)
{
    const auto lock = lockDeviceAccess();
    string strDevice = strName;
    if (strDevice.empty())
        strDevice = configuredMotionControllerName();

    // 控制器必须随 Service 生命期销毁：禁止函数内 static
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
                 "Process Service: controller '{}' is unavailable in this build; fallback is forbidden",
                 strDevice);
		m_strMotionControl.clear();
		return;
    }
    m_strMotionControl = strDevice;
}

string Service::configuredMotionControllerName() const
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

bool Service::configuredControllerRequiresDevice() const
{
    return configuredMotionControllerName() != "Simulator";
}

void Service::SetLaserDevice(string strName)
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
    m_pLaserDevice = m_LDFactory.GetLaserDevice(strDevice);
    if (!m_pLaserDevice) {
        LCNC_ERR(lcnc::LogCode::InternalUnexpectedState,
                 "Process Service: laser device '{}' is unavailable in this build; fallback is forbidden",
                 strDevice);
        m_strLaserDevice.clear();
        return;
    }
    m_strLaserDevice = strDevice;
}


void Service::SetToolTable()
{
    ClearToolDate();

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

void Service::ClearToolDate()
{
    m_ToolFactory.ToolClear();
}

void Service::SetMotionControlTable(const table& table_MotionControl)
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
            SetMotionControl(strMotionControl);
        }
        if (!m_motionControl) {
            LCNC_ERR(lcnc::LogCode::InternalUnexpectedState,
                     "Process Service: cannot apply motion table without a controller");
            return;
        }
        m_motionControl->SetMotionControlTable();
    }
    else
    {
        if (!m_motionControl) {
            LCNC_ERR(lcnc::LogCode::InternalUnexpectedState,
                     "Process Service: cannot apply explicit motion table without a controller");
            return;
        }
        m_motionControl->SetMotionControlTable(table_MotionControl);
    }
}

void Service::SetDigitalTable(const table& table_Digital)
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl) {
        LCNC_WARN(lcnc::LogCode::InternalUnexpectedState,
                  "Process Service: ignored digital IO table because no controller is available");
        return;
    }
    if (!table_Digital.size())
        m_motionControl->SetDigitalTable();
    else
        m_motionControl->SetDigitalTable(table_Digital);
}

void Service::SetAnalogTable(const table& table_Analog)
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl) {
        LCNC_WARN(lcnc::LogCode::InternalUnexpectedState,
                  "Process Service: ignored analog IO table because no controller is available");
        return;
    }
    if (!table_Analog.size())
        m_motionControl->SetAnalogTable();
    else
        m_motionControl->SetAnalogTable(table_Analog);
}

void Service::SetLaserTable(const table& table_Laser)
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
            SetLaserDevice(strLaserDevice);
        }

        if (!m_pLaserDevice) {
            LCNC_ERR(lcnc::LogCode::InternalUnexpectedState,
                     "Process Service: cannot apply laser table without a laser device");
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
            m_pLaserDevice->SetLaserTable();
    }
    else
    {
        if (!m_pLaserDevice) {
            LCNC_ERR(lcnc::LogCode::InternalUnexpectedState,
                     "Process Service: ignored explicit laser table because no laser device is available");
            return;
        }
        m_pLaserDevice->SetLaserTable(table_Laser);
    }
}

void Service::SetGasTable(const table& table_Gas)
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
