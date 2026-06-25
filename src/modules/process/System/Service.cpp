#include "RegexPatterns.h"
#include "Service.h"
#include "DataType.h"
#include "modules/process/device/MotionControl/ACSMotionControl.h"
#include "modules/process/device/MotionControl/SimulateCMHPMotionControl.h"
#if defined(LCNC_PROCESS_HAS_GTN) && LCNC_PROCESS_HAS_GTN
#include "modules/process/device/MotionControl/GTNMotionControl.h"
#endif

Service::Service(void):
    m_bCuttingHeadShow(false)
    ,m_bShowDirection(false)
    ,m_bShowCuttingPath(false)
    ,m_bShowPathID(false)
{
    m_pMotionControl = nullptr;
    m_pLaserDevice   = nullptr;
    SetCuttingDevice("NormalCutting");
    SetToolTable();
}

void Service::SetMotionControl(string strName)
{
    string strDevice = strName;
    if (DT::IsSimulatMode())
    {
        strDevice = "Simulator";
    }

    if (strDevice.empty())
    {
        SETTINGS->GetKeyValue("sType", strDevice, SettingSection::MotionControl, "MotionControl");
    }

    // 直接构造控制器实例（P3：MCFactory 已删除）。
    // ACSMotionControl → 真实控制器 TCP 连接；SimulateCMHPMotionControl → 本地模拟器。
    static ACSMotionControl          s_AcsMotion;          // ACS / 兜底
    static SimulateCMHPMotionControl s_SimMotion;          // 仿真器
#if defined(LCNC_PROCESS_HAS_GTN) && LCNC_PROCESS_HAS_GTN
    static GTNMotionControl          s_GtnMotion;
#endif

    if (strDevice == "Simulator" || strDevice == "SimulatorCMHP") {
        m_pMotionControl = &s_SimMotion;
    }
    else if (strDevice == "ACS") {
        m_pMotionControl = &s_AcsMotion;
    }
#if defined(LCNC_PROCESS_HAS_GTN) && LCNC_PROCESS_HAS_GTN
    else if (strDevice == "GTN") {
        m_pMotionControl = &s_GtnMotion;
    }
#endif
    else {
        m_pMotionControl = &s_SimMotion;    // 兜底：未知控制器名落到仿真器，避免 null
    }
    m_strMotionControl = strDevice;
}

void Service::SetLaserDevice(string strName)
{
    string strDevice = strName;
    if (DT::IsSimulatMode())
    {
        strDevice = "Simulator";
    }

    if (strDevice.empty())
    {
        SETTINGS->GetKeyValue("sType", strDevice, SettingSection::Laser, "Laser");
    }
    m_pLaserDevice = m_LDFactory.GetLaserDevice(strDevice);
    m_strLaserDevice = strDevice;
}


void Service::SetToolTable()
{
    ClearToolDate();

    // 从 TOOL 的 "ToolIndex" 子表中按 sTool_0, sTool_1, ... 键依次读取工具名，
    // 然后加载对应子表。规避 GetTable 返回表的大小受 sToolIndex / 其他杂键干扰。
    const table tabToolIndex = SETTINGS->GetTable(SettingSection::Tool, "ToolIndex");
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

        const table tabTool = SETTINGS->GetTable(SettingSection::Tool, strToolName);
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
    if (!table_MotionControl.size())
    {
        string strMotionControl;
        SETTINGS->GetKeyValue("sType", strMotionControl, SettingSection::MotionControl, "MotionControl");
        if (m_strMotionControl != strMotionControl)
        {
            if (m_pMotionControl)
                m_pMotionControl->Disconnect();
            SetMotionControl(strMotionControl);
        }
        m_pMotionControl->SetMotionControlTable();
    }
    else
    {
        m_pMotionControl->SetMotionControlTable(table_MotionControl);
    }
}

void Service::SetDigitalTable(const table& table_Digital)
{
    if (!table_Digital.size())
        m_pMotionControl->SetDigitalTable();
    else
        m_pMotionControl->SetDigitalTable(table_Digital);
}

void Service::SetAnalogTable(const table& table_Analog)
{
    if (!table_Analog.size())
        m_pMotionControl->SetAnalogTable();
    else
        m_pMotionControl->SetAnalogTable(table_Analog);
}

void Service::SetLaserTable(const table& table_Laser)
{
    if (!table_Laser.size())
    {
        string strLaserDevice;
        SETTINGS->GetKeyValue("sType", strLaserDevice, SettingSection::Laser, "Laser");
        if (m_strLaserDevice != strLaserDevice)
        {
            if (m_pLaserDevice)
                m_pLaserDevice->Disconnect();
            SetLaserDevice(strLaserDevice);
        }

        if (m_pLaserDevice->GetName() == "AnalogControl")
        {
            double dResolution;
            SETTINGS->GetKeyValue("fResolution", dResolution, SettingSection::Laser, "Laser");
            if (dResolution > 0)
            {
                double dEnergy = 0;
                SETTINGS->GetKeyValue("fEnergy", dEnergy, SettingSection::Laser, "Laser");
                double dValue = dEnergy / 100.0 * dResolution;
                m_pMotionControl->AnalogOutputSet(AnalogOUT::Laser, dValue, true);
            }
        }
        else
            m_pLaserDevice->SetLaserTable();
    }
    else
    {
        m_pLaserDevice->SetLaserTable(table_Laser);
    }
}

void Service::SetGasTable(const table& table_Gas)
{
    table tGas = table_Gas;
    if (!table_Gas.size())
        tGas = SETTINGS->GetTable(SettingSection::Gas);

    if (tGas.count("Gas") || tGas.count("GasSetting"))
    {
        double dPressure;
        int iConversions;
        SETTINGS->GetKeyValue("fPressure", dPressure, SettingSection::Gas, "Gas");
        SETTINGS->GetKeyValue("iConversions", iConversions, SettingSection::Gas, "GasSetting");
        if (m_pMotionControl)
            m_pMotionControl->AnalogOutputSet(AnalogOUT::Pressure, iConversions / 2.0 * dPressure);
    }
}

void Service::SetCompTable(const table& table_Comp)
{
    Q_UNUSED(table_Comp);
    // CompDevice removed
        // CompDevice removed
}

void Service::RedrawDrawing()
{
    // No-op: old QCad framework not available.
}
