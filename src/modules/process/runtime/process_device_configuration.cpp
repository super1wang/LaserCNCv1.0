#include "modules/process/runtime/process_device_runtime.h"

#include "core/logging/logger.h"
#include "modules/process/device/process_io_types.h"
#include "modules/process/settings/process_settings_service.h"

#include <string>

using lcnc::process::AnalogOUT;
using std::string;
using toml::table;

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
