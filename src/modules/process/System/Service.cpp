#include "Service.h"
#include "DataType.h"

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
    m_pMotionControl = m_MCFactory.GetMotionController(strDevice);
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
    table tabToolIndex = SETTINGS->GetTable(SettingSection::Tool, "ToolIndex");
    string strToolIndex, strToolName;
    for (int i = 0; i < (int)tabToolIndex.size() - 1; i++)
    {
        strToolIndex  = "sTool_" + std::to_string(i);
        strToolName   = tabToolIndex[strToolIndex].as_string();
        table tabTool = SETTINGS->GetTable(SettingSection::Tool, strToolName);

        Tool* curtool = new Tool();
        // Tool::SetFromTable stub;
        m_ToolFactory.SetTool(i, *curtool);
        delete curtool;
        curtool = nullptr;
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
