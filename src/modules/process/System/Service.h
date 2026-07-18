#pragma once

#include "LDFactory.h"
#include "ToolFactory.h"

#include <QTimer>
#include <QElapsedTimer>

#include "modules/process/device/MotionControl/MotionControl.h"

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
    Service(void);

    void SetMotionControl(string strName = "");
    MotionControl* GetMotionControl() { return m_pMotionControl; };

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
    MotionControl*  m_pMotionControl{nullptr};

    LDFactory       m_LDFactory;
    LaserDevice*    m_pLaserDevice{nullptr};

    ToolFactory     m_ToolFactory;

    string  m_strMotionControl;
    string  m_strLaserDevice;
};
