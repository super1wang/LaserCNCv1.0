#pragma once

#include "Settings.h"
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

    Settings* GetSETTING() { return &m_Settings; };

    void SetMotionControl(string strName = "");
    MotionControl* GetMotionControl() { return m_pMotionControl; };

    void SetLaserDevice(string strName = "");
    LaserDevice* GetLaserDevice() { return m_pLaserDevice; };

    void SetCompDevice(string = "") {}
    void* GetCompDevice() { return nullptr; }

    // CuttingDevice and SignalSource removed.
    void SetCuttingDevice(string) {}
    void* GetCuttingDevice() { return nullptr; }
    void* GetSignalSource()  { return nullptr; }

    void SetCuttingHeadShow(bool bFlag) { m_bCuttingHeadShow = bFlag; };
    bool GetCuttingHeadShow() { return m_bCuttingHeadShow; };

    void SetShowDirectionFlag(bool bFlag) { m_bShowDirection = bFlag; };
    bool GetShowDirectionFlag() { return m_bShowDirection; };

    void SetShowCuttingPath(bool bFlag) { m_bShowCuttingPath = bFlag; };
    bool GetShowCuttingPath() { return m_bShowCuttingPath; };

    void SetShowPathID(bool bFlag) { m_bShowPathID = bFlag; };
    bool GetShowPathID() { return m_bShowPathID; };

    void SetRedrawLayers(bool bFlag) { m_bRedrawLayers = bFlag; };
    bool GetRedrawLayers() { return m_bRedrawLayers; };

    void SetToolTable();
    void ClearToolDate();

    void SetMotionControlTable(const table& table_MotionControl = {});
    void SetDigitalTable(const table& table_Digital = {});
    void SetAnalogTable(const table& table_Analog = {});
    void SetLaserTable(const table& table_Laser = {});
    void SetSignalSourceTable(const table& = {}) {}
    void SetGasTable(const table& table_Gas = {});
    void SetCompTable(const table& table_Comp = {});

    void RedrawDrawing();

private:
    Settings        m_Settings;

    MotionControl*  m_pMotionControl{nullptr};

    LDFactory       m_LDFactory;
    LaserDevice*    m_pLaserDevice{nullptr};

    ToolFactory     m_ToolFactory;

    bool    m_bCuttingHeadShow{false};
    bool    m_bShowDirection{false};
    bool    m_bShowCuttingPath{false};
    bool    m_bShowPathID{false};
    bool    m_bRedrawLayers{false};

    string  m_strMotionControl;
    string  m_strLaserDevice;
    string  m_strCompDevice;

    QString m_qstrDirectionX;
    QString m_qstrDirectionY;
};
