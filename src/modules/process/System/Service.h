#pragma once
//#include <windows.h>
#include "Settings.h"
#include "MCFactory.h"
#include "LDFactory.h"
#include "COMPFactory.h"
#include "ToolFactory.h"
#include "CuttingFactory.h"
#include "SignalSource.h"

#include "MessageModule.h"
#include <QList>

#include <boost/thread.hpp>
#include <boost/atomic.hpp>

struct ButtonState
{
	QTimer*			pressTimer;
	QElapsedTimer	elapsedTimer;
	bool			bLongPress;
	bool			bPressed;
};

class  Service
{
public:
	Service(void);

public:

	Settings* GetSETTING()							{ return &m_Settings; };

	void SetMotionControl(string strName = "");
	MotionControl* GetMotionControl()				{ return m_pMotionControl; };
	
	void SetLaserDevice(string strName = "");
	LaserDevice* GetLaserDevice()					{ return m_pLaserDevice; };
	
	void SetCompDevice(string strName = "");
	CompDevice* GetCompDevice()						{ return m_pCompDevice; };

	void SetCuttingDevice(string);
	CuttingDevice* GetCuttingDevice()				{ return m_pCuttingDevice; };

	void SetCuttingHeadShow(bool bFlag)				{ m_bCuttingHeadShow = bFlag; };
	bool GetCuttingHeadShow()						{ return m_bCuttingHeadShow; };

	void SetShowDirectionFlag(bool bFlag)			{ m_bShowDirection = bFlag; };
	bool GetShowDirectionFlag()						{ return m_bShowDirection; };

	void SetShowCuttingPath(bool bFlag)				{ m_bShowCuttingPath = bFlag; };
	bool GetShowCuttingPath()						{ return m_bShowCuttingPath; };

	void SetShowPathID(bool bFlag)					{ m_bShowPathID = bFlag; };
	bool GetShowPathID()							{ return m_bShowPathID; };

	void SetRedrawLayers(bool bFlag)				{ m_bRedrawLayers = bFlag; };
	bool GetRedrawLayers()							{ return m_bRedrawLayers; };

	SignalSource* GetSignalSource()					{ return &m_SignalSource; };

	void SetToolTable();
	void ClearToolDate();	//清空工具数据

	void SetMotionControlTable	(const table& table_MotionControl	= {});
	void SetDigitalTable		(const table& table_Digital			= {});
	void SetAnalogTable			(const table& table_Analog			= {});
	void SetLaserTable			(const table& table_Laser			= {});
	void SetSignalSourceTable	(const table& table_SerialPort		= {});
	void SetGasTable			(const table& table_Gas				= {});
	void SetCompTable			(const table& table_Comp			= {});

	void RedrawDrawing();	// 重绘图纸部分

private:
	Settings		m_Settings;

	MCFactory		m_MCFactory;
	MotionControl*	m_pMotionControl;

	LDFactory		m_LDFactory;
	LaserDevice*	m_pLaserDevice;

	COMPFactory		m_COMPFactory;
	CompDevice*		m_pCompDevice;

	CuttingFactory	m_CuttingFactory;
	CuttingDevice*	m_pCuttingDevice;

	ToolFactory		m_ToolFactory;

	SignalSource	m_SignalSource;

private:
	//切割头坐标的显示和隐藏，当为绘图界面时，切割头不再显示和刷新
	bool	m_bCuttingHeadShow;		//是否显示切割头的十字光标
	bool	m_bShowDirection;		//是否显示开口方向、路径
	bool	m_bShowCuttingPath;		//切割路径的绘制
	bool	m_bShowPathID;			//已加入路径特征的ID
	bool	m_bRedrawLayers;		//重绘整个图纸

	string	m_strMotionControl;		//运动控制器型号
	string	m_strLaserDevice;		//激光器型号
	string	m_strCompDevice;		//补偿类设备型号

	QString m_qstrDirectionX;
	QString m_qstrDirectionY;

};

