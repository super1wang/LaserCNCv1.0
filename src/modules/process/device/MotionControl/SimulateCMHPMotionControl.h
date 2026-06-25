#pragma once
#include "ACSMotionControl.h"

/// 仿真器 CMHP：Connect() 调用 acsc_OpenCommSimulator() 开启本地模拟，
/// Disconnect() 调用 acsc_CloseSimulator() 关闭。
/// 与 ACSMotionControl（真实网络连接 acsc_OpenCommEthernetTCP）互斥。
class SimulateCMHPMotionControl : public ACSMotionControl
{
public:
	SimulateCMHPMotionControl();
	~SimulateCMHPMotionControl() override;
	bool Connect() override;
	bool Disconnect() override;
};
