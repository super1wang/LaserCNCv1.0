#pragma once
#include "ACSMotionControl.h"

class  SimulateCMHPMotionControl : public ACSMotionControl
{
public:
	SimulateCMHPMotionControl();
	~SimulateCMHPMotionControl();
	bool Connect();
	bool Disconnect();
};