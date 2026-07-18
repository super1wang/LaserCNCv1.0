/************************************************************************/
/*                            奥创激光器实现类                           */
/************************************************************************/

#ifndef _ULTRON_LASER_DEVICE_
#define _ULTRON_LASER_DEVICE_

#include "LaserDevice.h"
#include <string>
using namespace std;

class ULTRONLaserDevice : public LaserDevice
{
public:
	explicit ULTRONLaserDevice(lcnc::process::ProcessSettingsService& settings);
	virtual ErrorCode		SetLaserTable(const table& tableLaser = table{});

	virtual const string&	GetName();
	virtual bool			IsInited();
	virtual bool			IsAvailableData(const QByteArray& data);
	virtual bool			StartLaser();
	virtual bool			StopLaser(); 
	

	// 下发参数
	virtual bool			SetEnergy(double dEnergy);
	virtual bool			SetFrequency(double dFrequency);
	virtual bool			SetLaserParameter(const LaserParameter& parameter);
	virtual bool			SetPulseWidth(double dPulsePickerDivider);

	// 获取实际参数 
	virtual string			GetEnergy();
	virtual string			GetFrequency();
	virtual string			GetPulseWidth();
	
private:
	bool InitLaser();
	uint16_t crc16(const std::vector<uint8_t>& data);  //crc16校验
	uint16_t swap_bits(uint16_t& value);
	char* hextochs(char* ascii);
	string hexStrToDecString(const string& hexStr);
private:
	string					m_strName;
	bool					m_bIsInited;
	double					m_dMaxCurrent;
	double					m_dSimmerCurrent;
	int						m_iWaveShape;
	string					m_strTemperature;
public:
	// 非本型号函数
	virtual bool			StartAimingBeam() { return false; };
	virtual bool			StopAimingBeam() { return false; };

	//IPG
	virtual double			GetAveragePower() { return 0; };		//平均功率
	virtual string			GetTemperature() { return ""; };		//激光器温度
	virtual string			GetTroubleshooting() { return ""; };
};
#endif
