#ifndef SIGNALSOURCE
#define SIGNALSOURCE

#include "SerialPort.h"
#include "Settings.h"
#include "MessageModule.h"
#include <string>
using namespace std;

class SignalSource : public SerialPort
{
public:
	SignalSource();

	ErrorCode SetSignalSourceTable();
	ErrorCode SetSignalSourceTable(const table& tableLaser);
	
	bool 	SetFrequency(double dFrequency);
	bool 	SetPulseWidth(double dPulseWidth);
	bool 	SetFrequencyAndPulseWidth(double dFrequency, double dPulseWidth);
	
	int 	GetFrequency();
	int 	GetPulseWidth();
	void 	GetFrequencyAndPulseWidth();
	virtual bool			IsAvailableData(const QByteArray& data);

private:
	bool 	SetFrequencyValue(double dFrequency);
	bool 	SetPulseWidthValue(double dPulseWidth);
	bool 	SetFrequencyAndPulseWidthValue(double dFrequency, double dPulseWidth);

	int 	GetFrequencyValue();
	int 	GetPulseWidthValue();
	void 	GetFrequencyAndPulseWidthValue();

private:
	double  m_dFrequency;
	double  m_dPulseWidth;

	string 	m_strFrequency;					// 信号源的频率
	string 	m_strPulseWidth;				// 信号源的脉宽
	string 	m_strFrequencyAndPulseWidth;	// 读取的频率和脉宽数据

};
#endif //SIGNALSOURCE