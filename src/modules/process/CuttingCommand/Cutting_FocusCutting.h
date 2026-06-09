#ifndef _CUTTING_FOCUSCUTTINGI_
#define _CUTTING_FOCUSCUTTINGI_

 #include "CuttingDevice.h"
#include <QList>

class FocusCutting : public CuttingDevice
{
public:
	FocusCutting();

	virtual bool StartCutting(const map<QString, QString>& maps);
	virtual bool CuttingThread(double dOffsetX = 0, double dOffsetY = 0);

	// Feeding位置为不框选前提下的双图层minx，框选为框选的该范围的minx，要和正常feeding做好隔离
	virtual void SetEntitysLeftLimitPos();

private:
	void CreatCuttingListDucument();
	bool BuildCuttingCommand(RS_Entity* curEntity, double dOffsetX = 0, double dOffsetY = 0);
	bool EntityCommand(CuttingDevice::COMMAND_TYPE ctype, RS_Entity* curEntity, Tool& curTool, double dOffsetX = 0, double dOffsetY = 0);
	
public:
	QList<RS_Entity*>					m_CuttingList;
	QList<RS_Entity*>::iterator			m_itrCurList;

private:
	int									m_iFocus0Sum;			// |线总数
	int									m_iCurCuttingNumber;	// 当前开口数
	double								m_dHighCompensate;		// 切割高度补偿
	int									m_iStepInterval;		// 递增间隔数
	double								m_dEnergyStep;			// 激光能量递进
	double								m_dFrequencyStep;		// 激光频率递进
	double								m_dPulseWidthStep;		// 激光脉宽递进
	double								m_dCuttingHighStep;		// 切割高度递进

	int									m_iCurrentNum{0};
	int									m_iTotalNum{0};
};

#endif
