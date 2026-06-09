#ifndef _CUTTING_NORMAL_
#define _CUTTING_NORMAL_

#include "CuttingDevice.h"
#include <QList>

#define MAX_FLIGHT_CUTTING_BATCH_SIZE  10000

class CuttingNormal : public CuttingDevice
{
public:
	CuttingNormal();

	virtual bool	StartCutting(const map<QString, QString>& maps);
	virtual bool	CuttingThread(double dOffsetX = 0, double dOffsetY = 0);

private:
	void	CreatCuttingData(const int iStart, const int iEnd);
	bool	BuildCuttingCommand(RS_Entity* curEntity, Tool& curTool, double dOffsetX = 0, double dOffsetY = 0);
	void	BuildFlightCuttingCommand(const QList<RS_Entity*>& entityList, double dOffsetX = 0, double dOffsetY = 0);		//FlightCutting
	bool	EntityCommand(CuttingDevice::COMMAND_TYPE ctype, RS_Entity* curEntity, Tool& curTool, double dOffsetX = 0, double dOffsetY = 0);
	bool	EntityCommandGTN(CuttingDevice::COMMAND_TYPE ctype, RS_Entity* curEntity, Tool& curTool, double dOffsetX = 0, double dOffsetY = 0);
	bool	EntityFlightCuttingCommand(CuttingDevice::COMMAND_TYPE ctype, RS_Entity* curEntity, Tool& curTool, double dOffsetX = 0, double dOffsetY = 0);
	bool    MovingCuttingHead(Tool& curTool, double ptFirstX = 0, double ptFirstY = 0);
	bool	ProcessAndSendFlightCuttingBlock(double dOffsetX = 0, double dOffsetY = 0);
	void	GTNStartCommand();
public:
	QList<RS_Entity*>					m_CuttingList;
	QList<RS_Entity*>::iterator			m_itrCurList;

private:
	int									m_iCurrentNum{0};
	int									m_iTotalNum{0};
	bool                                m_bChangeTool{false};
	string                              m_strName;
	

	QList<RS_Entity*>					m_FlightCuttingList;
	//创建一个计数器，专门为GTN切割使用，记录当前切割了多少条线段，达到一定数量后就发送一次切割命令，避免一次性发送过多命令导致GTN处理不过来
	int									m_iGTNCuttingCount{0};
	//s是否已执行StartCommand   专门为GTN使用
	bool                                m_bStartGTNCommand{false};
};

#endif
