#pragma once

#include <vector>
#include <QList>
#include <string>
#include "ToolFactory.h"
#include "rs_document.h"
#include "DataType.h"
#include "LaserDevice.h"

using std::vector;
using std::string;

class CuttingDevice
{
public:
	enum EntityState
	{
		NONE,		// 还原层所在色
		SELECT,		// RS_Color(255,   0,   0)	红	选择的
		CUTTING,	// RS_Color(  0, 255,   0)	绿	切割中
		CUTTED		// RS_Color(255, 255, 255)	白	切割完
	};

	enum COMMAND_TYPE
	{
		COMMAND_BEGIN, COMMADN_MID, COMMAND_END, COMMAND_ALL
	};

public:
	virtual void	AxisBoundUpdate(const Tool& curTool);

	virtual void	SetDirectionX(QString qstr);
	virtual Axis	GetDirectionX();

	virtual void	SetDirectionY(QString qstr);
	virtual Axis	GetDirectionY();

	virtual void	LaserValueChange(const Tool& curTool);
	virtual void	LaserValueChange(const LaserParameter& parameter);
	virtual void    LaserEnergySet(double dEnergy);

	virtual bool	WaitTime(int iWaitTime);				// 等待时间，单位毫秒，内置暂停、停止检测，停止等待返回false 
	virtual bool	IsOffsetCutting(int iOverTime = 0);		// 暂停、停止监听，附带超时检测，单位s，缺省则不启用

	virtual void	ResetLeftLimitPos();					// 重设图纸最左侧坐标
	virtual void	SetEntitysLeftLimitPos();				// 设置切割部分最左侧坐标
	virtual double	GetEntitysLeftLimitPos();				// 获取切割部分最左侧坐标

	virtual void	GetCuttingBound(RS_Vector& vecMin, RS_Vector& vecMax);
	virtual void	GetCuttingBound(RS_Vector& vecMin, RS_Vector& vecMax, QList<RS_Entity*>& qlistCut);
	virtual void	GetBound(RS_Vector& vecMin, RS_Vector& vecMax,const QList<RS_Entity*>& qlist);

	virtual bool	IsEntityList();											// 有无图纸数据
	virtual bool	IsSelectionCutting();									// 是否框选切割
	virtual int		SequenceListSize();										// 含有的切割顺序大小
	virtual bool	CheckEntityLimit(RS_Entity* e);							// 检测线条状态
	virtual void	ChangeEntityState(EntityState eState, RS_Entity* e);	// 修改线条状态/颜色

	virtual void	SetCuttingTestFlag(bool bFlag)	{ m_bCuttingTest = bFlag; };
	virtual bool	GetCuttingTestFlag()			{ return m_bCuttingTest;  };

public:
	virtual bool	StartCutting(const map<QString, QString>& maps) = 0;
	virtual bool	CuttingThread(double dOffsetX = 0, double dOffsetY = 0) = 0;


protected:
	bool			m_bCuttingTest;
	bool			m_bComputeMinX;
	double			m_dMinX;

private:
	Axis			m_eDirectionX;
	Axis			m_eDirectionY;
};
