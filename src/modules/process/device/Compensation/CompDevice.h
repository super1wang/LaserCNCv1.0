#pragma once

#include "MessageCode.h"
#include "Settings.h"
#include "Expression.h"
#include <QStringList>
#include <map>

using std::map;
using Comps::CompValue;

class CompDevice
{
public:
	// 对存储的补偿值基本操作 
	void					SetCompValue(const QString& qstrName, const CompValue& value);
	bool					GetCompValue(const QString& qstrName, CompValue& value);
	bool					IsCompIndex(const QString& qstrName);

	QStringList				GetCompList();
	map<QString, CompValue>	GetComps();
	void					CleanComps();

public:
	// 获取补偿值的方法
	virtual const string&	GetName() = 0;
	virtual bool			IsInited() = 0;
	virtual	ErrorCode		SetSpecialTable(bool bReconnect = false) = 0;				// 下发参数并连接
	virtual ErrorCode		InitMeasurement() = 0;				// 初始化测量模块
	virtual ErrorCode		MeasurementComp(CompValue& cValue, int iIndex = 0) = 0;	// 获取测量值

protected:
	map<QString, CompValue>		map_Comps;	// 补偿值索引
};
