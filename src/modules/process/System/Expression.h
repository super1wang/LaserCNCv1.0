#pragma once
#include "MessageCode.h"

#include <QDebug>
#include <QRegularExpression>
#include <vector>
#include <string>
#include <map>

namespace Comps
{
	struct CompValue
	{
		QString X;
		QString Y;
	};
};

class Expression
{
public:
	// 解析式计算
	static ErrorCode GetResult(const QString& expression, QString& result);
	static ErrorCode GetLinearEquationResult(const QString& equation, double xValue, QString& result);
	static ErrorCode GetResult(const QString& expression, const Comps::CompValue& cValue, QString& result);
	static ErrorCode GetResult(const QString& expression, const std::map<QString, Comps::CompValue>& cMap, QString& result);

	// 逻辑式计算
	static ErrorCode Compare(const QString& expression, const std::map<QString, Comps::CompValue>& cMap, QString& result);

private:
	// 替换解析式中的引用值
	static ErrorCode Replacements(const QString& expression, const std::map<QString, Comps::CompValue>& cMap, QString& result);
};
