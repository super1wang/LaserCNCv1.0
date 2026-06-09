#include "Expression.h"
#include "TinyExpr/tinyexpr.h"

namespace
{
QString ReplaceIndependentX(const QString& expression, double xValue)
{
	QString replaced = expression;
	const QString xText = QString::number(xValue, 'f', 10);
	replaced.replace(QRegularExpression("(?<![A-Za-z0-9_])X(?![A-Za-z0-9_])",
		QRegularExpression::CaseInsensitiveOption), xText);
	return replaced;
}
}

ErrorCode Expression::GetResult(const QString& expression, QString& result)
{
	if (expression.isEmpty())
		return ErrorCode::ERROR_EXPRES_NONEEXPRESSION;

	// 处理max和min函数
	QString processedExpr = expression;

	// 正则表达式匹配max函数调用
	QRegularExpression maxRegex("max\\s*\\(([^\\(\\)]+)\\)", QRegularExpression::CaseInsensitiveOption);
	QRegularExpressionMatchIterator maxIt = maxRegex.globalMatch(processedExpr);

	while (maxIt.hasNext())
	{
		QRegularExpressionMatch maxMatch = maxIt.next();
		QString fullMatch = maxMatch.captured(0);
		QString argsStr = maxMatch.captured(1);

		// 分割参数
		QStringList args = argsStr.split(',', Qt::SkipEmptyParts);
		if (args.isEmpty())
		{
			result = QString("Invalid max function call: %1").arg(fullMatch);
			return ErrorCode::ERROR_EXPRES_EXPRESSIONINVALID;
		}

		// 计算每个参数的值
		QList<double> argValues;
		for (const QString& arg : args)
		{
			QString argResult;
			ErrorCode argCode = GetResult(arg.trimmed(), argResult);
			if (argCode != ErrorCode::ERROR_NONE)
			{
				result = QString("Error evaluating max argument: %1").arg(arg);
				return ErrorCode::ERROR_EXPRES_EXPRESSIONINVALID;
			}
			argValues.append(argResult.toDouble());
		}

		// 计算最大值
		double maxValue = argValues.first();
		for (double value : argValues)
		{
			if (value > maxValue)
				maxValue = value;
		}

		// 替换max函数调用为计算结果
		processedExpr.replace(fullMatch, QString::number(maxValue));
	}

	// 正则表达式匹配min函数调用
	QRegularExpression minRegex("min\\s*\\(([^\\(\\)]+)\\)", QRegularExpression::CaseInsensitiveOption);
	QRegularExpressionMatchIterator minIt = minRegex.globalMatch(processedExpr);

	while (minIt.hasNext())
	{
		QRegularExpressionMatch minMatch = minIt.next();
		QString fullMatch = minMatch.captured(0);
		QString argsStr = minMatch.captured(1);

		// 分割参数
		QStringList args = argsStr.split(',', Qt::SkipEmptyParts);
		if (args.isEmpty())
		{
			result = QString("Invalid min function call: %1").arg(fullMatch);
			return ErrorCode::ERROR_EXPRES_EXPRESSIONINVALID;
		}

		// 计算每个参数的值
		QList<double> argValues;
		for (const QString& arg : args)
		{
			QString argResult;
			ErrorCode argCode = GetResult(arg.trimmed(), argResult);
			if (argCode != ErrorCode::ERROR_NONE)
			{
				result = QString("Error evaluating min argument: %1").arg(arg);
				return ErrorCode::ERROR_EXPRES_EXPRESSIONINVALID;
			}
			argValues.append(argResult.toDouble());
		}

		// 计算最小值
		double minValue = argValues.first();
		for (double value : argValues)
		{
			if (value < minValue)
				minValue = value;
		}

		// 替换min函数调用为计算结果
		processedExpr.replace(fullMatch, QString::number(minValue));
	}

	QByteArray bytesExpr = processedExpr.toUtf8();
	const char* ccExpr = bytesExpr.constData();

	int iErr = 0;
	te_expr* expr = te_compile(ccExpr, nullptr, 0, &iErr);

	if (!expr)
	{
		result = QString("Unable to parse \"%1\"").arg(expression);
		return ErrorCode::ERROR_EXPRES_EXPRESSIONINVALID;
	}

	double evalResult = te_eval(expr);
	te_free(expr);

	result = QString::number(evalResult, 'f', 10);
	result.remove(QRegularExpression("\\.?0+$"));

	return ErrorCode::ERROR_NONE;
}

ErrorCode Expression::GetLinearEquationResult(const QString& equation, double xValue, QString& result)
{
	if (equation.trimmed().isEmpty())
		return ErrorCode::ERROR_EXPRES_NONEEXPRESSION;

	QString normalized = equation;
	normalized.remove(QRegularExpression("\\s+"));

	const int equalCount = normalized.count('=');
	if (equalCount != 1)
	{
		result = QString("Invalid equation \"%1\"").arg(equation);
		return ErrorCode::ERROR_EXPRES_EXPRESSIONINVALID;
	}

	const int equalIndex = normalized.indexOf('=');
	const QString left = normalized.left(equalIndex);
	const QString right = normalized.mid(equalIndex + 1);
	if (left.compare("Y", Qt::CaseInsensitive) != 0 || right.isEmpty())
	{
		result = QString("Invalid equation \"%1\"").arg(equation);
		return ErrorCode::ERROR_EXPRES_EXPRESSIONINVALID;
	}

	const QString replaced = ReplaceIndependentX(right, xValue);
	return GetResult(replaced, result);
}

ErrorCode Expression::GetResult(const QString& expression, const Comps::CompValue& cValue, QString& result)
{
	QString qstrExpr = expression;
	qstrExpr.replace("X", cValue.X, Qt::CaseInsensitive);
	qstrExpr.replace("Y", cValue.Y, Qt::CaseInsensitive);

	return GetResult(qstrExpr, result);
}

ErrorCode Expression::GetResult(const QString& expression, const std::map<QString, Comps::CompValue>& cMap, QString& result)
{
	QString qstrExpr;
	ErrorCode  eCode = Replacements(expression, cMap, qstrExpr);
	if (eCode != ErrorCode::ERROR_NONE)		return eCode;
	return GetResult(qstrExpr, result);
}

ErrorCode Expression::Compare(const QString& expression, const std::map<QString, Comps::CompValue>& cMap, QString& result)
{
	QString qstrExpr;
	ErrorCode eCode = Replacements(expression, cMap, qstrExpr);
	if (eCode != ErrorCode::ERROR_NONE)		return eCode;

	// 移除空格
	qstrExpr = qstrExpr.simplified();

	// 查找比较运算符的位置
	int opPos = -1;
	QString op;

	// 支持的比较运算符
	QStringList operators = { ">=", "<=", "==", "!=", ">", "<", "=" };
	for (const QString& currentOp : operators) {
		int pos = qstrExpr.indexOf(currentOp);
		if (pos != -1) {
			// 检查是否是真正的运算符（避免数字中的符号，如科学计数法 1.2e-3）
			bool isValid = true;

			// 检查运算符前后是否是有效的字符
			if (pos > 0) {
				QChar before = qstrExpr[pos - 1];
				// 运算符前不能是字母、数字或小数点（特殊情况：负号前面可以是运算符或开头）
				if (before.isLetterOrNumber() || before == '.' || before == 'E' || before == 'e') {
					// 处理特殊情况：比如 "a>=b" 是OK的，但 "1.2e-3" 中的 '-' 不是运算符
					// 检查是否科学计数法
					if (currentOp == "-" || currentOp == "+") {
						// 检查前一个字符是否是 'e' 或 'E'
						if (pos > 1 && (qstrExpr[pos - 1].toLower() == 'e')) {
							isValid = false;  // 是科学计数法，不是运算符
						}
					}
				}
			}

			if (isValid) {
				opPos = pos;
				op = currentOp;
				break;
			}
		}
	}

	// 没有运算符为无效解析
	if (opPos == -1)	
		return ErrorCode::ERROR_EXPRES_EXPRESSIONINVALID;

	// 计算左右两边的值
	QString qstrLResult, qstrRResult;
	eCode = GetResult(qstrExpr.left(opPos), qstrLResult);
	if (eCode != ErrorCode::ERROR_NONE)		return eCode;

	eCode = GetResult(qstrExpr.mid(opPos + op.length()), qstrRResult);
	if (eCode != ErrorCode::ERROR_NONE)		return eCode;

	double dLR = qstrLResult.toDouble();
	double dRR = qstrRResult.toDouble();
	
	// 根据运算符比较
	const double dE = 1e-10;  // 浮点数比较容差
	if (op == ">" || op == "=")
		result = dLR > dRR ? "T" : "F";
	else if (op == "<")
		result = dLR < dRR ? "T" : "F";
	else if (op == ">=")
		result = dLR >= dRR ? "T" : "F";
	else if (op == "<=")
		result = dLR <= dRR ? "T" : "F";
	else if (op == "==" || op == "=")
		result = fabs(dLR - dRR) < dE ? "T" : "F";
	else if (op == "!=")
		result = fabs(dLR - dRR) >= dE ? "T" : "F";
	
	return ErrorCode::ERROR_NONE;
}

ErrorCode Expression::Replacements(const QString& expression, const std::map<QString, Comps::CompValue>& cMap, QString& result)
{
	QString qstrExpr = expression;

	// 使用正则表达式查找所有[xxx.x]或[xxx.y]模式
	QRegularExpression regex("\\[([^\\[\\]\\.]+)\\.([xyXY])\\]", QRegularExpression::CaseInsensitiveOption);
	QRegularExpressionMatchIterator it = regex.globalMatch(expression);

	std::map<QString, QString> replacements; //公式中所有需要替换的值
	while (it.hasNext())
	{
		QRegularExpressionMatch match = it.next();
		QString fullMatch = match.captured(0);       // 如 "[Comp1.X]"
		QString compIndex = match.captured(1);       // 如 "Comp1"
		QString coord = match.captured(2).toLower(); // 如 "x" 或 "y"

		if (!cMap.count(compIndex))
		{
			result = QString("Invalid index \"%1\"").arg(compIndex);	// 输出不存在的索引
			return ErrorCode::ERROR_EXPRES_NONEINDEX;
		}
		// 获取对应的补偿值
		QString compValue;
		if (coord == "x")
			compValue = cMap.at(compIndex).X;
		else
			compValue = cMap.at(compIndex).Y;

		replacements[fullMatch] = compValue;
	}

	for (const auto& pair : replacements)
	{
		qstrExpr.replace(pair.first, pair.second);
	}
	result = qstrExpr;
	return ErrorCode::ERROR_NONE;
}
