#pragma once
#include "spdlog/spdlog.h"
#include "spdlog/async.h"
#include <QDateTime>
#include <QString>
#include <QStringList>
#include <memory>
#include <string>
#include <mutex>
#include <vector>

using std::string;
using std::mutex;
using std::shared_ptr;
using std::vector;

enum class LogType
{ 
	System, 
	Operator, 
	Process
};

enum class LogLevel
{
	Trace,
	Debug,
	Info,
	Warn,
	Error,
	Critical
};

#define LOG LogModule::instance()


#define LOG_SYS_TRACE(message)			LogModule::SystemLog	(LogLevel::Trace,		message)
#define LOG_SYS_DEBUG(message)			LogModule::SystemLog	(LogLevel::Debug,		message)
#define LOG_SYS_INFO(message)			LogModule::SystemLog	(LogLevel::Info,		message)
#define LOG_SYS_WARN(message)			LogModule::SystemLog	(LogLevel::Warn,		message)
#define LOG_SYS_ERROR(message)			LogModule::SystemLog	(LogLevel::Error,		message)
#define LOG_SYS_CRITICAL(message)		LogModule::SystemLog	(LogLevel::Critical,	message)

#define LOG_OPER_TRACE(message)			LogModule::OperatorLog	(LogLevel::Trace,		message)
#define LOG_OPER_DEBUG(message)			LogModule::OperatorLog	(LogLevel::Debug,		message)
#define LOG_OPER_INFO(message)			LogModule::OperatorLog	(LogLevel::Info,		message)
#define LOG_OPER_WARN(message)			LogModule::OperatorLog	(LogLevel::Warn,		message)
#define LOG_OPER_ERROR(message)			LogModule::OperatorLog	(LogLevel::Error,		message)
#define LOG_OPER_CRITICAL(message)		LogModule::OperatorLog	(LogLevel::Critical,	message)

#define LOG_PROCESS_TRACE(message)		LogModule::ProcessLog	(LogLevel::Trace,		message)
#define LOG_PROCESS_DEBUG(message)		LogModule::ProcessLog	(LogLevel::Debug,		message)
#define LOG_PROCESS_INFO(message)		LogModule::ProcessLog	(LogLevel::Info,		message)
#define LOG_PROCESS_WARN(message)		LogModule::ProcessLog	(LogLevel::Warn,		message)
#define LOG_PROCESS_ERROR(message)		LogModule::ProcessLog	(LogLevel::Error,		message)
#define LOG_PROCESS_CRITICAL(message)	LogModule::ProcessLog	(LogLevel::Critical,	message)

#define LOG_PROCESS_START()				LogModule::StartProcessLog()
#define LOG_PROCESS_STOP()				LogModule::StopProcessLog()
#define LOG_PROCESS_REFRESH()			LogModule::RefreshProcessLog()
#define LOG_REFRESH()					LogModule::RefreshLog()

struct CuttingLogSummary
{
	QString userName;
	QString permission;
	QString fileName;
	QString startTimeText;
	QString endTimeText;
	QDateTime startTime;
	QDateTime endTime;
	QString totalCutTime;
	int feedCount = 0;
	QStringList toolSummaryLines;
	QString logFilePath;
};

class LogModule {
public:
	LogModule();
	~LogModule();

	// 日志操作
	static void InitLog();
	static void StopLog();
	static void RefreshLog();

	// 函数宏指针
	static LogModule* instance();

	// 切换用户
	static void ChangeUser(const string& strPermission = m_strPermission, const string& strUserName = m_strUser);

	// 开始（创建）流程日志
	static void StartProcessLog();
	static void StopProcessLog();
	static void RefreshProcessLog();

	// 使用日志
	static void SystemLog(LogLevel eLevel, const string& strMessage);
	static void OperatorLog(LogLevel eLevel, const string& strMessage);
	static void ProcessLog(LogLevel eLevel, const string& strMessage);

	// 加工摘要
	static string GetCurrentProcessLogPath();
	static void WriteCuttingSummary(const QString& qstrFileName,
		const string& strStartTime,
		const string& strEndTime,
		const string& strTotalCutTime,
		int iFeedCount,
		const vector<string>& vecToolSummaryLines);
	static vector<CuttingLogSummary> GetCuttingLogSummaries();

private:
	static LogModule* uniqueInstance;

	static shared_ptr<spdlog::logger> log_System;
	static shared_ptr<spdlog::logger> log_Operator;
	static shared_ptr<spdlog::logger> log_Process;

public:
	static	string	m_strPermission;
	static	string	m_strUser;
	static	mutex	m_mutexUserChange;
	static	string	m_strCurrentProcessLogPath;
};

#include "spdlog/pattern_formatter.h"
// 权限标记
class PermissionFlagFormatter : public spdlog::custom_flag_formatter
{
public:
	void format(const spdlog::details::log_msg&, const std::tm&, spdlog::memory_buf_t& dest) override
	{
		dest.append(LogModule::m_strPermission.data(), LogModule::m_strPermission.data() + LogModule::m_strPermission.size());
	};
	std::unique_ptr<spdlog::custom_flag_formatter> clone() const override
	{
		return std::make_unique<PermissionFlagFormatter>();
	};
};

// 用户标记
class UserFlagFormatter : public spdlog::custom_flag_formatter
{
public:
	void format(const spdlog::details::log_msg&, const std::tm&, spdlog::memory_buf_t& dest) override
	{
		dest.append(LogModule::m_strUser.data(), LogModule::m_strUser.data() + LogModule::m_strUser.size());
	};
	std::unique_ptr<spdlog::custom_flag_formatter> clone() const override
	{
		return std::make_unique<UserFlagFormatter>();
	};
};
