#include "LogModule.h"
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QStringConverter>
#include <QTextStream>
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include <chrono>
#include <filesystem>
#include <algorithm>

namespace
{
const char* kProcessLogRootPath = "D:/Log/";
const char* kSummaryStartMarker = "=== Cutting Summary Start ===";
const char* kSummaryEndMarker = "=== Cutting Summary End ===";
const char* kUserNameLabel = "UserName:";
const char* kPermissionLabel = "Permission:";
const char* kFileNameLabel = "FileName:";
const char* kStartTimeLabel = "StartTime:";
const char* kEndTimeLabel = "EndTime:";
const char* kTotalCutTimeLabel = "TotalCutTime:";
const char* kFeedCountLabel = "FeedCount:";
const char* kToolsLabel = "Tools:";

QString payloadAfter(const QString& line, const QString& label)
{
	const int index = line.indexOf(label);
	if (index < 0) {
		return QString();
	}
	return line.mid(index + label.size()).trimmed();
}

QDateTime parseSummaryDateTime(const QString& text)
{
	if (text.isEmpty()) {
		return QDateTime();
	}

	QDateTime dateTime = QDateTime::fromString(text, Qt::ISODate);
	if (dateTime.isValid()) {
		return dateTime;
	}

	dateTime = QDateTime::fromString(text, "yyyy-MM-dd HH:mm:ss");
	if (dateTime.isValid()) {
		return dateTime;
	}

	return QDateTime::fromString(text, "yyyy-MM-dd HH:mm:ss.zzz");
}

vector<CuttingLogSummary> parseCuttingLogFile(const QString& logFilePath)
{
	vector<CuttingLogSummary> summaries;

	QFile file(logFilePath);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
		return summaries;
	}

	QTextStream stream(&file);
	stream.setEncoding(QStringConverter::Utf8);
	CuttingLogSummary currentSummary;
	bool inSummary = false;
	bool inTools = false;

	while (!stream.atEnd()) {
		const QString line = stream.readLine();

		if (line.contains(kSummaryStartMarker)) {
			currentSummary = CuttingLogSummary();
			currentSummary.logFilePath = logFilePath;
			inSummary = true;
			inTools = false;
			continue;
		}

		if (!inSummary) {
			continue;
		}

		if (line.contains(kSummaryEndMarker)) {
			currentSummary.startTime = parseSummaryDateTime(currentSummary.startTimeText);
			currentSummary.endTime = parseSummaryDateTime(currentSummary.endTimeText);
			summaries.push_back(currentSummary);
			currentSummary = CuttingLogSummary();
			inSummary = false;
			inTools = false;
			continue;
		}

		if (line.contains(kToolsLabel)) {
			inTools = true;
			continue;
		}

		if (inTools) {
			QString toolLine = line.trimmed();
			const int markerIndex = toolLine.lastIndexOf("- ");
			if (markerIndex >= 0) {
				toolLine = toolLine.mid(markerIndex + 2).trimmed();
			}
			if (!toolLine.isEmpty()) {
				currentSummary.toolSummaryLines.append(toolLine);
			}
			continue;
		}

		const QString userName = payloadAfter(line, kUserNameLabel);
		if (!userName.isEmpty()) {
			currentSummary.userName = userName;
			continue;
		}

		const QString permission = payloadAfter(line, kPermissionLabel);
		if (!permission.isEmpty()) {
			currentSummary.permission = permission;
			continue;
		}

		const QString fileName = payloadAfter(line, kFileNameLabel);
		if (!fileName.isEmpty()) {
			currentSummary.fileName = fileName;
			continue;
		}

		const QString startTime = payloadAfter(line, kStartTimeLabel);
		if (!startTime.isEmpty()) {
			currentSummary.startTimeText = startTime;
			continue;
		}

		const QString endTime = payloadAfter(line, kEndTimeLabel);
		if (!endTime.isEmpty()) {
			currentSummary.endTimeText = endTime;
			continue;
		}

		const QString totalCutTime = payloadAfter(line, kTotalCutTimeLabel);
		if (!totalCutTime.isEmpty()) {
			currentSummary.totalCutTime = totalCutTime;
			continue;
		}

		const QString feedCount = payloadAfter(line, kFeedCountLabel);
		if (!feedCount.isEmpty()) {
			bool ok = false;
			currentSummary.feedCount = feedCount.toInt(&ok);
			if (!ok) {
				currentSummary.feedCount = 0;
			}
		}
	}

	return summaries;
}
}

// 静态量初始化
shared_ptr<spdlog::logger> LogModule::log_System	= nullptr;
shared_ptr<spdlog::logger> LogModule::log_Operator	= nullptr;
shared_ptr<spdlog::logger> LogModule::log_Process	= nullptr;

string		LogModule::m_strPermission	= "Operator";
string		LogModule::m_strUser		= "User";
string		LogModule::m_strCurrentProcessLogPath;
mutex		LogModule::m_mutexUserChange;

void LogModule::InitLog()
{
	try
	{
		// 获取本地时间
		auto in_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
		std::tm local_time;
		localtime_s(&local_time, &in_time_t);

		// 创建日志目录
		string strYearMonth = fmt::format("{:%Y%m/}", local_time);	//子文件夹路径
		string strPath = "D:/Log/" + strYearMonth;
		std::filesystem::create_directories(strPath);

		// 修改用户
		ChangeUser(m_strPermission, m_strUser);
		spdlog::init_thread_pool(8192, 1);					// 初始化线程池（异步日志用）
		spdlog::flush_every(std::chrono::seconds(5));		// 设置每5秒自动刷新一次（异步日志用）

		// 构建日志结构
		auto formatter = std::make_unique<spdlog::pattern_formatter>();
		formatter->add_flag<PermissionFlagFormatter>('Z');  // %Z 表示权限
		formatter->add_flag<UserFlagFormatter>('U');		// %U 表示用户
		formatter->set_pattern("[%Y-%m-%d %H:%M:%S.%e][%l][%Z][%U] %v");
		auto formatterCopy = formatter->clone();

		// 构建系统日志（同步日志）
		string strSystemLog = strPath + "System/SystemLog" + ".log";
		auto sinkSystem = std::make_shared<spdlog::sinks::daily_file_sink_mt>(strSystemLog, 0, 0);	// 设置每日滚动，0:00刷新
		log_System = std::make_shared<spdlog::logger>("System_Logger", sinkSystem);					// 创建同步日志
		log_System->flush_on(spdlog::level::warn);			// 记录info及以上级别立即刷新
		log_System->set_level(spdlog::level::info);			// 最低记录等级info
		log_System->set_formatter(std::move(formatter));	// 应用日志结构
		log_System->info("****************  System Restart  ****************");


		// 构建操作日志（异步日志）
		string strOperatorLog = strPath + "Operator/OperatorLog" + ".log";
		auto sinkOperator = std::make_shared<spdlog::sinks::daily_file_sink_mt>(strOperatorLog, 0, 0);	// 设置每日滚动，0:00刷新
		log_Operator = std::make_shared<spdlog::async_logger>("Operator_Logger", sinkOperator, spdlog::thread_pool(), spdlog::async_overflow_policy::block);	// 创建异步日志	
		//log_Operator = std::make_shared<spdlog::logger>("Operator_Logger", sinkOperator);				// 创建同步日志
		log_Operator->flush_on(spdlog::level::warn);		// 记录Warn及以上级别立即刷新
		log_Operator->set_level(spdlog::level::info);		// 最低记录等级info
		log_Operator->set_formatter(std::move(formatterCopy));	// 应用日志结构
		spdlog::register_logger(log_Operator);				// 注册到全局
		log_Operator->info("****************  System Restart  ****************");

		// 虽然这两个日志均为每日滚动的日志，但如果不重新进入该函数，应该会产生创建日志文件夹路径不变的情况，但根据目前的文件夹路径，仅在跨月时出现，影响较小。
	}
	catch (const spdlog::spdlog_ex& ex)
	{
		//std::cerr << "Log initialization failed: " << ex.what() << std::endl;
	}
}

void LogModule::StopLog()
{
	StopProcessLog();

	if (log_Operator)
	{
		log_Operator->flush();
		spdlog::drop("Operator_Logger");
		log_Operator = nullptr;
	}

	if (log_System)
	{
		log_System->flush();
		spdlog::drop("System_Logger");
		log_System = nullptr;
	}
}

void LogModule::RefreshLog()
{
	if (log_Operator)
		log_Operator->flush();

	if (log_System)
		log_System->flush();
}

void LogModule::ChangeUser(const string& strPermission, const string& strUserName)
{
	std::lock_guard<mutex> lock(m_mutexUserChange);
	m_strPermission = strPermission;
	m_strUser = strUserName;
	//std::lock_guard<mutex> unlock(m_mutexUserChange);
}

void LogModule::StartProcessLog()
{
	try
	{
		if (log_Process)
		{
			StopProcessLog();
		}

		// 获取本地时间
		auto in_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
		std::tm local_time;
		localtime_s(&local_time, &in_time_t);

		// 创建日志目录
		string strYearMonth_Day = fmt::format("{:%Y%m/%d/}", local_time);	// 子文件夹路径
		string strTime = fmt::format("{:%Y-%m-%d_%H-%M-%S}", local_time);	// 文件名中时间
		string strPath = "D:/Log/" + strYearMonth_Day;
		std::filesystem::create_directories(strPath);

		// 构建日志
		string strProcessLog = strPath + "ProcessLog_" + strTime + ".log";
		auto sinkProcess = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(strProcessLog, 1024 * 1024 * 10, 1); // 10MB大小，保留1个文件
		log_Process = std::make_shared<spdlog::async_logger>("Process_Logger", sinkProcess, spdlog::thread_pool(), spdlog::async_overflow_policy::block);	// 创建异步日志	
		//log_Process = std::make_shared<spdlog::logger>("Process_Logger", sinkProcess);
		log_Process->set_pattern("[%Y-%m-%d %H:%M:%S.%e][%l] %v");	// 设置日志结构
		log_Process->flush_on(spdlog::level::warn);					// 记录Warn及以上级别立即刷新
		log_Process->set_level(spdlog::level::info);				// 最低记录等级info		
		spdlog::register_logger(log_Process);						// 注册到全局
		m_strCurrentProcessLogPath = strProcessLog;
	}
	catch (const spdlog::spdlog_ex& ex)
	{
		string strMessage = ex.what();;
		SystemLog(LogLevel::Error, "Process log failed:" + strMessage);
	}
}

void LogModule::StopProcessLog()
{
	if (log_Process)
	{
		log_Process->flush();
		spdlog::drop("Process_Logger");
		log_Process = nullptr;
	}
}

void LogModule::RefreshProcessLog()
{
	try
	{
		if (log_Process)
			log_Process->flush();
	}
	catch (const spdlog::spdlog_ex& ex)
	{
		
	}
}

void LogModule::SystemLog(LogLevel eLevel, const string& strMessage)
{
	if (!log_System)
		return;

	switch (eLevel)
	{
	case LogLevel::Trace:		log_System->trace(strMessage);		break;
	case LogLevel::Debug:		log_System->debug(strMessage);		break;
	case LogLevel::Info:		log_System->info(strMessage);		break;
	case LogLevel::Warn:		log_System->warn(strMessage);		break;
	case LogLevel::Error:		log_System->error(strMessage);		break;
	case LogLevel::Critical:	log_System->critical(strMessage);	break;
	}
}

void LogModule::OperatorLog(LogLevel eLevel, const string& strMessage)
{
	if (!log_Operator)
		return;

	switch (eLevel)
	{
	case LogLevel::Trace:		log_Operator->trace(strMessage);	break;
	case LogLevel::Debug:		log_Operator->debug(strMessage);	break;
	case LogLevel::Info:		log_Operator->info(strMessage);		break;
	case LogLevel::Warn:		log_Operator->warn(strMessage);		break;
	case LogLevel::Error:		log_Operator->error(strMessage);	break;
	case LogLevel::Critical:	log_Operator->critical(strMessage);	break;
	}
}

void LogModule::ProcessLog(LogLevel eLevel, const string& strMessage)
{
	if (!log_Process)
		return;

	switch (eLevel)
	{
	case LogLevel::Trace:		log_Process->trace(strMessage);		break;
	case LogLevel::Debug:		log_Process->debug(strMessage);		break;
	case LogLevel::Info:		log_Process->info(strMessage);		break;
	case LogLevel::Warn:		log_Process->warn(strMessage);		break;
	case LogLevel::Error:		log_Process->error(strMessage);		break;
	case LogLevel::Critical:	log_Process->critical(strMessage);	break;
	}
}

string LogModule::GetCurrentProcessLogPath()
{
	return m_strCurrentProcessLogPath;
}

void LogModule::WriteCuttingSummary(const QString& qstrFileName,
	const string& strStartTime,
	const string& strEndTime,
	const string& strTotalCutTime,
	int iFeedCount,
	const vector<string>& vecToolSummaryLines)
{
	if (!log_Process)
		return;

	log_Process->info("=== Cutting Summary Start ===");
	log_Process->info("UserName: {}", m_strUser);
	log_Process->info("Permission: {}", m_strPermission);
	const QByteArray fileNameUtf8 = qstrFileName.toUtf8();
	log_Process->info("FileName: {}", string(fileNameUtf8.constData(), fileNameUtf8.size()));
	log_Process->info("StartTime: {}", strStartTime);
	log_Process->info("EndTime: {}", strEndTime);
	log_Process->info("TotalCutTime: {}", strTotalCutTime);
	log_Process->info("FeedCount: {}", iFeedCount);
	log_Process->info("Tools:");

	for (const string& strToolSummary : vecToolSummaryLines)
	{
		log_Process->info("- {}", strToolSummary);
	}

	log_Process->info("=== Cutting Summary End ===");
	log_Process->flush();
}

vector<CuttingLogSummary> LogModule::GetCuttingLogSummaries()
{
	vector<CuttingLogSummary> summaries;

	QDirIterator iterator(QString::fromLatin1(kProcessLogRootPath),
		QStringList() << "ProcessLog_*.log",
		QDir::Files,
		QDirIterator::Subdirectories);

	while (iterator.hasNext()) {
		const QString logFilePath = iterator.next();
		vector<CuttingLogSummary> fileSummaries = parseCuttingLogFile(logFilePath);
		summaries.insert(summaries.end(), fileSummaries.begin(), fileSummaries.end());
	}

	std::sort(summaries.begin(), summaries.end(),
		[](const CuttingLogSummary& lhs, const CuttingLogSummary& rhs) {
			if (lhs.startTime.isValid() && rhs.startTime.isValid()) {
				return lhs.startTime > rhs.startTime;
			}
			if (lhs.startTime.isValid() != rhs.startTime.isValid()) {
				return lhs.startTime.isValid();
			}
			return lhs.logFilePath > rhs.logFilePath;
		});

	return summaries;
}
