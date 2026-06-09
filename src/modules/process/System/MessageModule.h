#ifndef MESSAGEMODULE_H
#define MESSAGEMODULE_H

#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <queue>
#include <memory>
#include "MessageCode.h"
#include "LogModule.h"

#define SHOW_SYS_INFO(code, ...)			MessageModule::instance().ReportMessage(LogType::System,	LogLevel::Info,		static_cast<int>(code), ##__VA_ARGS__)
#define SHOW_SYS_WARN(code, ...)			MessageModule::instance().ReportMessage(LogType::System,	LogLevel::Warn,		static_cast<int>(code), ##__VA_ARGS__)
#define SHOW_SYS_ERROR(code, ...)		    MessageModule::instance().ReportMessage(LogType::System,	LogLevel::Error,	static_cast<int>(code), ##__VA_ARGS__)

#define SHOW_OPER_INFO(code, ...)		    MessageModule::instance().ReportMessage(LogType::Operator,	LogLevel::Info,		static_cast<int>(code), ##__VA_ARGS__)
#define SHOW_OPER_WARN(code, ...)		    MessageModule::instance().ReportMessage(LogType::Operator,	LogLevel::Warn,		static_cast<int>(code), ##__VA_ARGS__)
#define SHOW_OPER_ERROR(code, ...)		    MessageModule::instance().ReportMessage(LogType::Operator,	LogLevel::Error,	static_cast<int>(code), ##__VA_ARGS__)

#define SHOW_PROCESS_INFO(code, ...)		MessageModule::instance().ReportMessage(LogType::Process,	LogLevel::Info,		static_cast<int>(code), ##__VA_ARGS__)
#define SHOW_PROCESS_WARN(code, ...)		MessageModule::instance().ReportMessage(LogType::Process,	LogLevel::Warn,		static_cast<int>(code), ##__VA_ARGS__)
#define SHOW_PROCESS_ERROR(code, ...)	    MessageModule::instance().ReportMessage(LogType::Process,	LogLevel::Error,	static_cast<int>(code), ##__VA_ARGS__)

class MessageModule : public QThread
{
	Q_OBJECT
public:
	static MessageModule& instance();

	void ReportMessage(LogType eType, LogLevel eLevel, int iCode, const QString& qstrMessage = QString());
	void stop();

protected:
	void run() override;

private:
	struct Message
	{
		LogType		eType;
		LogLevel	eLevel;
		int			iCode;
		QString     qstrCustomMessage;
	};

	MessageModule(QObject* parent = nullptr);
	~MessageModule();

	void ShowMessage(const Message& item);
	void WriteLog(const LogType& eType, const LogLevel& eLevel, const int& iCode, const QString& qstrMessage = QString());

	QString ExplainCode(LogLevel eLevel, int iCode)	const;
	QString ExplainErrorCode(ErrorCode eCode)		const;
	QString ExplainWarnCode(WarnCode eCode)			const;
	QString ExplainInfoCode(InfoCode eCode)			const;

private:
	QMutex				m_mutex;
	QWaitCondition		m_condition;
	std::queue<Message> m_MessageQueue;
	bool				m_stopped = false;
};

#endif // MESSAGEMODULE_H