#include "MessageModule.h"
#include "BuzzerControl.h"
#include <QMessageBox>
#include <QApplication>
#include <QTimer>
#include <QPointer>

MessageModule& MessageModule::instance()
{
	static MessageModule instance;
	return instance;
}

MessageModule::MessageModule(QObject* parent)
	: QThread(parent)
{
	start();
}

MessageModule::~MessageModule()
{
	stop();
	wait();
}

void MessageModule::run()
{
	while (true)
	{
		Message item;
		{
			QMutexLocker locker(&m_mutex);
			while (m_MessageQueue.empty() && !m_stopped)
			{
				m_condition.wait(&m_mutex);
			}

			if (m_stopped && m_MessageQueue.empty())
				break;

			if (!m_MessageQueue.empty())
			{
				item = m_MessageQueue.front();
				m_MessageQueue.pop();
			}
		}

		if (!m_stopped)
			ShowMessage(item);
	}
}

void MessageModule::stop()
{
	QMutexLocker locker(&m_mutex);
	m_stopped = true;
	m_condition.wakeAll();
}

void MessageModule::ReportMessage(LogType eType, LogLevel eLevel, int iCode, const QString& qstrMessage)
{
	WriteLog(eType, eLevel, iCode, qstrMessage);
	QMutexLocker locker(&m_mutex);
	m_MessageQueue.push({ eType, eLevel, iCode, qstrMessage });
	m_condition.wakeOne();
	if (eLevel == LogLevel::Error)
	{
		BuzzerControl::Beep(3000);
	}
}

void MessageModule::WriteLog(const LogType& eType, const LogLevel& eLevel, const int& iCode, const QString& qstrMessage)
{
	QString qstrLog = qstrMessage.isEmpty() ? ExplainCode(eLevel, iCode) : qstrMessage;
	switch (eType)
	{
	case LogType::System:
		LogModule::SystemLog(eLevel, qstrLog.toUtf8().data());
		break;
	case LogType::Operator:
		LogModule::OperatorLog(eLevel, qstrLog.toUtf8().data());
		break;
	case LogType::Process:
		LogModule::ProcessLog(eLevel, qstrLog.toUtf8().data());
		break;
	default:
		LogModule::SystemLog(LogLevel::Error, qstrLog.toUtf8().data());
		break;
	}
}

void MessageModule::ShowMessage(const Message& item)
{
	QMetaObject::invokeMethod(qApp, [this, item]() {
		QString qstrMessage = item.qstrCustomMessage.isEmpty() ?
			MessageModule::instance().ExplainCode(item.eLevel, item.iCode) :
			item.qstrCustomMessage;
		QString qstrTitle = "None";
		QMessageBox::Icon eIcon = QMessageBox::Information;
		switch (item.eLevel)
		{
		case LogLevel::Info:
			qstrTitle = tr("Info");
			eIcon = QMessageBox::Information;
			break;
		case LogLevel::Warn:
			qstrTitle = tr("Warn");
			eIcon = QMessageBox::Warning;
			break;
		case LogLevel::Error:
			qstrTitle = tr("Error");
			eIcon = QMessageBox::Critical;
			break;
		default:
			break;;
		}

		QMessageBox msgBox;
		msgBox.setIcon(eIcon);
		msgBox.setWindowTitle(qstrTitle);
		msgBox.setText(qstrMessage);
		msgBox.setStandardButtons(QMessageBox::Ok);
		msgBox.exec();
		});
}

QString MessageModule::ExplainCode(LogLevel eLevel, int iCode) const
{
	switch (eLevel)
	{
	case LogLevel::Info:
		return ExplainInfoCode(static_cast<InfoCode>(iCode));
	case LogLevel::Warn:
		return ExplainWarnCode(static_cast<WarnCode>(iCode));
	case LogLevel::Error:
		return ExplainErrorCode(static_cast<ErrorCode>(iCode));
	default:
		return tr("[FAULT]This is not ErrorLevel !");
	}
}

QString MessageModule::ExplainErrorCode(ErrorCode eCode) const
{
	switch (eCode)
	{
	case ErrorCode::ERROR_NONE:
		return tr("[ERROR_NONE]The system is normal.");

	case ErrorCode::ERROR_MC_CONNECTIONFAILED:
		return tr("[ERROR_MC_CONNECTIONFAILED]Motion controller connection failed.");
	case ErrorCode::ERROR_MC_HOMEFAILED:
		return tr("[ERROR_MC_HOMEFAILED]Axis find home point failed.");
	case ErrorCode::ERROR_MC_SETTINGFAILED:
		return tr("[ERROR_MC_SETTINGFAILED]Set axis parameters failed.");	
	case ErrorCode::ERROR_MC_MOVEFAILED:
		return tr("[ERROR_MC_MOVEFAILED]Axis move failed. For details, please check the system log.");

	case ErrorCode::ERROR_LASER_CONNECTIONFAILED:
		return tr("[ERROR_LASER_CONNECTIONFAILED]Laser connection failed.");
	case ErrorCode::ERROR_LASER_SETTINGFAILED:
		return tr("[ERROR_LASER_SETTINGFAILED]Set laser parameters failed.");

	case ErrorCode::ERROR_SIGNALSOURCE_CONNECTIONFAILED:
		return tr("[ERROR_SIGNALSOURCE_CONNECTIONFAILED]Signal source connection failed.");
	case ErrorCode::ERROR_SIGNALSOURCE_SETTINGFAILED:
		return tr("[ERROR_SIGNALSOURCE_SETTINGFAILED]Signal source set laser parameters failed.");
		
	case ErrorCode::ERROR_SERIALPORT_CONNECTIONFAILED:
		return tr("[ERROR_SERIALPORT_CONNECTIONFAILED]Serial port connection failed.");

	case ErrorCode::ERROR_TCP_CONNECTIONFAILED:
		return tr("[ERROR_TCP_CONNECTIONFAILED]TCP connection failed.");

	case ErrorCode::ERROR_UDP_CONNECTIONFAILED:
		return tr("[ERROR_UDP_CONNECTIONFAILED]UDP connection failed.");

	case ErrorCode::ERROR_CAMERA_COMMUNICATIONFAILURE:
		return tr("[ERROR_CAMERA_CONNECTIONFAILED]Camera communication failure.");
	case ErrorCode::ERROR_CAMERA_RECOGNITIONFAILED:
		return tr("[ERROR_CAMERA_RECOGNITIONFAILED]Camera recognition failed.");
	case ErrorCode::ERROR_CAMERA_WRITEFAILED:
		return tr("[ERROR_CAMERA_WRITEFAILED]Camera write command failed.");
	case ErrorCode::ERROR_CAMERA_READFAILED:
		return tr("[ERROR_CAMERA_READFAILED]Camera read raw failed.");

	case ErrorCode::ERROR_PROCESS_GROUPLOSS:
		return tr("[ERROR_PROCESS_GROUPLOSS]The group is missing in the process.");
	case ErrorCode::ERROR_PROCESS_OUTOFNESTING:
		return tr("[ERROR_PROCESS_OUTOFNESTING]Exceeding the maximum loop nesting.");
	case ErrorCode::ERROR_PROCESS_AXISOUTOFLIMIT:
		return tr("[ERROR_PROCESS_AXISOUTOFLIMIT]Axis target position is out of limit.");
	case ErrorCode::ERROR_PROCESS_AXISDISABLED:
		return tr("[ERROR_PROCESS_AXISDISABLED]Axis is not enabled.");
	case ErrorCode::ERROR_PROCESS_LASERSETFAILED:
		return tr("[ERROR_PROCESS_LASERSETFAILED]Set laser parameters failed.");
	case ErrorCode::ERROR_PROCESS_CUTOUTOFLIMIT:
		return tr("[ERROR_PROCESS_CUTOUTOFLIMIT]Cut number out of limit.");
	case ErrorCode::ERROR_PROCESS_NONECOMMANDSFILE:
		return tr("[ERROR_PROCESS_NONECOMMANDSFILE]Commands file path is invalid.");
	case ErrorCode::ERROR_PROCESS_COMMANDSINVALID:
		return tr("[ERROR_PROCESS_COMMANDSINVALID]Commands is invalid.");
	case ErrorCode::ERROR_PROCESS_IOINVALID:
		return tr("[ERROR_PROCESS_IOINVALID]IO index is invalid.");
	case ErrorCode::ERROR_PROCESS_GROUPTIMEOUT:
		return tr("[ERROR_PROCESS_GROUPTIMEOUT]Background thread execution timed out.");
	case ErrorCode::ERROR_PROCESS_GROUPRUNNING:
		return tr("[ERROR_PROCESS_GROUPRUNNING]Background thread is already running and cannot be created.");
	case ErrorCode::ERROR_PROCESS_IFTIMEOUT:
		return tr("[ERROR_PROCESS_IFTIMEOUT]IF item waiting timeout.");
	case ErrorCode::ERROR_PROCESS_OUTOFTOLERANCE:
		return tr("[ERROR_PROCESS_OUTOFTOLERANCE]The calculation result exceeds the margin of error.");

	case ErrorCode::ERROR_CUTTING_LISTEMPTY:
		return tr("[ERROR_CUTTING_LISTEMPTY]Cut list is empty.");
	case ErrorCode::ERROR_CUTTING_SENDCOMMAND:
		return tr("[ERROR_CUTTING_SENDCOMMAND]Sending command failed.");
	case ErrorCode::ERROR_CUTTING_OUTOFLIMIT:
		return tr("[ERROR_CUTTING_OUTOFLIMIT]The drawing is outside the processing area.");
	case ErrorCode::ERROR_CUTTING_OVERTIME:
		return tr("[ERROR_CUTTING_OVERTIME]The cutting buffer runs out of timeout.");
	case ErrorCode::ERROR_CUTTING_GETCOMPENSATIONFAILED:
		return tr("[ERROR_CUTTING_GETCOMPENSATIONFAILED]Get the compensation failed.");

	case ErrorCode::ERROR_OVERCUTTING_NODIVISIONLAYER:
		return tr("[ERROR_OVERCUTTING_NODIVISIONLAYER]The division layer does not exist.");
	case ErrorCode::ERROR_OVERCUTTING_DIVISIONLAYERERR:
		return tr("[ERROR_OVERCUTTING_DIVISIONLAYERERR]The division layer error.");
	case ErrorCode::ERROR_OVERCUTTING_DIVISIONEMPTY:
		return tr("[ERROR_OVERCUTTING_DIVISIONEMPTY]The division layer has no valid division lines.");
	case ErrorCode::ERROR_OVERCUTTING_NOCUTENTITY:
		return tr("[ERROR_OVERCUTTING_NOCUTENTITY]No valid over-cutting entities were found. Check the selected entities and division layer.");
	case ErrorCode::ERROR_OVERCUTTING_OUTOFLIMIT:
		return tr("[ERROR_OVERCUTTING_OUTOFLIMIT]The starting position out of limit.");
	case ErrorCode::ERROR_OVERCUTTING_LSUG_TOO_LARGE:
		return tr("[ERROR_OVERCUTTING_LSUG_TOO_LARGE]The suggested division length exceeds the available X travel.");
	case ErrorCode::ERROR_OVERCUTTING_NOGAPINLIMIT:
		return tr("[ERROR_OVERCUTTING_NOGAPINLIMIT]No valid division position was found within the available X travel.");
	case ErrorCode::ERROR_OVERCUTTING_SEGTOOSHORT:
		return tr("[ERROR_OVERCUTTING_SEGTOOSHORT]The auto division segment length is too short.");
	case ErrorCode::ERROR_OVERCUTTING_LIMITNOTREADY:
		return tr("[ERROR_OVERCUTTING_LIMITNOTREADY]X axis soft limits are not ready. Wait for initialization to finish before auto division.");
	case ErrorCode::ERROR_OVERCUTTING_INVALIDRANGE:
		return tr("[ERROR_OVERCUTTING_INVALIDRANGE]The X range of the selected entities is degenerate.");

	case ErrorCode::ERROR_AUTOFOCUS_NOFOCUSLAYER:
		return tr("[ERROR_AUTOFOCUS_NOFOCUSLAYER]The focus layer does not exist.");
	case ErrorCode::ERROR_AUTOFOCUS_NOODDNUMBER:
		return tr("[ERROR_AUTOFOCUS_NOODDNUMBER]The number of \"Focus|\" selections is not odd.");

	case ErrorCode::ERROR_MONITOR_AXISDISABLED:
		return tr("[ERROR_MONITOR_AXISDISABLED]Axis is not enabled.");
	case ErrorCode::ERROR_MONITOR_WATERLEAKAGE:
		return tr("[ERROR_MONITOR_WATERLEAKAGE]Water tank is leaking.");
	case ErrorCode::ERROR_MONITOR_WATERTANK:
		return tr("[ERROR_MONITOR_WATERTANK]Water tank has malfunctioned.");
	case ErrorCode::ERROR_MONITOR_PRESSURE:
		return tr("[ERROR_MONITOR_PRESSURE]Process gases pressure is abnormal.");

	case ErrorCode::ERROR_FILE_INVALIDCONTENT:
		return tr("[ERROR_FILE_INVALIDCONTENT]The entity of the file is invalid.");

	case ErrorCode::ERROR_EXPRES_NONEEXPRESSION:
		return tr("[ERROR_EXPRES_NONEEXPRESSION]Expression is empty.");
	case ErrorCode::ERROR_EXPRES_EXPRESSIONINVALID:
		return tr("[ERROR_EXPRES_EXPRESSIONINVALID]Failed to parse expression.");
	case ErrorCode::ERROR_EXPRES_CALCULATIONFAILED:
		return tr("[ERROR_EXPRES_CALCULATIONFAILED]Failed to calculation expression.");
	case ErrorCode::ERROR_EXPRES_NONEINDEX:
		return tr("[ERROR_EXPRES_NONEINDEX]Expression index is invalid.");

	case ErrorCode::ERROR_SPECIAL_LPCONNECTIONFAILED:
		return tr("[ERROR_SPECIAL_LPCONNECTIONFAILED]LP connection failed.");
	case ErrorCode::ERROR_SPECIAL_LPSENDFAILED:
		return tr("[ERROR_SPECIAL_LPSENDFAILED]LP send command failed.");
	case ErrorCode::ERROR_SPECIAL_LPRESPONSENONE:
		return tr("[ERROR_SPECIAL_LPRESPONSENONE]LP response is invalid.");
	case ErrorCode::ERROR_SPECIAL_LPNONEINDEX:
		return tr("[ERROR_SPECIAL_LPNONEINDEX]LP index is invalid.");

	default:
		return tr("[FAULT]This is not ErrorCode or the explain was not found !");
	}
}

QString MessageModule::ExplainWarnCode(WarnCode eCode) const
{
	switch (eCode)
	{
	case WarnCode::WARN_NONE:
		return tr("[WARN_NONE]The system is normal.");

	case WarnCode::WARN_MC_DISCONNECTED:
		return tr("[WARN_MC_DISCONNECTED]Motion controller not connected.");
	case WarnCode::WARN_MC_OUTOFLIMIT:
		return tr("[WARN_MC_OUTOFLIMIT]Target position is out of limit.");
	case WarnCode::WARN_MC_NONEINDEX:
		return tr("[WARN_MC_NONEINDEX]Index is invalid.");
		
	case WarnCode::WARN_PROCESS_ITEMEDITING:
		return tr("[WARN_PROCESS_ITEMEDITING]Item is being edited, please close the edit box first.");
	
	case WarnCode::WARN_LASER_DISCONNECTED:
		return tr("[WARN_LASER_DISCONNECTED]Laser not connected.");

	case WarnCode::WARN_MONITOR_INTERLOCK:
		return tr("[WARN_MONITOR_INTERLOCK]Door was opened.");
	case WarnCode::WARN_MONITOR_PRESSURE:
		return tr("[WARN_MONITOR_PRESSURE]Process gases pressure is abnormal.");
	case WarnCode::WARN_MONITOR_WATERPRESSURE:
		return tr("[WARN_MONITOR_WATERPRESSURE]Water pressure is abnormal.");
	case WarnCode::WARN_MONITOR_SAFETYLIGHTCURTAIN:
		return tr("[WARN_MONITOR_SAFETYLIGHTCURTAIN]Safety Light Curtain is abnormal.");
		
	case WarnCode::WARN_SETTING_SAMETOOLNAME:
		return tr("[WARN_SETTING_SAMETOOLNAME]The name already exists !");
	case WarnCode::WARN_SETTING_ONLYONETOOL:
		return tr("[WARN_SETTING_ONLYONETOOL]Please retain at least one tool.");
	case WarnCode::WARN_SETTING_IMPORTTOOL:
		return tr("[WARN_SETTING_IMPORTTOOL]Import tool parameters failed.");
		
	case WarnCode::WARN_FILE_OPENFAILD:
		return tr("[WARN_FILE_OPENFAILD]The file failed to open.");
	case WarnCode::WARN_FILE_IMPORTFAILD:
		return tr("[WARN_FILE_IMPORTFAILD]The import failed.");
	case WarnCode::WARN_FILE_EXPORTFAILD:
		return tr("[WARN_FILE_EXPORTFAILD]The export failed.");
	case WarnCode::WARN_FILE_INVALIDCONTENT:
		return tr("[WARN_FILE_INVALIDCONTENT]The motion controller is not connected, so the drawing cannot be created.");

	case WarnCode::WARN_FOCUS_CREATFILEERR:
		return tr("[WARN_FOCUS_CREATFILEERR]Create file failed, Total line Number is Zero.");
	case WarnCode::WARN_FOCUS_MCNOTREADY:
		return tr("[WARN_FOCUS_MCNOTREADY]MC not connect, Total line Number is Zero.");

	default:
		return tr("[FAULT]This is not WarnCode or the explain was not found !");
	}
}

QString MessageModule::ExplainInfoCode(InfoCode eCode) const
{
	switch (eCode)
	{
	case InfoCode::INFO_NONE:
		return tr("[INFO_NONE]The system is normal.");

	case InfoCode::INFO_MONITOR_WATERLEVEL:
		return tr("[INFO_MONITOR_WATERLEVEL]The water tank level is low, please add water.");

	case InfoCode::INFO_PROCESS_SEQUENCEZERO:
		return tr("[INFO_PROCESS_SEQUENCEZERO]No cutting sequence has been set.");

	case InfoCode::INFO_FILE_NONEENTITY:
		return tr("[INFO_FILE_NONEENTITY]No entity in the file.");
	case InfoCode::INFO_FILE_NONESETTINGS:
		return tr("[INFO_FILE_NONESETTINGS]No settings in the file.");
	case InfoCode::INFO_FILE_NONEPROCESS:
		return tr("[INFO_FILE_NONEPROCESS]No process in the file.");

	default:
		return tr("[FAULT]This is not InfoCode or the explain was not found !");
	}
}
