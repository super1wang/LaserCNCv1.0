#include "Setting_Camera.h"

Dialog_Setting_Camera::Dialog_Setting_Camera(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	setupLineEditValidators(this);
}

Dialog_Setting_Camera::~Dialog_Setting_Camera()
{
}

void Dialog_Setting_Camera::InitSetting()
{
	table t_Init;

	t_Init["Calibration"]["fPixelAccuracy"]	= 0.025;
	t_Init["Calibration"]["iXStandard"]		= 0;
	t_Init["Calibration"]["iYStandard"]		= 0;

	t_Init["Commands"]["sCommand1Name"]		= "指令1";
	t_Init["Commands"]["sCommand2Name"]		= "指令2";
	t_Init["Commands"]["sCommand3Name"]		= "指令3";
	t_Init["Commands"]["sCommand4Name"]		= "指令4";
	t_Init["Commands"]["sCommand1"]			= "T1";
	t_Init["Commands"]["sCommand2"]			= "T2";
	t_Init["Commands"]["sCommand3"]			= "T3";
	t_Init["Commands"]["sCommand4"]			= "T4";

	t_Init["Connect"]["sHost"]		= "127.0.0.1";
	t_Init["Connect"]["iPort"]		= 6800;
	t_Init["Connect"]["iTimeOut"]	= 1000;

	SETTINGS->SetTable(true, SettingSection::Camera, t_Init);
}

void Dialog_Setting_Camera::SetPage(table table_Set)
{
	if (!table_Set.size())
		table_Set = SETTINGS->GetTable(SettingSection::Camera);
	
	QStringList parts;
	for (QLineEdit* lineEdit : m_qlLineEditS)
	{
		parts = lineEdit->objectName().split('_');
		lineEdit->setText(QString::fromStdString(table_Set[parts[1].toStdString()][parts[2].toStdString()].as_string()));
	}

	for (QLineEdit* lineEdit : m_qlLineEditI)
	{
		parts = lineEdit->objectName().split('_');
		lineEdit->setText(QString::number(table_Set[parts[1].toStdString()][parts[2].toStdString()].as_integer()));
	}

	for (QLineEdit* lineEdit : m_qlLineEditF)
	{
		parts = lineEdit->objectName().split('_');
		lineEdit->setText(QString::number(table_Set[parts[1].toStdString()][parts[2].toStdString()].as_floating(), 'g', 16));
	}
}

void Dialog_Setting_Camera::GetPage(table& table_Page)
{
	QStringList parts;

	for (QLineEdit* lineEdit : m_qlLineEditS)
	{
		parts = lineEdit->objectName().split('_');
		table_Page[parts[1].toStdString()][parts[2].toStdString()] = lineEdit->text().toStdString();
	}

	for (QLineEdit* lineEdit : m_qlLineEditI)
	{
		parts = lineEdit->objectName().split('_');
		table_Page[parts[1].toStdString()][parts[2].toStdString()] = lineEdit->text().toInt();
	}

	for (QLineEdit* lineEdit : m_qlLineEditF)
	{
		parts = lineEdit->objectName().split('_');
		table_Page[parts[1].toStdString()][parts[2].toStdString()] = lineEdit->text().toDouble();
	}
}

bool Dialog_Setting_Camera::GetChanged(table table_Page, table& table_Changed)
{
	if (set_Changed.empty())
		return false;

	for (auto it = set_Changed.begin(); it != set_Changed.end(); ++it)
	{
		string	strTable = it->first;
		string	strKey = it->second;
		value	Value = table_Page[strTable][strKey];
		table_Changed[strTable][strKey] = Value;
		SETTINGS->SetKeyValue(strKey, Value, SettingSection::Camera, strTable);
		LOG_OPER_INFO(tr("Setting [Camera][%1][%2] %3").arg(tr(strTable.c_str()))
			.arg(tr(strKey.c_str())).arg(toml::format(Value).c_str()).toUtf8().data());
	}
	set<pair<string, string>> set_null;
	set_Changed.swap(set_null);
	return true;
}

void Dialog_Setting_Camera::lineEditChanged()
{
	QStringList parts	 = qobject_cast<QLineEdit*>(sender())->objectName().split("_");
	string		strTable = parts.at(parts.size() - 2).toStdString();
	string		strKey	 = parts.at(parts.size() - 1).toStdString();
	set_Changed.insert(make_pair(strTable, strKey));
}

void Dialog_Setting_Camera::setupLineEditValidators(QWidget* dialog)
{
	const QList<QLineEdit* > lineEdits = dialog->findChildren<QLineEdit*>();
	for (QLineEdit* lineEdit : lineEdits)
	{
		QStringList parts = lineEdit->objectName().split('_');
		if (parts.size() >= 3)
		{
			if (parts[2].left(1) == "s")
			{
				m_qlLineEditS.append(lineEdit);
			}
			else if (parts[2].left(1) == "i")
			{
				lineEdit->setValidator(new QRegularExpressionValidator(Regex_All_Int(, nullptr)));
				m_qlLineEditI.append(lineEdit);
			}
			else if (parts[2].left(1) == "f")
			{
				lineEdit->setValidator(new QRegularExpressionValidator(Regex_Nonnegative_Double(, nullptr)));
				m_qlLineEditF.append(lineEdit);
			}
			connect(lineEdit, SIGNAL(editingFinished()), this, SLOT(lineEditChanged()));
		}
	}

	ui.lineEdit_Connect_iPort	->setValidator(new QRegularExpressionValidator(Regex_Nonnegative_Int(, nullptr)));
	ui.lineEdit_Connect_iTimeOut->setValidator(new QRegularExpressionValidator(Regex_Nonnegative_Int(, nullptr)));
}

void Dialog_Setting_Camera::SetCommandNameEnabled(bool bEnabled)
{
	ui.lineEdit_Commands_sCommand1Name->setEnabled(bEnabled);
	ui.lineEdit_Commands_sCommand2Name->setEnabled(bEnabled);
	ui.lineEdit_Commands_sCommand3Name->setEnabled(bEnabled);
	ui.lineEdit_Commands_sCommand4Name->setEnabled(bEnabled);
