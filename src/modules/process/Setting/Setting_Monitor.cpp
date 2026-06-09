#include "Setting_Monitor.h"

Dialog_Setting_Monitor::Dialog_Setting_Monitor(QWidget* parent)
	: QDialog(parent)
	, set_Changed()
	{
	ui.setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::FramelessWindowHint);
	setupLineEditValidators(this);
	setupCheckBoxValidators(this);
	setupComboBoxValidators(this);
}

Dialog_Setting_Monitor::~Dialog_Setting_Monitor()
{
}

void Dialog_Setting_Monitor::InitSetting()
{
	table t_Init;
	t_Init["Cutting"]["bInterLock"]						= false;
	t_Init["Cutting"]["bSafetyLightCurtain"]			= false;
	t_Init["Gas"]["bPressureMonitor"]					= false;
	t_Init["Water"]["bWaterLeakageMonitor"]				= false;
	t_Init["Water"]["bWaterTankMonitor"]				= false;
	t_Init["Water"]["bWaterPressureMonitor"]			= false;
	t_Init["Water"]["fWaterPressureLimit"]				= 1.0;
	t_Init["Water"]["bWaterLevelMonitor"]				= false;
	t_Init["Water"]["fWaterLevelLimit"]					= 50.0;
	t_Init["WaterSetting"]["iWaterPressureConversions"] = 4096;
	t_Init["WaterSetting"]["iWaterLevelConversions"]	= 4096;
	t_Init["General"]["iFaultAction"]					= 1;

	SETTINGS->SetTable(true, SettingSection::Monitor, t_Init);
}

void Dialog_Setting_Monitor::SetPage(table table_Set)
{
	if (!table_Set.size())
		table_Set = SETTINGS->GetTable(SettingSection::Monitor);

	QStringList parts;
	for (QLineEdit* lineEdit : m_qlLineEditI)
	{
		parts = lineEdit->objectName().split('_');
		lineEdit->setText(QString::number(table_Set[parts[1].toStdString()][parts[2].toStdString()].as_integer()));
	}

	for (QLineEdit* lineEdit : m_qlLineEditF)
	{
		parts = lineEdit->objectName().split('_');
		lineEdit->setText(QString::number(table_Set[parts[1].toStdString()][parts[2].toStdString()].as_floating(), 'g', 3));
	}

	for (QCheckBox* checkBox : m_qlCheckBoxB)
	{
		parts = checkBox->objectName().split('_');
		checkBox->setChecked(table_Set[parts[1].toStdString()][parts[2].toStdString()].as_boolean());
	}

	for (QComboBox* comboBox : m_qlComboBoxI)
	{
		parts = comboBox->objectName().split('_');
		comboBox->setCurrentIndex(table_Set[parts[1].toStdString()][parts[2].toStdString()].as_integer());
	}
}

void Dialog_Setting_Monitor::GetPage(table& table_Page)
{
	QStringList parts;

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

	for (QCheckBox* checkBox : m_qlCheckBoxB)
	{
		parts = checkBox->objectName().split('_');
		table_Page[parts[1].toStdString()][parts[2].toStdString()] = checkBox->isChecked();
	}

	for (QComboBox* comboBox : m_qlComboBoxI)
	{
		parts = comboBox->objectName().split('_');
		table_Page[parts[1].toStdString()][parts[2].toStdString()] = comboBox->currentIndex();
	}
}

bool Dialog_Setting_Monitor::GetChanged(table table_Page, table& table_Changed)
{
	if (set_Changed.empty())
		return false;

	for (auto it = set_Changed.begin(); it != set_Changed.end(); ++it)
	{
		string	strTable	= it->first;
		string	strKey		= it->second;
		value	Value		= table_Page[strTable][strKey];
		table_Changed[strTable][strKey] = Value;
		SETTINGS->SetKeyValue(strKey, Value, SettingSection::Monitor, strTable);
		LOG_OPER_INFO(tr("Setting [Monitor][%1][%2] %3").arg(tr(strTable.c_str()))
			.arg(tr(strKey.c_str())).arg(toml::format(Value).c_str()).toUtf8().data());
	}
	set<pair<string, string>> set_null;
	set_Changed.swap(set_null);
	return true;
}

void Dialog_Setting_Monitor::setupLineEditValidators(QWidget* dialog)
{
	const QList<QLineEdit*> lineEdits = dialog->findChildren<QLineEdit*>();
	for (QLineEdit* lineEdit : lineEdits)
	{
		QStringList parts = lineEdit->objectName().split('_');
		if (parts.size() >= 3)
		{
			if (parts[2].left(1) == "i")
			{
				lineEdit->setValidator(new QRegExpValidator(Regex_Nonnegative_Int));
				m_qlLineEditI.append(lineEdit);
			}
			else if (parts[2].left(1) == "f")
			{
				lineEdit->setValidator(new QRegExpValidator(Regex_Nonnegative_Double));
				m_qlLineEditF.append(lineEdit);
			}
			connect(lineEdit, SIGNAL(editingFinished()), this, SLOT(lineEditChanged()));
		}
	}
}

void Dialog_Setting_Monitor::setupCheckBoxValidators(QWidget* dialog)
{
	const QList<QCheckBox*> checkBoxs = dialog->findChildren<QCheckBox*>();
	for (QCheckBox* checkBox : checkBoxs)
	{
		QStringList parts = checkBox->objectName().split('_');
		if (parts.size() >= 3)
		{
			m_qlCheckBoxB.append(checkBox);
			connect(checkBox, SIGNAL(clicked()), this, SLOT(checkBoxChanged()));
		}
	}
}

void Dialog_Setting_Monitor::setupComboBoxValidators(QWidget* dialog)
{
	const QList<QComboBox*> comboBoxs = dialog->findChildren<QComboBox*>();
	for (QComboBox* comboBox : comboBoxs)
	{
		QStringList parts = comboBox->objectName().split('_');
		if (parts.size() >= 3)
		{
			m_qlComboBoxI.append(comboBox);
			connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(comboBoxChanged()));
		}
	}
}

void Dialog_Setting_Monitor::lineEditChanged()
{
	QLineEdit*	lineEdit	= qobject_cast<QLineEdit*>(sender());
	QString		qstChanged	= lineEdit->objectName();

	QStringList parts		= qstChanged.split("_");
	string		strTable	= parts.at(parts.size() - 2).toStdString();
	string		strKey		= parts.at(parts.size() - 1).toStdString();

	set_Changed.insert(make_pair(strTable, strKey));
}

void Dialog_Setting_Monitor::checkBoxChanged()
{
	QCheckBox*	CheckBox	= qobject_cast<QCheckBox*>(sender());
	QString		qstChanged	= CheckBox->objectName();

	QStringList parts		= qstChanged.split("_");
	string		strTable	= parts.at(parts.size() - 2).toStdString();
	string		strKey		= parts.at(parts.size() - 1).toStdString();

	set_Changed.insert(make_pair(strTable, strKey));
}

void Dialog_Setting_Monitor::comboBoxChanged()
{
	QComboBox*	comboBox	= qobject_cast<QComboBox*>(sender());
	QString		qstChanged	= comboBox->objectName();

	QStringList parts		= qstChanged.split("_");
	string		strTable	= parts.at(parts.size() - 2).toStdString();
	string		strKey		= parts.at(parts.size() - 1).toStdString();

	set_Changed.insert(make_pair(strTable, strKey));
}

