#include "RegexPatterns.h"
#include "Setting_Monitor.h"

namespace
{
const char* kWaterSettingTable = "WaterSetting";
const char* kWaterPressureFormulaKey = "sWaterPressureConversions";
const char* kWaterLevelFormulaKey = "sWaterLevelConversions";
const char* kLegacyWaterPressureFormulaKey = "iWaterPressureConversions";
const char* kLegacyWaterLevelFormulaKey = "iWaterLevelConversions";

QString ToFormulaText(const value& field)
{
	if (field.is_string())
		return QString::fromStdString(field.as_string());

	double coefficient = 0.0;
	if (field.is_integer())
		coefficient = static_cast<double>(field.as_integer());
	else if (field.is_floating())
		coefficient = field.as_floating();

	return QString("Y=X*%1").arg(QString::number(coefficient, 'g', 15));
}

QString ReadWaterSettingFormula(const table& tableSet, const QString& key)
{
	if (!tableSet.count(kWaterSettingTable))
		return QString();

	const value& waterSettingValue = tableSet.at(kWaterSettingTable);
	if (!waterSettingValue.is_table())
		return QString();

	const table& waterSetting = waterSettingValue.as_table();
	const std::string keyStd = key.toStdString();
	if (waterSetting.count(keyStd))
		return ToFormulaText(waterSetting.at(keyStd));

	if (key == kWaterPressureFormulaKey && waterSetting.count(kLegacyWaterPressureFormulaKey))
		return ToFormulaText(waterSetting.at(kLegacyWaterPressureFormulaKey));

	if (key == kWaterLevelFormulaKey && waterSetting.count(kLegacyWaterLevelFormulaKey))
		return ToFormulaText(waterSetting.at(kLegacyWaterLevelFormulaKey));

	return QString();
}
}

Dialog_Setting_Monitor::Dialog_Setting_Monitor(QWidget* parent)
	: QDialog(parent)
	, set_Changed()
	{
	ui.setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::FramelessWindowHint);
	setupLineEditValidators(this);
	setupCheckBoxValidators(this);
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
	t_Init[kWaterSettingTable][kWaterPressureFormulaKey] = string("Y=X*4096");
	t_Init[kWaterSettingTable][kWaterLevelFormulaKey]	 = string("Y=X*4096");

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
		lineEdit->setText(QString::number(table_Set[parts[1].toStdString()][parts[2].toStdString()].as_floating(), 'g', 16));
	}

	for (QLineEdit* lineEdit : m_qlLineEditS)
	{
		parts = lineEdit->objectName().split('_');
		if (parts.size() < 3)
			continue;

		QString text = ReadWaterSettingFormula(table_Set, parts[2]);
		if (text.isEmpty() && table_Set.count(parts[1].toStdString()) &&
			table_Set.at(parts[1].toStdString()).is_table())
		{
			const table& currentTable = table_Set.at(parts[1].toStdString()).as_table();
			if (currentTable.count(parts[2].toStdString()) && currentTable.at(parts[2].toStdString()).is_string())
				text = QString::fromStdString(currentTable.at(parts[2].toStdString()).as_string());
		}
		lineEdit->setText(text);
	}

	for (QCheckBox* checkBox : m_qlCheckBoxB)
	{
		parts = checkBox->objectName().split('_');
		checkBox->setChecked(table_Set[parts[1].toStdString()][parts[2].toStdString()].as_boolean());
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

	for (QLineEdit* lineEdit : m_qlLineEditS)
	{
		parts = lineEdit->objectName().split('_');
		table_Page[parts[1].toStdString()][parts[2].toStdString()] = lineEdit->text().toStdString();
	}

	for (QCheckBox* checkBox : m_qlCheckBoxB)
	{
		parts = checkBox->objectName().split('_');
		table_Page[parts[1].toStdString()][parts[2].toStdString()] = checkBox->isChecked();
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
				lineEdit->setValidator(new QRegularExpressionValidator(Regex_Nonnegative_Int(), nullptr));
				m_qlLineEditI.append(lineEdit);
			}
			else if (parts[2].left(1) == "f")
			{
				lineEdit->setValidator(new QRegularExpressionValidator(Regex_Nonnegative_Double(), nullptr));
				m_qlLineEditF.append(lineEdit);
			}
			else if (parts[2].left(1) == "s")
			{
				lineEdit->setMaxLength(64);
				m_qlLineEditS.append(lineEdit);
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

