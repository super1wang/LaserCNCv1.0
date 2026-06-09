#include "Setting_Gas.h"

Dialog_Setting_Gas::Dialog_Setting_Gas(QWidget* parent)
	: QDialog(parent)
	, set_Changed()
{
	ui.setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::FramelessWindowHint);
	setupLineEditValidators(this);

	connect(ui.checkBox_Gas_bBlow,				 SIGNAL(clicked()),			this, SLOT(checkBoxChanged()));
}

Dialog_Setting_Gas::~Dialog_Setting_Gas()
{
}

void Dialog_Setting_Gas::InitSetting()
{
	table t_Init;
	t_Init["Gas"]["bBlow"]				 = true;
	t_Init["Gas"]["fBlowDelay"]			 = 0.0;
	t_Init["Gas"]["fPressure"]			 = 500.0;
	t_Init["GasSetting"]["iConversions"] = 65535;

	SETTINGS->SetTable(true, SettingSection::Gas, t_Init);
}

void Dialog_Setting_Gas::SetPage(table table_Set)
{
	if (!table_Set.size())
		table_Set = SETTINGS->GetTable(SettingSection::Gas);

	ui.checkBox_Gas_bBlow->setChecked(table_Set["Gas"]["bBlow"].as_boolean());
	
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
}

void Dialog_Setting_Gas::GetPage(table& table_Page)
{
	table_Page["Gas"]["bBlow"] = ui.checkBox_Gas_bBlow->isChecked();
	
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
}

bool Dialog_Setting_Gas::GetChanged(table table_Page, table & table_Changed)
{
	if (set_Changed.empty())
		return false;

	for (auto it = set_Changed.begin(); it != set_Changed.end(); ++it)
	{
		string	strTable = it->first;
		string	strKey = it->second;
		value	Value = table_Page[strTable][strKey];
		table_Changed[strTable][strKey] = Value;
		SETTINGS->SetKeyValue(strKey, Value, SettingSection::Gas, strTable);
		LOG_OPER_INFO(tr("Setting [Gas][%1][%2] %3").arg(tr(strTable.c_str()))
			.arg(tr(strKey.c_str())).arg(toml::format(Value).c_str()).toUtf8().data());
	}
	set<pair<string, string>> set_null;
	set_Changed.swap(set_null);
	return true;
}

void Dialog_Setting_Gas::setupLineEditValidators(QWidget* dialog)
{
	const QList<QLineEdit*> lineEdits = dialog->findChildren<QLineEdit*>();
	for (QLineEdit* lineEdit : lineEdits)
	{
		QStringList parts = lineEdit->objectName().split('_');
		if (parts.size() >= 3)
		{
			if (parts[2].left(1) == "f")
			{
				lineEdit->setValidator(new QRegularExpressionValidator(Regex_Nonnegative_Double(, nullptr)));
				m_qlLineEditF.append(lineEdit);
			}
			else if (parts[2].left(1) == "i")
			{
				lineEdit->setValidator(new QRegularExpressionValidator(Regex_Nonnegative_Int(, nullptr)));
				m_qlLineEditI.append(lineEdit);
			}
			connect(lineEdit, SIGNAL(editingFinished()), this, SLOT(lineEditChanged()));
		}
	}
}

void Dialog_Setting_Gas::lineEditChanged()
{
	QLineEdit*	lineEdit	= qobject_cast<QLineEdit*>(sender());
	QString		qstChanged	= lineEdit->objectName();

	QStringList parts		= qstChanged.split("_");
	string		strTable	= parts.at(parts.size() - 2).toStdString();
	string		strKey		= parts.at(parts.size() - 1).toStdString();

	set_Changed.insert(make_pair(strTable, strKey));
}

void Dialog_Setting_Gas::checkBoxChanged()
{
	QCheckBox*	CheckBox	= qobject_cast<QCheckBox*>(sender());
	QString		qstChanged	= CheckBox->objectName();

	QStringList parts		= qstChanged.split("_");
	string		strTable	= parts.at(parts.size() - 2).toStdString();
	string		strKey		= parts.at(parts.size() - 1).toStdString();

	set_Changed.insert(make_pair(strTable, strKey));
