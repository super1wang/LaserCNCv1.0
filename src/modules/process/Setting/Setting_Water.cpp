#include "RegexPatterns.h"
#include "Setting_Water.h"

Dialog_Setting_Water::Dialog_Setting_Water(QWidget* parent)
	: QDialog(parent)
	, set_Changed()
{
	ui.setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::FramelessWindowHint);
	setupLineEditValidators(this);

	connect(ui.checkBox_Water_bWater,		 SIGNAL(clicked()),			this, SLOT(checkBoxChanged()));
	connect(ui.lineEdit_Water_fWaterDelay,	 SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.checkBox_Pump_bPump,			 SIGNAL(clicked()),			this, SLOT(checkBoxChanged()));
	connect(ui.lineEdit_Pump_fPumpOpenTime,	 SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Pump_fPumpCloseTime, SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
}

Dialog_Setting_Water::~Dialog_Setting_Water()
{
}

void Dialog_Setting_Water::InitSetting()
{
	table t_Init;
	t_Init["Water"]["bWater"]		 = false;
	t_Init["Water"]["fWaterDelay"]	 = 0.0;
	t_Init["Pump"]["bPump"]			 = false;
	t_Init["Pump"]["fPumpOpenTime"]	 = 10.0;
	t_Init["Pump"]["fPumpCloseTime"] = 2.0;

	//SETTINGS->SetTable(t_Init, SettingSection::Water);
	SETTINGS->SetTable(true, SettingSection::Water, t_Init);
}

void Dialog_Setting_Water::SetPage(table table_Set)
{
	if (!table_Set.size())
		table_Set = SETTINGS->GetTable(SettingSection::Water);

	ui.checkBox_Water_bWater		->setChecked	(					table_Set["Water"]["bWater"]		.as_boolean());
	ui.lineEdit_Water_fWaterDelay	->setText		(QString::number(	table_Set["Water"]["fWaterDelay"]	.as_floating(), 'g', 16));
	ui.checkBox_Pump_bPump			->setChecked	(					table_Set["Pump"]["bPump"]			.as_boolean());
	ui.lineEdit_Pump_fPumpOpenTime	->setText		(QString::number(	table_Set["Pump"]["fPumpOpenTime"]	.as_floating(), 'g', 16));
	ui.lineEdit_Pump_fPumpCloseTime ->setText		(QString::number(	table_Set["Pump"]["fPumpCloseTime"] .as_floating(), 'g', 16));
}

void Dialog_Setting_Water::GetPage(table& table_Page)
{
	table_Page["Water"]["bWater"]			= ui.checkBox_Water_bWater			->isChecked();
	table_Page["Water"]["fWaterDelay"]		= ui.lineEdit_Water_fWaterDelay		->text().toDouble();
	table_Page["Pump"]["bPump"]				= ui.checkBox_Pump_bPump			->isChecked();
	table_Page["Pump"]["fPumpOpenTime"]		= ui.lineEdit_Pump_fPumpOpenTime	->text().toDouble();
	table_Page["Pump"]["fPumpCloseTime"]	= ui.lineEdit_Pump_fPumpCloseTime	->text().toDouble();
}

bool Dialog_Setting_Water::GetChanged(table table_Page, table & table_Changed)
{
	if (set_Changed.empty())
		return false;

	for (auto it = set_Changed.begin(); it != set_Changed.end(); ++it)
	{
		string	strTable = it->first;
		string	strKey = it->second;
		value	Value = table_Page[strTable][strKey];
		table_Changed[strTable][strKey] = Value;
		SETTINGS->SetKeyValue(strKey, Value, SettingSection::Water, strTable);
		LOG_OPER_INFO(tr("Setting [Water][%1][%2] %3").arg(tr(strTable.c_str()))
			.arg(tr(strKey.c_str())).arg(toml::format(Value).c_str()).toUtf8().data());
	}
	set<pair<string, string>> set_null;
	set_Changed.swap(set_null);
	return true;
}

void Dialog_Setting_Water::setupLineEditValidators(QWidget* dialog)
{
	const QList<QLineEdit*> lineEdits = dialog->findChildren<QLineEdit*>();
	for (QLineEdit* lineEdit : lineEdits)
	{
		QString objectName = lineEdit->objectName();
		QStringList parts = objectName.split('_');
		if (parts.size() >= 3)
		{
			QString typeIndicator = parts[2].left(1);
			if (typeIndicator == "f")
				lineEdit->setValidator(new QRegularExpressionValidator(Regex_Nonnegative_Double(), nullptr));
		}
	}
}

void Dialog_Setting_Water::lineEditChanged()
{
	QLineEdit*	lineEdit	= qobject_cast<QLineEdit*>(sender());
	QString		qstChanged	= lineEdit->objectName();

	QStringList parts		= qstChanged.split("_");
	string		strTable	= parts.at(parts.size() - 2).toStdString();
	string		strKey		= parts.at(parts.size() - 1).toStdString();

	set_Changed.insert(make_pair(strTable, strKey));
}

void Dialog_Setting_Water::checkBoxChanged()
{
	QCheckBox*	CheckBox	= qobject_cast<QCheckBox*>(sender());
	QString		qstChanged	= CheckBox->objectName();

	QStringList parts		= qstChanged.split("_");
	string		strTable	= parts.at(parts.size() - 2).toStdString();
	string		strKey		= parts.at(parts.size() - 1).toStdString();

	set_Changed.insert(make_pair(strTable, strKey));
}

