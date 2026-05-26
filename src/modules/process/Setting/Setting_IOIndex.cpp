#include "Setting_IOIndex.h"

Dialog_Setting_IOIndex::Dialog_Setting_IOIndex(QWidget* parent)
	: QDialog(parent)
	, set_Changed()
{
	ui.setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::FramelessWindowHint);
	setupLineEditValidators(this);
}

Dialog_Setting_IOIndex::~Dialog_Setting_IOIndex()
{
}

void Dialog_Setting_IOIndex::InitSetting()
{
	table t_InitDigital;
	t_InitDigital["DigitalOUT"]["aChuck"]				= std::array<string, 2>{ "夹头",		"0.0" };
	t_InitDigital["DigitalOUT"]["aPliers"]				= std::array<string, 2>{ "夹爪",		"0.1" };
	t_InitDigital["DigitalOUT"]["aBlow"]				= std::array<string, 2>{ "吹气",		"0.2" };
	t_InitDigital["DigitalOUT"]["aLaser"]				= std::array<string, 2>{ "激光",		"0.4" };
	t_InitDigital["DigitalOUT"]["aGreenLight"]			= std::array<string, 2>{ "绿灯",		"0.5" };
	t_InitDigital["DigitalOUT"]["aYellowLight"]			= std::array<string, 2>{ "黄灯",		"0.6" };
	t_InitDigital["DigitalOUT"]["aRedLight"]			= std::array<string, 2>{ "红灯",		"0.7" };
	t_InitDigital["DigitalOUT"]["aBuzzer"]				= std::array<string, 2>{ "蜂鸣器",	"1.0" };
	t_InitDigital["DigitalOUT"]["aWater"]				= std::array<string, 2>{ "出水泵",	"0.10" };
	t_InitDigital["DigitalOUT"]["aPump"]				= std::array<string, 2>{ "回水泵",	"0.11" };

	t_InitDigital["DigitalIN"]["aStart"]				= std::array<string, 2>{ "开始",		"0.0" };
	t_InitDigital["DigitalIN"]["aStop"]					= std::array<string, 2>{ "停止",		"0.1" };
	t_InitDigital["DigitalIN"]["aInterLock"]			= std::array<string, 2>{ "互锁",		"0.3" };
	t_InitDigital["DigitalIN"]["aSafetyLightCurtain"]	= std::array<string, 2>{ "安全光幕", "0.7" };
	t_InitDigital["DigitalIN"]["aPressureMonitor"]		= std::array<string, 2>{ "气压监控", "0.2" };
	t_InitDigital["DigitalIN"]["aRemnantsMonitor"]		= std::array<string, 2>{ "余料监控", "0.5" };
	t_InitDigital["DigitalIN"]["aWaterLeakageMonitor"]	= std::array<string, 2>{ "液位监控", "0.4" };
	t_InitDigital["DigitalIN"]["aWaterTankMonitor"]		= std::array<string, 2>{ "水箱监控", "0.6" };

	SETTINGS->SetTable(true, SettingSection::Digital, t_InitDigital);

	table t_InitAnalog;
	t_InitAnalog["AnalogOUT"]["aPressure"]		= std::array<string, 2>{ "气压", "0" };
	t_InitAnalog["AnalogOUT"]["aLaser"]			= std::array<string, 2>{ "激光", "1" };

	t_InitAnalog["AnalogIN"]["aWaterLevel"]		= std::array<string, 2>{ "液位", "0" };
	t_InitAnalog["AnalogIN"]["aWaterPressure"]	= std::array<string, 2>{ "水压", "1" };
	t_InitAnalog["AnalogIN"]["aPressure"]		= std::array<string, 2>{ "气压", "2" };

	SETTINGS->SetTable(true, SettingSection::Analog, t_InitAnalog);
}


void Dialog_Setting_IOIndex::SetPage(table table_SetD, table table_SetA)
{
	if (!table_SetD.size())
		table_SetD = SETTINGS->GetTable(SettingSection::Digital);

	ui.lineEdit_DigitalOUT_aChuck				->setText(QString::fromStdString(table_SetD["DigitalOUT"]["aChuck"][1]				.as_string()));
	ui.lineEdit_DigitalOUT_aPliers				->setText(QString::fromStdString(table_SetD["DigitalOUT"]["aPliers"][1]				.as_string()));
	ui.lineEdit_DigitalOUT_aBlow				->setText(QString::fromStdString(table_SetD["DigitalOUT"]["aBlow"][1]				.as_string()));
	ui.lineEdit_DigitalOUT_aLaser				->setText(QString::fromStdString(table_SetD["DigitalOUT"]["aLaser"][1]				.as_string()));
	ui.lineEdit_DigitalOUT_aGreenLight			->setText(QString::fromStdString(table_SetD["DigitalOUT"]["aGreenLight"][1]			.as_string()));
	ui.lineEdit_DigitalOUT_aYellowLight			->setText(QString::fromStdString(table_SetD["DigitalOUT"]["aYellowLight"][1]		.as_string()));
	ui.lineEdit_DigitalOUT_aRedLight			->setText(QString::fromStdString(table_SetD["DigitalOUT"]["aRedLight"][1]			.as_string()));
	ui.lineEdit_DigitalOUT_aBuzzer				->setText(QString::fromStdString(table_SetD["DigitalOUT"]["aBuzzer"][1]				.as_string()));
	ui.lineEdit_DigitalOUT_aWater				->setText(QString::fromStdString(table_SetD["DigitalOUT"]["aWater"][1]				.as_string()));
	ui.lineEdit_DigitalOUT_aPump				->setText(QString::fromStdString(table_SetD["DigitalOUT"]["aPump"][1]				.as_string()));


	ui.lineEdit_DigitalIN_aStart				->setText(QString::fromStdString(table_SetD["DigitalIN"]["aStart"][1]				.as_string()));
	ui.lineEdit_DigitalIN_aStop					->setText(QString::fromStdString(table_SetD["DigitalIN"]["aStop"][1]				.as_string()));
	ui.lineEdit_DigitalIN_aInterLock			->setText(QString::fromStdString(table_SetD["DigitalIN"]["aInterLock"][1]			.as_string()));
	ui.lineEdit_DigitalIN_aSafetyLightCurtain	->setText(QString::fromStdString(table_SetD["DigitalIN"]["aSafetyLightCurtain"][1]	.as_string()));
	ui.lineEdit_DigitalIN_aPressureMonitor		->setText(QString::fromStdString(table_SetD["DigitalIN"]["aPressureMonitor"][1]		.as_string()));
	ui.lineEdit_DigitalIN_aRemnantsMonitor		->setText(QString::fromStdString(table_SetD["DigitalIN"]["aRemnantsMonitor"][1]		.as_string()));
	ui.lineEdit_DigitalIN_aWaterLeakageMonitor	->setText(QString::fromStdString(table_SetD["DigitalIN"]["aWaterLeakageMonitor"][1]	.as_string()));
	ui.lineEdit_DigitalIN_aWaterTankMonitor		->setText(QString::fromStdString(table_SetD["DigitalIN"]["aWaterTankMonitor"][1]	.as_string()));

	if (!table_SetA.size())
		table_SetA = SETTINGS->GetTable(SettingSection::Analog);

	ui.lineEdit_AnalogOUT_aPressure				->setText(QString::fromStdString(table_SetA["AnalogOUT"]["aPressure"][1]			.as_string()));
	ui.lineEdit_AnalogOUT_aLaser				->setText(QString::fromStdString(table_SetA["AnalogOUT"]["aLaser"][1]				.as_string()));

	ui.lineEdit_AnalogIN_aPressure				->setText(QString::fromStdString(table_SetA["AnalogIN"]["aPressure"][1]				.as_string()));
	ui.lineEdit_AnalogIN_aWaterPressure			->setText(QString::fromStdString(table_SetA["AnalogIN"]["aWaterPressure"][1]		.as_string()));
	ui.lineEdit_AnalogIN_aWaterLevel			->setText(QString::fromStdString(table_SetA["AnalogIN"]["aWaterLevel"][1]			.as_string()));
}

void Dialog_Setting_IOIndex::GetPage(table& table_PageD, table& table_PageA)
{
	table t_SetDO = SETTINGS->GetTable(SettingSection::Digital, "DigitalOUT");
	t_SetDO["aChuck"][1]		= ui.lineEdit_DigitalOUT_aChuck			->text().toStdString();
	t_SetDO["aPliers"][1]		= ui.lineEdit_DigitalOUT_aPliers		->text().toStdString();
	t_SetDO["aBlow"][1]			= ui.lineEdit_DigitalOUT_aBlow			->text().toStdString();
	t_SetDO["aLaser"][1]		= ui.lineEdit_DigitalOUT_aLaser			->text().toStdString();
	t_SetDO["aGreenLight"][1]	= ui.lineEdit_DigitalOUT_aGreenLight	->text().toStdString();
	t_SetDO["aYellowLight"][1]	= ui.lineEdit_DigitalOUT_aYellowLight	->text().toStdString();
	t_SetDO["aRedLight"][1]		= ui.lineEdit_DigitalOUT_aRedLight		->text().toStdString();
	t_SetDO["aBuzzer"][1]		= ui.lineEdit_DigitalOUT_aBuzzer		->text().toStdString();
	t_SetDO["aWater"][1]		= ui.lineEdit_DigitalOUT_aWater			->text().toStdString();
	t_SetDO["aPump"][1]			= ui.lineEdit_DigitalOUT_aPump			->text().toStdString();

	table_PageD["DigitalOUT"]["aChuck"]			= t_SetDO["aChuck"];
	table_PageD["DigitalOUT"]["aPliers"]		= t_SetDO["aPliers"];
	table_PageD["DigitalOUT"]["aBlow"]			= t_SetDO["aBlow"];
	table_PageD["DigitalOUT"]["aLaser"]			= t_SetDO["aLaser"];
	table_PageD["DigitalOUT"]["aGreenLight"]	= t_SetDO["aGreenLight"];
	table_PageD["DigitalOUT"]["aYellowLight"]	= t_SetDO["aYellowLight"];
	table_PageD["DigitalOUT"]["aRedLight"]		= t_SetDO["aRedLight"];
	table_PageD["DigitalOUT"]["aBuzzer"]		= t_SetDO["aBuzzer"];
	table_PageD["DigitalOUT"]["aWater"]			= t_SetDO["aWater"];
	table_PageD["DigitalOUT"]["aPump"]			= t_SetDO["aPump"];

	table t_SetDI = SETTINGS->GetTable(SettingSection::Digital, "DigitalIN");
	t_SetDI["aStart"][1]				= ui.lineEdit_DigitalIN_aStart				->text().toStdString();
	t_SetDI["aStop"][1]					= ui.lineEdit_DigitalIN_aStop				->text().toStdString();
	t_SetDI["aInterLock"][1]			= ui.lineEdit_DigitalIN_aInterLock			->text().toStdString();
	t_SetDI["aSafetyLightCurtain"][1]	= ui.lineEdit_DigitalIN_aSafetyLightCurtain	->text().toStdString();
	t_SetDI["aPressureMonitor"][1]		= ui.lineEdit_DigitalIN_aPressureMonitor	->text().toStdString();
	t_SetDI["aRemnantsMonitor"][1]		= ui.lineEdit_DigitalIN_aRemnantsMonitor	->text().toStdString();
	t_SetDI["aWaterLeakageMonitor"][1]	= ui.lineEdit_DigitalIN_aWaterLeakageMonitor->text().toStdString();
	t_SetDI["aWaterTankMonitor"][1]		= ui.lineEdit_DigitalIN_aWaterTankMonitor	->text().toStdString();

	table_PageD["DigitalIN"]["aStart"]					= t_SetDI["aStart"];	
	table_PageD["DigitalIN"]["aStop"]					= t_SetDI["aStop"];
	table_PageD["DigitalIN"]["aInterLock"]				= t_SetDI["aInterLock"];
	table_PageD["DigitalIN"]["aSafetyLightCurtain"]		= t_SetDI["aSafetyLightCurtain"];
	table_PageD["DigitalIN"]["aPressureMonitor"]		= t_SetDI["aPressureMonitor"];
	table_PageD["DigitalIN"]["aRemnantsMonitor"]		= t_SetDI["aRemnantsMonitor"];
	table_PageD["DigitalIN"]["aWaterLeakageMonitor"]	= t_SetDI["aWaterLeakageMonitor"];
	table_PageD["DigitalIN"]["aWaterTankMonitor"]		= t_SetDI["aWaterTankMonitor"];

	table t_SetAO = SETTINGS->GetTable(SettingSection::Analog, "AnalogOUT");
	t_SetAO["aPressure"][1]	= ui.lineEdit_AnalogOUT_aPressure->text().toStdString();
	t_SetAO["aLaser"][1]	= ui.lineEdit_AnalogOUT_aLaser->text().toStdString();
	table_PageA["AnalogOUT"]["aPressure"]	= t_SetAO["aPressure"];
	table_PageA["AnalogOUT"]["aLaser"]		= t_SetAO["aLaser"];

	table t_SetAI = SETTINGS->GetTable(SettingSection::Analog, "AnalogIN");
	t_SetAI["aPressure"][1]			= ui.lineEdit_AnalogIN_aPressure->text().toStdString();
	t_SetAI["aWaterPressure"][1]	= ui.lineEdit_AnalogIN_aWaterPressure->text().toStdString();
	t_SetAI["aWaterLevel"][1]		= ui.lineEdit_AnalogIN_aWaterLevel->text().toStdString();
	table_PageA["AnalogIN"]["aPressure"]		= t_SetAI["aPressure"];
	table_PageA["AnalogIN"]["aWaterPressure"]	= t_SetAI["aWaterPressure"];
	table_PageA["AnalogIN"]["aWaterLevel"]		= t_SetAI["aWaterLevel"];
}

bool Dialog_Setting_IOIndex::GetChanged(table table_PageD, table table_PageA, table& table_ChangedD, table& table_ChangedA)
{
	if (set_Changed.empty())
		return false;

	for (auto it = set_Changed.begin(); it != set_Changed.end(); ++it)
	{
		string	strTable = it->first;
		string	strKey   = it->second;
		if ((strTable == "DigitalOUT") || (strTable == "DigitalIN"))
		{
			value Value = table_PageD[strTable][strKey];
			table_ChangedD[strTable][strKey] = Value;
			SETTINGS->SetKeyValue(strKey, Value, SettingSection::Digital, strTable);
			LOG_OPER_INFO(tr("Setting [Digital][%1][%2] %3").arg(tr(strTable.c_str()))
				.arg(tr(strKey.c_str())).arg(toml::format(Value).c_str()).toUtf8().data());
		}
		else
		{
			value Value = table_PageA[strTable][strKey];
			table_ChangedA[strTable][strKey] = Value;
			SETTINGS->SetKeyValue(strKey, Value, SettingSection::Analog, strTable);
			LOG_OPER_INFO(tr("Setting [Analog][%1][%2] %3").arg(tr(strTable.c_str()))
				.arg(tr(strKey.c_str())).arg(toml::format(Value).c_str()).toUtf8().data());
		}
	}
	set<pair<string, string>> set_null;
	set_Changed.swap(set_null);
	return true;
}

void Dialog_Setting_IOIndex::setupLineEditValidators(QWidget* dialog)
{
	const QList<QLineEdit*> lineEdits = dialog->findChildren<QLineEdit*>();
	for (QLineEdit* lineEdit : lineEdits)
	{
		QString objectName = lineEdit->objectName();
		QStringList parts = objectName.split('_');
		if (parts.size() >= 3)
		{
			QString typeIndicator = parts[2].left(1);
			if (parts[1] == "DigitalIN" || parts[1] == "DigitalOUT")
			{
				if (typeIndicator == "a")
					lineEdit->setValidator(new QRegExpValidator(Regex_Digital_Index));
			}
			else if(parts[1] == "AnalogIN" || parts[1] == "AnalogOUT")
			{
				if (typeIndicator == "a")
					lineEdit->setValidator(new QRegExpValidator(Regex_Analog_Index));
			}
			else
			{
				if (typeIndicator == "a")
					lineEdit->setValidator(new QRegExpValidator(Regex_Normal_String));
			}
		}
		connect(lineEdit, SIGNAL(editingFinished()), this, SLOT(lineEditChanged()));
	}
}

void Dialog_Setting_IOIndex::lineEditChanged()
{
	QLineEdit* lineEdit		= qobject_cast<QLineEdit*>(sender());
	QString		qstChanged	= lineEdit->objectName();

	QStringList parts		= qstChanged.split("_");
	string		strTable	= parts.at(parts.size() - 2).toStdString();
	string		strKey		= parts.at(parts.size() - 1).toStdString();

	set_Changed.insert(make_pair(strTable, strKey));
}

void Dialog_Setting_IOIndex::SetIndexEnabled(bool bEnabled)
{
	const QList<QLineEdit*> lineEdits = this->findChildren<QLineEdit*>();
	for (QLineEdit* lineEdit : lineEdits) {
		lineEdit->setEnabled(bEnabled);
	}
}
