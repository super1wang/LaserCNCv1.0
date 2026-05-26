#include "Setting_Laser.h"

Dialog_Setting_Laser::Dialog_Setting_Laser(QWidget* parent)
	: QDialog(parent)
	, set_Changed()
{
	ui.setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::FramelessWindowHint);
	setupLineEditValidators(this);
	setupComboBoxValidators(this);

	connect(ui.comboBox_Laser_sType,				SIGNAL(activated(int)),		this, SLOT(UpdatePage()));
	connect(ui.checkBox_SignalSource_bSignal,		SIGNAL(clicked()),			this, SLOT(UpdatePage()));
}

Dialog_Setting_Laser::~Dialog_Setting_Laser()
{
}

void Dialog_Setting_Laser::InitSetting()
{
	table t_Init;
	t_Init["Laser"]["sType"]					= "Simulator";
	t_Init["Laser"]["fEnergy"]					= 20.0;
	t_Init["Laser"]["fFrequency"]				= 20.0;
	t_Init["Laser"]["fPulseWidth"]				= 20.0;
	t_Init["Laser"]["fResolution"]				= 0.0;
	t_Init["Laser"]["fAttenuatorPercentage"]	= 30.0;
	t_Init["Laser"]["fPpDivider"]				= 2.0;
	t_Init["Laser"]["iDelay"]					= 0;

	t_Init["ComSetting"]["sPort"]			= "COM1";
	t_Init["ComSetting"]["sBaudRate"]		= "9600";
	t_Init["ComSetting"]["sDataBits"]		= "8";
	t_Init["ComSetting"]["sParity"]			= "NONE";
	t_Init["ComSetting"]["sStopBits"]		= "1";

	t_Init["SignalSource"]["bSignal"]		= false;
	t_Init["SignalSource"]["sPort"]			= "COM1";
	t_Init["SignalSource"]["sBaudRate"]		= "9600";
	t_Init["SignalSource"]["sDataBits"]		= "8";
	t_Init["SignalSource"]["sParity"]		= "NONE";
	t_Init["SignalSource"]["sStopBits"]		= "1";

	t_Init["HTTP"]["sHost"]					= "127.0.0.1";
	t_Init["HTTP"]["iPort"]					= 20020;
	t_Init["HTTP"]["sPath"]					= "v1/Basic";
	t_Init["HTTP"]["iTimeOut"]				= 1000;

	//SETTINGS->SetTable(t_Init, SettingSection::Laser);
	SETTINGS->SetTable(true, SettingSection::Laser, t_Init);
}

void Dialog_Setting_Laser::SetPage(table table_Set)
{
	if (!table_Set.size())
		table_Set = SETTINGS->GetTable(SettingSection::Laser);

	QSignalBlocker blockerTemp(ui.checkBox_SignalSource_bSignal);
	ui.checkBox_SignalSource_bSignal->setChecked(table_Set["SignalSource"]["bSignal"].as_boolean());

	QStringList parts;
	for (QComboBox* comboBox : m_qlComboBoxS)
	{
		parts = comboBox->objectName().split('_');
		comboBox->setCurrentText(QString::fromStdString(table_Set[parts[1].toStdString()][parts[2].toStdString()].as_string()));
	}

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
		lineEdit->setText(QString::number(table_Set[parts[1].toStdString()][parts[2].toStdString()].as_floating(), 'g', 3));
	}

	UpdatePage();
}

void Dialog_Setting_Laser::GetPage(table& table_Page)
{
	table_Page["SignalSource"]["bSignal"] = ui.checkBox_SignalSource_bSignal->isChecked();

	QStringList parts;
	for (QComboBox* comboBox : m_qlComboBoxS)
	{
		parts = comboBox->objectName().split('_');
		table_Page[parts[1].toStdString()][parts[2].toStdString()] = comboBox->currentText().toStdString();
	}

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

bool Dialog_Setting_Laser::GetChanged(table table_Page, table& table_Changed)
{
	if (set_Changed.empty())
		return false;

	for (auto it = set_Changed.begin(); it != set_Changed.end(); ++it)
	{
		string	strTable = it->first;
		string	strKey	 = it->second;
		value	Value	 = table_Page[strTable][strKey];
		table_Changed[strTable][strKey] = Value;
		SETTINGS->SetKeyValue(strKey, Value, SettingSection::Laser, strTable);
		LOG_OPER_INFO(tr("Setting [Laser][%1][%2] %3").arg(tr(strTable.c_str()))
			.arg(tr(strKey.c_str())).arg(toml::format(Value).c_str()).toUtf8().data());
	}
	set<pair<string, string>> set_null;
	set_Changed.swap(set_null);
	return true;
}

void Dialog_Setting_Laser::UpdatePage()
{
	QWidget* qlPharosWidgets[] =
	{
		ui.label_Laser_AttenuatorPercentage,
		ui.lineEdit_Laser_fAttenuatorPercentage,
		ui.label_Laser_PpDivider,
		ui.lineEdit_Laser_fPpDivider,
		ui.label_Laser_Delay,
		ui.lineEdit_Laser_iDelay
	};
	QWidget* qlNormalWidgets[] =
	{
		ui.checkBox_SignalSource_bSignal,
		ui.label_Laser_fEnergy,
		ui.lineEdit_Laser_fEnergy,
		ui.label_Laser_Resolution,
		ui.lineEdit_Laser_fResolution,
		ui.label_Laser_Frequency,
		ui.lineEdit_Laser_fFrequency,
		ui.label_Laser_Pulse,
		ui.lineEdit_Laser_fPulseWidth
	};
	for (QWidget* pWidget : qlPharosWidgets)
		pWidget->setHidden(true);
	for (QWidget* pWidget : qlNormalWidgets)
		pWidget->setHidden(false);
	ui.groupBox_HTTP->setHidden(true);

	bool bIsFactory = int(DT::getPermission()) >= (int)PermissionLevel::Factory;
	bool bSignalSource = ui.checkBox_SignalSource_bSignal->isChecked();
	string strMotionControlType;
	SETTINGS->GetKeyValue("sType", strMotionControlType, SettingSection::MotionControl, "MotionControl");
	if (bSignalSource)
		ui.groupBox_SignalSource->setHidden(strMotionControlType == "GTN");
	else
		ui.groupBox_SignalSource->setHidden(true);

	string strType = ui.comboBox_Laser_sType->currentText().toStdString();
	bool bIsPharos = (strType == "Pharos");
	bool bAnalogControl = (strType == "AnalogControl");

	ui.label_Laser_Resolution->setHidden(!(bAnalogControl || bIsPharos));
	ui.lineEdit_Laser_fResolution->setHidden(!(bAnalogControl || bIsPharos));

	if (strType == "Simulator")
	{
		ui.groupBox_ComSetting->setHidden(true);
		ui.label_Laser_Pulse->setText(tr("Pulse(μs)"));
		set_Changed.insert(make_pair("SignalSource", "bSignal"));
	}
	else if (bAnalogControl)
	{
		ui.groupBox_ComSetting->setHidden(true);
		ui.label_Laser_Pulse->setText(tr("Pulse(μs)"));
		ui.groupBox_HTTP->setHidden(true);
	}
	else if (strType == "ULTRON")
	{
		ui.groupBox_ComSetting->setHidden(false);
		ui.label_Laser_Pulse->setText(tr("PulsePickerDivider"));
	}
	else
	{
		ui.groupBox_ComSetting->setHidden(false);
		ui.label_Laser_Pulse->setText(tr("Pulse(μs)"));
	}
	if (!bIsFactory)
	{
		ui.groupBox_ComSetting->setHidden(true);
		ui.groupBox_SignalSource->setHidden(true);
	}

	if (bIsPharos)
	{
		for (QWidget* pWidget : qlNormalWidgets)
			pWidget->setHidden(true);
		for (QWidget* pWidget : qlPharosWidgets)
			pWidget->setHidden(false);

		ui.groupBox_ComSetting->setHidden(true);
		ui.groupBox_SignalSource->setHidden(true);
		ui.groupBox_HTTP->setHidden(!bIsFactory);
	}
}

void Dialog_Setting_Laser::setupLineEditValidators(QWidget* dialog)
{
	const QList<QLineEdit*> lineEdits = dialog->findChildren<QLineEdit*>();
	static const QRegExp Regex_HTTP_IP("^((25[0-5]|2[0-4]\\d|1\\d\\d|[1-9]?\\d)\\.){3}(25[0-5]|2[0-4]\\d|1\\d\\d|[1-9]?\\d)$");
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
				lineEdit->setValidator(new QRegExpValidator(Regex_All_Int));
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

	ui.lineEdit_HTTP_sHost		->setValidator(new QRegExpValidator(Regex_HTTP_IP));
	ui.lineEdit_HTTP_iPort		->setValidator(new QRegExpValidator(Regex_Nonnegative_Int));
	ui.lineEdit_HTTP_iTimeOut	->setValidator(new QRegExpValidator(Regex_Nonnegative_Int));
	ui.lineEdit_Laser_iDelay	->setValidator(new QRegExpValidator(Regex_Nonnegative_Int));
}

void Dialog_Setting_Laser::setupComboBoxValidators(QWidget* dialog)
{
	const QList<QComboBox*> comboBoxs = dialog->findChildren<QComboBox*>();
	for (QComboBox* comboBox : comboBoxs)
	{
		QStringList parts = comboBox->objectName().split('_');
		if (parts.size() >= 3)
		{
			m_qlComboBoxS.append(comboBox);
			connect(comboBox, SIGNAL(activated(int)), this, SLOT(comboBoxChanged()));
		}
	}
}

void Dialog_Setting_Laser::lineEditChanged()
{
	QLineEdit*	lineEdit	= qobject_cast<QLineEdit*>(sender());
	QString		qstChanged	= lineEdit->objectName();

	QStringList parts		= qstChanged.split("_");
	string		strTable	= parts.at(parts.size() - 2).toStdString();
	string		strKey		= parts.at(parts.size() - 1).toStdString();

	if (lineEdit == ui.lineEdit_HTTP_sHost)
	{
		QString qstrIP = lineEdit->text().trimmed();
		while (qstrIP.endsWith('.'))
			qstrIP.chop(1);

		if (qstrIP != lineEdit->text())
			lineEdit->setText(qstrIP);

		if (!lineEdit->hasAcceptableInput())
		{
			set_Changed.erase(make_pair(strTable, strKey));
			return;
		}
	}

	set_Changed.insert(make_pair(strTable, strKey));
}

void Dialog_Setting_Laser::comboBoxChanged()
{
	QComboBox*	ComboBox	= qobject_cast<QComboBox*>(sender());
	QString		qstChanged	= ComboBox->objectName();

	QStringList parts		= qstChanged.split("_");
	string		strTable	= parts.at(parts.size() - 2).toStdString();
	string		strKey		= parts.at(parts.size() - 1).toStdString();

	set_Changed.insert(make_pair(strTable, strKey));
}

void Dialog_Setting_Laser::checkBoxChanged()
{
	QCheckBox* CheckBox		= qobject_cast<QCheckBox*>(sender());
	QString		qstChanged	= CheckBox->objectName();

	QStringList parts		= qstChanged.split("_");
	string		strTable	= parts.at(parts.size() - 2).toStdString();
	string		strKey		= parts.at(parts.size() - 1).toStdString();

	set_Changed.insert(make_pair(strTable, strKey));
}
