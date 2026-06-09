#include "Setting_Internet.h"

Dialog_Setting_Internet::Dialog_Setting_Internet(QWidget* parent)
	: QDialog(parent)
	, set_Changed()
{
	ui.setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::FramelessWindowHint);
	setupLineEditValidators(this);

	connect(ui.comboBox_Internet_sProtocol, SIGNAL(activated(int)),		this, SLOT(comboBoxChanged()));
	connect(ui.lineEdit_Internet_sIP,		SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Internet_sPort,		SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Internet_sCommand,	SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
}

Dialog_Setting_Internet::~Dialog_Setting_Internet()
{
}

void Dialog_Setting_Internet::InitSetting()
{
	table t_Init;
	t_Init["Internet"]["sProtocol"] = "TCP";
	t_Init["Internet"]["sIP"]		= "127.0.0.1";
	t_Init["Internet"]["sPort"]		= "22";
	t_Init["Internet"]["sCommand"]	= "NULL";

	//SETTINGS->SetTable(t_Init, SettingSection::Internet);
	SETTINGS->SetTable(true, SettingSection::Internet, t_Init);
}

void Dialog_Setting_Internet::SetPage(table table_Set)
{
	if (!table_Set.size())
		table_Set = SETTINGS->GetTable(SettingSection::Internet);

	ui.comboBox_Internet_sProtocol	->setCurrentText(QString::fromStdString(table_Set["Internet"]["sProtocol"]	.as_string()));
	ui.lineEdit_Internet_sIP		->setText		(QString::fromStdString(table_Set["Internet"]["sIP"]		.as_string()));
	ui.lineEdit_Internet_sPort		->setText		(QString::fromStdString(table_Set["Internet"]["sPort"]		.as_string()));
	ui.lineEdit_Internet_sCommand	->setText		(QString::fromStdString(table_Set["Internet"]["sCommand"]	.as_string()));
}

void Dialog_Setting_Internet::GetPage(table& table_Page)
{
	table_Page["Internet"]["sProtocol"] = ui.comboBox_Internet_sProtocol->currentText().toStdString();
	table_Page["Internet"]["sIP"]		= ui.lineEdit_Internet_sIP		->text().toStdString();
	table_Page["Internet"]["sPort"]		= ui.lineEdit_Internet_sPort	->text().toStdString();
	table_Page["Internet"]["sCommand"]	= ui.lineEdit_Internet_sCommand	->text().toStdString();
}

bool Dialog_Setting_Internet::GetChanged(table table_Page, table & table_Changed)
{
	if (set_Changed.empty())
		return false;

	for (auto it = set_Changed.begin(); it != set_Changed.end(); ++it)
	{
		string	strTable = it->first;
		string	strKey	 = it->second;
		value	Value	 = table_Page[strTable][strKey];
		table_Changed[strTable][strKey] = Value;
		SETTINGS->SetKeyValue(strKey, Value, SettingSection::Internet, strTable);
		LOG_OPER_INFO(tr("Setting [Internet][%1][%2] %3").arg(tr(strTable.c_str()))
			.arg(tr(strKey.c_str())).arg(toml::format(Value).c_str()).toUtf8().data());
	}
	set<pair<string, string>> set_null;
	set_Changed.swap(set_null);
	return true;
}

void Dialog_Setting_Internet::setupLineEditValidators(QWidget* dialog)
{
	const QList<QLineEdit*> lineEdits = dialog->findChildren<QLineEdit*>();
	for (QLineEdit* lineEdit : lineEdits)
	{
		QString objectName = lineEdit->objectName();
		QStringList parts = objectName.split('_');
		if (parts.size() >= 3)
		{
			QString typeIndicator = parts[2].left(1);
			if (parts[2] == "sIP")
				lineEdit->setValidator(new QRegularExpressionValidator(Regex_Internet_IP(, nullptr)));
			else if (parts[2] == "sPort")
				lineEdit->setValidator(new QRegularExpressionValidator(Regex_Internet_Port(, nullptr)));
			else if (typeIndicator == "s")
				lineEdit->setValidator(new QRegularExpressionValidator(Regex_Normal_String(, nullptr)));
		}
	}
}

void Dialog_Setting_Internet::lineEditChanged()
{
	QLineEdit*	lineEdit	= qobject_cast<QLineEdit*>(sender());
	QString		qstChanged	= lineEdit->objectName();

	QStringList parts		= qstChanged.split("_");
	string		strTable	= parts.at(parts.size() - 2).toStdString();
	string		strKey		= parts.at(parts.size() - 1).toStdString();

	set_Changed.insert(make_pair(strTable, strKey));
}

void Dialog_Setting_Internet::comboBoxChanged()
{
	QComboBox*	ComboBox	= qobject_cast<QComboBox*>(sender());
	QString		qstChanged	= ComboBox->objectName();

	QStringList parts		= qstChanged.split("_");
	string		strTable	= parts.at(parts.size() - 2).toStdString();
	string		strKey		= parts.at(parts.size() - 1).toStdString();

	set_Changed.insert(make_pair(strTable, strKey));
}

