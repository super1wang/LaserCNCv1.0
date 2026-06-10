#include "RegexPatterns.h"
#include "Setting_Digital.h"

#include <QLabel>
#include <QLineEdit>
#include <QHBoxLayout>

Dialog_Setting_Digital::Dialog_Setting_Digital(QWidget* parent)
	: QDialog(parent)
	, set_Changed()
{
	ui.setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::FramelessWindowHint);

	// 此处与枚举值定义对应上限，便于代码层编写，临时定义为64
	m_iINRowCount  = 64;
	m_iOUTRowCount = 64;

	setRows(true,  m_iINRowCount);
	setRows(false, m_iOUTRowCount);
}

Dialog_Setting_Digital::~Dialog_Setting_Digital()
{
}

void Dialog_Setting_Digital::InitSetting()
{
	table t_Init = SETTINGS->GetTable(SettingSection::Digital);
	string strID, strNull;
	for (int i = 1; i < m_iINRowCount + 1; i++)
	{
		strID = "aIN" + std::to_string(i);
		t_Init["DigitalIN"][strID] = std::array<string, 2>{ strNull, strNull };
	}

	for (int i = 1; i < m_iOUTRowCount + 1; i++)
	{
		strID = "aOUT" + std::to_string(i);
		t_Init["DigitalOUT"][strID] = std::array<string, 2>{ strNull, strNull };
	}

	SETTINGS->SetTable(true, SettingSection::Digital, t_Init);
}

void Dialog_Setting_Digital::SetPage(table table_Set)
{
	if (!table_Set.size())
		table_Set = SETTINGS->GetTable(SettingSection::Digital);

	QRegularExpression regex("^rowWidget.*");
	const QList<QWidget*> widgetsIN = ui.groupBox_DigitalIN->findChildren<QWidget*>(regex);
	for (QWidget* widgetIN : widgetsIN)
	{
		QStringList parts		= widgetIN->objectName().split('_');
		string		key			= "a" + parts.at(parts.size() - 1).toStdString();
		QString		qstrID		= QString::fromStdString(table_Set["DigitalIN"][key][0].as_string());
		QString		qstrIndex	= QString::fromStdString(table_Set["DigitalIN"][key][1].as_string());

		const QList<QLineEdit*> lineEdits = widgetIN->findChildren<QLineEdit*>();
		for (QLineEdit* lineEdit : lineEdits)
		{
			QString type = lineEdit->objectName().right(1);
			if (type == "D")
				lineEdit->setText(qstrID);
			else
				lineEdit->setText(qstrIndex);
		}
	}

	const QList<QWidget*> widgetsOUT = ui.groupBox_DigitalOUT->findChildren<QWidget*>(regex);
	for (QWidget* widgetOUT : widgetsOUT)
	{
		QStringList parts		= widgetOUT->objectName().split('_');
		string		key			= "a" + parts.at(parts.size() - 1).toStdString();
		QString		qstrID		= QString::fromStdString(table_Set["DigitalOUT"][key][0].as_string());
		QString		qstrIndex	= QString::fromStdString(table_Set["DigitalOUT"][key][1].as_string());

		const QList<QLineEdit*> lineEdits = widgetOUT->findChildren<QLineEdit*>();
		for (QLineEdit* lineEdit : lineEdits)
		{
			QString type = lineEdit->objectName().right(1);
			if (type == "D")
				lineEdit->setText(qstrID);
			else
				lineEdit->setText(qstrIndex);
		}
	}
}

void Dialog_Setting_Digital::GetPage(table& table_Page)
{
	QRegularExpression regex("^rowWidget.*");
	const QList<QWidget*> widgetsIN = ui.groupBox_DigitalIN->findChildren<QWidget*>(regex);
	for (QWidget* widgetIN : widgetsIN)
	{
		QStringList parts = widgetIN->objectName().split('_');
		string key = "a" + parts.at(parts.size() - 1).toStdString();
		string strID, strIndex;

		const QList<QLineEdit*> lineEdits = widgetIN->findChildren<QLineEdit*>();
		for (QLineEdit* lineEdit : lineEdits)
		{
			QString type = lineEdit->objectName().right(1);
			if (type == "D")
				strID = lineEdit->text().toStdString();
			else
				strIndex = lineEdit->text().toStdString();
		}
		table_Page["DigitalIN"][key] = std::array<string, 2>{ strID, strIndex };
	}

	const QList<QWidget*> widgetsOUT = ui.groupBox_DigitalOUT->findChildren<QWidget*>(regex);
	for (QWidget* widgetOUT : widgetsOUT)
	{
		QStringList parts = widgetOUT->objectName().split('_');
		string key = "a" + parts.at(parts.size() - 1).toStdString();
		string strID, strIndex;

		const QList<QLineEdit*> lineEdits = widgetOUT->findChildren<QLineEdit*>();
		for (QLineEdit* lineEdit : lineEdits)
		{
			QString type = lineEdit->objectName().right(1);
			if (type == "D")
				strID = lineEdit->text().toStdString();
			else
				strIndex = lineEdit->text().toStdString();
		}
		table_Page["DigitalOUT"][key] = std::array<string, 2>{ strID, strIndex };
	}
}

bool Dialog_Setting_Digital::GetChanged(table table_Page, table& table_Changed)
{
	if (set_Changed.empty())
		return false;

	for (auto it = set_Changed.begin(); it != set_Changed.end(); ++it)
	{
		string	strTable = it->first;
		string	strKey	 = it->second;
		value	Value	 = table_Page[strTable][strKey];
		table_Changed[strTable][strKey] = Value;
		SETTINGS->SetKeyValue(strKey, Value, SettingSection::Digital, strTable);
		LOG_OPER_INFO(tr("Setting [Digital][%1][%2] %3").arg(tr(strTable.c_str()))
			.arg(tr(strKey.c_str())).arg(toml::format(Value).c_str()).toUtf8().data());
	}
	set<pair<string, string>> set_null;
	set_Changed.swap(set_null);
	return true;
}

void Dialog_Setting_Digital::lineEditChanged()
{
	// 特殊获取，从控件所在的widget获取
	QLineEdit*	lineEdit = qobject_cast<QLineEdit*>(sender());
	QStringList parts	 = lineEdit->parentWidget()->objectName().split('_');
	string		strTable = parts.at(parts.size() - 2).toStdString();
	string		strKey	 = "a" + parts.at(parts.size() - 1).toStdString();	// 每组的索引，即key值，如IN1 OUT12
	set_Changed.insert(make_pair(strTable, strKey));
}

void Dialog_Setting_Digital::addRow(bool bIN)
{
	int rowNumber = bIN ? m_iINRowCount + 1 : m_iOUTRowCount + 1;

	QWidget* rowWidget = new QWidget();
	QHBoxLayout* rowLayout = new QHBoxLayout(rowWidget);
	rowLayout->setContentsMargins(2, 2, 2, 2);

	QString qstrLabelName	= QString("label_%1%2")			.arg(bIN ? "DigitalIN_IN"  : "DigitalOUT_OUT")	.arg(rowNumber);
	QString qstrIDName		= QString("lineEdit_%1%2ID")	.arg(bIN ? "DigitalIN_sIN" : "DigitalOUT_sOUT")	.arg(rowNumber);
	QString qstrIndexName	= QString("lineEdit_%1%2Index")	.arg(bIN ? "DigitalIN_sIN" : "DigitalOUT_sOUT")	.arg(rowNumber);

	QLabel* label = new QLabel(QString("%1 %2").arg(bIN ? tr("IN") : tr("OUT")).arg(rowNumber));
	label->setObjectName(qstrLabelName);
	label->setFixedWidth(80);

	QLineEdit* lineEditID = new QLineEdit();
	lineEditID->setObjectName(qstrIDName);
	connect(lineEditID, SIGNAL(editingFinished()), this, SLOT(lineEditChanged()));

	QLineEdit* lineEditIndex = new QLineEdit();
	lineEditIndex->setObjectName(qstrIndexName);
	lineEditIndex->setValidator(new QRegularExpressionValidator(Regex_Digital_Index(), nullptr));
	connect(lineEditIndex, SIGNAL(editingFinished()), this, SLOT(lineEditChanged()));

	rowLayout->addWidget(label);
	rowLayout->addWidget(lineEditID);
	rowLayout->addWidget(lineEditIndex);

	rowWidget->setObjectName(QString("rowWidget_%1%2").arg(bIN ? "DigitalIN_IN" : "DigitalOUT_OUT").arg(rowNumber));

	if (bIN) {
		ui.verticalLayout_DigitalIN->addWidget(rowWidget);
		m_iINRowCount = rowNumber;
	}
	else {
		ui.verticalLayout_DigitalOUT->addWidget(rowWidget);
		m_iOUTRowCount = rowNumber;
	}
}

void Dialog_Setting_Digital::setRows(bool bIN, int totalRows)
{
	// 重置行计数
	QVBoxLayout* scrollLayout;
	if (bIN) {
		scrollLayout = ui.verticalLayout_DigitalIN;
		m_iINRowCount = 0;
	}
	else {
		scrollLayout = ui.verticalLayout_DigitalOUT;
		m_iOUTRowCount = 0;
	}

	// 清空现有内容
	QLayoutItem* item;
	while ((item = scrollLayout->takeAt(0)) != nullptr) {
		delete item->widget();
		delete item;
	}

	// 添加指定数量的行
	for (int i = 1; i <= totalRows; ++i) {
		addRow(bIN);
	}
}

void Dialog_Setting_Digital::SetIDEnabled(bool bEnabled)
{
	QRegularExpression regex(".*D$");
	const QList<QLineEdit*> lineEdits = this->findChildren<QLineEdit*>(regex);
	for (QLineEdit* lineEdit : lineEdits){
		lineEdit->setEnabled(bEnabled);
	}
}

void Dialog_Setting_Digital::SetIndexEnabled(bool bEnabled)
{
	QRegularExpression regex(".*x$");
	const QList<QLineEdit*> lineEdits = this->findChildren<QLineEdit*>(regex);
	for (QLineEdit* lineEdit : lineEdits) {
		lineEdit->setEnabled(bEnabled);
	}
}
