#include "RegexPatterns.h"
#include "Setting_Axis.h"

Dialog_Setting_Axis::Dialog_Setting_Axis(QWidget* parent)
	: QDialog(parent)
	, set_Changed()
{
	ui.setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::FramelessWindowHint);
	setupLineEditValidators(this);

	connect(ui.comboBox_Axis_sAxis,					SIGNAL(activated(int)),		this, SLOT(comboBoxChanged()));
	connect(ui.comboBox_Axis_sAxis,					SIGNAL(activated(int)),		this, SLOT(UpdatePage()));
}

Dialog_Setting_Axis::~Dialog_Setting_Axis()
{
}

void Dialog_Setting_Axis::CreatAxis(string strAxis, table& t_Init)
{
	t_Init[strAxis]["fLowSpeed"]		= 3.0;
	t_Init[strAxis]["fMediumSpeed"]		= 5.0;
	t_Init[strAxis]["fHighSpeed"]		= 10.0;
	t_Init[strAxis]["fPipeDiameter"]	= 1.6;
}

void Dialog_Setting_Axis::InitSetting()
{
	table t_Init;
	t_Init["Axis"]["sAxis"] = "X";

	ui.comboBox_Axis_sAxis->blockSignals(true);
	for (int i = 0; i < 8; i++)
	{
		if (DT::IsAxisUse((Axis)i))
		{
			string sAxis = enum_name((Axis)i).data();
			CreatAxis(sAxis, t_Init);
			ui.comboBox_Axis_sAxis->addItem(QString::fromUtf8(sAxis.c_str()));
		}
	}
	ui.comboBox_Axis_sAxis->blockSignals(false);

	SETTINGS->SetTable(true, SettingSection::Axis, t_Init);
}

void Dialog_Setting_Axis::SetPage(table table_Set)
{
	if (!table_Set.size())
		table_Set = SETTINGS->GetTable(SettingSection::Axis);

	str_Axis = table_Set["Axis"]["sAxis"].as_string();
	table_Temp["Axis"]["sAxis"] = str_Axis;
	
	ui.comboBox_Axis_sAxis	->setCurrentText(QString::fromStdString(str_Axis));
	ui.groupBox_Speed		->setTitle		(QString::fromStdString(str_Axis) + tr(" Speed"));
	ui.groupBox_PipeDiameter->setTitle		(QString::fromStdString(str_Axis) + tr(" PipeDiameter"));

	QStringList parts;
	for (QLineEdit* lineEdit : m_qlLineEditF)
	{
		parts = lineEdit->objectName().split('_');
		lineEdit->setText(QString::number(table_Set[str_Axis][parts[2].toStdString()].as_floating(), 'g', 16));
	}

	// 管径显隐
	bool bRotation;
	SETTINGS->GetKeyValue("bRotation", bRotation, SettingSection::MotionControl, str_Axis);
	if (bRotation)
		ui.groupBox_PipeDiameter->setHidden(false);
	else
		ui.groupBox_PipeDiameter->setHidden(true);
}

void Dialog_Setting_Axis::GetPage(table& table_Page)
{
	QStringList parts;
	for (QLineEdit* lineEdit : m_qlLineEditF)
	{
		parts = lineEdit->objectName().split('_');
		table_Temp[str_Axis][parts[2].toStdString()] = lineEdit->text().toDouble();
	}

	string str_AxisNew = ui.comboBox_Axis_sAxis->currentText().toStdString();
	if (str_Axis != str_AxisNew)
		table_Temp["Axis"]["sAxis"] = str_AxisNew;

	table_Page = table_Temp;
}

bool Dialog_Setting_Axis::GetChanged(table table_Page, table& table_Changed)
{
	if (set_Changed.empty())
		return false;

	for (auto it = set_Changed.begin(); it != set_Changed.end(); ++it)
	{
		string	strTable = it->first;
		string	strKey   = it->second;
		value	Value	 = table_Page[strTable][strKey];
		table_Changed[strTable][strKey] = Value;
		SETTINGS->SetKeyValue(strKey, Value, SettingSection::Axis, strTable);
		LOG_OPER_INFO(tr("Setting [Axis][%1][%2] %3").arg(tr(strTable.c_str()))
			.arg(tr(strKey.c_str())).arg(toml::format(Value).c_str()).toUtf8().data());
	}
	set<pair<string, string>> set_null;
	set_Changed.swap(set_null);
	return true;
}

void Dialog_Setting_Axis::UpdatePage()
{
	// 获取切换前轴的参数
	GetPage(table_Temp);
	// 更新当前选择轴系
	str_Axis = ui.comboBox_Axis_sAxis->currentText().toStdString();
	table_Temp["Axis"]["sAxis"] = str_Axis;
	if (table_Temp[str_Axis].is_empty())
		table_Temp[str_Axis] = SETTINGS->GetTable(SettingSection::Axis, str_Axis);
	// 设置选择轴系参数
	SetPage(table_Temp);
}

void Dialog_Setting_Axis::setupLineEditValidators(QWidget* dialog)
{
	const QList<QLineEdit*> lineEdits = dialog->findChildren<QLineEdit*>();
	for (QLineEdit* lineEdit : lineEdits)
	{
		QStringList parts = lineEdit->objectName().split('_');
		if (parts.size() >= 3)
		{
			if (parts[2].left(1) == "f")
			{
				lineEdit->setValidator(new QRegularExpressionValidator(Regex_Nonnegative_Double(), nullptr));
				m_qlLineEditF.append(lineEdit);
			}
			connect(lineEdit, SIGNAL(editingFinished()), this, SLOT(lineEditChanged()));
		}
	}
}

void Dialog_Setting_Axis::lineEditChanged()
{
	QLineEdit*  lineEdit	= qobject_cast<QLineEdit*>(sender());
	QString		qstChanged	= lineEdit->objectName();

	QStringList parts		= qstChanged.split("_");
	string		strTable	= parts.at(parts.size() - 2).toStdString();
	if (strTable != "Axis")
		strTable = str_Axis;
	string		strKey		= parts.at(parts.size() - 1).toStdString();

	set_Changed.insert(make_pair(strTable, strKey));
}

void Dialog_Setting_Axis::comboBoxChanged()
{
	QComboBox*	ComboBox	= qobject_cast<QComboBox*>(sender());
	QString		qstChanged	= ComboBox->objectName();

	QStringList parts = qstChanged.split("_");
	string		strTable	= parts.at(parts.size() - 2).toStdString();
	if (strTable != "Axis")
		strTable = str_Axis;
	string		strKey		= parts.at(parts.size() - 1).toStdString();

	set_Changed.insert(make_pair(strTable, strKey));
}
