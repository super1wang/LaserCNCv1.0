#include "RegexPatterns.h"
#include "Setting_MotionControl.h"

Dialog_Setting_MotionControl::Dialog_Setting_MotionControl(QWidget* parent)
	: QDialog(parent)
	, set_Changed()
{
	ui.setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::FramelessWindowHint);
	setupLineEditValidators(this);

	connect(ui.comboBox_MotionControl_sType,		SIGNAL(activated(int)),		this, SLOT(comboBoxChanged()));
	connect(ui.comboBox_MotionControl_sType,		SIGNAL(activated(int)),		this, SLOT(TypeChanged()));
	connect(ui.comboBox_MotionControl_sAxis,		SIGNAL(activated(int)),		this, SLOT(comboBoxChanged()));
	connect(ui.comboBox_MotionControl_sAxis,		SIGNAL(activated(int)),		this, SLOT(UpdatePage()));
	connect(ui.lineEdit_AxisSetting_iIndex,			SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_AxisSetting_iHomeIndex,		SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.comboBox_AxisSetting_bRotation,		SIGNAL(activated(int)),		this, SLOT(comboBoxChanged()));
	connect(ui.lineEdit_AxisSetting_fResolution,	SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_AxisSetting_fVel,			SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_AxisSetting_fAcc,			SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_AxisSetting_fJerk,			SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_AxisSetting_fLeftLimit,		SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_AxisSetting_fRightLimit,	SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
}

Dialog_Setting_MotionControl::~Dialog_Setting_MotionControl()
{
}

void Dialog_Setting_MotionControl::setUI()
{
	string strName;
	SETTINGS->GetKeyValue("sType", strName, SettingSection::MotionControl, "MotionControl");
	if (strName == "GTN")
	{
		ui.label_AxisSetting_HomeIndex->setHidden(true);
		ui.lineEdit_AxisSetting_iHomeIndex->setHidden(true);
		ui.label_AxisSetting_Jerk->setText(tr("smoothTime(ms)"));
		//ui.label_AxisSetting_Resolution->setText(tr("Conversion(pulse)"));
	}
	else
	{
		ui.label_AxisSetting_HomeIndex->setHidden(false);
		ui.lineEdit_AxisSetting_iHomeIndex->setHidden(false);
		ui.label_AxisSetting_Jerk->setText(tr("Jerk(mm/s^3)"));
		//ui.label_AxisSetting_Resolution->setText(tr("Resolution"));
	}
	//TypeChanged();
}

void Dialog_Setting_MotionControl::CreatAxis(string strAxis, int iIndex, table& t_Init)
{
	t_Init[strAxis]["iIndex"]		= iIndex;
	t_Init[strAxis]["iHomeIndex"]	= iIndex;
	t_Init[strAxis]["bRotation"]	= false;
	t_Init[strAxis]["fResolution"]	= 2000.0;
	t_Init[strAxis]["fVel"]			= 10.0;
	t_Init[strAxis]["fAcc"]			= 1000.0;
	t_Init[strAxis]["fJerk"]		= 10000.0;
	t_Init[strAxis]["fLeftLimit"]	= 0.0;
	t_Init[strAxis]["fRightLimit"]	= 50.0;

	// 回零参数（THomePrm），仅初始化默认值，不通过界面编辑
	t_Init[strAxis]["Home"]["iMode"]				= 10;		// HOME_MODE_LIMIT
	t_Init[strAxis]["Home"]["iMoveDir"]				= -1;		// 负方向搜索限位
	t_Init[strAxis]["Home"]["iIndexDir"]			= -1;		// 负方向搜索Index
	t_Init[strAxis]["Home"]["iEdge"]				= 0;		// 下降沿
	t_Init[strAxis]["Home"]["iTriggerIndex"]		= -1;		// 使用本轴触发器
	t_Init[strAxis]["Home"]["fVelHigh"]				= 5.0;		// 搜索Home速度(pulse/ms)
	t_Init[strAxis]["Home"]["fVelLow"]				= 1.0;		// 搜索Index速度(pulse/ms)
	t_Init[strAxis]["Home"]["fAcc"]					= 50.0;		// 加速度(pulse/ms^2)
	t_Init[strAxis]["Home"]["fDec"]					= 50.0;		// 减速度(pulse/ms^2)
	t_Init[strAxis]["Home"]["iSmoothTime"]			= 0;		// 平滑时间(ms)
	t_Init[strAxis]["Home"]["iHomeOffset"]			= 0;		// 原点偏移(pulse)
	t_Init[strAxis]["Home"]["iSearchHomeDistance"]	= 0;		// Home最大搜索距离，0不限制
	t_Init[strAxis]["Home"]["iSearchIndexDistance"]	= 0;		// Index最大搜索距离，0不限制
	t_Init[strAxis]["Home"]["iEscapeStep"]			= 20000;	// 脱离限位步长(pulse)
}

void Dialog_Setting_MotionControl::InitSetting()
{
	table t_Init;
	t_Init["MotionControl"]["sType"] = "SimulatorCMHP";
	t_Init["MotionControl"]["sAxis"] = "X";

	ui.comboBox_MotionControl_sAxis->blockSignals(true);
	for(int i = 0; i < 8; i++)
	{
		if (DT::IsAxisUse((Axis)i))
		{
			string sAxis = enum_name((Axis)i).data();
			CreatAxis(sAxis, i, t_Init);
			ui.comboBox_MotionControl_sAxis->addItem(QString::fromStdString(sAxis));
		}
	}
	ui.comboBox_MotionControl_sAxis->blockSignals(false);

	SETTINGS->SetTable(true, SettingSection::MotionControl, t_Init);
}

void Dialog_Setting_MotionControl::SetPage(table table_Set)
{
	if (!table_Set.size())
		table_Set = SETTINGS->GetTable(SettingSection::MotionControl);

	str_Axis = table_Set["MotionControl"]["sAxis"].as_string();
	table_Temp["MotionControl"]["sAxis"] = str_Axis;
	int iRotation = 0;
	if (table_Set[str_Axis]["bRotation"].as_boolean())
		iRotation = 1;

	ui.comboBox_MotionControl_sType		->setCurrentText	(QString::fromStdString(table_Set["MotionControl"]["sType"].as_string()));
	ui.comboBox_MotionControl_sAxis		->setCurrentText	(QString::fromStdString(str_Axis));
	
	ui.groupBox_AxisSetting				->setTitle			(tr("Axis ") + QString::fromStdString(str_Axis));
	ui.lineEdit_AxisSetting_iIndex		->setText			(QString::number(table_Set[str_Axis]["iIndex"]		.as_integer(), 'g', 16));
	ui.lineEdit_AxisSetting_iHomeIndex	->setText			(QString::number(table_Set[str_Axis]["iHomeIndex"]	.as_integer(),  'g', 16));
	ui.comboBox_AxisSetting_bRotation	->setCurrentIndex	(iRotation);
	ui.lineEdit_AxisSetting_fResolution	->setText			(QString::number(table_Set[str_Axis]["fResolution"]	.as_floating(), 'g', 16));
	ui.lineEdit_AxisSetting_fVel		->setText			(QString::number(table_Set[str_Axis]["fVel"]		.as_floating(), 'g', 16));
	ui.lineEdit_AxisSetting_fAcc		->setText			(QString::number(table_Set[str_Axis]["fAcc"]		.as_floating(), 'g', 16));
	ui.lineEdit_AxisSetting_fJerk		->setText			(QString::number(table_Set[str_Axis]["fJerk"]		.as_floating(), 'g', 16));
	ui.lineEdit_AxisSetting_fLeftLimit	->setText			(QString::number(table_Set[str_Axis]["fLeftLimit"]	.as_floating(), 'g', 16));
	ui.lineEdit_AxisSetting_fRightLimit	->setText			(QString::number(table_Set[str_Axis]["fRightLimit"]	.as_floating(), 'g', 16));
}

void Dialog_Setting_MotionControl::GetPage(table& table_Page)
{	
	table_Temp["MotionControl"]["sType"] = ui.comboBox_MotionControl_sType->currentText().toStdString();

	bool bRotation = false;
	if (ui.comboBox_AxisSetting_bRotation->currentIndex())
		bRotation = true;

	// 保存修改前的值
	table_Temp[str_Axis]["iIndex"]		= ui.lineEdit_AxisSetting_iIndex		->text().toInt();
	table_Temp[str_Axis]["iHomeIndex"]	= ui.lineEdit_AxisSetting_iHomeIndex	->text().toInt();
	table_Temp[str_Axis]["bRotation"]	= bRotation;
	table_Temp[str_Axis]["fResolution"] = ui.lineEdit_AxisSetting_fResolution	->text().toDouble();
	table_Temp[str_Axis]["fVel"]		= ui.lineEdit_AxisSetting_fVel			->text().toDouble();
	table_Temp[str_Axis]["fAcc"]		= ui.lineEdit_AxisSetting_fAcc			->text().toDouble();
	table_Temp[str_Axis]["fJerk"]		= ui.lineEdit_AxisSetting_fJerk			->text().toDouble();
	table_Temp[str_Axis]["fLeftLimit"]	= ui.lineEdit_AxisSetting_fLeftLimit	->text().toDouble();
	table_Temp[str_Axis]["fRightLimit"]	= ui.lineEdit_AxisSetting_fRightLimit	->text().toDouble();

	string str_AxisNew = ui.comboBox_MotionControl_sAxis->currentText().toStdString();
	if (str_Axis != str_AxisNew)
		table_Temp["MotionControl"]["sAxis"] = str_AxisNew;

	table_Page = table_Temp;
}

bool Dialog_Setting_MotionControl::GetChanged(table table_Page, table& table_Changed)
{
	if (set_Changed.empty())
		return false;

	for (auto it = set_Changed.begin(); it != set_Changed.end(); ++it)
	{
		string	strTable = it->first;
		string	strKey = it->second;
		value	Value = table_Page[strTable][strKey];
		table_Changed[strTable][strKey] = Value;
		SETTINGS->SetKeyValue(strKey, Value, SettingSection::MotionControl, strTable);
		LOG_OPER_INFO(tr("Setting [MotionControl][%1][%2] %3").arg(tr(strTable.c_str()))
			.arg(tr(strKey.c_str())).arg(toml::format(Value).c_str()).toUtf8().data());
	}
	set<pair<string, string>> set_null;
	set_Changed.swap(set_null);
	return true;
}

void Dialog_Setting_MotionControl::UpdatePage()
{
	// 获取切换前轴的参数
	GetPage(table_Temp);
	// 更新当前选择轴系
	str_Axis = ui.comboBox_MotionControl_sAxis->currentText().toStdString();
	table_Temp["MotionControl"]["sAxis"] = str_Axis;
	if (table_Temp[str_Axis].is_empty())
		table_Temp[str_Axis] = SETTINGS->GetTable(SettingSection::MotionControl, str_Axis);
	// 设置选择轴系参数
	SetPage(table_Temp);
}

void Dialog_Setting_MotionControl::setupLineEditValidators(QWidget* dialog)
{
	const QList<QLineEdit*> lineEdits = dialog->findChildren<QLineEdit*>();
	for (QLineEdit* lineEdit : lineEdits)
	{
		QString objectName = lineEdit->objectName();
		QStringList parts = objectName.split('_');
		if (parts.size() >= 3)
		{
			QString typeIndicator = parts[2].left(1);
			if (parts[2] == "fLeftLimit" || parts[2] == "fRightLimit")
				lineEdit->setValidator(new QRegularExpressionValidator(Regex_All_Double(), nullptr));
			else if (typeIndicator == "f")
				lineEdit->setValidator(new QRegularExpressionValidator(Regex_Nonnegative_Double(), nullptr));
			else if (typeIndicator == "i")
				lineEdit->setValidator(new QRegularExpressionValidator(Regex_Nonnegative_Int(), nullptr));
		}
	}
}

void Dialog_Setting_MotionControl::TypeChanged()
{
	string strType = ui.comboBox_MotionControl_sType->currentText().toStdString();
	string strName;
	SETTINGS->GetKeyValue("sType", strName, SettingSection::MotionControl, "MotionControl");
	if (strType == "GTN")
	{
		ui.label_AxisSetting_HomeIndex->setHidden(true);
		ui.lineEdit_AxisSetting_iHomeIndex->setHidden(true);
		ui.label_AxisSetting_Jerk->setText(tr("smoothTime(ms)"));
		ui.label_AxisSetting_Resolution->setText(tr("Conversion(pulse)"));
	}
	else
	{
		ui.label_AxisSetting_HomeIndex->setHidden(false);
		ui.lineEdit_AxisSetting_iHomeIndex->setHidden(false);
		ui.label_AxisSetting_Jerk->setText(tr("Jerk(mm/s^3)"));
		ui.label_AxisSetting_Resolution->setText(tr("Resolution"));
	}
}

void Dialog_Setting_MotionControl::lineEditChanged()
{
	QLineEdit*	lineEdit	= qobject_cast<QLineEdit*>(sender());
	QString		qstChanged	= lineEdit->objectName();

	QStringList parts		= qstChanged.split("_");
	string		strTable	= parts.at(parts.size() - 2).toStdString();
	if (strTable != "MotionControl")
		strTable = str_Axis;
	string		strKey		= parts.at(parts.size() - 1).toStdString();

	set_Changed.insert(make_pair(strTable, strKey));
}

void Dialog_Setting_MotionControl::comboBoxChanged()
{
	QComboBox*	ComboBox	= qobject_cast<QComboBox*>(sender());
	QString		qstChanged	= ComboBox->objectName();

	QStringList parts		= qstChanged.split("_");
	string		strTable	= parts.at(parts.size() - 2).toStdString();
	if (strTable != "MotionControl")
		strTable = str_Axis;
	string		strKey		= parts.at(parts.size() - 1).toStdString();

	set_Changed.insert(make_pair(strTable, strKey));
}
