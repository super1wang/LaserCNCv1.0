#include "Setting_Tool.h"
#include <QFileDialog>
#include <QFileInfo>

Dialog_Setting_Tool::Dialog_Setting_Tool(QWidget* parent)
	: QDialog(parent)
	, set_Changed()
	, m_pService(nullptr)
{
	ui.setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::FramelessWindowHint);
	setupLineEditValidators(this);

	connect(ui.pushButton_Tool_CreatTool,				SIGNAL(clicked()),			this, SLOT(CreatTool()));
	connect(ui.pushButton_Tool_CopyTool,				SIGNAL(clicked()),			this, SLOT(CopyTool()));
	connect(ui.pushButton_Tool_DeleteTool,				SIGNAL(clicked()),			this, SLOT(DeleteTool()));
	connect(ui.pushButton_Tool_RenameTool,				SIGNAL(clicked()),			this, SLOT(RenameTool()));
	connect(ui.pushButton_Tool_ExportTool,				SIGNAL(clicked()),			this, SLOT(ExportTool()));
	connect(ui.pushButton_Tool_ImportTool,				SIGNAL(clicked()),			this, SLOT(ImportTool()));

	connect(ui.comboBox_Tool_sToolIndex,				SIGNAL(activated(int)),		this, SLOT(UpdatePage()));
	//connect(ui.comboBox_Tool_sToolIndex,				SIGNAL(activated(int)),		this, SLOT(comboBoxChanged()));
	connect(ui.lineEdit_Cutting_fLineVel,				SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Cutting_fArcVel,				SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Cutting_fCutAcc,				SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Cutting_fCutJerk,				SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Idel_fXVel,						SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Idel_fYVel,						SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Idel_fAVel,						SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Idel_fAVel,						SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Idel_fA1Vel,					SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Idel_fX1Vel,					SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Idel_fY1Vel,					SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Idel_fZVel,						SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Idel_fIdelAcc,					SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Idel_fIdelJerk,					SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Laser_fEnergy,					SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Laser_fPluse,					SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Laser_fFrequency,				SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Laser_fAttenuatorPercentage,	SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Laser_fPpDivider,				SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Laser_iDelay,					SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	
	connect(ui.lineEdit_LaserDelay_fBeforeOpenLaser,	SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_LaserDelay_fAfterOpenLaser,		SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_LaserDelay_fAfterCloseLaser,	SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_LaserDelay_fBeforeCloseLaser,	SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_ACSCutting_fCornerVelocity,		SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_ACSCutting_fCornerAngle,		SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_ACSCutting_fXSEGVelocity,		SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));

	connect(ui.lineEdit_Cutting_fCutSmoothTime,			SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Cutting_fCutSmoothK,			SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Axis_fAxisSmoothTime,			SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Axis_fAxisSmoothK,				SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));


	connect(ui.lineEdit_Height_fCuttingHeight,			SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Height_fIdleHeight,				SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.comboBox_Directions_sDirectionsX,		SIGNAL(activated(int)),		this, SLOT(comboBoxChanged()));
	connect(ui.comboBox_Directions_sDirectionsY,		SIGNAL(activated(int)),		this, SLOT(comboBoxChanged()));
	connect(ui.checkBox_General_bPunch,					SIGNAL(clicked()),			this, SLOT(checkBoxChanged()));
	connect(ui.checkBox_General_bStopBlow,				SIGNAL(clicked()),			this, SLOT(checkBoxChanged()));
	connect(ui.checkBox_SetPos_bSetPosA,				SIGNAL(clicked()),			this, SLOT(checkBoxChanged()));
	connect(ui.lineEdit_SetPos_fSetPosA,				SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.checkBox_SetPos_bSetPosA1,				SIGNAL(clicked()),			this, SLOT(checkBoxChanged()));
	connect(ui.lineEdit_SetPos_fSetPosA1,				SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.checkBox_MovePos_bMovePosX,				SIGNAL(clicked()),			this, SLOT(checkBoxChanged()));
	connect(ui.lineEdit_MovePos_fMovePosX,				SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.checkBox_MovePos_bMovePosX1,				SIGNAL(clicked()),			this, SLOT(checkBoxChanged()));
	connect(ui.lineEdit_MovePos_fMovePosX1,				SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.checkBox_MovePos_bMovePosA,				SIGNAL(clicked()),			this, SLOT(checkBoxChanged()));
	connect(ui.lineEdit_MovePos_fMovePosA,				SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.checkBox_MovePos_bMovePosA1,				SIGNAL(clicked()),			this, SLOT(checkBoxChanged()));
	connect(ui.lineEdit_MovePos_fMovePosA1,				SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.checkBox_MovePos_bMovePosY,				SIGNAL(clicked()),			this, SLOT(checkBoxChanged()));
	connect(ui.lineEdit_MovePos_fMovePosY,				SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.checkBox_MovePos_bMovePosY1,				SIGNAL(clicked()),			this, SLOT(checkBoxChanged()));
	connect(ui.lineEdit_MovePos_fMovePosY1,				SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));

	connect(ui.checkBox_Servo_bCuttingHead,				SIGNAL(clicked()),			this, SLOT(checkBoxChanged()));
	connect(ui.checkBox_Servo_bCrossBridge,				SIGNAL(clicked()),			this, SLOT(checkBoxChanged()));
 	connect(ui.lineEdit_Servo_fServoCuttingHeight,		SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));

	connect(ui.checkBox_Linkage_bAxisZLinkage,			SIGNAL(clicked()),			this, SLOT(checkBoxChanged()));
	connect(ui.lineEdit_Linkage_fLinkedDelay,			SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.comboBox_Linkage_sLinkedDirection,		SIGNAL(activated(int)),		this, SLOT(comboBoxChanged()));
	connect(ui.comboBox_Linkage_iLinkedMode,			SIGNAL(activated(int)),		this, SLOT(comboBoxChanged()));
	connect(ui.comboBox_Linkage_iLinkedMode,			SIGNAL(activated(int)),		this, SLOT(LinkedModeChanged()));
	connect(ui.lineEdit_Linkage_fLinkageParameterA,		SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Linkage_fLinkageParameterB,		SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Linkage_sLinkedFormula,			SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));

	connect(ui.checkBox_Trough_bTrough,					SIGNAL(clicked()),			this, SLOT(checkBoxChanged()));
	connect(ui.lineEdit_Trough_iRunBuffer,				SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Trough_fCHCompensate,			SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Trough_fExtendSctart,			SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Trough_fExtendEnd,				SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Trough_fAccTime,				SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.lineEdit_Trough_fDelay,					SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));

	connect(ui.checkBox_FlightCutting_bFlightCutting,	SIGNAL(clicked()),			this, SLOT(checkBoxChanged()));
	connect(ui.lineEdit_FlightCutting_fMotorDelay,		SIGNAL(editingFinished()),	this, SLOT(lineEditChanged()));
	connect(ui.checkBox_EnergySwitch_bEnergySwitch,		SIGNAL(clicked()),			this, SLOT(checkBoxChanged()));
}

Dialog_Setting_Tool::~Dialog_Setting_Tool()
{
}


void Dialog_Setting_Tool::SetService(Service* pService)
{
	m_pService = pService;
}

void Dialog_Setting_Tool::InitSetting()
{
	// 构建默认的工具及索引
	str_ToolName = "Default";
	SETTINGS->SetKeyValue("sToolIndex", str_ToolName, SettingSection::Tool, "ToolIndex");
	CreatTool(str_ToolName);
	RebuildToolIndex(false);
}

void Dialog_Setting_Tool::SetPage(table table_Set)
{
	if (!table_Set.size())
		table_Set = SETTINGS->GetTable(SettingSection::Tool);

	str_ToolName = table_Set["ToolIndex"]["sToolIndex"].as_string();
	
	ui.comboBox_Tool_sToolIndex				->setCurrentText(QString::fromUtf8(str_ToolName.c_str()));

	ui.lineEdit_Cutting_fLineVel			->setText		(QString::number(		table_Set[str_ToolName]["fLineVel"]			.as_floating(), 'g', 16));
	ui.lineEdit_Cutting_fArcVel				->setText		(QString::number(		table_Set[str_ToolName]["fArcVel"]			.as_floating(), 'g', 16));
	ui.lineEdit_Cutting_fCutAcc				->setText		(QString::number(		table_Set[str_ToolName]["fCutAcc"]			.as_floating(), 'g', 16));
	ui.lineEdit_Cutting_fCutJerk			->setText		(QString::number(		table_Set[str_ToolName]["fCutJerk"]			.as_floating(), 'g', 16));
	ui.lineEdit_Idel_fXVel					->setText		(QString::number(		table_Set[str_ToolName]["fXVel"]			.as_floating(), 'g', 16));
	ui.lineEdit_Idel_fYVel					->setText		(QString::number(		table_Set[str_ToolName]["fYVel"]			.as_floating(), 'g', 16));
	ui.lineEdit_Idel_fAVel					->setText		(QString::number(		table_Set[str_ToolName]["fAVel"]			.as_floating(), 'g', 16));
	ui.lineEdit_Idel_fA1Vel					->setText		(QString::number(		table_Set[str_ToolName]["fA1Vel"]			.as_floating(), 'g', 16));
	ui.lineEdit_Idel_fX1Vel					->setText		(QString::number(		table_Set[str_ToolName]["fX1Vel"]			.as_floating(), 'g', 16));
	ui.lineEdit_Idel_fY1Vel					->setText		(QString::number(		table_Set[str_ToolName]["fY1Vel"]			.as_floating(), 'g', 16));
	ui.lineEdit_Idel_fZVel					->setText		(QString::number(		table_Set[str_ToolName]["fZVel"]			.as_floating(), 'g', 16));
	ui.lineEdit_Idel_fIdelAcc				->setText		(QString::number(		table_Set[str_ToolName]["fIdelAcc"]			.as_floating(), 'g', 16));
	ui.lineEdit_Idel_fIdelJerk				->setText		(QString::number(		table_Set[str_ToolName]["fIdelJerk"]		.as_floating(), 'g', 16));
	ui.lineEdit_Laser_fEnergy				->setText		(QString::number(		table_Set[str_ToolName]["fEnergy"]			.as_floating(), 'g', 16));
	ui.lineEdit_Laser_fPluse				->setText		(QString::number(		table_Set[str_ToolName]["fPluse"]			.as_floating(), 'g', 16));
	ui.lineEdit_Laser_fFrequency			->setText		(QString::number(		table_Set[str_ToolName]["fFrequency"]		.as_floating(), 'g', 16));
	ui.lineEdit_Laser_fAttenuatorPercentage	->setText		(QString::number(		table_Set[str_ToolName]["fAttenuatorPercentage"].as_floating(), 'g', 16));
	ui.lineEdit_Laser_fPpDivider			->setText		(QString::number(		table_Set[str_ToolName]["fPpDivider"]		.as_floating(), 'g', 16));
	ui.lineEdit_Laser_iDelay				->setText		(QString::number(		table_Set[str_ToolName]["iDelay"]			.as_integer(),  'g', 16));
	
	ui.lineEdit_LaserDelay_fBeforeOpenLaser	->setText		(QString::number(		table_Set[str_ToolName]["fBeforeOpenLaser"]	.as_floating(), 'g', 16));
	ui.lineEdit_LaserDelay_fAfterOpenLaser	->setText		(QString::number(		table_Set[str_ToolName]["fAfterOpenLaser"]	.as_floating(), 'g', 16));
	ui.lineEdit_LaserDelay_fAfterCloseLaser	->setText		(QString::number(		table_Set[str_ToolName]["fAfterCloseLaser"]	.as_floating(), 'g', 16));
	ui.lineEdit_LaserDelay_fBeforeCloseLaser->setText		(QString::number(		table_Set[str_ToolName]["fBeforeCloseLaser"].as_floating(), 'g', 16));
	ui.lineEdit_ACSCutting_fCornerVelocity	->setText		(QString::number(		table_Set[str_ToolName]["fCornerVelocity"]	.as_floating(), 'g', 16));
	ui.lineEdit_ACSCutting_fCornerAngle		->setText		(QString::number(		table_Set[str_ToolName]["fCornerAngle"]		.as_floating(), 'g', 16));
	ui.lineEdit_ACSCutting_fXSEGVelocity	->setText		(QString::number(		table_Set[str_ToolName]["fXSEGVelocity"]	.as_floating(), 'g', 16));

	ui.lineEdit_Cutting_fCutSmoothTime		->setText		(QString::number(		table_Set[str_ToolName]["fCutSmoothTime"]	.as_floating(), 'g', 16));
	ui.lineEdit_Cutting_fCutSmoothK			->setText		(QString::number(		table_Set[str_ToolName]["fCutSmoothK"]		.as_floating(), 'g', 16));
	ui.lineEdit_Axis_fAxisSmoothTime		->setText		(QString::number(		table_Set[str_ToolName]["fAxisSmoothTime"]	.as_floating(), 'g', 16));
	ui.lineEdit_Axis_fAxisSmoothK			->setText		(QString::number(		table_Set[str_ToolName]["fAxisSmoothK"]		.as_floating(), 'g', 16));

	ui.lineEdit_Height_fCuttingHeight		->setText		(QString::number(		table_Set[str_ToolName]["fCuttingHeight"]	.as_floating(), 'g', 16));
	ui.lineEdit_Height_fIdleHeight			->setText		(QString::number(		table_Set[str_ToolName]["fIdleHeight"]		.as_floating(), 'g', 16));
	ui.checkBox_General_bPunch				->setChecked	(						table_Set[str_ToolName]["bPunch"]			.as_boolean());
	ui.checkBox_General_bStopBlow			->setChecked	(						table_Set[str_ToolName]["bStopBlow"]		.as_boolean());
	ui.checkBox_SetPos_bSetPosA				->setChecked	(						table_Set[str_ToolName]["bSetPosA"]			.as_boolean());
	ui.lineEdit_SetPos_fSetPosA				->setText		(QString::number(		table_Set[str_ToolName]["fSetPosA"]			.as_floating(), 'g', 16));
	ui.checkBox_SetPos_bSetPosA1			->setChecked	(						table_Set[str_ToolName]["bSetPosA1"]		.as_boolean());
	ui.lineEdit_SetPos_fSetPosA1			->setText		(QString::number(		table_Set[str_ToolName]["fSetPosA1"]		.as_floating(), 'g', 16));
	ui.checkBox_MovePos_bMovePosX			->setChecked	(						table_Set[str_ToolName]["bMovePosX"]		.as_boolean());
	ui.lineEdit_MovePos_fMovePosX			->setText		(QString::number(		table_Set[str_ToolName]["fMovePosX"]		.as_floating(), 'g', 16));
	ui.checkBox_MovePos_bMovePosX1			->setChecked	(						table_Set[str_ToolName]["bMovePosX1"]		.as_boolean());
	ui.lineEdit_MovePos_fMovePosX1			->setText		(QString::number(		table_Set[str_ToolName]["fMovePosX1"]		.as_floating(), 'g', 16));
	ui.checkBox_MovePos_bMovePosA			->setChecked	(						table_Set[str_ToolName]["bMovePosA"]		.as_boolean());
	ui.lineEdit_MovePos_fMovePosA			->setText		(QString::number(		table_Set[str_ToolName]["fMovePosA"]		.as_floating(), 'g', 16));
	ui.checkBox_MovePos_bMovePosA1			->setChecked	(						table_Set[str_ToolName]["bMovePosA1"]		.as_boolean());
	ui.lineEdit_MovePos_fMovePosA1			->setText		(QString::number(		table_Set[str_ToolName]["fMovePosA1"]		.as_floating(), 'g', 16));
	ui.checkBox_MovePos_bMovePosY			->setChecked	(						table_Set[str_ToolName]["bMovePosY"]		.as_boolean());
	ui.lineEdit_MovePos_fMovePosY			->setText		(QString::number(		table_Set[str_ToolName]["fMovePosY"]		.as_floating(), 'g', 16));
	ui.checkBox_MovePos_bMovePosY1			->setChecked	(						table_Set[str_ToolName]["bMovePosY1"]		.as_boolean());
	ui.lineEdit_MovePos_fMovePosY1			->setText		(QString::number(		table_Set[str_ToolName]["fMovePosY1"]		.as_floating(), 'g', 16));
	
	QString qstrDirectionsX = QString::fromUtf8(table_Set[str_ToolName]["sDirectionsX"].as_string().c_str());
	if (qstrlist_DirectionsX.contains(qstrDirectionsX))
		ui.comboBox_Directions_sDirectionsX->setCurrentText(qstrDirectionsX);
	else
	{
		ui.comboBox_Directions_sDirectionsX->setCurrentIndex(0);
		string strDirectionsX = ui.comboBox_Directions_sDirectionsX->currentText().toStdString();
		SETTINGS->SetKeyValue("sDirectionsX", strDirectionsX, SettingSection::Tool, str_ToolName);
		ToolFactory::GetTool(str_ToolName)->m_strDirectionX = strDirectionsX;
	}
	QString qstrDirectionsY = QString::fromUtf8(table_Set[str_ToolName]["sDirectionsY"].as_string().c_str());
	if (qstrlist_DirectionsY.contains(qstrDirectionsY))
		ui.comboBox_Directions_sDirectionsY->setCurrentText(qstrDirectionsY);
	else
	{
		ui.comboBox_Directions_sDirectionsY->setCurrentIndex(0);
		string strDirectionsY = ui.comboBox_Directions_sDirectionsY->currentText().toStdString();
		SETTINGS->SetKeyValue("sDirectionsY", strDirectionsY, SettingSection::Tool, str_ToolName);
		ToolFactory::GetTool(str_ToolName)->m_strDirectionY = strDirectionsY;
	}

 	ui.checkBox_Servo_bCuttingHead			->setChecked	(						table_Set[str_ToolName]["bCuttingHead"]			.as_boolean());
 	ui.checkBox_Servo_bCrossBridge			->setChecked	(						table_Set[str_ToolName]["bCrossBridge"]			.as_boolean());
	ui.lineEdit_Servo_fServoCuttingHeight	->setText		(QString::number(		table_Set[str_ToolName]["fServoCuttingHeight"]	.as_floating(), 'g', 16));

	ui.checkBox_Linkage_bAxisZLinkage		->setChecked	(						table_Set[str_ToolName]["bAxisZLinkage"]		.as_boolean());
	ui.lineEdit_Linkage_fLinkedDelay		->setText		(QString::number(		table_Set[str_ToolName]["fLinkedDelay"]			.as_floating(), 'g', 16));
	ui.comboBox_Linkage_sLinkedDirection	->setCurrentText(QString::fromStdString(table_Set[str_ToolName]["sLinkedDirection"]		.as_string()));
	ui.comboBox_Linkage_iLinkedMode			->setCurrentIndex(						table_Set[str_ToolName]["iLinkedMode"]			.as_integer());
	ui.lineEdit_Linkage_fLinkageParameterA	->setText		(QString::number(		table_Set[str_ToolName]["fLinkageParameterA"]	.as_floating(), 'g', 16));
	ui.lineEdit_Linkage_fLinkageParameterB	->setText		(QString::number(		table_Set[str_ToolName]["fLinkageParameterB"]	.as_floating(), 'g', 16));
	ui.lineEdit_Linkage_sLinkedFormula		->setText		(QString::fromStdString(table_Set[str_ToolName]["sLinkedFormula"]		.as_string()));

	ui.checkBox_Trough_bTrough				->setChecked	(						table_Set[str_ToolName]["bTrough"]			.as_boolean());
	ui.lineEdit_Trough_iRunBuffer			->setText		(QString::number(		table_Set[str_ToolName]["iRunBuffer"]		.as_integer(),  'g', 16));
	ui.lineEdit_Trough_fCHCompensate		->setText		(QString::number(		table_Set[str_ToolName]["fCHCompensate"]	.as_floating(), 'g', 16));
	ui.lineEdit_Trough_fExtendSctart		->setText		(QString::number(		table_Set[str_ToolName]["fExtendSctart"]	.as_floating(), 'g', 16));
	ui.lineEdit_Trough_fExtendEnd			->setText		(QString::number(		table_Set[str_ToolName]["fExtendEnd"]		.as_floating(), 'g', 16));
	ui.lineEdit_Trough_fAccTime				->setText		(QString::number(		table_Set[str_ToolName]["fAccTime"]			.as_floating(), 'g', 16));
	ui.lineEdit_Trough_fDelay				->setText		(QString::number(		table_Set[str_ToolName]["fDelay"]			.as_floating(), 'g', 16));

	ui.checkBox_FlightCutting_bFlightCutting->setChecked	(						table_Set[str_ToolName]["bFlightCutting"]	.as_boolean());
	ui.lineEdit_FlightCutting_fMotorDelay	->setText		(QString::number(		table_Set[str_ToolName]["fMotorDelay"]		.as_floating(), 'g', 16));
	ui.checkBox_EnergySwitch_bEnergySwitch	->setChecked	(						table_Set[str_ToolName].count("bEnergySwitch")
		? table_Set[str_ToolName]["bEnergySwitch"].as_boolean()
		: false);
	
	LinkedModeChanged();
	MotionControlTypeChanged();
}

void Dialog_Setting_Tool::GetPage(table& table_Page)
{
	table_Temp[str_ToolName]["fLineVel"]			= ui.lineEdit_Cutting_fLineVel				->text().toDouble();
	table_Temp[str_ToolName]["fArcVel"]				= ui.lineEdit_Cutting_fArcVel				->text().toDouble();
	table_Temp[str_ToolName]["fCutAcc"]				= ui.lineEdit_Cutting_fCutAcc				->text().toDouble();
	table_Temp[str_ToolName]["fCutJerk"]			= ui.lineEdit_Cutting_fCutJerk				->text().toDouble();
	table_Temp[str_ToolName]["fXVel"]				= ui.lineEdit_Idel_fXVel					->text().toDouble();
	table_Temp[str_ToolName]["fYVel"]				= ui.lineEdit_Idel_fYVel					->text().toDouble();
	table_Temp[str_ToolName]["fAVel"]				= ui.lineEdit_Idel_fAVel					->text().toDouble();
	table_Temp[str_ToolName]["fA1Vel"]				= ui.lineEdit_Idel_fA1Vel					->text().toDouble();
	table_Temp[str_ToolName]["fX1Vel"]				= ui.lineEdit_Idel_fX1Vel					->text().toDouble();
	table_Temp[str_ToolName]["fY1Vel"]				= ui.lineEdit_Idel_fY1Vel					->text().toDouble();
	table_Temp[str_ToolName]["fZVel"]				= ui.lineEdit_Idel_fZVel					->text().toDouble();
	table_Temp[str_ToolName]["fIdelAcc"]			= ui.lineEdit_Idel_fIdelAcc					->text().toDouble();
	table_Temp[str_ToolName]["fIdelJerk"]			= ui.lineEdit_Idel_fIdelJerk				->text().toDouble();
	table_Temp[str_ToolName]["fEnergy"]				= ui.lineEdit_Laser_fEnergy					->text().toDouble();
	table_Temp[str_ToolName]["fPluse"]				= ui.lineEdit_Laser_fPluse					->text().toDouble();
	table_Temp[str_ToolName]["fFrequency"]			= ui.lineEdit_Laser_fFrequency				->text().toDouble();
	table_Temp[str_ToolName]["fAttenuatorPercentage"] = ui.lineEdit_Laser_fAttenuatorPercentage	->text().toDouble();
	table_Temp[str_ToolName]["fPpDivider"]			= ui.lineEdit_Laser_fPpDivider				->text().toDouble();
	table_Temp[str_ToolName]["iDelay"]				= ui.lineEdit_Laser_iDelay					->text().toInt();
	
	table_Temp[str_ToolName]["fBeforeOpenLaser"]	= ui.lineEdit_LaserDelay_fBeforeOpenLaser	->text().toDouble();
	table_Temp[str_ToolName]["fAfterOpenLaser"]		= ui.lineEdit_LaserDelay_fAfterOpenLaser	->text().toDouble();
	table_Temp[str_ToolName]["fAfterCloseLaser"]	= ui.lineEdit_LaserDelay_fAfterCloseLaser	->text().toDouble();
	table_Temp[str_ToolName]["fBeforeCloseLaser"]	= ui.lineEdit_LaserDelay_fBeforeCloseLaser	->text().toDouble();
	table_Temp[str_ToolName]["fCornerVelocity"]		= ui.lineEdit_ACSCutting_fCornerVelocity	->text().toDouble();
	table_Temp[str_ToolName]["fCornerAngle"]		= ui.lineEdit_ACSCutting_fCornerAngle		->text().toDouble();
	table_Temp[str_ToolName]["fXSEGVelocity"]		= ui.lineEdit_ACSCutting_fXSEGVelocity		->text().toDouble();

	table_Temp[str_ToolName]["fCutSmoothTime"]		= ui.lineEdit_Cutting_fCutSmoothTime		->text().toDouble();
	table_Temp[str_ToolName]["fCutSmoothK"]			= ui.lineEdit_Cutting_fCutSmoothK			->text().toDouble();
	table_Temp[str_ToolName]["fAxisSmoothTime"]		= ui.lineEdit_Axis_fAxisSmoothTime			->text().toDouble();
	table_Temp[str_ToolName]["fAxisSmoothK"]		= ui.lineEdit_Axis_fAxisSmoothK				->text().toDouble();
	
	table_Temp[str_ToolName]["fCuttingHeight"]		= ui.lineEdit_Height_fCuttingHeight			->text().toDouble();
	table_Temp[str_ToolName]["fIdleHeight"]			= ui.lineEdit_Height_fIdleHeight			->text().toDouble();
	table_Temp[str_ToolName]["sDirectionsX"]		= ui.comboBox_Directions_sDirectionsX		->currentText().toStdString();
	table_Temp[str_ToolName]["sDirectionsY"]		= ui.comboBox_Directions_sDirectionsY		->currentText().toStdString();
	table_Temp[str_ToolName]["bPunch"]				= ui.checkBox_General_bPunch				->isChecked();
	table_Temp[str_ToolName]["bStopBlow"]			= ui.checkBox_General_bStopBlow				->isChecked();
	table_Temp[str_ToolName]["bSetPosA"]			= ui.checkBox_SetPos_bSetPosA				->isChecked();
	table_Temp[str_ToolName]["fSetPosA"]			= ui.lineEdit_SetPos_fSetPosA				->text().toDouble();
	table_Temp[str_ToolName]["bSetPosA1"]			= ui.checkBox_SetPos_bSetPosA1				->isChecked();
	table_Temp[str_ToolName]["fSetPosA1"]			= ui.lineEdit_SetPos_fSetPosA1				->text().toDouble();
	table_Temp[str_ToolName]["bMovePosX"]			= ui.checkBox_MovePos_bMovePosX				->isChecked();
	table_Temp[str_ToolName]["fMovePosX"]			= ui.lineEdit_MovePos_fMovePosX				->text().toDouble();
	table_Temp[str_ToolName]["bMovePosX1"]			= ui.checkBox_MovePos_bMovePosX1			->isChecked();
	table_Temp[str_ToolName]["fMovePosX1"]			= ui.lineEdit_MovePos_fMovePosX1			->text().toDouble();
	table_Temp[str_ToolName]["bMovePosA"]			= ui.checkBox_MovePos_bMovePosA				->isChecked();
	table_Temp[str_ToolName]["fMovePosA"]			= ui.lineEdit_MovePos_fMovePosA				->text().toDouble();
	table_Temp[str_ToolName]["bMovePosA1"]			= ui.checkBox_MovePos_bMovePosA1			->isChecked();
	table_Temp[str_ToolName]["fMovePosA1"]			= ui.lineEdit_MovePos_fMovePosA1			->text().toDouble();
	table_Temp[str_ToolName]["bMovePosY"]			= ui.checkBox_MovePos_bMovePosY				->isChecked();
	table_Temp[str_ToolName]["fMovePosA"]			= ui.lineEdit_MovePos_fMovePosA				->text().toDouble();
	table_Temp[str_ToolName]["bMovePosY"]			= ui.checkBox_MovePos_bMovePosY				->isChecked();
	table_Temp[str_ToolName]["fMovePosY"]			= ui.lineEdit_MovePos_fMovePosY				->text().toDouble();
	table_Temp[str_ToolName]["bMovePosY1"]			= ui.checkBox_MovePos_bMovePosY1			->isChecked();
	table_Temp[str_ToolName]["fMovePosY1"]			= ui.lineEdit_MovePos_fMovePosY1			->text().toDouble();

	table_Temp[str_ToolName]["bCuttingHead"]		= ui.checkBox_Servo_bCuttingHead			->isChecked();
	table_Temp[str_ToolName]["bCrossBridge"]		= ui.checkBox_Servo_bCrossBridge			->isChecked();
	table_Temp[str_ToolName]["fServoCuttingHeight"]	= ui.lineEdit_Servo_fServoCuttingHeight		->text().toDouble();

	table_Temp[str_ToolName]["bAxisZLinkage"]		= ui.checkBox_Linkage_bAxisZLinkage			->isChecked();
	table_Temp[str_ToolName]["fLinkedDelay"]		= ui.lineEdit_Linkage_fLinkedDelay			->text().toDouble();
	table_Temp[str_ToolName]["sLinkedDirection"]	= ui.comboBox_Linkage_sLinkedDirection		->currentText().toStdString();
	table_Temp[str_ToolName]["iLinkedMode"]			= ui.comboBox_Linkage_iLinkedMode			->currentIndex();
	table_Temp[str_ToolName]["fLinkageParameterA"]	= ui.lineEdit_Linkage_fLinkageParameterA	->text().toDouble();
	table_Temp[str_ToolName]["fLinkageParameterB"]	= ui.lineEdit_Linkage_fLinkageParameterB	->text().toDouble();
	table_Temp[str_ToolName]["sLinkedFormula"]		= ui.lineEdit_Linkage_sLinkedFormula		->text().toStdString();

	table_Temp[str_ToolName]["bTrough"]				= ui.checkBox_Trough_bTrough				->isChecked();
	table_Temp[str_ToolName]["iRunBuffer"]			= ui.lineEdit_Trough_iRunBuffer				->text().toInt();
	table_Temp[str_ToolName]["fCHCompensate"]		= ui.lineEdit_Trough_fCHCompensate			->text().toDouble();
	table_Temp[str_ToolName]["fExtendSctart"]		= ui.lineEdit_Trough_fExtendSctart			->text().toDouble();
	table_Temp[str_ToolName]["fExtendEnd"]			= ui.lineEdit_Trough_fExtendEnd				->text().toDouble();
	table_Temp[str_ToolName]["fAccTime"]			= ui.lineEdit_Trough_fAccTime				->text().toDouble();
	table_Temp[str_ToolName]["fDelay"]				= ui.lineEdit_Trough_fDelay					->text().toDouble();

	table_Temp[str_ToolName]["bFlightCutting"]		= ui.checkBox_FlightCutting_bFlightCutting	->isChecked();
	table_Temp[str_ToolName]["fMotorDelay"]			= ui.lineEdit_FlightCutting_fMotorDelay		->text().toDouble();
	bool bEnergySwitchUse = DT::getCustomerID() == "MaiTong"
		|| int(DT::getPermission()) > (int)PermissionLevel::Factory;
	table_Temp[str_ToolName]["bEnergySwitch"]		= bEnergySwitchUse
		? ui.checkBox_EnergySwitch_bEnergySwitch->isChecked()
		: false;

	
	string str_ToolNameNew = ui.comboBox_Tool_sToolIndex->currentText().toUtf8().data();
	if (str_ToolName != str_ToolNameNew)
		table_Temp["ToolIndex"]["sToolIndex"] = str_ToolNameNew;

	table_Page = table_Temp;
}

bool Dialog_Setting_Tool::GetChanged(table table_Page, table& table_Changed)
{
	if (set_Changed.empty())
		return false;

	for (auto it = set_Changed.begin(); it != set_Changed.end(); ++it)
	{
		string	strTable = it->first;
		string	strKey = it->second;
		value	Value = table_Page[strTable][strKey];
		table_Changed[strTable][strKey] = Value;
		SETTINGS->SetKeyValue(strKey, Value, SettingSection::Tool, strTable);
		LOG_OPER_INFO(tr("Setting [Tool][%1][%2] %3").arg(tr(strTable.c_str()))
			.arg(tr(strKey.c_str())).arg(toml::format(Value).c_str()).toUtf8().data());
	}
	set<pair<string, string>> set_null;
	set_Changed.swap(set_null);
	return true;
}

void Dialog_Setting_Tool::RebuildToolIndex(bool bLoad)
{
	string strToolIndex;
	// 读入索引
	if (bLoad)
	{
		vec_ToolNames.clear();
		table t_temp = SETTINGS->GetTable(SettingSection::Tool, "ToolIndex");
		for (int i = 0; i < t_temp.size() - 1; i++)
		{
			strToolIndex = "sTool_" + std::to_string(i);
			vec_ToolNames.push_back(t_temp[strToolIndex].as_string());
		}
	}
	else
	{
		// 更新选择工具，保存索引，使用table节点覆盖的方式，直接清掉不存在的工具，仅涉及了索引
		table t_temp;
		str_ToolName = ui.comboBox_Tool_sToolIndex->currentText().toUtf8().data();
		t_temp["sToolIndex"] = str_ToolName;
		for (int i = 0; i < vec_ToolNames.size(); i++)
		{
			strToolIndex = "sTool_" + std::to_string(i);
			t_temp[strToolIndex] = vec_ToolNames[i];
		}
		//SETTINGS->SetTable(t_temp2, SettingSection::Tool, "ToolIndex");
		SETTINGS->SetTable(true, SettingSection::Tool, t_temp, "ToolIndex");
	}

	// 重构Combox索引
	ui.comboBox_Tool_sToolIndex->blockSignals(true);
	string strLastTool = ui.comboBox_Tool_sToolIndex->currentText().toUtf8().data();
	int iLastTool = ui.comboBox_Tool_sToolIndex->currentIndex();

	QStringList	qsl_ToolList;
	for (int i = 0; i < vec_ToolNames.size(); i++)
	{
		qsl_ToolList.append(QString::fromUtf8(vec_ToolNames[i].c_str()));
	}
	int iLastSize = ui.comboBox_Tool_sToolIndex->count();
	ui.comboBox_Tool_sToolIndex->clear();
	ui.comboBox_Tool_sToolIndex->addItems(qsl_ToolList);

	if (iLastSize < qsl_ToolList.size() || iLastTool > qsl_ToolList.size() - 1)
	{
		ui.comboBox_Tool_sToolIndex->setCurrentIndex(0);
		table tableSet = SETTINGS->GetTable(SettingSection::Tool);
		strLastTool = ui.comboBox_Tool_sToolIndex->currentText().toUtf8().data();
		tableSet["ToolIndex"]["sToolIndex"] = strLastTool;
		SetPage(tableSet);
		tableSet.clear();
	}
	else
	{
		ui.comboBox_Tool_sToolIndex->setCurrentIndex(iLastTool);
		str_ToolName = qsl_ToolList[iLastTool].toUtf8().data();
		strLastTool = str_ToolName;
	}
	ui.comboBox_Tool_sToolIndex->blockSignals(false);
	
	SETTINGS->SetKeyValue("sToolIndex", strLastTool, SettingSection::Tool, "ToolIndex");
}

bool Dialog_Setting_Tool::IsSaved()
{
	if (set_Changed.empty())
		return true;
	QMessageBox::StandardButton box;
	box = QMessageBox::question(this, tr("Save Tool"), tr("You need to save tools before you do it, do you want to save it ?"), QMessageBox::Yes | QMessageBox::No);
	if (box == QMessageBox::Yes)
	{
		table tableSet;
		GetPage(tableSet);
		GetChanged(tableSet, tableSet);
		tableSet.clear();
		LOG_OPER_INFO(tr("Click Setting save tools .").toUtf8().data());
		return true;
	}
	return false;
}

void Dialog_Setting_Tool::UpdatePage()
{
	// 获取切换前工具的参数
	GetPage(table_Temp);
	// 更新当前选择工具
	str_ToolName = ui.comboBox_Tool_sToolIndex->currentText().toUtf8().data();
	table_Temp["ToolIndex"]["sToolIndex"] = str_ToolName;
	if (table_Temp[str_ToolName].is_empty())
		table_Temp[str_ToolName] = SETTINGS->GetTable(SettingSection::Tool, str_ToolName);
	// 设置选择的工具
	SetPage(table_Temp);
	string strDevice;
	SETTINGS->GetKeyValue("sType", strDevice, SettingSection::Laser, "Laser");
	QWidget* qlPharosWidgets[] =
	{
		ui.label,
		ui.lineEdit_Laser_fAttenuatorPercentage,
		ui.label_2,
		ui.lineEdit_Laser_fPpDivider,
		ui.label_3,
		ui.lineEdit_Laser_iDelay
	};
	QWidget* qlNormalWidgets[] =
	{
		ui.label_Laser_Energy,
		ui.lineEdit_Laser_fEnergy,
		ui.label_Laser_Frequency,
		ui.lineEdit_Laser_fFrequency,
		ui.label_Laser_Pluse,
		ui.lineEdit_Laser_fPluse
	};
	bool bIsPharos = (strDevice == "Pharos");
	for (QWidget* pWidget : qlPharosWidgets)
		pWidget->setHidden(!bIsPharos);
	for (QWidget* pWidget : qlNormalWidgets)
		pWidget->setHidden(bIsPharos);
	if (bIsPharos)
	{
		ui.lcdNumber_Laser_DutyCycle->setHidden(true);
		ui.lable_Laser_DutyCycle->setHidden(true);
		return;
	}
	if (strDevice == "ULTRON")
	{
		ui.lcdNumber_Laser_DutyCycle->setHidden(true);
		ui.lable_Laser_DutyCycle->setHidden(true);
		ui.label_Laser_Pluse->setText(tr("PulsePickerDivider"));
		return;
	}
	else
	{
		ui.lcdNumber_Laser_DutyCycle->setHidden(false);
		ui.lable_Laser_DutyCycle->setHidden(false);
		ui.label_Laser_Pluse->setText(tr("Pluse(μs)"));
	}
	// 填入占空比
	if (!m_pService || !m_pService->GetLaserDevice())
	{
		ui.lcdNumber_Laser_DutyCycle->display(0);
		return;
	}
	
	double dPulseWidth = table_Temp[str_ToolName]["fPluse"].as_floating();
	double dFrequency = table_Temp[str_ToolName]["fFrequency"].as_floating();
	if (m_pService->GetLaserDevice()->GetName() == "JPT" && (dFrequency * dPulseWidth))
	{
		double dDutyCycle = (dFrequency * dPulseWidth) / 10000.0;
		ui.lcdNumber_Laser_DutyCycle->display(QString::number(dDutyCycle, 'f', 1));
	}
	else
	{
		int iDutyCycle = (dFrequency * dPulseWidth + 5000.0) / 10000;
		ui.lcdNumber_Laser_DutyCycle->display(iDutyCycle);
	}
}

void Dialog_Setting_Tool::LinkedModeChanged()
{
	ui.lineEdit_Linkage_fLinkageParameterA->setValidator(new QRegularExpressionValidator(Regex_Pos_Double(, nullptr)));

	int iMode = ui.comboBox_Linkage_iLinkedMode->currentIndex();
	switch (iMode)
	{
	case 0:
		ui.label_Linkage_LinkageParameterA		->setHidden(false);
		ui.label_Linkage_LinkageParameterB		->setHidden(false);
		ui.lineEdit_Linkage_fLinkageParameterA	->setHidden(false);
		ui.lineEdit_Linkage_fLinkageParameterB	->setHidden(false);
		ui.label_Linkage_LinkageParameterA		->setText(tr("StartPos"));
		ui.label_Linkage_LinkageParameterB		->setText(tr("Slope"));
		ui.label_Linkage_LinkedFormula			->setHidden(true);
		ui.lineEdit_Linkage_sLinkedFormula		->setHidden(true);
		break;
	case 1:
		ui.label_Linkage_LinkageParameterA		->setHidden(false);
		ui.label_Linkage_LinkageParameterB		->setHidden(false);
		ui.lineEdit_Linkage_fLinkageParameterA	->setHidden(false);
		ui.lineEdit_Linkage_fLinkageParameterB	->setHidden(false);
		ui.label_Linkage_LinkageParameterA		->setText(tr("AxisPos"));
		ui.label_Linkage_LinkageParameterB		->setText(tr("ArcDiameter(mm)"));
		ui.label_Linkage_LinkedFormula			->setHidden(true);
		ui.lineEdit_Linkage_sLinkedFormula		->setHidden(true);
		break;
	case 2:
		ui.label_Linkage_LinkedFormula			->setHidden(false);
		ui.lineEdit_Linkage_sLinkedFormula		->setHidden(false);
		ui.label_Linkage_LinkageParameterA		->setHidden(true);
		ui.label_Linkage_LinkageParameterB		->setHidden(true);
		ui.lineEdit_Linkage_fLinkageParameterA	->setHidden(true);
		ui.lineEdit_Linkage_fLinkageParameterB	->setHidden(true);
		break;
	default:
		break;
	}
}

void Dialog_Setting_Tool::CreatTool(string strToolName)
{
	bool bCreat = false;
	if (strToolName == "")
	{
		strToolName = QInputDialog::getText(this, tr("Create Tool"), tr("Enter Tool name:"), QLineEdit::Normal, "", &bCreat).toUtf8().data();
		if (bCreat && !strToolName.empty())
		{
			// 已存在相同名称
			if (std::find(vec_ToolNames.begin(), vec_ToolNames.end(), strToolName) != vec_ToolNames.end())
			{
				SHOW_OPER_WARN(WarnCode::WARN_SETTING_SAMETOOLNAME);
				return;
			}
			else
				LOG_OPER_INFO(tr("Click Setting creat tool %1.").arg(strToolName.c_str()).toUtf8().data());
		}
		else
			return;
	}

	table t_Init;
	t_Init["fLineVel"]			= 10.0;
	t_Init["fArcVel"]			= 10.0;
	t_Init["fCutAcc"]			= 100.0;
	t_Init["fCutJerk"]			= 1000.0;
	t_Init["fXVel"]				= 20.0;
	t_Init["fYVel"]				= 20.0;
	t_Init["fAVel"]				= 10.0;
	t_Init["fA1Vel"] = 10.0;
	t_Init["fX1Vel"]			= 20.0;
	t_Init["fY1Vel"]			= 20.0;
	t_Init["fZVel"]				= 10.0;
	t_Init["fIdelAcc"]			= 100.0;
	t_Init["fIdelJerk"]			= 1000.0;
	t_Init["fEnergy"]			= 20.0;
	t_Init["fPluse"]			= 20.0;
	t_Init["fFrequency"]		= 30.0;
	t_Init["fAttenuatorPercentage"] = 30.0;
	t_Init["fPpDivider"]		= 2.0;
	t_Init["iDelay"]			= 0;
	t_Init["fPulsePickerDivider"] = 30.0;
	t_Init["fBeforeOpenLaser"]	= 0.0;
	t_Init["fAfterOpenLaser"]	= 0.0;
	t_Init["fAfterCloseLaser"]	= 0.0;
	t_Init["fBeforeCloseLaser"]	= 0.0;
	t_Init["fCornerVelocity"]	= 1.0;
	t_Init["fCornerAngle"]		= 1.0;
	t_Init["fXSEGVelocity"]		= 1.0;

	t_Init["fCutSmoothTime"]	= 0.0;
	t_Init["fCutSmoothK"]		= 0.0;
	t_Init["fAxisSmoothTime"]	= 0.0;
	t_Init["fAxisSmoothK"]		= 0.0;
	
	t_Init["fCuttingHeight"]	= 0.0;
	t_Init["fIdleHeight"]		= 0.0;
	t_Init["sDirectionsX"]		= "X";
	if (DT::IsAxisUse(Axis::Y))	t_Init["sDirectionsY"] = "Y";
	else						t_Init["sDirectionsY"] = "A";
	t_Init["bStopBlow"]			= false;
	t_Init["bPunch"]			= false;
	t_Init["bSetPosA"]			= false;
	t_Init["fSetPosA"]			= 0.0;
	t_Init["bSetPosA1"]			= false;
	t_Init["fSetPosA1"]			= 0.0;
	t_Init["bMovePosX"]			= false;
	t_Init["fMovePosX"]			= 0.0;
	t_Init["bMovePosX1"]		= false;
	t_Init["fMovePosX1"]		= 0.0;
	t_Init["bMovePosA"]			= false;
	t_Init["fMovePosA"]			= 0.0;
	t_Init["bMovePosA1"]		= false;
	t_Init["fMovePosA1"]		= 0.0;
	t_Init["bMovePosY"]			= false;
	t_Init["fMovePosY"]			= 0.0;
	t_Init["bMovePosY1"]		= false;
	t_Init["fMovePosY1"]		= 0.0;

	t_Init["bCuttingHead"]		= false;
	t_Init["bCrossBridge"]		= false;
	t_Init["fServoCuttingHeight"] = 0.0;

	t_Init["bAxisZLinkage"]		= false;
	t_Init["fLinkedDelay"]		= 50.0;
	t_Init["sLinkedDirection"]	= "X";
	t_Init["iLinkedMode"]		= 0;
	t_Init["fLinkageParameterA"] = 0.0;
	t_Init["fLinkageParameterB"] = 0.0;
	t_Init["sLinkedFormula"]	= "";

	t_Init["bTrough"]			= false;
	t_Init["iRunBuffer"]		= 0;
	t_Init["fCHCompensate"]		= 0.0;
	t_Init["fExtendSctart"]		= 0.0;
	t_Init["fExtendEnd"]		= 0.0;
	t_Init["fAccTime"]			= 0.0;
	t_Init["fDelay"]			= 0.0;

	t_Init["bFlightCutting"]	= false;
	t_Init["fMotorDelay"]		= 0.0;
	t_Init["bEnergySwitch"]	= false;

	SETTINGS->SetTable(true, SettingSection::Tool, t_Init, strToolName);

	vec_ToolNames.push_back(strToolName);

	// 按钮创建，立刻更新索引
	if (bCreat)
	{
		RebuildToolIndex(false);
		m_pService->SetToolTable();
	}
}

void Dialog_Setting_Tool::MotionControlTypeChanged()
{
	
}

void Dialog_Setting_Tool::CopyTool()
{
	if (!IsSaved())
		return;

	bool bCopy = false;
	QStringList	qslCopyToList;
	string strCopy = ui.comboBox_Tool_sToolIndex->currentText().toUtf8().data();
	for (int i = 0; i < vec_ToolNames.size(); i++)
	{
		if (vec_ToolNames[i] != strCopy)
			qslCopyToList.append(vec_ToolNames[i].c_str());
	}
	string strCopyTo = QInputDialog::getItem(this, tr("Copy Tool"), tr("Copy to :"), qslCopyToList, 0, true, &bCopy, Qt::WindowFlags()).toUtf8().data();
	if (bCopy && !strCopyTo.empty())
	{
		table tableCopy = SETTINGS->GetTable(SettingSection::Tool, strCopy);
		SETTINGS->DelTable(SettingSection::Tool, strCopyTo);
		//SETTINGS->SetTable(tableCopy, SettingSection::Tool, strCopyTo);
		SETTINGS->SetTable(true, SettingSection::Tool, tableCopy, strCopyTo);
		table_Temp.erase(strCopyTo);

		// 复制到新工具
		if (std::find(vec_ToolNames.begin(), vec_ToolNames.end(), strCopyTo) == vec_ToolNames.end())
		{
			vec_ToolNames.push_back(strCopyTo);
			RebuildToolIndex(false);
		}
		LOG_OPER_INFO(tr("Click Setting copy tool %1 to %2.").arg(strCopy.c_str()).arg(QString::fromStdString(strCopyTo)).toUtf8().data());
	}
}

void Dialog_Setting_Tool::DeleteTool()
{
	if (!IsSaved())
		return;

	if (vec_ToolNames.size() == 1)
	{
		SHOW_OPER_WARN(WarnCode::WARN_SETTING_ONLYONETOOL);
		return;
	}

	bool bDelete = false;
	string strDeleteTool = ui.comboBox_Tool_sToolIndex->currentText().toUtf8().data();
	QMessageBox::StandardButton box = QMessageBox::question(this, tr("Delete Tool"), tr("Are you sure you want to delete this ?"), QMessageBox::Yes | QMessageBox::No);
	if (box == QMessageBox::Yes)
	{
		SETTINGS->DelTable(SettingSection::Tool, strDeleteTool);
		// 工具名容器处理
		auto pose = std::find(vec_ToolNames.begin(), vec_ToolNames.end(), strDeleteTool);
		vec_ToolNames.erase(pose);
		RebuildToolIndex(false);
		m_pService->SetToolTable();
		LOG_OPER_INFO(tr("Click Setting delete tool %1.").arg(QString::fromStdString(strDeleteTool)).toUtf8().data());
	}
}

void Dialog_Setting_Tool::RenameTool()
{
	if (!IsSaved())
		return;

	bool bRename = false;
	string strOldName = ui.comboBox_Tool_sToolIndex->currentText().toUtf8().data();
	string strNewName = QInputDialog::getText(this, tr("Rename Tool"), tr("Enter new name :"), QLineEdit::Normal, "", &bRename).toUtf8().data();
	if (bRename && !strNewName.empty())
	{
		// 已存在相同名称
		if (std::find(vec_ToolNames.begin(), vec_ToolNames.end(), strNewName) != vec_ToolNames.end())
		{
			SHOW_OPER_WARN(WarnCode::WARN_SETTING_SAMETOOLNAME);
			return;
		}
		table tableOldTool = SETTINGS->GetTable(SettingSection::Tool, strOldName);
		SETTINGS->DelTable(SettingSection::Tool, strOldName);
		//SETTINGS->SetTable(tableOldTool, SettingSection::Tool, strNewName);
		SETTINGS->SetTable(true, SettingSection::Tool, tableOldTool, strNewName);

		// 工具名容器处理
		auto pose = std::find(vec_ToolNames.begin(), vec_ToolNames.end(), strOldName);
		*pose = strNewName;
		RebuildToolIndex(false);
		m_pService->SetToolTable();
		LOG_OPER_INFO(tr("Click Setting rename tool %1 to %2.").arg(strOldName.c_str()).arg(strNewName.c_str()).toUtf8().data());
	}
}

void Dialog_Setting_Tool::ExportTool()
{
	if (!IsSaved())
		return;

	QString qstrToolName = ui.comboBox_Tool_sToolIndex->currentText();
	QString qstrFileName = QFileDialog::getSaveFileName(this, tr("Save As"), "./" + qstrToolName + ".toml", tr("toml(*.toml)"));
	if (qstrFileName.isEmpty())
		return;
	SETTINGS->SaveSectionSetting(qstrFileName.toLocal8Bit().constData(), SettingSection::Tool, qstrToolName.toStdString());
	LOG_OPER_INFO(tr("Click Setting export tool %1.").arg(qstrToolName).toUtf8().data());
}

void Dialog_Setting_Tool::ImportTool()// 工具内的多工具文件导入，
{
	if (!IsSaved())
		return;

	QStringList qlistFiles = QFileDialog::getOpenFileNames(this, tr("open a file"), "./", tr("toml(*.toml)"));
	if (qlistFiles.isEmpty())
		return;

	QStringList qlistErrorFiles;
	bool bCreatTool = true;
	QString qstrLastToolName;
	for (const QString& qstrFile : qlistFiles)
	{
		// 不需要考虑工具内外名称不一样的情况，以文件名为准，采用set，内外不一致直接导入失败（即不允许导出后文件修改名称，强制设置）。反之以工具内参数为准，需要采用load
		QFileInfo qFileInfo(qstrFile);
		string strToolName = qFileInfo.baseName().toUtf8().data();
		// 是否存在重名检测
		if (std::find(vec_ToolNames.begin(), vec_ToolNames.end(), strToolName) != vec_ToolNames.end())
		{
			QMessageBox::StandardButton box = QMessageBox::question(this, tr("Override Tool"), tr("Tool \"%1\" already exists. Overwrite?").arg(qFileInfo.baseName()), QMessageBox::Yes | QMessageBox::No);
			if (box == QMessageBox::Yes)
				bCreatTool = false;
			else
				continue;
		}
		// 导入，采用先创建后修改模式，实现不完整的参数读入，使得工具可以正常使用
		try
		{
			value valInput = toml::parse(string(qstrFile.toUtf8().data()));
			const auto& tableTool = valInput.at("Setting").at("Tool").at(strToolName);
			if (bCreatTool)
				CreatTool(strToolName);
			SETTINGS->SetTable(false, SettingSection::Tool, tableTool.as_table(), strToolName);//覆盖有问题，不应该。因为都是先创建后覆盖。
			bCreatTool = true;
			qstrLastToolName = qFileInfo.baseName();
			LOG_OPER_INFO(tr("Click Setting import tool: %1").arg(qstrLastToolName).toUtf8().data());
		}
		catch (const std::exception& e)
		{
			qlistErrorFiles << qstrFile;
			continue;
		}
	}
	RebuildToolIndex(false);

	// 导入失败的提示
	if (qlistErrorFiles.count())
		SHOW_OPER_WARN(WarnCode::WARN_SETTING_IMPORTTOOL, tr("Import failed: %1").arg(qlistErrorFiles.join("\n")).toUtf8().data());

	// 自动选择最后导入的工具
	if (!qstrLastToolName.isEmpty())
	{
		if (qstrLastToolName.toUtf8().data() != str_ToolName)
		{
			ui.comboBox_Tool_sToolIndex->setCurrentText(qstrLastToolName);
			UpdatePage();
		}
		else
		{
			SetPage();
		}
		m_pService->SetToolTable();
	}
}

void Dialog_Setting_Tool::setupLineEditValidators(QWidget* dialog)
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
			{
				if (parts[1] == "SetPos" || parts[1] == "MovePos")
					lineEdit->setValidator(new QRegularExpressionValidator(Regex_Pos_Double(, nullptr)));
				else
					lineEdit->setValidator(new QRegularExpressionValidator(Regex_Nonnegative_Double(, nullptr)));
				lineEdit->setMaxLength(8);  // 最多输入8个字符
			}
			else if (typeIndicator == "i")
				lineEdit->setValidator(new QRegularExpressionValidator(Regex_Nonnegative_Int(, nullptr)));
		}
	}
	ui.lineEdit_Height_fCuttingHeight->setValidator(new QRegularExpressionValidator(Regex_Pos_Double(, nullptr)));
	ui.lineEdit_Height_fIdleHeight->setValidator(new QRegularExpressionValidator(Regex_Pos_Double(, nullptr)));
}

void Dialog_Setting_Tool::setUI()
{
	string strName;
	SETTINGS->GetKeyValue("sType", strName, SettingSection::MotionControl, "MotionControl");
	if (strName == "GTN")
	{
		ui.groupBox_GTNCutting->setHidden(false);
		ui.groupBox_ACSCutting->setHidden(true);
		ui.label_Cutting_Jerk->setText(tr("smoothTime(ms)"));
		ui.label_Idel_Jerk->setText(tr("smoothTime(ms)"));
	}
	else
	{
		ui.groupBox_GTNCutting->setHidden(true);
		ui.groupBox_ACSCutting->setHidden(false);
		ui.label_Cutting_Jerk->setText(tr("Jerk(mm/s^3)"));
		ui.label_Idel_Jerk->setText(tr("Jerk(mm/s^3)"));
	}
	string strDevice;
	SETTINGS->GetKeyValue("sType", strDevice, SettingSection::Laser, "Laser");
	QWidget* qlPharosWidgets[] =
	{
		ui.label,
		ui.lineEdit_Laser_fAttenuatorPercentage,
		ui.label_2,
		ui.lineEdit_Laser_fPpDivider,
		ui.label_3,
		ui.lineEdit_Laser_iDelay
	};
	QWidget* qlNormalWidgets[] =
	{
		ui.label_Laser_Energy,
		ui.lineEdit_Laser_fEnergy,
		ui.label_Laser_Frequency,
		ui.lineEdit_Laser_fFrequency,
		ui.label_Laser_Pluse,
		ui.lineEdit_Laser_fPluse
	};
	bool bIsPharos = (strDevice == "Pharos");
	for (QWidget* pWidget : qlPharosWidgets)
		pWidget->setHidden(!bIsPharos);
	for (QWidget* pWidget : qlNormalWidgets)
		pWidget->setHidden(bIsPharos);
	if (bIsPharos)
	{
		ui.lcdNumber_Laser_DutyCycle->setHidden(true);
		ui.lable_Laser_DutyCycle->setHidden(true);
	}
	else if (strDevice == "ULTRON")
	{
		ui.lable_Laser_DutyCycle->setHidden(false);
		ui.lcdNumber_Laser_DutyCycle->setHidden(true);
		//ui.label_Laser_Pluse->setText(tr("PulsePickerDivider"));
		return;
	}
	else
	{
		ui.lable_Laser_DutyCycle->setHidden(false);
		ui.lcdNumber_Laser_DutyCycle->setHidden(false);
		//ui.label_Laser_Pluse->setText(tr("Pluse(μs)"));
	}
	// 方向索引
	qstrlist_DirectionsX = DT::getDirectionX();
	ui.comboBox_Directions_sDirectionsX->clear();
	ui.comboBox_Directions_sDirectionsX->addItems(qstrlist_DirectionsX);
	qstrlist_DirectionsY = DT::getDirectionY();
	ui.comboBox_Directions_sDirectionsY->clear();
	ui.comboBox_Directions_sDirectionsY->addItems(qstrlist_DirectionsY);

	if (qstrlist_DirectionsX.size() == 1)
	{
		ui.label_Directions_DirectionsX->hide();
		ui.comboBox_Directions_sDirectionsX->hide();
		set_Changed.insert(make_pair(str_ToolName, "sDirectionsX"));
	}
	if (qstrlist_DirectionsY.size() == 1)
	{
		ui.label_Directions_DirectionsY->hide();
		ui.comboBox_Directions_sDirectionsY->hide();
		set_Changed.insert(make_pair(str_ToolName, "sDirectionsY"));
	}
	if (qstrlist_DirectionsX.size() == 1 && qstrlist_DirectionsY.size() == 1)
	{
		ui.groupBox_Directions->hide();
	}

	// 移位 & 空程速度
	if (!DT::IsAxisUse(Axis::X))
	{
		ui.checkBox_MovePos_bMovePosX->hide();
		ui.lineEdit_MovePos_fMovePosX->hide();
		ui.label_Idel_XVel->hide();
		ui.lineEdit_Idel_fXVel->hide();
	}
	if (!DT::IsAxisUse(Axis::X1))
	{
		ui.checkBox_MovePos_bMovePosX1->hide();
		ui.lineEdit_MovePos_fMovePosX1->hide();
		ui.label_Idel_X1Vel->hide();
		ui.lineEdit_Idel_fX1Vel->hide();
	}
	if (!DT::IsAxisUse(Axis::A))
	{
		ui.checkBox_MovePos_bMovePosA->hide();
		ui.lineEdit_MovePos_fMovePosA->hide();
		ui.label_Idel_AVel->hide();
		ui.lineEdit_Idel_fAVel->hide();

		// 置位
		ui.groupBox_SetPos->hide();
	}
	if (!DT::IsAxisUse(Axis::A1))
	{
		ui.checkBox_MovePos_bMovePosA1->hide();
		ui.lineEdit_MovePos_fMovePosA1->hide();
		ui.label_Idel_A1Vel->hide();
		ui.lineEdit_Idel_fA1Vel->hide();

		// 置位
		ui.checkBox_SetPos_bSetPosA1->hide();
		ui.lineEdit_SetPos_fSetPosA1->hide();
	}
	if (!DT::IsAxisUse(Axis::Y))
	{
		ui.checkBox_MovePos_bMovePosY->hide();
		ui.lineEdit_MovePos_fMovePosY->hide();
		ui.label_Idel_YVel->hide();
		ui.lineEdit_Idel_fYVel->hide();
	}
	if (!DT::IsAxisUse(Axis::Y1))
	{
		ui.checkBox_MovePos_bMovePosY1->hide();
		ui.lineEdit_MovePos_fMovePosY1->hide();
		ui.label_Idel_Y1Vel->hide();
		ui.lineEdit_Idel_fY1Vel->hide();
	}
	if (!DT::IsAxisUse(Axis::Z))
	{
		ui.label_Idel_ZVel->hide();
		ui.lineEdit_Idel_fZVel->hide();
	}
}

void Dialog_Setting_Tool::lineEditChanged()
{
	QLineEdit*	lineEdit	= qobject_cast<QLineEdit*>(sender());
	QString		qstChanged	= lineEdit->objectName();

	QStringList parts		= qstChanged.split("_");
	string		strKey		= parts.at(parts.size() - 1).toStdString();

	set_Changed.insert(make_pair(str_ToolName, strKey));
}

void Dialog_Setting_Tool::comboBoxChanged()
{
	QComboBox*	ComboBox	= qobject_cast<QComboBox*>(sender());
	QString		qstChanged	= ComboBox->objectName();

	QStringList parts 		= qstChanged.split("_");
	string		strKey		= parts.at(parts.size() - 1).toStdString();

	set_Changed.insert(make_pair(str_ToolName, strKey));
}

void Dialog_Setting_Tool::checkBoxChanged()
{
	QCheckBox*	CheckBox	= qobject_cast<QCheckBox*>(sender());
	QString		qstChanged	= CheckBox->objectName();

	QStringList parts		= qstChanged.split("_");
	string		strKey		= parts.at(parts.size() - 1).toStdString();

	set_Changed.insert(make_pair(str_ToolName, strKey));
}
