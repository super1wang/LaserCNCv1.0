#include "qg_dlgsetting.h"
#include <QSignalBlocker>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <functional>

QG_dlgSetting* QG_dlgSetting::uniqueInstance = nullptr;

QG_dlgSetting::QG_dlgSetting(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	ui.pushButton_Setting_Apply->setFocus();
	connect(ui.treeWidget_Setting_Menu, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(SwitchItem(QTreeWidgetItem*, int)));

	// 右侧设置页
	dlgMotionControlSetting = new Dialog_Setting_MotionControl	(this);
	dlgIOIndexSetting		= new Dialog_Setting_IOIndex		(this);
	dlgDigitalSetting		= new Dialog_Setting_Digital		(this);
	dlgAnalogSetting		= new Dialog_Setting_Analog			(this);
	dlgLaserSetting			= new Dialog_Setting_Laser			(this);
	dlgInternetSetting		= new Dialog_Setting_Internet		(this);

	dlgToolSetting			= new Dialog_Setting_Tool			(this);
	dlgGasSetting			= new Dialog_Setting_Gas			(this);
	dlgWaterSetting			= new Dialog_Setting_Water			(this);
	dlgMonitorSetting		= new Dialog_Setting_Monitor		(this);
	dlgLoadingPosSetting	= new Dialog_Setting_LoadingPos		(this);
	dlgCameraSetting		= new Dialog_Setting_Camera			(this);
	
	ui.stackedWidget_Setting_Content->insertWidget(Page::MotionController,	dlgMotionControlSetting);
	ui.stackedWidget_Setting_Content->insertWidget(Page::IOIndex,			dlgIOIndexSetting);
	ui.stackedWidget_Setting_Content->insertWidget(Page::Digital, 			dlgDigitalSetting);
	ui.stackedWidget_Setting_Content->insertWidget(Page::Analog,			dlgAnalogSetting);
	ui.stackedWidget_Setting_Content->insertWidget(Page::Laser,				dlgLaserSetting);
	ui.stackedWidget_Setting_Content->insertWidget(Page::Internet,			dlgInternetSetting);

	ui.stackedWidget_Setting_Content->insertWidget(Page::Tool,				dlgToolSetting);
	ui.stackedWidget_Setting_Content->insertWidget(Page::Gas,				dlgGasSetting);
	ui.stackedWidget_Setting_Content->insertWidget(Page::Water,				dlgWaterSetting);
	ui.stackedWidget_Setting_Content->insertWidget(Page::Monitor,			dlgMonitorSetting);
	ui.stackedWidget_Setting_Content->insertWidget(Page::LoadingPos,		dlgLoadingPosSetting);
	ui.stackedWidget_Setting_Content->insertWidget(Page::Camera,			dlgCameraSetting);

	ui.stackedWidget_Setting_Content->setCurrentIndex(Page::MotionController);

	connect(ui.pushButton_Setting_Apply,		SIGNAL(clicked()), this, SLOT(clickApply()));
	connect(ui.pushButton_Setting_OK,			SIGNAL(clicked()), this, SLOT(clickOK()));
	connect(ui.pushButton_Setting_Cancel,		SIGNAL(clicked()), this, SLOT(clickCancel()));
	connect(ui.pushButton_Setting_ExportConfig, SIGNAL(clicked()), this, SLOT(clickExportConfig()));
	connect(ui.pushButton_Setting_ImportConfig, SIGNAL(clicked()), this, SLOT(clickImportConfig()));

	CreateMenu();
	InitSetting();
	UpdatePage();
	m_tableSettings = table{ {"MotionControl", table{}}, {"Axis", table{}}, {"Digital", table{}}, {"Analog", table{}}, {"Laser", table{}}, {"Internet", table{}}, {"Tool", table{}}, {"Gas", table{}}, {"Water", table{}}, {"Monitor", table{}}, {"LoadingPos", table{}}, {"Camera", table{}} };
	m_tableChanged  = table{ {"MotionControl", table{}}, {"Axis", table{}}, {"Digital", table{}}, {"Analog", table{}}, {"Laser", table{}}, {"Internet", table{}}, {"Tool", table{}}, {"Gas", table{}}, {"Water", table{}}, {"Monitor", table{}}, {"LoadingPos", table{}}, {"Camera", table{}} };
	
	LoadSetting();
}

QG_dlgSetting::~QG_dlgSetting()
{}

QG_dlgSetting* QG_dlgSetting::instance(QG_dlgSetting* pdlgSetting)
{
	if (!uniqueInstance)
		uniqueInstance = pdlgSetting;
	return uniqueInstance;
}

void QG_dlgSetting::SetService(Service* pService)
{
	m_pService = pService;
	dlgToolSetting->SetService(m_pService);
}

void QG_dlgSetting::CreateMenu()
{
	// 清空配置
	ui.treeWidget_Setting_Menu->clear();
	// 设置列数
	ui.treeWidget_Setting_Menu->setColumnCount(1);
	ui.treeWidget_Setting_Menu->setHeaderLabel("Settings");

	// External
	QTreeWidgetItem* ExternalItem = new QTreeWidgetItem(ui.treeWidget_Setting_Menu);
	ExternalItem->setText(0, tr("External"));
	ExternalItem->setData(0, Qt::UserRole, EXTERNAL);
	m_mapMenu[EXTERNAL] = ExternalItem;

	// Motion Controller
	QTreeWidgetItem* MotionControllerItem = new QTreeWidgetItem(ExternalItem);
	MotionControllerItem->setText(0, tr("Motion Controller"));
	MotionControllerItem->setData(0, Qt::UserRole, MOTION_CONTROLLER);
	m_mapMenu[MOTION_CONTROLLER] = MotionControllerItem;

	// Axis
	QTreeWidgetItem* AxisSpeedItem = new QTreeWidgetItem(ExternalItem);
	AxisSpeedItem->setText(0, tr("Axis"));
	AxisSpeedItem->setData(0, Qt::UserRole, AXIS_SPEED);
	m_mapMenu[AXIS_SPEED] = AxisSpeedItem;

	// I/O Index
	QTreeWidgetItem* IOIndexItem = new QTreeWidgetItem(ExternalItem);
	IOIndexItem->setText(0, tr("I/O Index"));
	IOIndexItem->setData(0, Qt::UserRole, IO_INDEX);
	m_mapMenu[IO_INDEX] = IOIndexItem;

	// Digital IN/OUT
	QTreeWidgetItem* DigitalIOItem = new QTreeWidgetItem(IOIndexItem);
	DigitalIOItem->setText(0, tr("Digital IN/OUT"));
	DigitalIOItem->setData(0, Qt::UserRole, DIGITAL_IO);
	m_mapMenu[DIGITAL_IO] = DigitalIOItem;

	// Analog IN/OUT
	QTreeWidgetItem* AnalogIOItem = new QTreeWidgetItem(IOIndexItem);
	AnalogIOItem->setText(0, tr("Analog IN/OUT"));
	AnalogIOItem->setData(0, Qt::UserRole, ANALOG_IO);
	m_mapMenu[ANALOG_IO] = AnalogIOItem;

	// Laser
	QTreeWidgetItem* LaserItem = new QTreeWidgetItem(ExternalItem);
	LaserItem->setText(0, tr("Laser"));
	LaserItem->setData(0, Qt::UserRole, LASER);
	m_mapMenu[LASER] = LaserItem;

	// Internet
	QTreeWidgetItem* InternetItem = new QTreeWidgetItem(ExternalItem);
	InternetItem->setText(0, tr("Internet"));
	InternetItem->setData(0, Qt::UserRole, INTERNET);
	m_mapMenu[INTERNET] = InternetItem;


	// Processing
	QTreeWidgetItem* ProcessingItem = new QTreeWidgetItem(ui.treeWidget_Setting_Menu);
	ProcessingItem->setText(0, tr("Processing"));
	ProcessingItem->setData(0, Qt::UserRole, PROCESSING);
	m_mapMenu[PROCESSING] = ProcessingItem;

	// Tool
	QTreeWidgetItem* ToolItem = new QTreeWidgetItem(ProcessingItem);
	ToolItem->setText(0, tr("Tool"));
	ToolItem->setData(0, Qt::UserRole, TOOL);
	m_mapMenu[TOOL] = ToolItem;

	// Motion&Laser
	QTreeWidgetItem* MotionLaserItem = new QTreeWidgetItem(ToolItem);
	MotionLaserItem->setText(0, tr("Motion&Laser"));
	MotionLaserItem->setData(0, Qt::UserRole, MOTION_LASER);
	m_mapMenu[MOTION_LASER] = MotionLaserItem;

	// General
	QTreeWidgetItem* GeneralItem = new QTreeWidgetItem(ToolItem);
	GeneralItem->setText(0, tr("General"));
	GeneralItem->setData(0, Qt::UserRole, GENERAL);
	m_mapMenu[GENERAL] = GeneralItem;

	// Servo
	QTreeWidgetItem* ServoItem = new QTreeWidgetItem(ToolItem);
	ServoItem->setText(0, tr("Servo"));
	ServoItem->setData(0, Qt::UserRole, SERVO);
	m_mapMenu[SERVO] = ServoItem;

	// Special
	QTreeWidgetItem* SpecialItem = new QTreeWidgetItem(ToolItem);
	SpecialItem->setText(0, tr("Special"));
	SpecialItem->setData(0, Qt::UserRole, SPECIAL);
	m_mapMenu[SPECIAL] = SpecialItem;

	// Gas
	QTreeWidgetItem* GasItem = new QTreeWidgetItem(ProcessingItem);
	GasItem->setText(0, tr("Gas"));
	GasItem->setData(0, Qt::UserRole, GAS);
	m_mapMenu[GAS] = GasItem;

	// Water
	QTreeWidgetItem* WaterItem = new QTreeWidgetItem(ProcessingItem);
	WaterItem->setText(0, tr("Water"));
	WaterItem->setData(0, Qt::UserRole, WATER);
	m_mapMenu[WATER] = WaterItem;

	// Monitor
	QTreeWidgetItem* MonitorItem = new QTreeWidgetItem(ProcessingItem);
	MonitorItem->setText(0, tr("Monitor"));
	MonitorItem->setData(0, Qt::UserRole, MONITOR);
	m_mapMenu[MONITOR] = MonitorItem;

	// LoadingPos
	QTreeWidgetItem* LoadingPosItem = new QTreeWidgetItem(ProcessingItem);
	LoadingPosItem->setText(0, tr("LoadingPos"));
	LoadingPosItem->setData(0, Qt::UserRole, LOADINGPOS);
	m_mapMenu[LOADINGPOS] = LoadingPosItem;

	// Camera
	QTreeWidgetItem* CameraItem = new QTreeWidgetItem(ProcessingItem);
	CameraItem->setText(0, tr("Camera"));
	CameraItem->setData(0, Qt::UserRole, CAMERA);
	m_mapMenu[CAMERA] = CameraItem;
}

void QG_dlgSetting::UpdateMenu(int iPermissionLevel)
{
	// 隐藏所有页面
	for (auto page : m_mapMenu)
	{
		page.second->setHidden(true);
	}

	// IO配置界面锁定
	dlgDigitalSetting->SetIDEnabled(false);
	dlgAnalogSetting->SetIDEnabled(false);

	dlgIOIndexSetting->SetIndexEnabled(false);
	dlgDigitalSetting->SetIndexEnabled(false);
	dlgAnalogSetting->SetIndexEnabled(false);

	// 相机页面指令名称锁定
	dlgCameraSetting->SetCommandNameEnabled(false);

	// 客户定制页跳转
	if (!CustomerMenu(iPermissionLevel))
	{
		// 操作员
		if (iPermissionLevel > (int)PermissionLevel::None)
		{
			m_mapMenu[EXTERNAL]->setHidden(false);
			m_mapMenu[AXIS_SPEED]->setHidden(false);
		}

		// 技术员
		if (iPermissionLevel > (int)PermissionLevel::Operator)
		{
			m_mapMenu[PROCESSING]->setHidden(false);
			m_mapMenu[TOOL]->setHidden(false);
			m_mapMenu[TOOL]->setExpanded(true);
			m_mapMenu[MOTION_LASER]->setHidden(false);
			m_mapMenu[GENERAL]->setHidden(false);
			m_mapMenu[SERVO]->setHidden(false);
			m_mapMenu[GAS]->setHidden(false);
			m_mapMenu[WATER]->setHidden(false);
			m_mapMenu[MONITOR]->setHidden(false);
			m_mapMenu[LOADINGPOS]->setHidden(false);

			m_mapMenu[LASER]->setHidden(false);
			dlgLaserSetting->ui.groupBox_ComSetting->setHidden(true);
			dlgLaserSetting->ui.groupBox_SignalSource->setHidden(true);
			dlgLaserSetting->ui.comboBox_Laser_sType->setEnabled(false);
			dlgLaserSetting->ui.checkBox_SignalSource_bSignal->setEnabled(false);
			dlgLaserSetting->ui.lineEdit_Laser_fResolution->setEnabled(false);

			m_mapMenu[MOTION_CONTROLLER]->setHidden(false);
			dlgMotionControlSetting->ui.comboBox_MotionControl_sType->setEnabled(false);
		}

		// 管理员
		if (iPermissionLevel > (int)PermissionLevel::Technician)
		{
			m_mapMenu[IO_INDEX]->setHidden(false);
			m_mapMenu[IO_INDEX]->setExpanded(false);
		}
	}

	// 临时做法，后续添加界面设置之后，设置界面移除客户标识设置
	if (DT::getCustomerID() == "SZK")
		m_mapMenu[CAMERA]->setHidden(false);

	// 厂家
	if (iPermissionLevel > (int)PermissionLevel::Administrator)
	{
		m_mapMenu[DIGITAL_IO]->setHidden(false);
		m_mapMenu[ANALOG_IO]->setHidden(false);

		m_mapMenu[CAMERA]->setHidden(false);

		dlgMotionControlSetting->ui.comboBox_MotionControl_sType->setEnabled(true);

		dlgLaserSetting->UpdatePage();
		dlgLaserSetting->ui.comboBox_Laser_sType->setEnabled(true);
		dlgLaserSetting->ui.checkBox_SignalSource_bSignal->setEnabled(true);
		dlgLaserSetting->ui.lineEdit_Laser_fResolution->setEnabled(true);

		dlgIOIndexSetting->SetIndexEnabled(true);
		dlgDigitalSetting->SetIndexEnabled(true);
		dlgAnalogSetting->SetIndexEnabled(true);

		dlgCameraSetting->SetCommandNameEnabled(true);
	}

	// 开发者
	if (iPermissionLevel > (int)PermissionLevel::Factory)
	{
		m_mapMenu[INTERNET]->setHidden(false);
		m_mapMenu[SERVO]->setHidden(false);
		m_mapMenu[SPECIAL]->setHidden(false);

		dlgAnalogSetting->SetIDEnabled(true);
		dlgDigitalSetting->SetIDEnabled(true);

		for (auto page : m_mapMenu) {
			page.second->setHidden(false);
		}
	}

	if (!DT::IsUseCamera())
	{
		m_mapMenu[CAMERA]->setHidden(true);
		dlgCameraSetting->SetCommandNameEnabled(false);
	}

	// 展开
	m_mapMenu[EXTERNAL]->setExpanded(true);
	m_mapMenu[PROCESSING]->setExpanded(true);

	ui.stackedWidget_Setting_Content->setCurrentIndex(Page::MotionController);
}

void QG_dlgSetting::SwitchItem(QTreeWidgetItem* item, int column)
{
	int iSettingPage = item->data(column, Qt::UserRole).toInt();
	int iPermission  = int(DT::getPermission());

	switch (iSettingPage)
	{
	case Menu::PROCESSING:
	{
		if (iPermission >= (int)PermissionLevel::Factory)
			ui.stackedWidget_Setting_Content->setCurrentIndex(Page::MotionController);
		break;
	}
	case Menu::MOTION_CONTROLLER:	ui.stackedWidget_Setting_Content->setCurrentIndex(Page::MotionController);	break;
	case Menu::AXIS_SPEED:			ui.stackedWidget_Setting_Content->setCurrentIndex(Page::MotionController);				break;
	case Menu::IO_INDEX:			ui.stackedWidget_Setting_Content->setCurrentIndex(Page::IOIndex);			break;
	case Menu::DIGITAL_IO:			ui.stackedWidget_Setting_Content->setCurrentIndex(Page::Digital);			break;
	case Menu::ANALOG_IO:			ui.stackedWidget_Setting_Content->setCurrentIndex(Page::Analog);			break;
	case Menu::LASER:				ui.stackedWidget_Setting_Content->setCurrentIndex(Page::Laser);				break;
	case Menu::INTERNET:			ui.stackedWidget_Setting_Content->setCurrentIndex(Page::Internet);			break;
	case Menu::MOTION_LASER: {
		dlgToolSetting->ui.stackedWidget->setCurrentIndex(0);
		ui.stackedWidget_Setting_Content->setCurrentIndex(Page::Tool);
		break; }
	case Menu::GENERAL: {
		dlgToolSetting->ui.stackedWidget->setCurrentIndex(1);
		ui.stackedWidget_Setting_Content->setCurrentIndex(Page::Tool);
		break; }
	case Menu::SERVO: {
		dlgToolSetting->ui.stackedWidget->setCurrentIndex(2);
		ui.stackedWidget_Setting_Content->setCurrentIndex(Page::Tool);
		break; }
	case Menu::GAS:					ui.stackedWidget_Setting_Content->setCurrentIndex(Page::Gas);				break;
	case Menu::WATER:				ui.stackedWidget_Setting_Content->setCurrentIndex(Page::Water);				break;
	case Menu::MONITOR:				ui.stackedWidget_Setting_Content->setCurrentIndex(Page::Monitor);			break;
	case Menu::LOADINGPOS:			ui.stackedWidget_Setting_Content->setCurrentIndex(Page::LoadingPos);		break;
	case Menu::CAMERA:				ui.stackedWidget_Setting_Content->setCurrentIndex(Page::Camera);			break;
	default: break;
	}
}

void QG_dlgSetting::InitSetting()
{
	try {
		dlgMotionControlSetting	->InitSetting();
		dlgMotionControlSetting->setUI();
		dlgIOIndexSetting		->InitSetting();
		dlgDigitalSetting		->InitSetting();
		dlgAnalogSetting		->InitSetting();
		dlgLaserSetting			->InitSetting();
		dlgInternetSetting		->InitSetting();
		SETTINGS->SetTable(true, SettingSection::Special, table{});

		dlgToolSetting			->InitSetting();
		dlgToolSetting			->setUI();
		dlgGasSetting			->InitSetting();
		dlgWaterSetting			->InitSetting();
		dlgMonitorSetting		->InitSetting();
		dlgLoadingPosSetting	->InitSetting();
		dlgCameraSetting		->InitSetting();
	}
	catch (const std::exception& e)
	{
		LOG_SYS_INFO(tr("InitSetting exception: %1").arg(QString::fromLocal8Bit(e.what())).toUtf8().data());
	}
}

// Helper: safely call a SetPage-like function, log and continue on toml errors
static void SafeSetPage(const char* name, std::function<void()> fn)
{
	try { fn(); }
	catch (const std::exception& e) {
		LOG_SYS_INFO(QString("SetPage [%1] exception: %2")
			.arg(QString::fromLatin1(name))
			.arg(QString::fromLocal8Bit(e.what())).toUtf8().data());
	}
}

void QG_dlgSetting::UpdatePage()
{
	SafeSetPage("MotionControl",	[&]{ dlgMotionControlSetting	->SetPage(); });
	SafeSetPage("IOIndex",			[&]{ dlgIOIndexSetting		->SetPage(); });
	SafeSetPage("Digital",			[&]{ dlgDigitalSetting		->SetPage(); });
	SafeSetPage("Analog",			[&]{ dlgAnalogSetting		->SetPage(); });
	SafeSetPage("Laser",			[&]{ dlgLaserSetting			->SetPage(); });
	SafeSetPage("Internet",			[&]{ dlgInternetSetting		->SetPage(); });

	SafeSetPage("Tool.Rebuild",	[&]{ dlgToolSetting			->RebuildToolIndex(true); });
	SafeSetPage("Tool",				[&]{ dlgToolSetting			->SetPage(); });

	SafeSetPage("Gas",				[&]{ dlgGasSetting			->SetPage(); });
	SafeSetPage("Water",			[&]{ dlgWaterSetting			->SetPage(); });
	SafeSetPage("Monitor",			[&]{ dlgMonitorSetting		->SetPage(); });
	SafeSetPage("LoadingPos",		[&]{ dlgLoadingPosSetting	->SetPage(); });
	SafeSetPage("Camera",			[&]{ dlgCameraSetting		->SetPage(); });
}

void QG_dlgSetting::GetChanged()
{
	bool bMotionControlChanged = false;
	dlgMotionControlSetting->GetPage(m_tableSettings["MotionControl"].as_table());
	// MotionControl page now handles both MotionControl and Axis sections.
	// GetChanged writes to both m_tableChanged["MotionControl"] and m_tableChanged["Axis"].
	{
		table& mcChanged  = m_tableChanged["MotionControl"].as_table();
		table& axisChanged = m_tableChanged["Axis"].as_table();
		bool hasMCChanges = dlgMotionControlSetting->GetChanged(m_tableSettings["MotionControl"].as_table(), mcChanged);
		// Axis section changes are already written to SETTINGS by GetChanged;
		// collect them from mcChanged's "Axis" sub-table if present.
		if (mcChanged.count("Axis"))
		{
			table& axisSub = mcChanged.at("Axis").as_table();
			for (const auto& kv : axisSub)
				axisChanged[kv.first] = kv.second;
			mcChanged.erase("Axis");
		}

		if (hasMCChanges && m_pService->GetMotionControl())
		{
			if (m_pService->GetMotionControl()->IsConnected())
			{
				if (!mcChanged["sType"].is_empty())
				{
					bMotionControlChanged = true;
					m_pService->SetMotionControlTable();
				}
				else
					m_pService->SetMotionControlTable(mcChanged);
			}
		}
	}

	// PipeDiameter / axis speed changes are now written to SettingSection::Axis
	// by the MotionControl page's GetChanged.
	if (m_tableChanged["Axis"].as_table().size() || bMotionControlChanged)
	{
		if (m_pService->GetMotionControl() && m_pService->GetMotionControl()->IsConnected())
		{
			if (bMotionControlChanged)
				m_pService->GetMotionControl()->SetPipeDiameterTable();
			else
				m_pService->GetMotionControl()->SetPipeDiameterTable(m_tableChanged["Axis"].as_table());
		}
	}

	dlgDigitalSetting->GetPage(m_tableSettings["Digital"].as_table());
	dlgDigitalSetting->GetChanged(m_tableSettings["Digital"].as_table(), m_tableChanged["Digital"].as_table());

	dlgAnalogSetting->GetPage(m_tableSettings["Analog"].as_table());
	dlgAnalogSetting->GetChanged(m_tableSettings["Analog"].as_table(), m_tableChanged["Analog"].as_table());
	
	dlgIOIndexSetting->GetPage(m_tableSettings["Digital"].as_table(), m_tableSettings["Analog"].as_table());
	dlgIOIndexSetting->GetChanged(m_tableSettings["Digital"].as_table(), m_tableSettings["Analog"].as_table(), m_tableChanged["Digital"].as_table(), m_tableChanged["Analog"].as_table());


	if (m_pService->GetMotionControl() && m_pService->GetMotionControl()->IsConnected())
	{
		if (m_tableChanged["Digital"].as_table().size())
			m_pService->GetMotionControl()->SetDigitalTable(m_tableChanged["Digital"].as_table());
		if (m_tableChanged["Analog"].as_table().size())
			m_pService->GetMotionControl()->SetAnalogTable(m_tableChanged["Analog"].as_table());
	}

	dlgLaserSetting->GetPage(m_tableSettings["Laser"].as_table());
	if (dlgLaserSetting->GetChanged(m_tableSettings["Laser"].as_table(), m_tableChanged["Laser"].as_table()))
	{
		if (m_pService->GetLaserDevice() && m_pService->GetLaserDevice()->IsInited())
		{
			if (!m_tableChanged["Laser"]["sType"].is_empty())
				m_pService->SetLaserTable();
			else
				m_pService->SetLaserTable(m_tableChanged["Laser"].as_table());
		}
	}

	dlgInternetSetting->GetPage(m_tableSettings["Internet"].as_table());
	dlgInternetSetting->GetChanged(m_tableSettings["Internet"].as_table(), m_tableChanged["Internet"].as_table());

	dlgToolSetting->GetPage(m_tableSettings["Tool"].as_table());
	if (dlgToolSetting->GetChanged(m_tableSettings["Tool"].as_table(), m_tableChanged["Tool"].as_table()))
		m_pService->SetToolTable();

	dlgGasSetting->GetPage(m_tableSettings["Gas"].as_table());
	if ((dlgGasSetting->GetChanged(m_tableSettings["Gas"].as_table(), m_tableChanged["Gas"].as_table()) || bMotionControlChanged))
	{
		if (m_pService->GetMotionControl() && m_pService->GetMotionControl()->IsConnected())
		{
			if (bMotionControlChanged)
				m_pService->SetGasTable();
			else
				m_pService->SetGasTable(m_tableChanged["Gas"].as_table());
		}
	}

	dlgWaterSetting->GetPage(m_tableSettings["Water"].as_table());
	dlgWaterSetting->GetChanged(m_tableSettings["Water"].as_table(), m_tableChanged["Water"].as_table());

	dlgMonitorSetting->GetPage(m_tableSettings["Monitor"].as_table());
	dlgMonitorSetting->GetChanged(m_tableSettings["Monitor"].as_table(), m_tableChanged["Monitor"].as_table());

	dlgLoadingPosSetting->GetPage(m_tableSettings["LoadingPos"].as_table());
	dlgLoadingPosSetting->GetChanged(m_tableSettings["LoadingPos"].as_table(), m_tableChanged["LoadingPos"].as_table());
	
	dlgCameraSetting->GetPage(m_tableSettings["Camera"].as_table());
	dlgCameraSetting->GetChanged(m_tableSettings["Camera"].as_table(), m_tableChanged["Camera"].as_table());


	m_tableSettings = table{ {"MotionControl", table{}}, {"Axis", table{}}, {"Digital", table{}}, {"Analog", table{}}, {"Laser", table{}}, {"Internet", table{}}, {"Tool", table{}}, {"Gas", table{}}, {"Water", table{}}, {"Monitor", table{}}, {"LoadingPos", table{}}, {"Camera", table{}} };
	m_tableChanged  = table{ {"MotionControl", table{}}, {"Axis", table{}}, {"Digital", table{}}, {"Analog", table{}}, {"Laser", table{}}, {"Internet", table{}}, {"Tool", table{}}, {"Gas", table{}}, {"Water", table{}}, {"Monitor", table{}}, {"LoadingPos", table{}}, {"Camera", table{}} };
}

void QG_dlgSetting::AppSettings(int iMode)
{
	// 0为全下发，其他mode自定
	if (iMode == 1)	// 仅工艺
	{
		if (m_pService->GetMotionControl())
			m_pService->GetMotionControl()->SetPipeDiameterTable();
		m_pService->SetToolTable();
		m_pService->SetGasTable();
	}
	else
	{
		m_pService->SetMotionControlTable();
		if (m_pService->GetMotionControl())
		{
			m_pService->GetMotionControl()->SetPipeDiameterTable();
			m_pService->GetMotionControl()->SetDigitalTable();
			m_pService->GetMotionControl()->SetAnalogTable();
		}
		m_pService->SetLaserTable();
		m_pService->SetToolTable();
		m_pService->SetGasTable();
		m_pService->SetCompTable();
	}
}

void QG_dlgSetting::clickApply()
{
	// 此处额外写一个备份文件的操作
	LOG_OPER_INFO(tr("Click Setting Apply.").toUtf8().data());

	GetChanged();

	// 外设参数
	SETTINGS->SavePeripheralSetting("./Peripheral.toml");
	LOG_SYS_INFO(tr("Peripheral setting saved.").toUtf8().data());

	// 工艺参数
	SETTINGS->SaveTechnologySetting("./config.toml");
	LOG_SYS_INFO(tr("Save config setting.").toUtf8().data());
}

void QG_dlgSetting::clickOK()
{
	clickApply();
	QDialog::close();
}

void QG_dlgSetting::clickCancel()
{
	// 从settings重设界面参数
	UpdatePage();

	// 清除修改内容
	dlgMotionControlSetting	->ClearChange();
	dlgIOIndexSetting		->ClearChange();
	dlgDigitalSetting		->ClearChange();
	dlgAnalogSetting		->ClearChange();
	dlgLaserSetting			->ClearChange();
	dlgInternetSetting		->ClearChange();

	dlgToolSetting			->ClearChange();
	dlgGasSetting			->ClearChange();
	dlgWaterSetting			->ClearChange();
	dlgMonitorSetting		->ClearChange();
	dlgLoadingPosSetting	->ClearChange();
}


void QG_dlgSetting::clickExportConfig()
{
	LOG_OPER_INFO(tr("Click Setting ExportConfig.").toUtf8().data());

	QString Filenamestr = QFileDialog::getSaveFileName(this, tr("Export Setting"), "", tr("toml(*.toml)"));
	if (Filenamestr.isEmpty())
		return;

	GetChanged();

	// 若为GTN控制器，从控制器读取回零参数并同步至SETTINGS
	MotionControl* pMC = m_pService->GetMotionControl();
	if (pMC && pMC->GetName() == "GTN")
	{
		for (const auto& eAxis : magic_enum::enum_values<::Axis>())
		{
			if (DT::IsAxisUse(eAxis))
			{
				table tableHome;
				if (pMC->GetAxisHomePrm(eAxis, tableHome))
				{
					string strAxis = enum_name(eAxis).data();
					// 包装为轴子表，利用 SetTable(false) 仅更新已有 Home 字段
					table axisWrapper;
					axisWrapper["Home"] = tableHome;
					SETTINGS->SetTable(false, SettingSection::MotionControl, axisWrapper, strAxis);
				}
			}
		}
	}

	// 根据权限等级导出
	int iPermissionLevel = 100;	
	if (iPermissionLevel > (int)PermissionLevel::Administrator)
	{
		QMessageBox Confirmation(QMessageBox::NoIcon, tr("Export Setting"), tr("Whether to export all settings ?"), QMessageBox::Yes | QMessageBox::No, NULL);
		if (Confirmation.exec() == QMessageBox::Yes)
		{
			SETTINGS->SaveSettings(Filenamestr.toStdString());
			LOG_SYS_INFO(tr("All setting exported.").toUtf8().data());
			return;
		}
	}

	// 导出工艺参数
	SETTINGS->SaveTechnologySetting(Filenamestr.toStdString());
	LOG_SYS_INFO(tr("Config setting exported.").toUtf8().data());
}

void QG_dlgSetting::clickImportConfig()
{
	LOG_OPER_INFO(tr("Click Setting ImportConfig.").toUtf8().data());

	QString Filenamestr = QFileDialog::getOpenFileName(this, tr("Import Setting"), "", tr("toml(*.toml)"));
	
	if (Filenamestr.isEmpty())
		return;

	try
	{
		QByteArray c_path = Filenamestr.toLocal8Bit();
		toml::value v_input = toml::parse(string(c_path));
		value vSettings;
		vSettings["Setting"] = std::move(v_input.at("Setting").as_table());

		map<string, bool> mapObject;
		for (Technology eTechnology : magic_enum::enum_values<Technology>())
		{
			string strCategory = magic_enum::enum_name(eTechnology).data();
			bool bHave = vSettings["Setting"].count(strCategory);
			if (bHave)
				mapObject.insert(pair<string, bool>(strCategory, bHave));
		}
		for (Peripheral ePeripheral : magic_enum::enum_values<Peripheral>())
		{
			string strCategory = magic_enum::enum_name(ePeripheral).data();
			bool bHave = vSettings["Setting"].count(strCategory);
			if (bHave)
				mapObject.insert(pair<string, bool>(strCategory, bHave));
		}

		bool bSelect = false;
		// 内容判定
		for (auto& pair : mapObject)
		{
			if (pair.second)//有true则继续，反之return
			{
				bSelect = true;
				continue;
			}
		}
		if (!bSelect)
			return;

		ChooseDialog(tr("Load Selection"), mapObject);
		
		// 勾选判定
		bSelect = false;
		for (auto& pair : mapObject)
		{
			if (pair.second)//有true则继续，反之return
			{
				bSelect = true;
				continue;
			}
		}
		if (!bSelect)
			return;

		// 导入工具则先对工具索引读入
		if (mapObject["Tool"])
		{
			table tToolIndex;
			tToolIndex["Setting"]["Tool"]["ToolIndex"] =
				vSettings.at("Setting").at("Tool").at("ToolIndex").as_table();
			SETTINGS->LoadSectionSetting(tToolIndex, SettingSection::Tool, "ToolIndex");
			DLGSETTING->RebuildToolList();
		}
		
		// 按节点读入
		for (auto& pair : mapObject)
		{
			if (pair.second)
			{
				table tSection;
				tSection["Setting"][pair.first] = std::move(vSettings.at("Setting").at(pair.first).as_table());
				SettingSection eSection = magic_enum::enum_cast<SettingSection>(pair.first).value();
				SETTINGS->LoadSectionSetting(tSection, eSection);
			}
		}

		UpdatePage();
		// 下发
		AppSettings();
		LOG_SYS_INFO(tr("Config setting imported.").toUtf8().data());
	}
	catch (const std::exception& e)
	{
		SHOW_OPER_WARN(WarnCode::WARN_FILE_OPENFAILD, QString::fromLocal8Bit(e.what()));
		return;
	}
}

void QG_dlgSetting::LoadSetting()
{
	// 外设参数
	SETTINGS->LoadSettings("./Peripheral.toml");
	LOG_SYS_INFO(tr("Peripheral setting imported.").toUtf8().data());

	// 软件参数
	LoadToolConfig("./config.toml");		// 读入工具参数
	SETTINGS->LoadSettings("./config.toml");
	LOG_SYS_INFO(tr("Config setting imported.").toUtf8().data());
	dlgToolSetting->setUI();
	dlgMotionControlSetting->setUI();
	UpdatePage();
}

void QG_dlgSetting::LoadToolConfig(string FilePath)
{
	// 直接将Setting中整个Tool节点，清空仅留ToolIndex，再重新创建各个工具
	SETTINGS->LoadSectionSetting(FilePath, SettingSection::Tool, "ToolIndex");
	RebuildToolList();
}

void QG_dlgSetting::RebuildToolList()
{
	try {
		dlgToolSetting->ClearChange();
		table t_temp = SETTINGS->GetTable(SettingSection::Tool, "ToolIndex");
		for (int i = 0; i < t_temp.size() - 1; i++)
		{
			string strToolIndex = "sTool_" + std::to_string(i);
			if (!t_temp.count(strToolIndex))
				break;
			if (!t_temp[strToolIndex].is_string())
				break;
			string strToolName = t_temp[strToolIndex].as_string();
			dlgToolSetting->CreatTool(strToolName);
		}
		dlgToolSetting->RebuildToolIndex(true);
	}
	catch (const std::exception& e)
	{
		LOG_SYS_INFO(tr("RebuildToolList exception: %1").arg(QString::fromLocal8Bit(e.what())).toUtf8().data());
	}
}

void QG_dlgSetting::ChooseDialog(const QString& qstrTitle, map<string, bool>& mapObject)
{
	QDialog dialog;
	dialog.setWindowTitle(qstrTitle);

	QVBoxLayout* layout = new QVBoxLayout(&dialog);

	// 存储CheckBox指针的map，key与输入mapObject一致
	map<string, QCheckBox*> checkBoxMap;

	// 动态创建CheckBox
	for (auto& pair : mapObject)
	{
		QCheckBox* checkBox = new QCheckBox(QObject::tr(pair.first.c_str()), &dialog);
		checkBox->setChecked(pair.second); // 使用传入的bool值作为初始状态
		layout->addWidget(checkBox);
		checkBoxMap[pair.first] = checkBox;
	}

	// 添加确定/取消按钮
	QDialogButtonBox* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
	layout->addWidget(buttons);

	QObject::connect(buttons, &QDialogButtonBox::accepted, [&]() {
		// 更新mapObject中的值
		for (auto& pair : checkBoxMap)
		{
			mapObject[pair.first] = pair.second->isChecked();
		}
		dialog.accept();
		});

	QObject::connect(buttons, &QDialogButtonBox::rejected, [&]() {
		// 取消操作，将所有值设为false
		for (auto& pair : mapObject)
		{
			pair.second = false;
		}
		dialog.reject();
		});

	dialog.exec();
}

bool QG_dlgSetting::CustomerMenu(int iPermissionLevel)
{
	int iCustomerID = 0;
	if		(DT::getCustomerID() == "Standard")		return false;
	else if (DT::getCustomerID() == "Mindray")		iCustomerID = 1;
	else if (DT::getCustomerID() == "JAPHL")		iCustomerID = 2;

	switch (iCustomerID)
	{
	case 1:
	{
		// 操作员
		// 设备复位、报警清除、设备暂停、设备启动
		if (iPermissionLevel > (int)PermissionLevel::None)
		{
			m_mapMenu[EXTERNAL]->setHidden(false);
			m_mapMenu[AXIS_SPEED]->setHidden(false);
		}

		// 工程师
		// 权限:包含操作员权限外，可以操作设备气缸动作、同服运动、电机运行、安全设置(光栅、门禁); 
		//		不可以修改工艺参数
		if (iPermissionLevel > (int)PermissionLevel::Operator)
		{
			m_mapMenu[PROCESSING]->setHidden(false);
			m_mapMenu[MONITOR]->setHidden(false);
			dlgMonitorSetting->ui.groupBox_Water->setHidden(true);
			dlgMonitorSetting->ui.groupBox_WaterSetting->setHidden(true);

			m_mapMenu[LASER]->setHidden(false);
			dlgLaserSetting->ui.groupBox_ComSetting->setHidden(true);
			dlgLaserSetting->ui.groupBox_SignalSource->setHidden(true);
			dlgLaserSetting->ui.comboBox_Laser_sType->setEnabled(false);
			dlgLaserSetting->ui.checkBox_SignalSource_bSignal->setEnabled(false);
			dlgLaserSetting->ui.lineEdit_Laser_fResolution->setEnabled(false);

			m_mapMenu[MOTION_CONTROLLER]->setHidden(false);
			dlgMotionControlSetting->ui.comboBox_MotionControl_sType->setEnabled(false);
		}

		// 管理员
		// 管理员有修改密码的权限，含工程师权限外，可以修改各种延时参数、闽值参数、驱动参数(电机速度、加速度等)
		if (iPermissionLevel > (int)PermissionLevel::Technician)
		{
			m_mapMenu[TOOL]->setHidden(false);
			m_mapMenu[TOOL]->setExpanded(true);
			m_mapMenu[MOTION_LASER]->setHidden(false);
			m_mapMenu[GENERAL]->setHidden(false);
			m_mapMenu[SERVO]->setHidden(false);
			dlgToolSetting->ui.checkBox_General_bPunch->setHidden(true);
			dlgToolSetting->ui.groupBox_FlightCutting->setHidden(true);
			dlgToolSetting->ui.groupBox_Servo->setHidden(true);
			
			m_mapMenu[LOADINGPOS]->setHidden(false);
		}
		break;
	}

	case 2:
	{
		// 操作员
		// 设备复位、报警清除、设备暂停、设备启动
		if (iPermissionLevel > (int)PermissionLevel::None)
		{
			m_mapMenu[EXTERNAL]->setHidden(false);
			m_mapMenu[AXIS_SPEED]->setHidden(false);
		}

		// 工程师
		// 权限:包含操作员权限外，可以操作设备气缸动作、同服运动、电机运行、安全设置(光栅、门禁); 
		//		不可以修改工艺参数
		if (iPermissionLevel > (int)PermissionLevel::Operator)
		{
			m_mapMenu[PROCESSING]->setHidden(false);
			m_mapMenu[MONITOR]->setHidden(false);
			dlgMonitorSetting->ui.groupBox_Water->setHidden(true);
			dlgMonitorSetting->ui.groupBox_WaterSetting->setHidden(true);

			m_mapMenu[LASER]->setHidden(false);
			dlgLaserSetting->ui.groupBox_ComSetting->setHidden(true);
			dlgLaserSetting->ui.groupBox_SignalSource->setHidden(true);
			dlgLaserSetting->ui.comboBox_Laser_sType->setEnabled(false);
			dlgLaserSetting->ui.checkBox_SignalSource_bSignal->setEnabled(false);
			dlgLaserSetting->ui.lineEdit_Laser_fResolution->setEnabled(false);

			m_mapMenu[MOTION_CONTROLLER]->setHidden(false);
			dlgMotionControlSetting->ui.comboBox_MotionControl_sType->setEnabled(false);
		}

		// 管理员//随动
		// 管理员有修改密码的权限，含工程师权限外，可以修改各种延时参数、闽值参数、驱动参数(电机速度、加速度等)
		if (iPermissionLevel > (int)PermissionLevel::Technician)
		{
			m_mapMenu[TOOL]->setHidden(false);
			m_mapMenu[TOOL]->setExpanded(true);
			m_mapMenu[MOTION_LASER]->setHidden(false);
			m_mapMenu[GENERAL]->setHidden(false);
			m_mapMenu[SERVO]->setHidden(false);
			dlgToolSetting->ui.checkBox_General_bPunch->setHidden(true);
			m_mapMenu[GAS]->setHidden(false);

			dlgGasSetting->ui.groupBox_GasSetting->setHidden(true);
			m_mapMenu[LOADINGPOS]->setHidden(false);

			m_mapMenu[IO_INDEX]->setHidden(false);
			m_mapMenu[IO_INDEX]->setExpanded(false);
			m_mapMenu[DIGITAL_IO]->setHidden(false);
			m_mapMenu[ANALOG_IO]->setHidden(false);

		}
		break;
	}

	default:
		return false;
	}
	return true;
}
