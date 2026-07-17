#include "qg_IOWidget.h"
#include "modules/process/process_module.h"
#include "modules/process/settings/process_settings_service.h"
#include <QTimer>
#include <QElapsedTimer>

QG_IOWidget* QG_IOWidget::uniqueInstance = nullptr;

QG_IOWidget::QG_IOWidget(QWidget* parent, const char* name)
	: QWidget(parent)
	, m_iLightState(0), m_iBlowState(0), m_iLaserState(0), m_iWaterState(0)
	, m_iPumpState(0), m_iChuckState(0), m_iPliersState(0), m_iOut1State(0)
{
	ui.setupUi(this);
	setObjectName(name);
	setupQPushButton(this);

	m_qiconGraw		= CreateIcon(":/IOAndAxis/IO_light01.svg");
	m_qiconRed		= CreateIcon(":/IOAndAxis/IO_redlight.svg");
	m_qiconGreen	= CreateIcon(":/IOAndAxis/IO_greenlight.svg");
	m_qiconYellow	= CreateIcon(":/IOAndAxis/IO_yellowlight.svg");
	ui.pushButton_IO_Light->setIconSize(QSize(40, 40));
	ui.pushButton_IO_Light->setIcon(m_qiconGraw);
	ui.label_SystemStatus_SystemStatus->setText(tr("UnInit"));
	ui.label_SystemStatus_SystemStatus->setStyleSheet("QLabel:disabled { color: inherit; border: inherit; }");

	QTimer* timer = new QTimer(this);
	connect(timer, &QTimer::timeout, this, &QG_IOWidget::UpdateIOState);
	timer->start(200);

	SetupUI();
}

QG_IOWidget::~QG_IOWidget()
{
}

QG_IOWidget* QG_IOWidget::instance(QG_IOWidget* pIOWidget)
{
	if (!uniqueInstance)
		uniqueInstance = pIOWidget;
	return uniqueInstance;
}

void QG_IOWidget::SetService(Service* pService)
{
	m_pService = pService;
}

void QG_IOWidget::ClearTimer()
{
	for (auto& state : m_buttonStates)
	{
		if (state.pressTimer && state.pressTimer->isActive())
		{
			state.pressTimer->stop();
		}
		state.bPressed = false;
		state.bLongPress = false;
	}
}

QIcon QG_IOWidget::CreateIcon(const QString& path)
{
	QPixmap pixmap(path);
	QIcon icon;
	icon.addPixmap(pixmap, QIcon::Normal);
	icon.addPixmap(pixmap, QIcon::Disabled);  // 禁用状态使用相同图标
	return icon;
}

void QG_IOWidget::SetupUI()
{
	m_iCustomerID = 0;
	if		(DT::getCustomerID() == "Standard")		m_iCustomerID = 0;
	else if (DT::getCustomerID() == "Mindray")		m_iCustomerID = 1;
	else if (DT::getCustomerID() == "JAPHL")		m_iCustomerID = 2;
	switch (m_iCustomerID)
	{
	case 0:
	{
		m_bWaterCutting = true;
		ui.pushButton_IO_OUT1	->setHidden(true);
		ui.label_IO_OUT1		->setHidden(true);
		break;
	}
	case 1:
	{
		m_bWaterCutting = false;
		ui.pushButton_IO_Pump	->setHidden(true);
		ui.label_IO_Pump		->setHidden(true);
		ui.pushButton_IO_Water	->setHidden(true);
		ui.label_IO_Water		->setHidden(true);
		ui.pushButton_IO_OUT1	->setHidden(true);
		ui.label_IO_OUT1		->setHidden(true);
		break;
	}
	case 2:
	{
		m_bWaterCutting = false;
		ui.pushButton_IO_Pliers	->setHidden(true);
		ui.label_IO_Pliers		->setHidden(true);
		ui.pushButton_IO_Pump	->setHidden(true);
		ui.label_IO_Pump		->setHidden(true);
		ui.pushButton_IO_Water	->setHidden(true);
		ui.label_IO_Water		->setHidden(true);
		ui.label_IO_Chuck		->setText(tr("LChuck"));
		ui.label_IO_OUT1		->setText(tr("RChuck"));
		break;
	}
	default:
		break;
	}
}

void QG_IOWidget::setupQPushButton(QWidget* dialog)
{
	const QList<QPushButton*> PushButtons = dialog->findChildren<QPushButton*>();
	for (QPushButton* button : PushButtons)
	{
		setupButtonActions(button);
	}

	// 借用函数设置下布局
	// 垂直布局，设置水平居中
	const QList<QHBoxLayout*> HLayouts = dialog->findChildren<QHBoxLayout*>();
	for (QHBoxLayout* layout : HLayouts)	
	{
		if (layout->objectName() != "groupBox_IO")
			layout->setAlignment(Qt::AlignHCenter);
	}
	// 水平布局，设置向右对齐
	const QList<QVBoxLayout*> VLayouts = dialog->findChildren<QVBoxLayout*>();
	for (QVBoxLayout* layout : VLayouts)	
	{
		layout->setAlignment(Qt::AlignLeft);
	}
}

void QG_IOWidget::setupButtonActions(QPushButton* PushButton)
{
	// 确保每个按钮只设置一次
	if (m_buttonStates.contains(PushButton)) {
		return;
	}

	ButtonState& state = m_buttonStates[PushButton];
	state.pressTimer = new QTimer(PushButton);
	state.pressTimer->setSingleShot(true);
	state.bLongPress = false;
	state.bPressed = false;

	// 按下事件
	QObject::connect(PushButton, &QPushButton::pressed, this, [this, PushButton]() {
		ButtonState& btnState = m_buttonStates[PushButton];
		if (btnState.bPressed)		{ return; } // 防止重复处理
		btnState.bPressed = true;
		btnState.bLongPress = false;
		btnState.elapsedTimer.start();
		btnState.pressTimer->start(500); // 500ms长按阈值
		});

	// 长按超时处理
	QObject::connect(state.pressTimer, &QTimer::timeout, this, [this, PushButton]() {
		ButtonState& btnState = m_buttonStates[PushButton];
		if (!btnState.bPressed)		{ return; }
		btnState.bLongPress = true;
		// 长按按钮
		OnClickedIOState(1, PushButton);
		});

	// 释放事件
	QObject::connect(PushButton, &QPushButton::released, this, [this, PushButton]() {
		ButtonState& btnState = m_buttonStates[PushButton];
		if (!btnState.bPressed)		{ return; }	
		btnState.bPressed = false;

		if (btnState.pressTimer->isActive()) {
			btnState.pressTimer->stop();
			if (!btnState.bLongPress && btnState.elapsedTimer.elapsed() < 500) {
				// 点击按钮
				OnClickedIOState(2, PushButton);
			}
		}
		else if (btnState.bLongPress) {
			// 松按钮
			OnClickedIOState(0, PushButton);
			btnState.bLongPress = false;
		}
		});
}

void QG_IOWidget::OnClickedIOState(int iType, QPushButton* PushButton)
{
	if (!m_pService->GetMotionControl() || !m_pService->GetMotionControl()->IsConnected())
	{
		PushButton->setChecked(false);
		SHOW_OPER_WARN(WarnCode::WARN_MC_DISCONNECTED);
		return;	
	}

	string strIO = PushButton->objectName().mid(14).toStdString();
	//需要处理所有的类型，该函数为通用IO控制函数，再额外传入一个int值，2为按钮状态，0，1为开关。用于按钮本身的控制
	if (strIO == "Light")
	{
		return;
	}
	else if (strIO == "AimingBeam")
	{
		switch (iType)
		{
		case 0:
			LOG_OPER_INFO(tr("Lift AimingBeam = 0").toUtf8().constData());
			OnClickedAimingBeam(false);
			return;
		case 1:
			LOG_OPER_INFO(tr("Press AimingBeam = 1").toUtf8().constData());
			OnClickedAimingBeam(true);
			return;
		case 2:
			char cState = PushButton->isChecked() ? '1' : '0';
			LOG_OPER_INFO(tr("Click AimingBeam = %1").arg(cState).toUtf8().constData());
			OnClickedAimingBeam(PushButton->isChecked());
			return;
		}
	}
	else
	{
		DigitalOUT IO = enum_cast<DigitalOUT>(strIO).value();
		bool bError;
		switch (iType)
		{
		case 0:
			LOG_OPER_INFO(tr("Lift %1 = 0").arg(tr(strIO.c_str())).toUtf8().constData());
			//bool bError;
			if (strIO == "Laser" && m_pService->GetMotionControl()->GetName() == "GTN")
				bError = m_pService->GetMotionControl()->GSN_SetLaserEnablePro(false);
			else
				bError = m_pService->GetMotionControl()->DigitalOutputSet(IO, 0, true);
			if (!bError)
				SHOW_OPER_WARN(WarnCode::WARN_MC_NONEINDEX, tr("%1 digital out index is not compliant")
					.arg(tr(strIO.c_str())).toUtf8().constData());
			return;
		case 1:
			LOG_OPER_INFO(tr("Press %1 = 1").arg(tr(strIO.c_str())).toUtf8().constData());
			//bool bError;
			if (strIO == "Laser" && m_pService->GetMotionControl()->GetName() == "GTN")
			{
                table t_Laser = lcnc::process::ProcessSettingsService::current()
                    ? lcnc::process::ProcessSettingsService::current()->rawTable(lcnc::process::ProcessConfigArea::Devices, "Laser") : table{};
				double dFrequency = t_Laser["fFrequency"].as_floating();
				double dPulseWidth = t_Laser["fPulseWidth"].as_floating();
				m_pService->GetMotionControl()->GSN_SetLaserParameterApplication(dFrequency, dPulseWidth, 0.0);
				bError = m_pService->GetMotionControl()->GSN_SetLaserEnablePro(true);
			}
				
			else
				bError = m_pService->GetMotionControl()->DigitalOutputSet(IO, 1, true);
			if (!bError)
				SHOW_OPER_WARN(WarnCode::WARN_MC_NONEINDEX, tr("%1 digital out index is not compliant")
					.arg(tr(strIO.c_str())).toUtf8().constData());
			return;
		case 2:
			char cState = PushButton->isChecked() ? '1' : '0';
			LOG_OPER_INFO(tr("Click %1 = %2").arg(tr(strIO.c_str())).arg(cState).toUtf8().constData());
			//bool bError;
			if (strIO == "Laser" && m_pService->GetMotionControl()->GetName() == "GTN")
			{
				if (PushButton->isChecked())
				{
                    table t_Laser = lcnc::process::ProcessSettingsService::current()
                        ? lcnc::process::ProcessSettingsService::current()->rawTable(lcnc::process::ProcessConfigArea::Devices, "Laser") : table{};
					double dFrequency = t_Laser["fFrequency"].as_floating();
					double dPulseWidth = t_Laser["fPulseWidth"].as_floating();
					m_pService->GetMotionControl()->GSN_SetLaserParameterApplication(dFrequency, dPulseWidth, 0.0);
				}
				bError = m_pService->GetMotionControl()->GSN_SetLaserEnablePro(PushButton->isChecked());
			}	
			else
				bError = m_pService->GetMotionControl()->DigitalOutputSet(IO, (int)PushButton->isChecked(), true);
			if (!bError)
				SHOW_OPER_WARN(WarnCode::WARN_MC_NONEINDEX, tr("%1 digital out index is not compliant")
					.arg(tr(strIO.c_str())).toUtf8().constData());
			return;
		}
	}
}

void QG_IOWidget::OnClickedAimingBeam(bool bState)
{
	// 同步按钮状态
	ui.pushButton_IO_AimingBeam->setChecked(bState);
	emit updateAimingBeam(bState);
	
	if (!m_pService->GetLaserDevice())
	{ 
		ui.pushButton_IO_AimingBeam->setChecked(false);
		emit updateAimingBeam(false);
		return;
	}

	if (bState)
	{
		if (m_pService->GetLaserDevice()->GetName() == "Raycus" || m_pService->GetLaserDevice()->GetName() == "RaycusQCW")
			m_pService->GetLaserDevice()->StopLaser();
		else
			m_pService->GetLaserDevice()->StartAimingBeam();
	}
	else
	{
		if (m_pService->GetLaserDevice()->GetName() == "Raycus" || m_pService->GetLaserDevice()->GetName() == "RaycusQCW")
			m_pService->GetLaserDevice()->StartLaser();
		else
			m_pService->GetLaserDevice()->StopAimingBeam();
	}
}

void QG_IOWidget::SetLight(DigitalOUT eLight)
{
	switch (eLight)
	{
	case DigitalOUT::RedLight:
		m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::RedLight,		1);
		m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::GreenLight,	0);
		m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::YellowLight,	0);
		break;
	case DigitalOUT::GreenLight:
		m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::RedLight,		0);
		m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::GreenLight,	1);
		m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::YellowLight,	0);
		break;
	case DigitalOUT::YellowLight:
		m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::RedLight,		0);
		m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::GreenLight,	0);
		m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::YellowLight,	1);
		break;
	default:
		break;
	}
}

void QG_IOWidget::UpdateIOState()
{
	if (!m_pService->GetMotionControl() || !m_pService->GetMotionControl()->IsConnected())
		return;

	if (m_eState != g_eState.load())
	{
		QString qstrState;
		m_eState = g_eState;
		switch (m_eState)
		{
		case SystemStatus::UnInit:			qstrState = tr("UnInit");			SetLight(DigitalOUT::YellowLight);	break;
		case SystemStatus::Initializing:	qstrState = tr("Initializing");		SetLight(DigitalOUT::GreenLight);	break;
		case SystemStatus::Idle:			qstrState = tr("Idle");				SetLight(DigitalOUT::YellowLight);	break;
		case SystemStatus::Paused:			qstrState = tr("Paused");			SetLight(DigitalOUT::YellowLight);	break;
		case SystemStatus::Pausing:			qstrState = tr("Pausing");			SetLight(DigitalOUT::GreenLight);	break;
		case SystemStatus::Processing:		qstrState = tr("Processing");		SetLight(DigitalOUT::GreenLight);	break;
		case SystemStatus::LaserProcessing: qstrState = tr("LaserProcessing");	SetLight(DigitalOUT::GreenLight);	break;
		case SystemStatus::Error:			qstrState = tr("Error");			SetLight(DigitalOUT::RedLight);		break;
		default:							break;
		}
		ui.label_SystemStatus_SystemStatus->setText(qstrState);
	}

	int iLightState = 0;
	m_pService->GetMotionControl()->DigitalOutputGet(DigitalOUT::RedLight,		m_iRed);
	m_pService->GetMotionControl()->DigitalOutputGet(DigitalOUT::GreenLight,	m_iGreen);
	m_pService->GetMotionControl()->DigitalOutputGet(DigitalOUT::YellowLight,	m_iYellow);
	m_iRed		? iLightState += 1 : 0;
	m_iGreen	? iLightState += 2 : 0;
	m_iYellow	? iLightState += 4 : 0;

	if (iLightState != m_iLightState)
	{
		ui.pushButton_IO_Light->setEnabled(true);
		m_iLightState = iLightState;
		switch (m_iLightState)
		{
		case 0:		ui.pushButton_IO_Light->setIcon(m_qiconGraw);	break;
		case 1:		ui.pushButton_IO_Light->setIcon(m_qiconRed);	break;
		case 2:		ui.pushButton_IO_Light->setIcon(m_qiconGreen);  break;
		case 4:		ui.pushButton_IO_Light->setIcon(m_qiconYellow);	break;
		default:	break;
		}
	}

	m_pService->GetMotionControl()->DigitalOutputGet(DigitalOUT::Blow,		m_iBlowState);

	m_pService->GetMotionControl()->DigitalOutputGet(DigitalOUT::Laser,		m_iLaserState);
	m_pService->GetMotionControl()->DigitalOutputGet(DigitalOUT::Chuck,		m_iChuckState);
	m_pService->GetMotionControl()->DigitalOutputGet(DigitalOUT::Pliers,	m_iPliersState);

	ui.pushButton_IO_Blow	->setChecked((bool)m_iBlowState);
	//ui.pushButton_IO_Laser	->setChecked((bool)m_iLaserState);
	ui.pushButton_IO_Chuck	->setChecked((bool)m_iChuckState);
	ui.pushButton_IO_Pliers	->setChecked((bool)m_iPliersState);

	if (m_pService->GetMotionControl()->GetName() !="GTN")
	{
		m_pService->GetMotionControl()->DigitalOutputGet(DigitalOUT::Laser, m_iLaserState);
		ui.pushButton_IO_Laser->setChecked((bool)m_iLaserState);
	}
	else
	{
		m_pService->GetMotionControl()->GSN_LaserOnStatus(m_iLaserState);
		ui.pushButton_IO_Laser->setChecked((bool)m_iLaserState);
	}
	if (m_bWaterCutting)
	{
		ui.pushButton_IO_Water->setChecked((bool)m_iWaterState);
		ui.pushButton_IO_Pump->setChecked((bool)m_iPumpState);
		m_pService->GetMotionControl()->DigitalOutputGet(DigitalOUT::Water, m_iWaterState);
		m_pService->GetMotionControl()->DigitalOutputGet(DigitalOUT::Pump, m_iPumpState);
	}

	switch (m_iCustomerID)
	{
	case 2: {	//JAPHL
		m_pService->GetMotionControl()->DigitalOutputGet(DigitalOUT::OUT1, m_iOut1State);
		ui.pushButton_IO_OUT1->setChecked((bool)m_iOut1State);
		break; }
	default:
		break;
	}


}

