#include "qg_AxisWidget.h"
#include "core/kernel/kernel.h"
#include "core/kinematics/machine_configuration_service.h"
#include "RegexPatterns.h"
#include <QTimer>
#include <QElapsedTimer>
#include <QRegularExpressionValidator>

string m_strSpeedMode = "fMediumSpeed";
QG_AxisWidget* QG_AxisWidget::uniqueInstance = nullptr;

QG_AxisWidget::QG_AxisWidget(QWidget *parent, const char* name)
	: QWidget(parent)
	, m_vecControls(NULL)
{
	ui.setupUi(this);
	setObjectName(name);
	SetupUI();
	setupQLcdNumber(this);
	setupQPushButton(this);

	connect(ui.pushButton_AxisMove_AimingBeam,		SIGNAL(clicked()), this, SLOT(OnClickedAimingBeam()));

	connect(ui.radioButton_AxisMove_LowSpeed,		SIGNAL(clicked()), this, SLOT(OnClickedradioButtonSpeedModelLow()));
	connect(ui.radioButton_AxisMove_MediumSpeed,	SIGNAL(clicked()), this, SLOT(OnClickedradioButtonSpeedModelMedium()));
	connect(ui.radioButton_AxisMove_HighSpeed,		SIGNAL(clicked()), this, SLOT(OnClickedradioButtonSpeedModelHigh()));
	
	ui.radioButton_AxisMove_MediumSpeed->setChecked(true);
	ui.lineEdit_AxisMove_dPos->setValidator(new QRegularExpressionValidator(Regex_Nonnegative_Double(), this));

	QTimer* timer = new QTimer(this);
	connect(timer, &QTimer::timeout, this, &QG_AxisWidget::UpdateAxisState);
	timer->start(50);

	ui.pushButton_AxisMove_AimingBeam->setToolTip(tr("AimingBeam"));
}

QG_AxisWidget::~QG_AxisWidget()
{
}

QG_AxisWidget* QG_AxisWidget::instance(QG_AxisWidget* pAxisWidget)
{
	if (!uniqueInstance)
		uniqueInstance = pAxisWidget;
	return uniqueInstance;
}

void QG_AxisWidget::SetService(Service* pService)
{
	m_pService = pService;
}

void QG_AxisWidget::ClearTimer()
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

void QG_AxisWidget::setupQLcdNumber(QWidget* dialog)
{
	const QList<QLCDNumber*> LCDNumbers = dialog->findChildren<QLCDNumber*>();
	for (QLCDNumber* LCD : LCDNumbers)
	{
		// 设置lcd位数，整数部分4位 + 小数点 + 小数部分2位 = 总共7字符
		LCD->setDigitCount(7);
		// 十进制模式
		LCD->setMode(QLCDNumber::Dec);
		//小数点占位 
		LCD->setSmallDecimalPoint(true);

		string strAxis = LCD->objectName().mid(20, 2).toStdString();
		if (strAxis[1] != '1')//判断是否是两位的轴
			strAxis = strAxis[0];

		ControlInfos CI;
		CI.type = ControlType::LCD_NUMBER;
		CI.qlcd = LCD;
		CI.qbtn = NULL;
		CI.axis = enum_cast<Axis>(strAxis).value();
		CI.lastValue = 0;
		m_vecControls.push_back(CI);
	}
}

void QG_AxisWidget::setupQPushButton(QWidget* dialog)
{
	const QList<QPushButton*> PushButtons = dialog->findChildren<QPushButton*>();
	for (QPushButton* button : PushButtons)
	{
		string strFunction = button->objectName().mid(15, 1).toStdString();  // 使能"S" 移动"M"
		if (strFunction == "S")
		{
			connect(button, SIGNAL(clicked()), this, SLOT(OnClickedAxisEnable()));

			string	strAxis = button->objectName().mid(21, 2).toStdString();
			if (strAxis[1] != '1')//判断是否是两位的轴
				strAxis = strAxis[0];

			ControlInfos CI;
			CI.type = ControlType::PUSH_BUTTON;
			CI.qlcd = NULL;
			CI.qbtn = button;
			CI.axis = enum_cast<Axis>(strAxis).value();
			CI.lastValue = 0;
			m_vecControls.push_back(CI);
		}
		else
		{
			// 此处直接用长度排除IO的按钮，后续若修改则修改
			if (button->objectName().size()<27)
				setupButtonActions(button);
		}
	}
}

void QG_AxisWidget::setupButtonActions(QPushButton* PushButton)
{
	// 确保每个按钮只设置一次
	if (m_buttonStates.contains(PushButton))
		return;

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
		OnClickedAxisMove(1, PushButton);
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
				OnClickedAxisMove(2, PushButton);
			}
		}
		else if (btnState.bLongPress) {
			// 松按钮
			OnClickedAxisMove(0, PushButton);
			btnState.bLongPress = false;
		}
		});
}

void QG_AxisWidget::OnClickedAxisMove(int iType, QPushButton* PushButton)
{
	if (!m_pService->GetMotionControl() || !m_pService->GetMotionControl()->IsConnected())
	{
		SHOW_OPER_WARN(WarnCode::WARN_MC_DISCONNECTED);
		return;
	}

	string	strAxis = PushButton->objectName().mid(20, 2).toStdString();
	if (strAxis[1] != '1')//判断是否是两位的轴
		strAxis = strAxis[0];
	Axis	eAxis = enum_cast<Axis>(strAxis).value();
	string	strDirection = PushButton->objectName().right(1).toStdString();// 正反向"s" 负方向"g"

	double	dPos = ui.lineEdit_AxisMove_dPos->text().toDouble();

	double dVel, dAcc, dDec, dJerk;
	bool bDirection = true;
	if (strDirection == "g")
	{
		dPos = -dPos;
		bDirection = false;
	}
	dVel = 10.0;
	if (auto machine = lcnc::Kernel::current().services().getService<lcnc::MachineConfigurationService>()) {
		for (const auto& axis : machine->axisConfigurations()) {
			if (axis.axis.name.compare(QString::fromStdString(strAxis), Qt::CaseInsensitive) != 0) continue;
			dVel = m_strSpeedMode == "fLowSpeed" ? axis.lowSpeed
				: (m_strSpeedMode == "fHighSpeed" ? axis.highSpeed : axis.mediumSpeed);
			break;
		}
	}

	switch (iType)
	{
	case 0:
	{
		LOG_OPER_INFO(tr("Lift %1 stop.").arg(strAxis.c_str()).toUtf8().constData());
		m_pService->GetMotionControl()->StopMotion(eAxis);
		break;
	}
	case 1:
	{
		dPos = bDirection ? 10 : -10;
		if (!m_pService->GetMotionControl()->IsReachPos(eAxis, true, dPos))
			return;

		LOG_OPER_INFO(tr("Press %1 use %2 jog %3").arg(strAxis.c_str())
			.arg(m_strSpeedMode.c_str()).arg(bDirection ? '+' : '-').toUtf8().constData());
		m_pService->GetMotionControl()->Jog(eAxis, bDirection, dVel);
		break;
	}
	case 2:
	{
		if (m_pService->GetMotionControl()->IsAxisMoving(eAxis))
			return;
		if (!m_pService->GetMotionControl()->IsReachPos(eAxis, true, dPos))
		{
			double dTargetPos;
			m_pService->GetMotionControl()->GetActualPos(eAxis, dTargetPos);
			dTargetPos += dPos;
			SHOW_OPER_WARN(WarnCode::WARN_MC_OUTOFLIMIT, tr("%1 %2 target position %3 is out of limit")
				.arg("[WARN_MC_OUTOFLIMIT]").arg(strAxis.c_str()).arg(dTargetPos).toUtf8().constData());
			return;
		}
		LOG_OPER_INFO(tr("Click %1 use %2 move %3").arg(strAxis.c_str())
			.arg(tr(m_strSpeedMode.c_str())).arg(dPos).toUtf8().constData());
		m_pService->GetMotionControl()->MoveRelative(eAxis, dPos, dVel);
		break;
	}
	}
}

void QG_AxisWidget::OnClickedAxisEnable()
{
	if (!m_pService->GetMotionControl() || !m_pService->GetMotionControl()->IsConnected())
	{
		SHOW_OPER_WARN(WarnCode::WARN_MC_DISCONNECTED);
		return;
	}

	QPushButton* PushButton = qobject_cast<QPushButton*>(sender());
	string strAxis = PushButton->objectName().mid(21, 2).toStdString();
	if (strAxis[1] != '1')// 判断是否是第二组轴
		strAxis = strAxis[0];
	Axis eAxis = enum_cast<Axis>(strAxis).value();
	m_pService->GetMotionControl()->SetAxisEnable(eAxis, PushButton->isChecked());
}

void QG_AxisWidget::UpdateAimingBeam(bool bState)
{
	if (bState)
	{
		LOG_OPER_INFO(tr("Click AimingBeam = 1").toUtf8().data());
		ui.pushButton_AxisMove_AimingBeam->setChecked(true);
	}
	else
	{
		LOG_OPER_INFO(tr("Click AimingBeam = 0").toUtf8().data());
		ui.pushButton_AxisMove_AimingBeam->setChecked(false);
	}
}

void QG_AxisWidget::OnClickedAimingBeam()
{
	emit setAimingBeam(ui.pushButton_AxisMove_AimingBeam->isChecked());
}

void QG_AxisWidget::OnClickedradioButtonSpeedModelLow()
{
	m_strSpeedMode = "fLowSpeed";
}

void QG_AxisWidget::OnClickedradioButtonSpeedModelMedium()
{
	m_strSpeedMode = "fMediumSpeed";
}

void QG_AxisWidget::OnClickedradioButtonSpeedModelHigh()
{
	m_strSpeedMode = "fHighSpeed";
}


void QG_AxisWidget::UpdateAxisState()
{
	if (!m_pService->GetMotionControl() || !m_pService->GetMotionControl()->IsConnected())
		return;

	bool bEnable;
	double dPos;
	for (ControlInfos &Control : m_vecControls)
	{
		switch (Control.type)
		{
		case ControlType::LCD_NUMBER:
		{
			m_pService->GetMotionControl()->GetActualPos(Control.axis, dPos);
			if (dPos != Control.lastValue.toDouble())
			{
				Control.lastValue = dPos;
				Control.qlcd->display(QString::number(dPos, 'f', 3));
			}
			break;
		}	
		case ControlType::PUSH_BUTTON:
		{
			bEnable = m_pService->GetMotionControl()->IsEnabled(Control.axis);
			if (bEnable != Control.lastValue.toBool())
			{
				Control.lastValue = bEnable;
				Control.qbtn->setChecked(bEnable);
			}
			break;
		}
		default:
			break;
		}
	}
}

void QG_AxisWidget::SetupUI()
{
	bool bIsAxis;
	if (!DT::isExtensionAxis("A1") && !DT::isExtensionAxis("X1"))
	{
		delete ui.pushButton_AxisMove_X1Neg;
		delete ui.pushButton_AxisMove_X1Plus;
		delete ui.pushButton_AxisState_X1Enable;
		delete ui.lcdNumber_AxisState_X1Pos;

		ui.pushButton_AxisMove_X1Neg		= nullptr;
		ui.pushButton_AxisMove_X1Plus		= nullptr;
		ui.pushButton_AxisState_X1Enable	= nullptr;
		ui.lcdNumber_AxisState_X1Pos		= nullptr;
	}
	if(!DT::isExtensionAxis("Y1") && !DT::isExtensionAxis("Z1") && !DT::isExtensionAxis("A1"))
	{
		delete ui.lcdNumber_AxisState_Y1Pos;
		delete ui.pushButton_AxisMove_Y1Neg;
		delete ui.pushButton_AxisMove_Y1Plus;
		delete ui.pushButton_AxisState_Y1Enable;

		ui.pushButton_AxisMove_Y1Neg		= nullptr;
		ui.pushButton_AxisMove_Y1Plus		= nullptr;
		ui.pushButton_AxisState_Y1Enable	= nullptr;
		ui.lcdNumber_AxisState_Y1Pos		= nullptr;
	}


	for (const auto& eAxis : magic_enum::enum_values<Axis>())
	{
		if (DT::IsAxisUse(eAxis))
		{
			string sAxis = enum_name(eAxis).data();
			switch (eAxis)
			{
			case Axis::Y:
				break;
			case Axis::A:
				if (!DT::IsAxisUse(Axis::Y))//判断是否存在Y轴
				{
					ui.pushButton_AxisMove_YPlus->setObjectName("pushButton_AxisMove_APlus");
					ui.pushButton_AxisMove_YNeg->setObjectName("pushButton_AxisMove_ANeg");
					ui.pushButton_AxisState_YEnable->setObjectName("pushButton_AxisState_AEnable");
					ui.lcdNumber_AxisState_YPos->setObjectName("lcdNumber_AxisState_APos");
				}
				else
				{
					ui.pushButton_AxisMove_Y1Plus->setObjectName("pushButton_AxisMove_A1Plus");
					ui.pushButton_AxisMove_Y1Neg->setObjectName("pushButton_AxisMove_A1Neg");
					ui.pushButton_AxisState_Y1Enable->setObjectName("pushButton_AxisState_A1Enable");
					ui.lcdNumber_AxisState_Y1Pos->setObjectName("lcdNumber_AxisState_A1Pos");
				}
				break;
			default:
				break;
			}
		}
	}
}
