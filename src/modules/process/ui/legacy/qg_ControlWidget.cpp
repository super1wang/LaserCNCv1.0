#include "qg_ControlWidget.h"
#include "qc_applicationwindow.h"
#include "qg_graphicview.h"
#include "qc_mdiwindow.h"
#include "rs_selection.h"

#include <QButtonGroup>
#include <QDialogButtonBox>

QG_ControlWidget::QG_ControlWidget(QWidget* parent, const char* name)
	: QWidget(parent)
{
	ui.setupUi(this);

	connect(ui.pushButton_Init,			SIGNAL(clicked()),	this, SLOT(OnClickedButtonInit()));
	connect(ui.pushButton_Run,			SIGNAL(clicked()),	this, SLOT(OnClickedButtonRun()));
	connect(ui.pushButton_Continue,		SIGNAL(clicked()),	this, SLOT(OnClickedButtonContinue()));
	connect(ui.pushButton_Pause,		SIGNAL(clicked()),	this, SLOT(OnClickedButtonPause()));
	connect(ui.pushButton_Stop,			SIGNAL(clicked()),	this, SLOT(OnClickedButtonStop()));
	connect(ui.pushButton_BackToIdle,	SIGNAL(clicked()),	this, SLOT(OnClickedButtonBackToIdle()));
	connect(this,	   SIGNAL(SignalUpdateControlState()),	this, SLOT(UpdateControlState()));


	// 右键点击逻辑（改为弹出确认框）
	ui.pushButton_Run->setContextMenuPolicy(Qt::CustomContextMenu); // 允许右键自定义事件
	QObject::connect(ui.pushButton_Run, &QPushButton::customContextMenuRequested, [=]() {
		// 创建对话框
		QDialog* dialog = new QDialog();
		dialog->setWindowTitle(tr("Start Test Model"));

		// 创建布局和组件
		QVBoxLayout* layout = new QVBoxLayout(dialog);

		QButtonGroup* buttonGroup = new QButtonGroup(dialog);
		QRadioButton* radioCuttingTest = new QRadioButton(tr("Cutting Test"), dialog);
		QRadioButton* radioProcessTest = new QRadioButton(tr("Process Test"), dialog);

		// 设置默认选择
		radioCuttingTest->setChecked(true);

		// 将radio button添加到按钮组实现互斥
		buttonGroup->addButton(radioCuttingTest);
		buttonGroup->addButton(radioProcessTest);

		// 创建按钮框
		QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, dialog);
		
		buttonBox->button(QDialogButtonBox::Ok)->setText(tr("Start"));
		buttonBox->button(QDialogButtonBox::Cancel)->setText(tr("Cancel"));

		// 添加到布局
		layout->addWidget(radioCuttingTest);
		layout->addWidget(radioProcessTest);
		layout->addWidget(buttonBox);

		// 连接按钮信号
		QObject::connect(buttonBox, &QDialogButtonBox::accepted, dialog, &QDialog::accept);
		QObject::connect(buttonBox, &QDialogButtonBox::rejected, dialog, &QDialog::reject);

		// 显示对话框并等待用户选择
		if (dialog->exec() == QDialog::Accepted)
		{
			if (radioCuttingTest->isChecked())
				PROCESSMODULE->Start(RunMode::CuttingTest);
			else if (radioProcessTest->isChecked())
				PROCESSMODULE->Start(RunMode::ProcessTest);
		}
		});

	m_nTimerId = QObject::startTimer(50);
}

QG_ControlWidget::~QG_ControlWidget()
{
}

void QG_ControlWidget::closeEvent(QCloseEvent* event)
{
	QAction* action = action_map["ViewControl"];
	action->setChecked(false);
}

void QG_ControlWidget::timerEvent(QTimerEvent* event)
{
	if (event->timerId() == m_nTimerId)
		emit SignalUpdateControlState();

	if (event->timerId() == m_nTimerId)
	{
		int iState = 0;
		if (g_eState.load() == SystemStatus::Idle &&
			m_pService->GetMotionControl() &&
			m_pService->GetMotionControl()->IsConnected() &&
			m_pService->GetMotionControl()->DigitalInputGet(DigitalIN::Start, iState) &&
			iState)
		{
			// 获取当前时间
			qint64 currentTime = QDateTime::currentMSecsSinceEpoch();

			// 检查是否超过去抖时间
			if (currentTime - m_lastTriggerTime >= DEBOUNCE_DELAY)
			{
				m_lastTriggerTime = currentTime; // 更新触发时间
				PROCESSMODULE->Start();
			}
		}

		if (g_eState.load() == SystemStatus::Processing &&
			m_pService->GetMotionControl() &&
			m_pService->GetMotionControl()->IsConnected() &&
			m_pService->GetMotionControl()->DigitalInputGet(DigitalIN::Stop, iState) &&
			iState)
		{
			// 获取当前时间
			qint64 currentTime = QDateTime::currentMSecsSinceEpoch();

			// 检查是否超过去抖时间
			if (currentTime - m_lastTriggerTime1 >= DEBOUNCE_DELAY)
			{
				m_lastTriggerTime1 = currentTime; // 更新触发时间
				PROCESSMODULE->Stop();
			}
		}
	}

}

void QG_ControlWidget::SetService(Service* pService)
{
	m_pService = pService;
}

void QG_ControlWidget::OnClickedButtonInit()
{
	QMessageBox::StandardButton rb = QMessageBox::question(NULL, tr("INIT"), tr("Do you want to Init?"), QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
	if (rb == QMessageBox::Yes)
	{
		PROCESSMODULE->Init();
	}
}

void QG_ControlWidget::OnClickedButtonRun()
{
	PROCESSMODULE->Start();
}

void QG_ControlWidget::OnClickedButtonContinue()
{
	PROCESSMODULE->Continue();
}

void QG_ControlWidget::OnClickedButtonPause()
{
	PROCESSMODULE->Pause();
}

void QG_ControlWidget::OnClickedButtonStop()
{
	PROCESSMODULE->Stop();
}

void QG_ControlWidget::OnClickedButtonBackToIdle()
{
	PROCESSMODULE->BackToIdle();
	RS_GraphicView* gv = QC_ApplicationWindow::getAppWindow()->getGraphicView();
	QC_MDIWindow* m = QC_ApplicationWindow::getAppWindow()->getMDIWindow();
	RS_EntityContainer* d = QC_ApplicationWindow::getAppWindow()->getDocument();
	if (gv && m && m->getDocument()) {
		gv->killAllActions();
		RS_Selection s((RS_EntityContainer&)*m->getDocument(), gv);
		s.selectAll(false);
		for (auto e : *d)
		{
			e->setPen(RS_Pen(RS_Color(RS2::FlagByLayer),
				RS2::WidthByLayer,
				RS2::LineByLayer));
		}
		m->getGraphicView()->RedrawDrawing();
	}
}

void QG_ControlWidget::UpdateControlState()
{
	SystemStatus eState = g_eState.load();
	if (m_eCurrentStatus == eState)
		return;

	m_eCurrentStatus = eState;
	switch (eState)
	{
	case SystemStatus::UnInit:			emit SignalEnabledUI(true);		m_currentState = IDLE;			break;
	case SystemStatus::Initializing:	emit SignalEnabledUI(false);	m_currentState = IDLE;			break;
	case SystemStatus::Idle:			emit SignalEnabledUI(true);		m_currentState = IDLE;			break;
	case SystemStatus::Paused:			emit SignalEnabledUI(true);		m_currentState = PAUSED;		break;
	case SystemStatus::Pausing:			emit SignalEnabledUI(false);	m_currentState = PAUSED;		break;
	case SystemStatus::Processing:		emit SignalEnabledUI(false);	m_currentState = RUNNING;		break;
	case SystemStatus::LaserProcessing:	emit SignalEnabledUI(false);	m_currentState = RUNNING;		break;
	case SystemStatus::Error:			emit SignalEnabledUI(true);		m_currentState = STOP;			break;
	default: break;
	}

	ui.pushButton_Init		->setHidden((m_currentState & INIT_BIT)			== 0);
	ui.pushButton_Run		->setHidden((m_currentState & RUN_BIT)			== 0);
	ui.pushButton_Pause		->setHidden((m_currentState & PAUSE_BIT)		== 0);
	ui.pushButton_Continue	->setHidden((m_currentState & CONTINUE_BIT)		== 0);
	ui.pushButton_Stop		->setHidden((m_currentState & STOP_BIT)			== 0);
	ui.pushButton_BackToIdle->setHidden((m_currentState & BACK_TO_IDLE_BIT) == 0);
}

