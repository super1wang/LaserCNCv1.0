#pragma once

#include <QWidget>
#include "ui_qg_ControlWidget.h"

#include "Service.h"
#include "ProcessModule.h"
#include <qmessagebox.h>
#include <QAction>
#include <QDateTime>

class QG_ControlWidget : public QWidget
{
	Q_OBJECT

	// 按钮数据位
	enum StateMask : uint8_t
	{
		INIT_BIT			= 0x01, // 00000001 - 第0位
		RUN_BIT				= 0x02, // 00000010 - 第1位  
		PAUSE_BIT			= 0x04, // 00000100 - 第2位
		CONTINUE_BIT		= 0x08, // 00001000 - 第3位
		STOP_BIT			= 0x10, // 00010000 - 第4位
		BACK_TO_IDLE_BIT	= 0x20, // 00100000 - 第5位
		// 0x40, 0x80 保留位
	};

	// 状态下显示的按钮
	enum PredefinedStates : uint8_t {
		IDLE	= INIT_BIT | RUN_BIT | STOP_BIT | BACK_TO_IDLE_BIT,	// 00110011
		RUNNING	= STOP_BIT | PAUSE_BIT, 							// 00010100
		PAUSED	= STOP_BIT | CONTINUE_BIT,							// 00011000
		STOP	= INIT_BIT | RUN_BIT | STOP_BIT | BACK_TO_IDLE_BIT,	// 00110011
	};


public:
	QG_ControlWidget(QWidget* parent = 0, const char* name = 0);
	~QG_ControlWidget();


protected:
	void	closeEvent(QCloseEvent* event) override;
	void	timerEvent(QTimerEvent* event);			// QObject定时器响应

public:
	void	SetService(Service*);

signals:
	void	SignalUpdateControlState();
	void	SignalEnabledUI(bool bEnable);

private slots:
	void	UpdateControlState();
	void	OnClickedButtonInit();
	void	OnClickedButtonRun();
	void	OnClickedButtonContinue();
	void	OnClickedButtonPause();
	void	OnClickedButtonStop();
	void	OnClickedButtonBackToIdle();

private:
	Service*		m_pService;
	int				m_nTimerId;					// 刷新定时器ID
	SystemStatus	m_eCurrentStatus;
	uint8_t			m_currentState		= IDLE;	// 按钮状态
	qint64			m_lastTriggerTime	= 0;	// 上次触发时间戳
	qint64			m_lastTriggerTime1	= 0;	// 上次触发时间戳
	const qint64	DEBOUNCE_DELAY		= 1000;	// 去抖延时1000ms

public:
	QMap<QString, QAction*> action_map;

private:
	Ui::QG_ControlWidgetClass ui;
};

