#ifndef QG_IOWIDGET_H
#define QG_IOWIDGET_H
#pragma once

#include <QIcon>
#include <QWidget>
#include "ui_qg_IOWidget.h"
#include "Service.h"

#define IOWIDGET QG_IOWidget::instance()

class QG_IOWidget : public QWidget/*, public Ui::QG_IOWidgetClass*/
{
	Q_OBJECT

public:
	QG_IOWidget(QWidget* parent = 0, const char* name = 0);
	~QG_IOWidget();
	static QG_IOWidget* instance(QG_IOWidget* pIOWidget = nullptr);

public:
	void	SetService(Service*);
	void	ClearTimer();

private slots:
	void	UpdateIOState();

public slots:
	void	OnClickedAimingBeam(bool bState);

signals:
	void	updateAimingBeam(bool bState);

protected:
	QIcon	CreateIcon(const QString& path);
	void	SetupUI();
	void	setupQPushButton(QWidget* dialog);						// 给窗体中按钮连接信号
	void 	setupButtonActions(QPushButton* button);
	void	OnClickedIOState(int iType, QPushButton* PushButton);	// 按钮数字量IO下发通用函数
	void	SetLight(DigitalOUT eLight);
private:
	int		m_iRed;
	int		m_iGreen;
	int		m_iYellow;
	int		m_iLightState;
	int		m_iBlowState;
	int		m_iLaserState;
	int		m_iWaterState;
	int		m_iPumpState;
	int		m_iChuckState;
	int		m_iPliersState;
	int		m_iOut1State;
	
	int		m_nTimerId;			// 刷新定时器ID
	int		m_iCustomerID;		// 用户ID，控制部分IO刷新
	bool	m_bWaterCutting;	// 湿切相关IO

	SystemStatus m_eState;

protected:
	QIcon m_qiconGraw;
	QIcon m_qiconRed;
	QIcon m_qiconYellow;
	QIcon m_qiconGreen;

private:
	Ui::QG_IOWidgetClass			ui;
	Service*						m_pService;
	QMap<QPushButton*, ButtonState> m_buttonStates;		// 按钮状态
	static QG_IOWidget*				uniqueInstance;		// 类指针
};
#endif