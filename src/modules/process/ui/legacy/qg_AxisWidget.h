#ifndef QG_AXISWIDGET_H
#define QG_AXISWIDGET_H
#pragma once

#include <QWidget>
#include "ui_qg_AxisWidget.h"
#include "Service.h"
#include <QIcon>
#include <QSettings>

#define AXISWIDGET QG_AxisWidget::instance()

class QG_IOWidget;

enum ControlType
{
	LCD_NUMBER,
	PUSH_BUTTON,
	UNKNOWN
};

struct ControlInfos
{
	ControlType		type;       // 控件类型
	QLCDNumber*		qlcd;		// 控件指针
	QPushButton*	qbtn;		// 控件指针
	Axis			axis;       // 所属轴系
	QVariant		lastValue;	// 上一次的值
};

class QG_AxisWidget : public QWidget/*, public Ui::QG_AxisWidgetClass*/
{
	Q_OBJECT

public:
	QG_AxisWidget(QWidget *parent = 0, const char* name = 0);
	~QG_AxisWidget();
	static QG_AxisWidget* instance(QG_AxisWidget* pAxisWidget = nullptr);

public:
	void	SetService(Service*);
	void	ClearTimer();

public slots:
	void	UpdateAimingBeam(bool bState);
	void	OnClickedAimingBeam();
	void	OnClickedAxisEnable();
	void	OnClickedradioButtonSpeedModelLow();
	void	OnClickedradioButtonSpeedModelMedium();
	void	OnClickedradioButtonSpeedModelHigh();

private slots:
	void	UpdateAxisState();

signals:
	void	setAimingBeam(bool bState);

protected:
	void    SetupUI();
	void    setupQLcdNumber(QWidget* dialog);
	void	setupQPushButton(QWidget* dialog);		// 给窗体中按钮连接信号
	void 	setupButtonActions(QPushButton* button);
	void	OnClickedAxisMove(int iType, QPushButton* PushButton);			// 0停止 1Jog 2相对

private:
	QMap<QPushButton*, ButtonState> m_buttonStates;		// 控件状态
	vector<ControlInfos>			m_vecControls;		// 控件信息整理
	double							m_dPos;

private:
	Ui::QG_AxisWidgetClass			ui;
	Service*						m_pService;
	static QG_AxisWidget*			uniqueInstance;		// 类指针
};
extern string						m_strSpeedMode;
#endif