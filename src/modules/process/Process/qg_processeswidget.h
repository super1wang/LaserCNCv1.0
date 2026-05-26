#pragma once

#include <QWidget>
#include "ui_qg_processeswidget.h"
//#include "service.h"

class QG_ProcessesWidget : public QWidget
{
	Q_OBJECT

public:
	QG_ProcessesWidget(QWidget* parent = 0, const char* name = 0/*, Service* pService = nullptr*/);
	~QG_ProcessesWidget();

	ProcessTreeView*			GetTreeView();

private:

	Ui::QG_dlgProcessesClass	ui;
	//Service* m_pService;
};
