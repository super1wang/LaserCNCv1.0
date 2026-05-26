#include "qg_processeswidget.h"

QG_ProcessesWidget::QG_ProcessesWidget(QWidget* parent, const char* name/*, Service* pService*/)
	: QWidget(parent)
	//, m_pService(pService)
{
	ui.setupUi(this);
}

QG_ProcessesWidget::~QG_ProcessesWidget()
{}

ProcessTreeView* QG_ProcessesWidget::GetTreeView()
{
	return ui.treeView_Processes;
}
