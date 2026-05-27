#include "qg_processeswidget.h"

#include "modules/process/ui/process_flow_model.h"
#include "modules/process/ui/process_flow_tree_view.h"

#include <QVBoxLayout>

QG_ProcessesWidget::QG_ProcessesWidget(QWidget* parent, const char* name/*, Service* pService*/)
	: QWidget(parent)
{
	Q_UNUSED(name);
	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(0, 0, 0, 0);

	m_flowModel = new lcnc::process::ProcessFlowModel(this);
	m_flowTreeView = new lcnc::process::ProcessFlowTreeView(this);
	m_flowTreeView->setFlowModel(m_flowModel);
	layout->addWidget(m_flowTreeView);
}

QG_ProcessesWidget::~QG_ProcessesWidget()
{}

void QG_ProcessesWidget::setFlowDocument(lcnc::process::ProcessFlowDocument* document)
{
	m_flowModel->setDocument(document);
	m_flowTreeView->expandAll();
}

void QG_ProcessesWidget::reloadFlowModel()
{
	m_flowModel->resetFromDocument();
	m_flowTreeView->expandAll();
}
