#include "modules/process/ui/process_flow_widget.h"

#include "modules/process/ui/process_flow_model.h"
#include "modules/process/ui/process_flow_tree_view.h"

#include <QVBoxLayout>

ProcessFlowWidget::ProcessFlowWidget(QWidget* parent, const char* name)
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

ProcessFlowWidget::~ProcessFlowWidget() = default;

void ProcessFlowWidget::setFlowDocument(lcnc::process::ProcessFlowDocument* document)
{
    m_flowModel->setDocument(document);
    m_flowTreeView->expandAll();
}

void ProcessFlowWidget::reloadFlowModel()
{
    m_flowModel->resetFromDocument();
    m_flowTreeView->expandAll();
}
