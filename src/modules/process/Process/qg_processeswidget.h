#pragma once

#include <QWidget>

namespace lcnc::process {
class ProcessFlowDocument;
class ProcessFlowModel;
class ProcessFlowTreeView;
}

class QG_ProcessesWidget : public QWidget
{
	Q_OBJECT

public:
	QG_ProcessesWidget(QWidget* parent = 0, const char* name = 0/*, Service* pService = nullptr*/);
	~QG_ProcessesWidget();

	void					setFlowDocument(lcnc::process::ProcessFlowDocument* document);
	void					reloadFlowModel();
	lcnc::process::ProcessFlowTreeView* flowTreeView() const { return m_flowTreeView; }

private:
	lcnc::process::ProcessFlowModel* m_flowModel{nullptr};
	lcnc::process::ProcessFlowTreeView* m_flowTreeView{nullptr};
};
