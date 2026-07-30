#pragma once

#include <QWidget>

namespace lcnc::process {
class ProcessFlowDocument;
class ProcessFlowModel;
class ProcessFlowTreeView;
}

class ProcessFlowWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ProcessFlowWidget(QWidget* parent = nullptr, const char* name = nullptr);
    ~ProcessFlowWidget();

    void setFlowDocument(lcnc::process::ProcessFlowDocument* document);
    void reloadFlowModel();
    lcnc::process::ProcessFlowTreeView* flowTreeView() const { return m_flowTreeView; }

private:
    lcnc::process::ProcessFlowModel* m_flowModel{nullptr};
    lcnc::process::ProcessFlowTreeView* m_flowTreeView{nullptr};
};
