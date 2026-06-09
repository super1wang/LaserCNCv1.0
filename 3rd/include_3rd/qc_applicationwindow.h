#pragma once
#include <QMainWindow>
class QC_MDIWindow;
class QG_GraphicView;
class Service;

class QC_ApplicationWindow : public QMainWindow {
public:
    static QC_ApplicationWindow* getAppWindow();
    QC_MDIWindow* getMDIWindow();
    Service* getService() { return m_service; }
    void setService(Service* s) { m_service = s; }
private:
    static QC_ApplicationWindow* s_instance;
    Service* m_service{nullptr};
};
