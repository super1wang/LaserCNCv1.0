#include "app/app_context.h"
#include "core/kernel/kernel.h"
#include "app/main_window.h"
#include "core/project/lcnc_project_manager.h"
#include "core/task/task_manager.h"
#include "view/gui_application.h"
#include "view/gui_document.h"
#include "modules/cad/cad_module.h"
#include "modules/cam/cam_module.h"
#include "modules/process/process_module.h"

AppContext::AppContext(MainWindow* mainWindow, QObject* parent)
    : QObject(parent)
    , m_mainWindow(mainWindow)
{}

lcnc::LcncProjectManager* AppContext::projectManager() const { return lcnc::Kernel::current().projectManager(); }
GuiApplication*  AppContext::guiApp() const { return lcnc::Kernel::current().guiApp();  }
TaskManager*     AppContext::taskMgr()const { return lcnc::Kernel::current().taskManager();      }

GuiDocument* AppContext::activeGuiDocument() const
{
    return lcnc::Kernel::current().guiApp()->activeGuiDocument();
}

DocumentId AppContext::workpieceDocumentId() const
{
    return projectManager()->workpieceDocumentId();
}

DocumentId AppContext::machineDocumentId() const
{
    return projectManager()->machineDocumentId();
}

DocumentId AppContext::camDocumentId() const
{
    return projectManager()->camDocumentId();
}

LcncDocument* AppContext::workpieceDocument() const
{
    return projectManager()->workpieceDocument();
}

LcncDocument* AppContext::machineDocument() const
{
    if (CamModule* cam = camModule())
        return cam->machineDocument();
    return projectManager()->machineDocument();
}

LcncDocument* AppContext::camDocument() const
{
    if (CamModule* cam = camModule())
        return cam->camDocument();
    return projectManager()->camDocument();
}

WidgetOccView* AppContext::occView() const
{
    return m_mainWindow ? m_mainWindow->occView() : nullptr;
}

CadModule* AppContext::cadModule() const
{
    return lcnc::Kernel::current().service<CadModule>();
}

CamModule* AppContext::camModule() const
{
    return lcnc::Kernel::current().service<CamModule>();
}

ProcessModule* AppContext::processModule() const
{
    return lcnc::Kernel::current().service<ProcessModule>();
}

void AppContext::updateCommandStates()
{
    // Delegated to MainWindow via its CommandContainer
    if (m_mainWindow)
        m_mainWindow->updateCommandStates();
}

bool AppContext::isMachineViewActive() const
{
    return m_mainWindow && m_mainWindow->isMachineViewActive();
}
