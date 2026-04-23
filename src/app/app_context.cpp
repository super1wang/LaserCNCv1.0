#include "app/app_context.h"
#include "core/kernel/kernel.h"
#include "app/main_window.h"
#include "core/document/lcnc_application.h"
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

LcncApplication* AppContext::app()    const { return lcnc::Kernel::current().app(); }
GuiApplication*  AppContext::guiApp() const { return lcnc::Kernel::current().guiApp();  }
TaskManager*     AppContext::taskMgr()const { return lcnc::Kernel::current().taskManager();      }

DocumentId AppContext::activeDocumentId() const
{
    return lcnc::Kernel::current().app()->activeDocumentId();
}

LcncDocument* AppContext::activeDocument() const
{
    return lcnc::Kernel::current().app()->activeDocument();
}

GuiDocument* AppContext::activeGuiDocument() const
{
    return lcnc::Kernel::current().guiApp()->activeGuiDocument();
}

LcncDocument* AppContext::machineDocument() const
{
    return lcnc::Kernel::current().app()->machineDocument();
}

GuiDocument* AppContext::machineGuiDocument() const
{
    DocumentId id = lcnc::Kernel::current().app()->machineDocumentId();
    return lcnc::Kernel::current().guiApp()->guiDocument(id);
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
