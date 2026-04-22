#include "app/app_context.h"
#include "app/mainwindow.h"
#include "base/lcnc_application.h"
#include "base/task_manager.h"
#include "gui/gui_application.h"
#include "gui/gui_document.h"
#include "modules/cad_module.h"
#include "modules/cam_module.h"
#include "modules/process_module.h"

AppContext::AppContext(MainWindow* mainWindow, QObject* parent)
    : QObject(parent)
    , m_mainWindow(mainWindow)
{}

LcncApplication* AppContext::app()    const { return LcncApplication::instance(); }
GuiApplication*  AppContext::guiApp() const { return GuiApplication::instance();  }
TaskManager*     AppContext::taskMgr()const { return TaskManager::instance();      }

DocumentId AppContext::activeDocumentId() const
{
    return LcncApplication::instance()->activeDocumentId();
}

LcncDocument* AppContext::activeDocument() const
{
    return LcncApplication::instance()->activeDocument();
}

GuiDocument* AppContext::activeGuiDocument() const
{
    return GuiApplication::instance()->activeGuiDocument();
}

LcncDocument* AppContext::machineDocument() const
{
    return LcncApplication::instance()->machineDocument();
}

GuiDocument* AppContext::machineGuiDocument() const
{
    DocumentId id = LcncApplication::instance()->machineDocumentId();
    return GuiApplication::instance()->guiDocument(id);
}

WidgetOccView* AppContext::occView() const
{
    return m_mainWindow ? m_mainWindow->occView() : nullptr;
}

CadModule* AppContext::cadModule() const
{
    return CadModule::instance();
}

CamModule* AppContext::camModule() const
{
    return CamModule::instance();
}

ProcessModule* AppContext::processModule() const
{
    return ProcessModule::instance();
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
