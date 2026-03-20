#include "app/app_context.h"
#include "app/mainwindow.h"
#include "base/lcnc_application.h"
#include "base/task_manager.h"
#include "gui/gui_application.h"
#include "gui/gui_document.h"

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

void AppContext::updateCommandStates()
{
    // Delegated to MainWindow via its CommandContainer
    if (m_mainWindow)
        m_mainWindow->updateCommandStates();
}
