#pragma once

#include <QObject>
#include "app/i_app_context.h"

class MainWindow;
class CadModule;
class CamModule;
class ProcessModule;

/**
 * @brief Concrete IAppContext that holds raw pointers to all singletons.
 *
 * Constructed by MainWindow and passed to every Command.
 */
class AppContext : public QObject, public IAppContext
{
    Q_OBJECT
public:
    explicit AppContext(MainWindow* mainWindow, QObject* parent = nullptr);

    LcncApplication* app()     const override;
    GuiApplication*  guiApp()  const override;
    TaskManager*     taskMgr() const override;

    DocumentId    activeDocumentId()  const override;
    LcncDocument* activeDocument()    const override;
    GuiDocument*  activeGuiDocument() const override;

    LcncDocument* machineDocument()    const override;
    GuiDocument*  machineGuiDocument() const override;

    CadModule*     cadModule()     const override;
    CamModule*     camModule()     const override;
    ProcessModule* processModule() const override;

    bool isMachineViewActive() const override;

    void updateCommandStates() override;

private:
    MainWindow* m_mainWindow;
};
