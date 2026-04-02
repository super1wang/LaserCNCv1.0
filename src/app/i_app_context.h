#pragma once

#include "base/lcnc_application.h"

class GuiApplication;
class GuiDocument;
class TaskManager;
class CadModule;
class CamModule;
class ProcessModule;

/**
 * @brief Thin context interface injected into every Command.
 *
 * Provides access to the application singletons without creating hard
 * circular dependencies between the command files and the main window.
 */
class IAppContext
{
public:
    virtual ~IAppContext() = default;

    virtual LcncApplication* app()      const = 0;
    virtual GuiApplication*  guiApp()   const = 0;
    virtual TaskManager*     taskMgr()  const = 0;

    virtual DocumentId        activeDocumentId()   const = 0;
    virtual LcncDocument*     activeDocument()     const = 0;
    virtual GuiDocument*      activeGuiDocument()  const = 0;

    virtual LcncDocument*     machineDocument()    const = 0;
    virtual GuiDocument*      machineGuiDocument() const = 0;

    virtual CadModule*        cadModule()         const = 0;
    virtual CamModule*        camModule()         const = 0;
    virtual ProcessModule*    processModule()     const = 0;

    /// Returns true when the 准备 (machine) tab is currently active.
    virtual bool isMachineViewActive() const = 0;

    /// Ask the main window to refresh enabled/disabled state of all actions.
    virtual void updateCommandStates() = 0;
};
