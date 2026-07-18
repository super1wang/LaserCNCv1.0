#pragma once

#include "core/project/project_types.h"

class GuiApplication;
class GuiDocument;
class LcncDocument;
class TaskManager;
class CadModule;
class CamModule;
class ProcessModule;
class WidgetOccView;
namespace lcnc { class LcncProjectManager; }

/**
 * @brief UI command context injected into QAction-backed commands.
 *
 * Lives in the app boundary because it exposes GUI documents, widgets, and
 * module facades needed by UI command adapters. Core command primitives only
 * keep an opaque pointer to this interface.
 */
class IAppContext
{
public:
    virtual ~IAppContext() = default;

    virtual lcnc::LcncProjectManager* projectManager() const = 0;
    virtual GuiApplication*  guiApp()   const = 0;
    virtual TaskManager*     taskMgr()  const = 0;

    virtual GuiDocument*      activeGuiDocument() const = 0;

    virtual DocumentId        workpieceDocumentId() const = 0;
    virtual DocumentId        machineDocumentId()   const = 0;
    virtual DocumentId        camDocumentId()       const = 0;
    virtual LcncDocument*     workpieceDocument() const = 0;
    virtual LcncDocument*     machineDocument()   const = 0;
    virtual LcncDocument*     camDocument()       const = 0;
    virtual WidgetOccView*    occView()            const = 0;

    virtual CadModule*        cadModule()         const = 0;
    virtual CamModule*        camModule()         const = 0;
    virtual ProcessModule*    processModule()     const = 0;

    /// Returns true when the 准备 (machine) tab is currently active.
    virtual bool isMachineViewActive() const = 0;

    /// Ask the main window to refresh enabled/disabled state of all actions.
    virtual void updateCommandStates() = 0;
};