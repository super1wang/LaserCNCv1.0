#pragma once

#include <QObject>
#include "app/app_command_context.h"

class MainWindow;
class CadModule;
class CamModule;
class ProcessModule;
class WidgetOccView;

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

    lcnc::LcncProjectManager* projectManager() const override;
    GuiApplication*  guiApp()  const override;
    TaskManager*     taskMgr() const override;

    GuiDocument*  activeGuiDocument() const override;

    DocumentId    workpieceDocumentId() const override;
    DocumentId    machineDocumentId()   const override;
    DocumentId    camDocumentId()       const override;
    LcncDocument* workpieceDocument() const override;
    LcncDocument* machineDocument()   const override;
    LcncDocument* camDocument()       const override;
    WidgetOccView* occView()           const override;

    CadModule*     cadModule()     const override;
    CamModule*     camModule()     const override;
    ProcessModule* processModule() const override;
    lcnc::cad::ICadProjectExplorerProjection* cadProjectExplorerProjection() const override;
    lcnc::cam::ICamProjectExplorerProjection* camProjectExplorerProjection() const override;

    bool isMachineViewActive() const override;

    void updateCommandStates() override;

private:
    MainWindow* m_mainWindow;
};
