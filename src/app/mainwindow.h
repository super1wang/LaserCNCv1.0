#pragma once

#include <SARibbonMainWindow.h>
#include "base/lcnc_application.h"

class AppContext;
class CommandContainer;
class WidgetOccView;
class WidgetModelTree;
class WidgetMachinePanel;
class WidgetLaserControl;
class WidgetToolpathPanel;
class DialogTaskManager;
class GraphicsScene;
class QTabWidget;
class QStackedWidget;
class QSplitter;
class QLabel;
class QTimer;
class QTreeWidget;
class QTreeWidgetItem;

/**
 * @brief The application's main window.
 *
 * Inherits SARibbonMainWindow to get the Ribbon-style toolbar.
 * Layout:
 *   ┌──────────────── Ribbon ──────────────────────────────┐
 *   │ ┌──────┬──────────────────────┬────────────────────┐ │
 *   │ │ Left │    3D Viewport       │   Right Panel      │ │
 *   │ │ Panel│  (WidgetOccView)     │  (Stacked)         │ │
 *   │ │ Tab  │                      │  Prepare → Machine │ │
 *   │ │ 准备 │                      │  Execute → Laser   │ │
 *   │ │ 执行 │                      │  Control           │ │
 *   │ └──────┴──────────────────────┴────────────────────┘ │
 *   └─────────────────── Status Bar ───────────────────────┘
 */
class MainWindow : public SARibbonMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    /// Called by AppContext to refresh Ribbon button states.
    void updateCommandStates();
    WidgetOccView* occView() const { return m_occView; }

    /// Returns true when the 准备 (machine) tab is currently active.
    bool isMachineViewActive() const;

protected:
    void closeEvent(QCloseEvent*) override;

private:
    // ── Initialization steps ─────────────────────────────────────────────────
    void createContext();
    void createCommands();
    void createCentralLayout();
    void createLeftPanel();
    void createRightPanel();
    void create3DView();
    void createRibbon();
    void createStatusBar();

    // ── Ribbon tab builders ───────────────────────────────────────────────────
    void buildFileTab(class SARibbonCategory* cat);
    void buildCadTab(class SARibbonCategory* cat);
    void buildCamTab(class SARibbonCategory* cat);
    void buildLaserTab(class SARibbonCategory* cat);
    void rebuildDocumentTree();
    void rebuildContourListWidget();
    void restorePersistedCamState();
    void syncMachineWorkspaceUi();
    void syncMachineWorkspaceUiInternal(bool rebuildTree);
    /// Route 3D view to the machine workspace document.
    void showMachineView();
    /// Route 3D view to the specified workpiece document (defaults to active).
    void showWorkpieceView(DocumentId id = kInvalidDocumentId);

    // ── Slots ─────────────────────────────────────────────────────────────────
    void onLeftTabChanged(int index);
    void onDocumentAdded(DocumentId id);
    void onDocumentClosed(DocumentId id);
    void onActiveDocumentChanged(DocumentId id);
    void onDocumentModified(DocumentId id);
    void onDocumentTreeItemClicked(QTreeWidgetItem* item, int column);

    // ── Members ───────────────────────────────────────────────────────────────
    AppContext*        m_appContext{nullptr};
    CommandContainer*  m_cmdContainer{nullptr};

    // Widgets
    QSplitter*         m_splitter{nullptr};
    QTabWidget*        m_leftTabs{nullptr};
    WidgetOccView*     m_occView{nullptr};
    QStackedWidget*    m_rightStack{nullptr};
    WidgetModelTree*   m_modelTree{nullptr};
    QTreeWidget*       m_documentTree{nullptr};
    QTreeWidget*       m_contourListWidget{nullptr};  ///< contour list in "刀路" tab (drag-reorder)
    QTreeWidget*       m_processTree{nullptr};
    WidgetMachinePanel*   m_machinePanel{nullptr};
    WidgetToolpathPanel*   m_toolpathPanel{nullptr};
    WidgetLaserControl*    m_laserControl{nullptr};
    DialogTaskManager*     m_taskDialog{nullptr};
    GraphicsScene*         m_defaultScene{nullptr};

    // Status bar labels
    QLabel* m_sbDocName{nullptr};
    QLabel* m_sbCoords{nullptr};
    QLabel* m_sbStatus{nullptr};
    QTimer* m_machineRefreshTimer{nullptr};
    QString m_pendingCalibrationTarget;
};
