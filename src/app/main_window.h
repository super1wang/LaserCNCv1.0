#pragma once

#include <SARibbonMainWindow.h>
#include <QSet>
#include <cstdint>
#include <memory>
#include "core/project/project_types.h"
#include "app/project_explorer_model.h"

class AppContext;
class CommandContainer;
class GuiDocument;
class WidgetOccView;
class WidgetMachinePanel;
class WidgetMachineTree;
class WidgetLaserControl;
class WidgetToolpathPanel;
namespace lcnc::cam::ui { class WidgetCollisionDetectionPanel; }
class DialogTaskManager;
namespace lcnc::cad::ui { class WidgetCadTaskPanel; }
namespace lcnc::cam::ui { class DialogAxisCalibrationWizard; }
namespace lcnc::app {
class ProjectExplorerController;
class CadTaskPanelController;
class StartGuideWidget;
class ViewStateController;
class WorkspacePresenter;
}
class GraphicsScene;
class QStackedWidget;
class QSplitter;
class QLabel;
class QProgressBar;
class QTabBar;
class QTabWidget;
class QTreeWidget;
class QTreeWidgetItem;
class QWidget;
class SARibbonCategory;

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
    WidgetOccView* createOccView(QWidget* parent);
    void connectOccViewSignals(WidgetOccView* view);
    WidgetOccView* ensureWorkspaceOccView(ProjectWorkspaceId id);
    void activateWorkspaceOccView(ProjectWorkspaceId id, GuiDocument* document);
    void removeWorkspaceOccView(ProjectWorkspaceId id);
    void showDefaultOccView();
    void createRibbon();
    void createStatusBar();

    // ── Ribbon tab builders ───────────────────────────────────────────────────
    void buildFileTab(class SARibbonCategory* cat);
    void buildViewTab(class SARibbonCategory* cat);
    void buildCadTab(class SARibbonCategory* cat);
    void buildCamTab(class SARibbonCategory* cat);
    void buildLaserTab(class SARibbonCategory* cat);
    void buildSimulationTab(class SARibbonCategory* cat);
    void enterOfflineSimulation();
    void exitOfflineSimulation();
    void rebuildProjectExplorer();
    void restorePersistedCamState();
    void syncMachineWorkspaceUi();
    void syncMachineWorkspaceUiInternal(bool rebuildTree);
    void updateCadPrimitivePreview();
    void updateCadFeaturePreview();
    void updateCadTransformPreview();
    void updateCadTaskPanelState();
    void refreshSketchElementsView();
    void refreshFinishedSketchesView();
    void updateCadSketchOverlay();
    /// Highlight the contour AIS corresponding to the selected toolpath node.
    void highlightContourInView(int contourIndex);
    void handleCadSketchOverlayPicked(const QString& key);
    void handleCadSketchOverlayDrag(const QString& key, double deltaX, double deltaY);
    /// Route 3D view to the machine workspace.
    void showMachineView();
    /// Route CAD context through the unified machine 3D view (defaults to active workpiece).
    void showWorkpieceView(DocumentId id = kInvalidDocumentId);
    /// Keep the right-side parameter page in sync with the active Ribbon page.
    void syncRightPanelForRibbonIndex(int index);
    void applyPersistedViewState();
    void persistViewDisplayMode(int displayMode, bool faceBoundary);
    void persistViewToggleState();
    void syncMachineTreeVisibilityState();
    void showStartGuide();
    void showViewTab();
    void refreshStartGuide();
    void refreshDocumentTabs();
    void openStartGuideFile(const QString& filePath);
    void addRecentFile(const QString& filePath);
    void scheduleRecentThumbnailCapture(const QString& filePath);
    void captureRecentThumbnail(const QString& filePath);
    QString recentThumbnailPath(const QString& filePath) const;
    bool isStartGuideSupportedFile(const QString& filePath) const;

    // ── Slots ─────────────────────────────────────────────────────────────────
    void onProjectExplorerCurrentItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* previous);
    void onProjectExplorerItemChanged(QTreeWidgetItem* item, int column);
    void onProjectExplorerItemDoubleClicked(QTreeWidgetItem* item, int column);
    void onProjectExplorerContextMenuRequested(const QPoint& pos);
    void onProjectReset();
    void onProjectDomainChanged(lcnc::ProjectDomain domain);
    void selectProjectExplorerContour(int contourIndex);
    void selectProjectExplorerContourById(lcnc::cam::ContourId contourId, int fallbackIndex = -1);
    void selectProjectExplorerContours(const QList<int>& contourIndexes);
    void selectProjectExplorerEntries(DocumentId docId, const QStringList& entries);
    /// 收集当前选中的轮廓 id（视图拾取 + 工程树多选的并集），供"移动到图层"等操作使用。
    QList<lcnc::cam::ContourId> gatherSelectedContourIds() const;

    // ── Members ───────────────────────────────────────────────────────────────
    AppContext*        m_appContext{nullptr};
    CommandContainer*  m_cmdContainer{nullptr};

    // Widgets
    QSplitter*         m_splitter{nullptr};
    QTabWidget*        m_centerTabs{nullptr};
    lcnc::app::StartGuideWidget* m_startGuide{nullptr};
    QTabBar*           m_documentTabs{nullptr};
    QStackedWidget*    m_viewStack{nullptr};
    QTabWidget*        m_leftTabs{nullptr};
    QTreeWidget*       m_projectExplorerTree{nullptr};
    QWidget*           m_processLeftPanel{nullptr};
    WidgetMachineTree* m_machineTree{nullptr};
    WidgetOccView*     m_occView{nullptr};        ///< Active OCC viewport; use occView() at call time.
    WidgetOccView*     m_defaultOccView{nullptr};
    std::unique_ptr<lcnc::app::WorkspacePresenter> m_workspacePresenter;
    QStackedWidget*    m_rightStack{nullptr};
    QTabWidget*        m_camRightTabs{nullptr};
    WidgetMachinePanel*   m_machinePanel{nullptr};
    lcnc::cad::ui::WidgetCadTaskPanel* m_cadTaskPanel{nullptr};
    WidgetToolpathPanel*   m_toolpathPanel{nullptr};
    lcnc::cam::ui::WidgetCollisionDetectionPanel* m_collisionDetectionPanel{nullptr};
    WidgetLaserControl*    m_laserControl{nullptr};
    DialogTaskManager*     m_taskDialog{nullptr};
    GraphicsScene*         m_defaultScene{nullptr};
    QAction*               m_actRotaryAxisGuides{nullptr};
    QAction*               m_actCutterHeadGuide{nullptr};
    QAction*               m_actMachineModelVisible{nullptr};
    SARibbonCategory*      m_simulationRibbonCategory{nullptr};
    QWidget*               m_simulationPage{nullptr};

    // Status bar labels
    QLabel* m_sbDocName{nullptr};
    QLabel* m_sbCoords{nullptr};
    QLabel* m_sbStatus{nullptr};
    QProgressBar* m_sbDeviceProgress{nullptr};
    QString m_pendingCalibrationTarget;
    lcnc::cam::ui::DialogAxisCalibrationWizard* m_axisCalibWizard{nullptr};
    lcnc::app::ProjectExplorerSnapshot m_projectExplorerSnapshot;
    std::unique_ptr<lcnc::app::ProjectExplorerController> m_projectExplorerController;
    std::unique_ptr<lcnc::app::CadTaskPanelController> m_cadTaskPanelController;
    std::unique_ptr<lcnc::app::ViewStateController> m_viewStateController;
    bool m_machineWorkspaceActive{false};
    bool m_blockProjectExplorerSignals{false};
    bool m_skipNextSourceRecent{false};
    QString m_pendingRecentThumbnailPath;
    QSet<std::uint64_t> m_lastExplorerContourSelection; ///< 上一帧 Explorer 选中的 contourId，用于差分推 SelectionService
};
