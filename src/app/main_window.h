#pragma once

#include <SARibbonMainWindow.h>
#include <QSet>
#include <cstdint>
#include "core/project/project_types.h"
#include "app/project_explorer_model.h"

class AppContext;
class CommandContainer;
class WidgetOccView;
class WidgetMachinePanel;
class WidgetMachineTree;
class WidgetLaserControl;
class WidgetToolpathPanel;
class DialogTaskManager;
namespace lcnc::cad::ui { class WidgetCadTaskPanel; }
namespace lcnc::cam::ui { class DialogAxisCalibrationWizard; }
namespace lcnc::app { class StartGuideWidget; }
class GraphicsScene;
class QStackedWidget;
class QSplitter;
class QLabel;
class QTabWidget;
class QTreeWidget;
class QTreeWidgetItem;
class QWidget;

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
    void buildViewTab(class SARibbonCategory* cat);
    void buildCadTab(class SARibbonCategory* cat);
    void buildCamTab(class SARibbonCategory* cat);
    void buildLaserTab(class SARibbonCategory* cat);
    void rebuildProjectExplorer();
    void handleProjectExplorerRowsMoved();
    void restorePersistedCamState();
    void syncMachineWorkspaceUi();
    void syncMachineWorkspaceUiInternal(bool rebuildTree);
    /// Regenerate the transient CAD primitive preview from the right task panel.
    void updateCadPrimitivePreview();
    /// Regenerate the transient CAD feature preview from the right task panel.
    void updateCadFeaturePreview();
    /// Regenerate the transient CAD transform preview and transform gizmo.
    void updateCadTransformPreview();
    /// Refresh CAD TaskPanel command availability from active document context.
    void updateCadTaskPanelState();
    /// Sync the TaskPanel sketch element list from the CAD module session state.
    void refreshSketchElementsView();
    /// Sync the TaskPanel home-page finished sketches list.
    void refreshFinishedSketchesView();
    /// Sync CAD sketch overlays from module snapshots to the OCC view.
    void updateCadSketchOverlay();
    /// Highlight the contour AIS corresponding to the selected toolpath node.
    void highlightContourInView(int contourIndex);
    /// Select a CAD sketch overlay item emitted by the OCC view.
    void handleCadSketchOverlayPicked(const QString& key);
    /// Move an active sketch overlay item by a local sketch-plane delta.
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
    void selectProjectExplorerEntries(DocumentId docId, const QStringList& entries, bool cadOnly);

    // ── Members ───────────────────────────────────────────────────────────────
    AppContext*        m_appContext{nullptr};
    CommandContainer*  m_cmdContainer{nullptr};

    // Widgets
    QSplitter*         m_splitter{nullptr};
    QTabWidget*        m_centerTabs{nullptr};
    lcnc::app::StartGuideWidget* m_startGuide{nullptr};
    QTabWidget*        m_leftTabs{nullptr};
    QTreeWidget*       m_projectExplorerTree{nullptr};
    QWidget*           m_processLeftPanel{nullptr};
    WidgetMachineTree* m_machineTree{nullptr};
    WidgetOccView*     m_occView{nullptr};
    QStackedWidget*    m_rightStack{nullptr};
    QTabWidget*        m_camRightTabs{nullptr};   // CAM ribbon 右栏：机床面板 / 刀路参数面板 两个 tab
    WidgetMachinePanel*   m_machinePanel{nullptr};
    lcnc::cad::ui::WidgetCadTaskPanel* m_cadTaskPanel{nullptr};
    WidgetToolpathPanel*   m_toolpathPanel{nullptr};
    WidgetLaserControl*    m_laserControl{nullptr};
    DialogTaskManager*     m_taskDialog{nullptr};
    GraphicsScene*         m_defaultScene{nullptr};
    QAction*               m_actRotaryAxisGuides{nullptr};
    QAction*               m_actCutterHeadGuide{nullptr};
    QAction*               m_actMachineModelVisible{nullptr};

    // Status bar labels
    QLabel* m_sbDocName{nullptr};
    QLabel* m_sbCoords{nullptr};
    QLabel* m_sbStatus{nullptr};
    QString m_pendingCalibrationTarget;
    lcnc::cam::ui::DialogAxisCalibrationWizard* m_axisCalibWizard{nullptr};
    lcnc::app::ProjectExplorerSnapshot m_projectExplorerSnapshot;
    bool m_machineWorkspaceActive{false};
    bool m_blockProjectExplorerSignals{false};
    bool m_skipNextSourceRecent{false};
    QString m_pendingRecentThumbnailPath;
    QSet<std::uint64_t> m_lastExplorerContourSelection; ///< 上一帧 Explorer 选中的 contourId，用于差分推 SelectionService
};
