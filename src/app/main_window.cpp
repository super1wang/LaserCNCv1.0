#include "app/main_window.h"

#include "core/kernel/kernel.h"
#include "core/services/selection_service.h"
#include "app/app_context.h"
#include "app/command_registry.h"
#include "app/controllers/project_explorer_controller.h"
#include "app/controllers/cad_task_panel_controller.h"
#include "app/controllers/view_state_controller.h"
#include "app/controllers/workspace_presenter.h"
#include "app/project_explorer_tree_utils.h"
#include "app/start_guide_widget.h"
#include "core/command/commands_api.h"
#include "core/logging/logger.h"
#include "modules/cad/commands/commands_file.h"
#include "modules/cad/commands/commands_edit.h"
#include "modules/cad/commands/commands_cad.h"
#include "modules/cam/commands/commands_machine.h"
#include "modules/cam/commands/commands_cam.h"
#include "app/commands/commands_display.h"
#include "modules/cad/ui/ribbon_cad_tab.h"
#include "modules/cad/ui/widget_cad_task_panel.h"
#include "modules/cam/ui/ribbon_cam_tab.h"
#include "modules/process/ui/ribbon_process_tab.h"
#include "modules/process/ui/process_flow_widget.h"
#include "view/widget_occ_view.h"
#include "modules/cam/ui/widget_machine_panel.h"
#include "modules/cam/ui/widget_machine_tree.h"
#include "modules/cam/ui/widget_toolpath_panel.h"
#include "modules/cam/ui/dialog_axis_calibration_wizard.h"
#include "modules/process/ui/widget_laser_control.h"
#include "app/dialog/dialog_task_manager.h"
#include "modules/cam/settings/cam_config.h"
#include "core/algorithms/cam/laser_toolpath.h"
#include "core/document/lcnc_document.h"
#include "core/project/lcnc_project_manager.h"
#include "core/project/lcnc_project_package.h"
#include "core/project/project_workspace.h"
#include "core/kinematics/machine_configuration_service.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/document/xcaf_utils.h"
#include "view/graphics_scene.h"
#include "view/gui_application.h"
#include "view/gui_document.h"
#include "view/rendering_manager.h"
#include "view/sketch_overlay_renderer.h"
#include "view/world_axes_renderer.h"
#include "modules/cad/cad_module.h"
#include "modules/cad/selection/cad_selection_resolver.h"
#include "modules/cam/cam_module.h"
#include "core/machine/machine_workspace.h"
#include "modules/process/cutting/process_cutting_plan_service.h"
#include "modules/process/process_module.h"
#include "modules/process/workflow/process_workflow_service.h"

#include <SARibbonBar.h>
#include <SARibbonCategory.h>
#include <SARibbonPanel.h>

#include <QSplitter>
#include <QStackedWidget>
#include <QTreeWidget>
#include <QTreeWidgetItemIterator>
#include <QTabBar>
#include <QTabWidget>
#include <QHeaderView>
#include <QStatusBar>
#include <QLabel>
#include <QMenu>
#include <QAction>
#include <QActionGroup>
#include <QProgressBar>
#include <QSizePolicy>
#include <QToolTip>

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QComboBox>
#include <QCloseEvent>
#include <QMessageBox>
#include <QInputDialog>
#include <QLineEdit>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QPushButton>
#include <QColorDialog>
#include <QApplication>
#include <QFileInfo>
#include <QIcon>
#include <QPixmap>
#include <QTimer>
#include <QStyle>
#include <QSignalBlocker>
#include <QSet>
#include <QVariantMap>
#include <QCryptographicHash>
#include <QDir>
#include <QStandardPaths>
#include <functional>
#include <AIS_InteractiveContext.hxx>
#include <V3d_TypeOfOrientation.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopoDS_Shape.hxx>
#include <XCAFDoc_ShapeTool.hxx>

// ─────────────────────────────────────────────────────────────────────────────

namespace {
constexpr int kRoleDocId = lcnc::app::ProjectExplorerRoles::DocId;
constexpr int kRoleEntry = lcnc::app::ProjectExplorerRoles::Entry;
constexpr int kRoleNodeKey = lcnc::app::ProjectExplorerRoles::NodeKey;
constexpr int kRoleLeafEntries = lcnc::app::ProjectExplorerRoles::LeafEntries;
constexpr int kRoleContourIndex = lcnc::app::ProjectExplorerRoles::ContourIndex;
constexpr int kRoleContourId = lcnc::app::ProjectExplorerRoles::ContourId;
constexpr int kRoleLayerId = lcnc::app::ProjectExplorerRoles::LayerId;
using lcnc::app::projectNodeKind;

QString primitiveToolId(int primitiveIndex)
{
    switch (primitiveIndex) {
    case 1: return QStringLiteral("cad.primitive.cylinder");
    case 2: return QStringLiteral("cad.primitive.sphere");
    case 3: return QStringLiteral("cad.primitive.cone");
    case 4: return QStringLiteral("cad.primitive.torus");
    default: return QStringLiteral("cad.primitive.box");
    }
}

QString featureToolId(int featureIndex)
{
    switch (featureIndex) {
    case 1: return QStringLiteral("cad.feature.revolve");
    case 2: return QStringLiteral("cad.feature.sweep");
    default: return QStringLiteral("cad.feature.extrude");
    }
}

QVariantMap primitiveParams(double sizeX, double sizeY, double sizeZ, double radius1, double radius2)
{
    return {{QStringLiteral("sizeX"), sizeX}, {QStringLiteral("sizeY"), sizeY},
            {QStringLiteral("sizeZ"), sizeZ}, {QStringLiteral("radius1"), radius1},
            {QStringLiteral("radius2"), radius2}};
}

QVariantMap featureParams(double length, double angleDeg)
{
    return {{QStringLiteral("length"), length}, {QStringLiteral("angle"), angleDeg}};
}

CadModule::TransformParameters transformParams(double translateX, double translateY,
                                               double translateZ, double rotateX,
                                               double rotateY, double rotateZ,
                                               int referenceMode)
{
    CadModule::TransformParameters params;
    params.translateX = translateX;
    params.translateY = translateY;
    params.translateZ = translateZ;
    params.rotateX = rotateX;
    params.rotateY = rotateY;
    params.rotateZ = rotateZ;
    params.referenceMode = referenceMode;
    return params;
}

constexpr int kRibbonFileIndex = 0;
constexpr int kRibbonViewIndex = 1;
constexpr int kRibbonCadIndex = 2;
constexpr int kRibbonCamIndex = 3;
constexpr int kRibbonLaserIndex = 4;

bool isStepFile(const QString& filePath)
{
    const QString ext = QFileInfo(filePath).suffix().toLower();
    return ext == QStringLiteral("stp") || ext == QStringLiteral("step");
}

}

MainWindow::MainWindow(QWidget* parent)
    : SARibbonMainWindow(parent)
{
    // 中文翻译：LaserCNC — 五轴激光加工CAM软件
    setWindowTitle(tr("LaserCNC — five-axis laser processing CAM software"));
    setWindowIcon(QIcon("themeicons:app_icon.svg"));
    resize(1440, 900);

    // Initialise in dependency order
    createContext();        // AppContext (needs singletons)
    createCentralLayout();  // Central splitter — creates m_occView first
    createCommands();       // Commands connect to m_occView (must exist)
    createRibbon();         // Ribbon uses m_cmdContainer (must exist)
    // SARibbon installs its own stylesheet. Apply the selected native theme
    // after the ribbon hierarchy exists, then restore the shared tab geometry.
    QTimer::singleShot(0, this, [this] {
        const bool lightTheme =
            qApp->property("lcnc.theme").toString() == QStringLiteral("light");
        setRibbonTheme(lightTheme
                           ? SARibbonTheme::RibbonThemeOffice2021Blue
                           : SARibbonTheme::RibbonThemeDark2);
        ribbonBar()->setStyleSheet(lightTheme
            ? "SARibbonTabBar::tab { color:#3D5663; padding:3px 15px 8px; margin:0 2px; }"
              "SARibbonTabBar::tab:selected { color:#FFFFFF; background:#167A9A; }"
            : "SARibbonTabBar::tab { padding:3px 15px 8px; margin:0 2px; }");
    });
    createStatusBar();

        auto* project = lcnc::Kernel::current().projectManager();
        connect(project, &lcnc::LcncProjectManager::projectReset,
            this, &MainWindow::onProjectReset);
        connect(project, &lcnc::LcncProjectManager::domainDataChanged,
            this, &MainWindow::onProjectDomainChanged);
        connect(project, &lcnc::LcncProjectManager::projectOpened,
            this, [this](const QString& filePath) {
                m_skipNextSourceRecent = true;
                addRecentFile(filePath);
                showViewTab();
                showWorkpieceView();
                scheduleRecentThumbnailCapture(filePath);
            });
        connect(project, &lcnc::LcncProjectManager::projectMachineConfigurationMismatch,
            this, [this](const QString& filePath, const QString&, const QString&) {
                // 中文翻译：机台构型不匹配
                QMessageBox::warning(this, tr("Machine configuration does not match"),
                    // 中文翻译：工程“%1”保存时使用的机台构型与当前机台不一致。
                    tr("The machine configuration used when saving project \"%1\" is inconsistent with the current machine."
                       // 中文翻译：可以继续查看或仿真，但真实加工已被禁止；请确认机台、轴映射和安全 IO 后重新保存工程。
                       "You can continue to view or simulate, but real processing has been prohibited; please confirm the machine, axis mapping and safety IO before re-saving the project.")
                        .arg(QFileInfo(filePath).fileName()));
            });
        connect(project, &lcnc::LcncProjectManager::projectSaved,
            this, [this](const QString& filePath) {
                addRecentFile(filePath);
                scheduleRecentThumbnailCapture(filePath);
            });
        connect(project, &lcnc::LcncProjectManager::workspaceAdded,
            this, [this](ProjectWorkspaceId) { refreshDocumentTabs(); });
        connect(project, &lcnc::LcncProjectManager::workspaceClosed,
            this, [this](ProjectWorkspaceId) { refreshDocumentTabs(); });
        connect(project, &lcnc::LcncProjectManager::activeWorkspaceChanged,
            this, [this](ProjectWorkspaceId) {
                refreshDocumentTabs();
                rebuildProjectExplorer();
                if (m_toolpathPanel && m_appContext && m_appContext->camModule()) {
                    auto* cam = m_appContext->camModule();
                    m_toolpathPanel->setToolpath(cam->hasToolpath() ? &cam->toolpathRef() : nullptr);
                    // Cam data belongs to the active project workspace.  Refresh
                    // all mode-dependent controls here as well as through CAM
                    // signals, so a just-opened STEP project cannot retain the
                    // previous workspace's five-axis-looking UI while its CAM
                    // data still has the enum default.
                    // 中文翻译：切换工程时同步加工模式、轴布局和工件安装姿态。
                    m_toolpathPanel->setMachiningModes(
                        cam->supportedMachiningModes(), cam->machiningMode());
                    if (const auto* camData = cam->camData())
                        m_toolpathPanel->setMachineAxisLayout(camData->machineAxisLayout());
                }
                syncMachineTreeVisibilityState();
                updateCommandStates();
            });

    // 世界坐标系渲染器与活动 GuiDocument 生命周期绑定；每个 workspace
    // 拥有自己的 WidgetOccView，切换时只切 QStackedWidget 页面。
    if (auto* guiApp = lcnc::Kernel::current().guiApp()) {
        connect(guiApp, &GuiApplication::guiDocumentReady,
            this, [this](ProjectWorkspaceId id, GuiDocument* gd) {
                WidgetOccView* view = ensureWorkspaceOccView(id);
                if (view && gd)
                    view->attachDocument(gd);
            });
        connect(guiApp, &GuiApplication::guiDocumentAboutToClose,
            this, [this](ProjectWorkspaceId id, GuiDocument* gd) {
                if (!gd)
                    return;
                lcnc::view::WorldAxesRenderer::instance().detach(gd->scene());
                removeWorkspaceOccView(id);
            });
        connect(guiApp, &GuiApplication::activeWorkspaceDocumentChanged,
            this, [this](ProjectWorkspaceId id, GuiDocument* gd) {
                activateWorkspaceOccView(id, gd);
                if (gd && gd->scene()) {
                    LCNC_DEBUG(lcnc::LogCode::Generic,
                               "WorldAxes attach for active workspace");
                    lcnc::view::WorldAxesRenderer::instance().attach(gd->scene());
                }
            });
        if (auto* gd = guiApp->activeGuiDocument(); gd && gd->scene()) {
            activateWorkspaceOccView(lcnc::Kernel::current().projectManager()->activeWorkspaceId(), gd);
            LCNC_DEBUG(lcnc::LogCode::Generic,
                       "WorldAxes auto-attach for workspace");
            lcnc::view::WorldAxesRenderer::instance().attach(gd->scene());
        } else {
            showDefaultOccView();
        }
    }
    refreshDocumentTabs();

    restorePersistedCamState();
    applyPersistedViewState();
    showMachineView();
    refreshStartGuide();
    showStartGuide();

    updateCommandStates();
}

MainWindow::~MainWindow() = default;

// ── Initialization ─────────────────────────────────────────────────────────────
void MainWindow::createContext()
{
    m_appContext = new AppContext(this, this);

    // Ensure GUI application singleton is alive
    lcnc::Kernel::current().guiApp();
    // Task manager
    lcnc::Kernel::current().taskManager();
    lcnc::Kernel::current().projectManager()->ensureProject();
    if (auto* settings = lcnc::Kernel::current().appSettings()) {
        m_viewStateController = std::make_unique<lcnc::app::ViewStateController>(
            *settings,
            [settings] { return settings->saveDefault(); });
    }

    connect(m_appContext->cadModule(), &CadModule::operationFailed,
            this, [this](const QString& title, const QString& message) {
                QMessageBox::critical(this, title, message);
            });

    connect(m_appContext->camModule(), &CamModule::operationFailed,
            this, [this](const QString& title, const QString& message) {
                QMessageBox::critical(this, title, message);
            });

    connect(m_appContext->camModule(), &CamModule::operationWarning,
            this, [this](const QString& title, const QString& message) {
                QMessageBox::warning(this, title, message);
            });

    connect(m_appContext->processModule(), &ProcessModule::statusMessageChanged,
            this, [this](const QString& status) {
                if (m_sbStatus)
                    m_sbStatus->setText(status);
            });
    connect(m_appContext->processModule(), &ProcessModule::connectionChanged,
            this, [this](bool) {
                // 连接/断开都在后台完成；完成后必须立即重算 Ribbon 动作，
                // 否则“连接设备”会保持断开前的禁用状态。
                updateCommandStates();
            });
    connect(m_appContext->processModule(), &ProcessModule::stateChanged,
            this, [this](lcnc::ProcessRunState state) {
                // 预检/运行期错误在后台切换状态；必须立即刷新“停止复位”等
                // Ribbon 命令，不能等待下一次用户交互。
                updateCommandStates();
                if (m_toolpathPanel) {
                    const bool editable = state == lcnc::ProcessRunState::Idle
                        || state == lcnc::ProcessRunState::Stopped
                        || state == lcnc::ProcessRunState::Error;
                    m_toolpathPanel->setMachineSetupEditingEnabled(editable);
                }
            });

    connect(m_appContext->processModule(), &ProcessModule::deviceConnectProgress,
            this, [this](const QString& deviceName, int percent, const QString& step) {
                if (!m_sbDeviceProgress)
                    return;
                m_sbDeviceProgress->setVisible(true);
                m_sbDeviceProgress->setValue(qBound(0, percent, 100));
                m_sbDeviceProgress->setFormat(tr("%1: %2 (%p%)").arg(deviceName, step));
            });
    connect(m_appContext->processModule(), &ProcessModule::deviceConnectFinished,
            this, [this](bool allSuccess, const QString& summary) {
                if (!m_sbDeviceProgress)
                    return;
                m_sbDeviceProgress->setValue(allSuccess ? 100 : 0);
                m_sbDeviceProgress->setVisible(false);
                if (m_sbStatus)
                    m_sbStatus->setText(summary);
            });
}

void MainWindow::createCommands()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "MainWindow::createCommands begin");
    m_cmdContainer = new CommandContainer(m_appContext, this);
    lcnc::app::registerAllCommands(m_cmdContainer, m_appContext, m_occView, this);
    LCNC_DEBUG(lcnc::LogCode::Generic, "MainWindow::createCommands end");
}

void MainWindow::createCentralLayout()
{
    // ── 3D View ────────────────────────────────────────────────────────────
    create3DView();

    m_startGuide = new lcnc::app::StartGuideWidget(this);
    connect(m_startGuide, &lcnc::app::StartGuideWidget::fileActivated,
            this, &MainWindow::openStartGuideFile);

    auto* viewPage = new QWidget(this);
    auto* viewLayout = new QVBoxLayout(viewPage);
    viewLayout->setContentsMargins(0, 0, 0, 0);
    viewLayout->setSpacing(0);
    m_documentTabs = new QTabBar(viewPage);
    m_documentTabs->setDocumentMode(true);
    m_documentTabs->setExpanding(false);
    m_documentTabs->setTabsClosable(true);
    viewLayout->addWidget(m_documentTabs);
    viewLayout->addWidget(m_viewStack, 1);
    connect(m_documentTabs, &QTabBar::currentChanged, this, [this](int index) {
        if (!m_documentTabs || index < 0)
            return;
        const auto id = static_cast<ProjectWorkspaceId>(m_documentTabs->tabData(index).toInt());
        if (id != kInvalidProjectWorkspaceId) {
            lcnc::Kernel::current().projectManager()->setActiveWorkspace(id);
            showViewTab();
            updateCadSketchOverlay();
        }
    });
    connect(m_documentTabs, &QTabBar::tabCloseRequested, this, [this](int index) {
        auto* project = lcnc::Kernel::current().projectManager();
        if (!project)
            return;
        if (!m_documentTabs || index < 0)
            return;
        const auto id = static_cast<ProjectWorkspaceId>(m_documentTabs->tabData(index).toInt());
        if (id != kInvalidProjectWorkspaceId)
            project->closeWorkspace(id);
    });

    m_centerTabs = new QTabWidget(this);
    m_centerTabs->setDocumentMode(true);
    // 中文翻译：开始
    m_centerTabs->addTab(m_startGuide, tr("start"));
    // 中文翻译：视图
    m_centerTabs->addTab(viewPage, tr("view"));

    // ── Left panel ─────────────────────────────────────────────────────────
    createLeftPanel();

    // ── Right panel ────────────────────────────────────────────────────────
    createRightPanel();

    // ── Splitter ───────────────────────────────────────────────────────────
    m_splitter = new QSplitter(Qt::Horizontal, this);
    m_splitter->addWidget(m_leftTabs);
    m_splitter->addWidget(m_centerTabs);
    m_splitter->addWidget(m_rightStack);
    // 两侧 tab/stack 中包含多个页面；QStackedWidget 会把所有页面的最小高度
    // 汇总给 QSplitter。Ribbon 展开时这会把主窗口最小高度推过屏幕可用高度，
    // 导致最大化窗口向下溢出，也使普通窗口无法缩短。
    // 中文翻译：侧栏允许垂直收缩，内容由各自的视图和滚动控件处理。
    for (QWidget* sidePanel : {static_cast<QWidget*>(m_leftTabs),
                               static_cast<QWidget*>(m_rightStack)}) {
        QSizePolicy policy = sidePanel->sizePolicy();
        policy.setVerticalPolicy(QSizePolicy::Ignored);
        sidePanel->setSizePolicy(policy);
        sidePanel->setMinimumHeight(0);
    }
    m_splitter->setMinimumHeight(0);
    m_splitter->setStretchFactor(0, 1);
    m_splitter->setStretchFactor(1, 5);
    m_splitter->setStretchFactor(2, 2);
    m_splitter->setSizes({250, 900, 290});

    setCentralWidget(m_splitter);
    // Keep the restored window resizable even after SARibbon switches between
    // its minimum and normal modes. The centre viewport still retains its own
    // 300px safety minimum.
    // 中文翻译：限制主窗口的最小高度，保证非最大化窗口可调且 Ribbon 恢复时不越过屏幕。
    setMinimumHeight(480);

    // Task dialog (non-modal, floats on top)
    m_taskDialog = new DialogTaskManager(this);

        connect(m_appContext->cadModule(), &CadModule::workpieceStructureChanged,
            this, &MainWindow::rebuildProjectExplorer);
        connect(m_appContext->cadModule(), &CadModule::workpieceViewRequested,
            this, &MainWindow::showWorkpieceView);
        connect(m_appContext->camModule(), &CamModule::machineViewRequested,
            this, &MainWindow::showMachineView);
            connect(m_appContext->camModule(), &CamModule::machineWorkspaceChanged,
                this, &MainWindow::syncMachineWorkspaceUi);
        connect(m_appContext->camModule(), &CamModule::machiningFacesChanged,
            this, &MainWindow::rebuildProjectExplorer);

    connect(m_appContext->camModule(), &CamModule::selectionChanged, this,
            [this](const QStringList& entries) {
                const QStringList sourceEntries = m_appContext->camModule()
                    ->sourceWorkpieceEntriesForMountedEntries(entries);
                if (!sourceEntries.isEmpty())
                    selectProjectExplorerEntries(
                        m_appContext->workpieceDocumentId(), sourceEntries);
                else
                    selectProjectExplorerEntries(kInvalidDocumentId, {});
                m_machinePanel->setSelectedEntries(entries);
            });
    connect(m_appContext->camModule(), &CamModule::toolpathContourSelected, this,
            [this](int contourIndex) {
                const auto contourId = m_appContext->camModule()->contourIdAt(contourIndex);
                m_appContext->camModule()->setActiveContourId(contourId);
                selectProjectExplorerContourById(contourId, contourIndex);
                m_toolpathPanel->showContourCoordinates(contourIndex);
            });
    connect(m_appContext->camModule(), &CamModule::toolpathContoursSelected, this,
            [this](const QList<int>& contourIndexes) {
                selectProjectExplorerContours(contourIndexes);
            });

    connect(m_appContext->cadModule(), &CadModule::selectionChanged, this,
            [this](DocumentId docId, const QStringList& entries) {
                selectProjectExplorerEntries(docId, entries);
                updateCommandStates();
            });

    rebuildProjectExplorer();
    syncMachineWorkspaceUiInternal(true);
}

void MainWindow::create3DView()
{
    m_viewStack = new QStackedWidget(this);
    m_defaultScene = new GraphicsScene(this);
    m_defaultOccView = createOccView(m_viewStack);
    m_defaultOccView->attachDefaultScene(m_defaultScene);
    m_viewStack->addWidget(m_defaultOccView);
    m_workspacePresenter =
        std::make_unique<lcnc::app::WorkspacePresenter>(m_viewStack, m_defaultOccView);
    m_occView = m_defaultOccView;
}

WidgetOccView* MainWindow::createOccView(QWidget* parent)
{
    auto* view = new WidgetOccView(parent);
    connectOccViewSignals(view);
    return view;
}

void MainWindow::connectOccViewSignals(WidgetOccView* view)
{
    if (!view)
        return;

    connect(view, &WidgetOccView::cursorPositionChanged, this,
            [this](double x, double y, double z) {
                if (!m_sbCoords)
                    return;

                const auto* machineConfig =
                    lcnc::Kernel::current().service<lcnc::MachineConfigurationService>();
                const QList<MachineAxisDef> axes = machineConfig
                    ? machineConfig->axisDefinitions()
                    : QList<MachineAxisDef>{};
                const auto axisCoordinate = [&axes, x, y, z](const QString& name,
                                                               double fallback) {
                    for (const MachineAxisDef& axis : axes) {
                        if (axis.motionType == MachineAxisDef::Linear
                            && axis.name.compare(name, Qt::CaseInsensitive) == 0) {
                            // 机台线性轴坐标以其配置的正方向为基准；与刀路
                            // 坐标求解保持同一投影语义。
                            return x * axis.direction.X()
                                + y * axis.direction.Y()
                                + z * axis.direction.Z();
                        }
                    }
                    return fallback;
                };
                m_sbCoords->setText(
                    tr("X: %1  Y: %2  Z: %3")
                        .arg(axisCoordinate(QStringLiteral("X"), x), 0, 'f', 3)
                        .arg(axisCoordinate(QStringLiteral("Y"), y), 0, 'f', 3)
                        .arg(axisCoordinate(QStringLiteral("Z"), z), 0, 'f', 3));
            });

    // ── 3D selection → module coordination ────────────────────────────────
    connect(view, &WidgetOccView::selectionChanged, this, [this] {
        if (isMachineViewActive()) {
            m_appContext->camModule()->syncSelectionFromView();
            return;
        }

        m_appContext->cadModule()->syncSelectionFromView(
            m_appContext->cadModule()->workpieceDocumentId());
    });

    connect(view, &WidgetOccView::sketchOverlayPicked,
            this, &MainWindow::handleCadSketchOverlayPicked);
    connect(view, &WidgetOccView::sketchOverlayDragMoved,
            this, &MainWindow::handleCadSketchOverlayDrag);
    connect(view, &WidgetOccView::sketchOverlayDragCanceled,
            this, &MainWindow::handleCadSketchOverlayDrag);
    connect(view, &WidgetOccView::transformGizmoDragMoved,
            this, [this](int operation, int axis, double delta) {
                if (!m_cadTaskPanel || !m_cadTaskPanel->isTransformPageActive())
                    return;
                m_cadTaskPanel->addTransformDragDelta(operation, axis, delta);
            });

    connect(view, &WidgetOccView::leadInPickMoved, this,
            [this, view](const QPoint& pos) {
                m_appContext->camModule()->updateLeadInPreview(view, pos);
            });
    connect(view, &WidgetOccView::leadInPickConfirmed, this,
            [this, view](const QPoint& pos) {
                if (m_appContext->camModule()->commitLeadInPreview(view, pos))
                    view->endLeadInPick();
            });
    connect(view, &WidgetOccView::leadInPickCanceled, this,
            [this, view]() {
                view->endLeadInPick();
                m_appContext->camModule()->cancelLeadInPreview();
            });

    connect(view, &WidgetOccView::facePickConfirmed, this,
            [this, view](const QPoint& pos) {
                if (m_pendingCalibrationTarget.isEmpty()) {
                    // Not axis-calibration -> manual machining-face selection.
                    QString err;
                    if (m_appContext->camModule()->pickMachiningFace(view, pos, &err)) {
                        // Keep the face picker active: a machining-face set is
                        // deliberately built from multiple operator picks and is
                        // committed only by the explicit Apply action.
                        QToolTip::showText(view->mapToGlobal(pos),
                            // 中文翻译：已加入加工面；可继续选择，右键或 Esc 结束后点击“应用加工面并继续”。
                            tr("The machining surface has been added; you can continue to select, right-click or press Esc and click \"Apply machining surface and continue\"."),
                            view);
                    } else if (!err.isEmpty()) {
                        QToolTip::showText(view->mapToGlobal(pos), err, view);
                    }
                    return;
                }

                bool handled = false;
                // 标定向导是唯一的拾取入口；旧的 A/C/CUTTER_HEAD 直传分支已移除。
                if (m_pendingCalibrationTarget.startsWith(QStringLiteral("WIZARD_"))
                    && m_axisCalibWizard) {
                    using lcnc::cam::ui::DialogAxisCalibrationWizard;
                    gp_Pnt center;
                    QString errMsg;
                    if (m_appContext->camModule()->pickReferenceFaceCenter(
                            view, pos, center, &errMsg)) {
                        DialogAxisCalibrationWizard::Stage stage = DialogAxisCalibrationWizard::Stage::AAxis;
                        if (m_pendingCalibrationTarget == QStringLiteral("WIZARD_C"))
                            stage = DialogAxisCalibrationWizard::Stage::CAxis;
                        else if (m_pendingCalibrationTarget == QStringLiteral("WIZARD_HEAD"))
                            stage = DialogAxisCalibrationWizard::Stage::CutterHead;
                        m_axisCalibWizard->applyPickResult(stage, center);
                        m_axisCalibWizard->raise();
                        m_axisCalibWizard->activateWindow();
                        handled = true;
                    } else {
                        LCNC_WARN(lcnc::LogCode::Generic,
                                  "Wizard face pick failed: {}", errMsg.toStdString());
                        // 拾取失败时保持拾取模式，便于用户重试
                        return;
                    }
                }

                if (!handled)
                    return;

                view->endFacePick();
                m_pendingCalibrationTarget.clear();
                if (m_machinePanel)
                    m_machinePanel->setCalibrationPickAxis(QString());
            });
    connect(view, &WidgetOccView::facePickCanceled, this,
            [this, view]() {
                view->endFacePick();
                if (m_pendingCalibrationTarget.startsWith(QStringLiteral("WIZARD_"))
                    && m_axisCalibWizard) {
                    m_axisCalibWizard->cancelPickInProgress();
                    m_axisCalibWizard->raise();
                    m_axisCalibWizard->activateWindow();
                }
                m_pendingCalibrationTarget.clear();
                if (m_machinePanel)
                    m_machinePanel->setCalibrationPickAxis(QString());
            });
}

WidgetOccView* MainWindow::ensureWorkspaceOccView(ProjectWorkspaceId id)
{
    if (id == kInvalidProjectWorkspaceId || !m_viewStack)
        return nullptr;
    if (m_workspacePresenter) {
        if (auto* existing =
                qobject_cast<WidgetOccView*>(m_workspacePresenter->view(id))) {
            return existing;
        }
    }

    WidgetOccView* view = createOccView(m_viewStack);
    if (m_workspacePresenter)
        m_workspacePresenter->registerView(id, view);
    return view;
}

void MainWindow::activateWorkspaceOccView(ProjectWorkspaceId id, GuiDocument* document)
{
    if (!document || id == kInvalidProjectWorkspaceId) {
        showDefaultOccView();
        return;
    }

    WidgetOccView* view = ensureWorkspaceOccView(id);
    if (!view) {
        showDefaultOccView();
        return;
    }

    m_occView = view;
    if (m_workspacePresenter)
        (void)m_workspacePresenter->activate(id);
    view->attachDocument(document);
}

void MainWindow::removeWorkspaceOccView(ProjectWorkspaceId id)
{
    WidgetOccView* view = m_workspacePresenter
        ? qobject_cast<WidgetOccView*>(m_workspacePresenter->take(id))
        : nullptr;
    if (!view)
        return;

    const bool wasActive = (m_occView == view);
    view->attachDefaultScene(nullptr);
    view->deleteLater();

    if (wasActive)
        showDefaultOccView();
}

void MainWindow::showDefaultOccView()
{
    if (!m_defaultOccView)
        return;
    m_occView = m_defaultOccView;
    if (m_workspacePresenter)
        m_workspacePresenter->showDefault();
    m_defaultOccView->attachDefaultScene(m_defaultScene);
}

void MainWindow::createLeftPanel()
{
    m_leftTabs = new QTabWidget(this);
    m_leftTabs->setDocumentMode(true);
    m_leftTabs->setMinimumWidth(240);
    m_leftTabs->setMaximumWidth(380);

    m_projectExplorerTree = new QTreeWidget(this);
    m_projectExplorerController =
        std::make_unique<lcnc::app::ProjectExplorerController>(m_projectExplorerTree);
    m_projectExplorerTree->setColumnCount(2);
    // 中文翻译：项目；信息
    m_projectExplorerTree->setHeaderLabels({tr("Project"), tr("information")});
    m_projectExplorerTree->header()->setStretchLastSection(false);
    m_projectExplorerTree->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    m_projectExplorerTree->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    m_projectExplorerTree->setAnimated(true);
    m_projectExplorerTree->setSelectionMode(QAbstractItemView::ExtendedSelection);
    // 禁用工程树节点的拖拽（轮廓重排改由显式命令承担，避免误操作）。
    m_projectExplorerTree->setDragDropMode(QAbstractItemView::NoDragDrop);
    m_projectExplorerTree->setContextMenuPolicy(Qt::CustomContextMenu);
    m_projectExplorerTree->setStyleSheet(
        "QTreeWidget::item:selected { background-color: #2A6FDB; color: white; }"
        "QTreeWidget::item:selected:!active { background-color: #2A6FDB; color: white; }");

    connect(m_projectExplorerTree, &QTreeWidget::currentItemChanged,
            this, &MainWindow::onProjectExplorerCurrentItemChanged);
    connect(m_projectExplorerTree, &QTreeWidget::itemChanged,
            this, &MainWindow::onProjectExplorerItemChanged);
        connect(m_projectExplorerTree, &QTreeWidget::itemDoubleClicked,
            this, &MainWindow::onProjectExplorerItemDoubleClicked);
    connect(m_projectExplorerTree, &QTreeWidget::customContextMenuRequested,
            this, &MainWindow::onProjectExplorerContextMenuRequested);
    connect(m_projectExplorerTree, &QTreeWidget::itemSelectionChanged, this,
            [this]() {
                if (m_blockProjectExplorerSignals || !m_projectExplorerTree)
                    return;
                // ── 推送到 SelectionService（轮廓选择顺序记录）─────────────────
                // 只要选中集合里有 Contour 节点，就把新增/移除差分推给服务。
                auto selSvc = lcnc::Kernel::current()
                                  .services()
                                  .getService<lcnc::core::SelectionService>();
                if (selSvc) {
                    QSet<std::uint64_t> nowSelected;
                    QVector<lcnc::core::SelectionEntry> newlyAdded;
                    const auto selectedItems = m_projectExplorerTree->selectedItems();
                    for (QTreeWidgetItem* item : selectedItems) {
                        const auto id = static_cast<std::uint64_t>(
                            item->data(0, kRoleContourId).toULongLong());
                        if (id == 0) continue;
                        nowSelected.insert(id);
                        if (!m_lastExplorerContourSelection.contains(id)) {
                            lcnc::core::SelectionEntry e;
                            e.contourId = id;
                            e.entry     = item->data(0, kRoleEntry).toString();
                            e.source    = lcnc::core::SelectionEntry::ProjectExplorer;
                            newlyAdded.append(e);
                        }
                    }
                    if (!newlyAdded.isEmpty())
                        selSvc->recordSelectedBatch(newlyAdded);
                    // 取消选中差分
                    for (auto id : m_lastExplorerContourSelection) {
                        if (!nowSelected.contains(id))
                            selSvc->removeContour(id);
                    }
                    m_lastExplorerContourSelection = nowSelected;
                }
            });

    auto* processWidget = new ProcessFlowWidget(m_leftTabs);
    m_processLeftPanel = processWidget;
    if (auto workflow = lcnc::Kernel::current()
            .services()
            .getService<lcnc::process::IProcessWorkflowService>()) {
        processWidget->setWorkflowService(workflow.get());
        connect(workflow->notifier(), SIGNAL(flowChanged()),
                processWidget, SLOT(reloadFlowModel()));
    }
    // 机台模型树：独立 tab，展示已加载的机台几何结构（按轴分组）。
    m_machineTree = new WidgetMachineTree(m_leftTabs);
    if (auto* cam = m_appContext->camModule()) {
        m_machineTree->setWorkspace(cam->machineWorkspace());
        // 机台加载/卸载/配置变化后自动刷新
        if (auto* ws = cam->machineWorkspace()) {
            connect(ws, &lcnc::cam::MachineWorkspace::machineModelChanged,
                    m_machineTree, &WidgetMachineTree::rebuild);
        }
        connect(cam, &CamModule::axisAssignmentsChanged,
                m_machineTree, &WidgetMachineTree::rebuild);
        // checkbox 勾选 → 显示/隐藏对应 AIS
        connect(m_machineTree, &WidgetMachineTree::shapeVisibilityChanged,
                cam, &CamModule::setEntityVisible);
        connect(cam, &CamModule::machineVisibilityChanged,
                this, &MainWindow::syncMachineTreeVisibilityState);
        syncMachineTreeVisibilityState();
    }

    // 中文翻译：项目
    m_leftTabs->addTab(m_projectExplorerTree, tr("Project"));
    // 中文翻译：机台
    m_leftTabs->addTab(m_machineTree, tr("machine"));
    // 中文翻译：执行
    m_leftTabs->addTab(m_processLeftPanel, tr("execute"));
    connect(m_leftTabs, &QTabWidget::currentChanged, this, [this](int index) {
        if (index == 1) {
            // 中文翻译：机台
            // "machine" tab：切到机台视图
            m_appContext->camModule()->requestMachineView();
        } else if (index == 2) {
            // 中文翻译：执行
            // "execute" tab：也切到机台视图（仿真监控）
            m_appContext->camModule()->requestMachineView();
        } else if (m_projectExplorerTree && m_projectExplorerTree->currentItem()) {
            onProjectExplorerCurrentItemChanged(m_projectExplorerTree->currentItem(), nullptr);
        } else {
            showWorkpieceView();
        }
        updateCommandStates();
    });
}

void MainWindow::createRightPanel()
{
    m_machinePanel   = new WidgetMachinePanel(this);
    m_cadTaskPanel   = new lcnc::cad::ui::WidgetCadTaskPanel(this);
    m_toolpathPanel  = new WidgetToolpathPanel(this);
    m_laserControl   = new WidgetLaserControl(this);

    CamModule* cam = m_appContext->camModule();
    CadModule* cad = m_appContext->cadModule();
    m_toolpathPanel->setLeadInLength(cam->leadInLength());
    m_toolpathPanel->setDiscretizationInterval(cam->deflection());
    m_toolpathPanel->setSmoothAngle(cam->smoothAngle());
    m_toolpathPanel->setExtractionStrategy(cam->extractionStrategy());
    m_toolpathPanel->setShowNormals(cam->showNormals());
    m_toolpathPanel->setNormalSampleStep(cam->normalSampleStep());
    m_toolpathPanel->setMachiningModes(cam->supportedMachiningModes(), cam->machiningMode());
    if (const auto* camData = cam->camData())
        m_toolpathPanel->setMachineAxisLayout(camData->machineAxisLayout());

    m_rightStack = new QStackedWidget(this);

    // CAM 右栏：机床、刀路参数和机床坐标页面。
    m_camRightTabs = new QTabWidget(this);
    m_camRightTabs->setTabPosition(QTabWidget::North);
    m_camRightTabs->setDocumentMode(true);
    // 中文翻译：机床
    m_camRightTabs->addTab(m_machinePanel,  tr("Machine tools"));
    // 中文翻译：刀路参数
    m_camRightTabs->addTab(m_toolpathPanel, tr("Tool path parameters"));
    // 中文翻译：机床坐标
    m_camRightTabs->addTab(m_toolpathPanel->machineCoordinatesPage(), tr("Machine coordinates"));

    m_rightStack->addWidget(m_camRightTabs);   // index 0 — CAM ribbon page
    m_rightStack->addWidget(m_laserControl);   // index 1 — laser/process ribbon page
    m_rightStack->addWidget(m_cadTaskPanel);   // index 2 — CAD ribbon page
    m_rightStack->setMinimumWidth(320);
    m_rightStack->setMaximumWidth(420);
    m_rightStack->setCurrentIndex(2);
    m_cadTaskPanelController = std::make_unique<lcnc::app::CadTaskPanelController>(
        m_cadTaskPanel,
        cad,
        [this] { return m_occView; },
        [this] { return m_rightStack && m_rightStack->currentWidget() == m_cadTaskPanel; });

    connect(m_cadTaskPanel, &lcnc::cad::ui::WidgetCadTaskPanel::commandRequested,
            this, [this](const QString& commandId) {
        if (m_occView) {
            m_occView->clearCadPreview();
            m_occView->clearTransformGizmo();
        }
        if (auto* command = m_cmdContainer->findCommand(commandId)) {
            if (command->isEnabled())
                command->execute();
        } else {
            LCNC_WARN(lcnc::LogCode::Generic,
                      "CAD TaskPanel requested unknown command: {}",
                      commandId.toStdString());
        }
        updateCommandStates();
        });

    connect(cad, &CadModule::primitiveToolRequested,
            this, [this](int primitiveIndex) {
        showWorkpieceView();
        m_occView->clearTransformGizmo();
        m_cadTaskPanel->showPrimitivePage(primitiveIndex);
        m_cadTaskPanelController->updatePrimitivePreview();
        updateCommandStates();
        });

    connect(m_cadTaskPanel, &lcnc::cad::ui::WidgetCadTaskPanel::sketchCreateRequested,
            this, [this](int planeIndex) {
        if (m_appContext->cadModule()->beginSketch(planeIndex)) {
            // 自动切换到所选工作平面，便于直接绘制。
            if (m_occView) {
                switch (planeIndex) {
                case 1: m_occView->setOrientation(V3d_Xpos); break;  // YZ
                case 2: m_occView->setOrientation(V3d_Ypos); break;  // ZX
                case 0:
                default: m_occView->setOrientation(V3d_Zpos); break; // XY
                }
            }
            updateCommandStates();
        }
        });

    connect(m_cadTaskPanel, &lcnc::cad::ui::WidgetCadTaskPanel::sketchExitRequested,
            this, [this]() {
        if (m_appContext->cadModule()->finishSketch()) {
            m_occView->clearCadPreview();
            m_cadTaskPanel->showHomePage();
            updateCommandStates();
        }
        });

    connect(m_cadTaskPanel, &lcnc::cad::ui::WidgetCadTaskPanel::sketchCanceled,
            this, [this]() {
        m_appContext->cadModule()->cancelModelingOperation();
        m_occView->clearCadPreview();
        updateCommandStates();
        });

    connect(m_cadTaskPanel, &lcnc::cad::ui::WidgetCadTaskPanel::sketchToolChanged,
            this, [this](int toolKind) {
        m_appContext->cadModule()->setSketchTool(toolKind);
        });

    connect(m_cadTaskPanel, &lcnc::cad::ui::WidgetCadTaskPanel::sketchElementAddRequested,
            this, [this](int toolKind, QVector<double> params) {
        QString errMsg;
        if (m_appContext->cadModule()->addSketchElement(toolKind, params, &errMsg) < 0) {
            LCNC_WARN(lcnc::LogCode::Generic,
                      "MainWindow: addSketchElement failed: {}", errMsg.toStdString());
        }
        });

    connect(m_cadTaskPanel, &lcnc::cad::ui::WidgetCadTaskPanel::sketchElementRemoveRequested,
            this, [this](int elementId) {
        m_appContext->cadModule()->removeSketchElement(elementId);
        });

    connect(cad, &CadModule::sketchToolChanged,
            this, [this](int toolKind) {
        if (m_cadTaskPanel)
            m_cadTaskPanel->setActiveSketchTool(toolKind);
        });

    connect(cad, &CadModule::sketchElementsChanged,
            this, [this]() {
        m_cadTaskPanelController->refreshSketchElements();
        rebuildProjectExplorer();
        m_cadTaskPanelController->updateSketchOverlay();
        m_cadTaskPanelController->refreshPanelState();
        });

    connect(cad, &CadModule::finishedSketchesChanged,
            this, [this](DocumentId) {
        m_cadTaskPanelController->refreshFinishedSketches();
        rebuildProjectExplorer();
        m_cadTaskPanelController->updateSketchOverlay();
        m_cadTaskPanelController->refreshPanelState();
        });

    connect(cad, &CadModule::sketchSelectionChanged,
            this, [this](int) {
        m_cadTaskPanelController->refreshFinishedSketches();
        m_cadTaskPanelController->updateSketchOverlay();
        m_cadTaskPanelController->refreshPanelState();
        });

    connect(m_cadTaskPanel, &lcnc::cad::ui::WidgetCadTaskPanel::sketchSelectionChanged,
            this, [this](int sketchId) {
        m_appContext->cadModule()->setSelectedSketchId(sketchId);
        });
    connect(m_cadTaskPanel, &lcnc::cad::ui::WidgetCadTaskPanel::sketchDeleteRequested,
            this, [this](int sketchId) {
        m_appContext->cadModule()->deleteSketch(kInvalidDocumentId, sketchId);
        });
    connect(m_cadTaskPanel, &lcnc::cad::ui::WidgetCadTaskPanel::sketchVisibilityToggled,
            this, [this](int sketchId, bool visible) {
        m_appContext->cadModule()->setSketchVisible(kInvalidDocumentId, sketchId, visible);
        });

    connect(m_cadTaskPanel, &lcnc::cad::ui::WidgetCadTaskPanel::primitiveParametersChanged,
            this, [this](int, double, double, double, double, double) {
        if (m_cadTaskPanel->isPrimitivePageActive())
            m_cadTaskPanelController->updatePrimitivePreview();
        });

    connect(m_cadTaskPanel, &lcnc::cad::ui::WidgetCadTaskPanel::featureParametersChanged,
            this, [this](int, double, double) {
        if (m_cadTaskPanel->isFeaturePageActive())
            m_cadTaskPanelController->updateFeaturePreview();
        });

    connect(m_cadTaskPanel, &lcnc::cad::ui::WidgetCadTaskPanel::transformParametersChanged,
            this, [this](double, double, double, double, double, double, int) {
        if (m_cadTaskPanel->isTransformPageActive())
            m_cadTaskPanelController->updateTransformPreview();
        });

    connect(m_cadTaskPanel, &lcnc::cad::ui::WidgetCadTaskPanel::previewToggled,
            this, [this](bool enabled) {
        if (enabled && m_cadTaskPanel->isPrimitivePageActive())
            m_cadTaskPanelController->updatePrimitivePreview();
        else if (enabled && m_cadTaskPanel->isFeaturePageActive())
            m_cadTaskPanelController->updateFeaturePreview();
        else if (enabled && m_cadTaskPanel->isTransformPageActive())
            m_cadTaskPanelController->updateTransformPreview();
        else
            m_occView->clearCadPreview();
        });

    connect(m_cadTaskPanel, &lcnc::cad::ui::WidgetCadTaskPanel::primitiveApplyRequested,
            this, [this](int primitiveIndex,
                         double sizeX,
                         double sizeY,
                         double sizeZ,
                         double radius1,
                         double radius2) {
        QString errMsg;
        if (m_appContext->cadModule()->executeTool(
            primitiveToolId(primitiveIndex),
            primitiveParams(sizeX, sizeY, sizeZ, radius1, radius2),
            &errMsg)) {
            m_occView->clearCadPreview();
            m_cadTaskPanel->showHomePage();
            updateCommandStates();
        }
        });

    connect(m_cadTaskPanel, &lcnc::cad::ui::WidgetCadTaskPanel::primitiveCanceled,
            this, [this]() {
        m_occView->clearCadPreview();
        updateCommandStates();
        });

    connect(m_cadTaskPanel, &lcnc::cad::ui::WidgetCadTaskPanel::featureApplyRequested,
            this, [this](int featureIndex, double length, double angleDeg) {
        QString errMsg;
        if (m_appContext->cadModule()->executeTool(
            featureToolId(featureIndex), featureParams(length, angleDeg), &errMsg)) {
            m_occView->clearCadPreview();
            m_cadTaskPanel->showHomePage();
            updateCommandStates();
        }
        });

    connect(m_cadTaskPanel, &lcnc::cad::ui::WidgetCadTaskPanel::featureCanceled,
            this, [this]() {
        m_occView->clearCadPreview();
        updateCommandStates();
        });

    connect(m_cadTaskPanel, &lcnc::cad::ui::WidgetCadTaskPanel::transformApplyRequested,
            this, [this](double translateX,
                         double translateY,
                         double translateZ,
                         double rotateX,
                         double rotateY,
                         double rotateZ,
                         int referenceMode) {
        QString errMsg;
        if (m_appContext->cadModule()->applyTransform(
                transformParams(translateX, translateY, translateZ,
                                rotateX, rotateY, rotateZ, referenceMode),
                &errMsg)) {
            m_occView->clearCadPreview();
            m_occView->clearTransformGizmo();
            {
                QSignalBlocker blocker(m_cadTaskPanel);
                m_cadTaskPanel->showHomePage();
            }
            updateCommandStates();
        } else if (!errMsg.isEmpty()) {
            LCNC_WARN(lcnc::LogCode::Generic,
                      "MainWindow: applyTransform failed: {}",
                      errMsg.toStdString());
        }
        });

    connect(m_cadTaskPanel, &lcnc::cad::ui::WidgetCadTaskPanel::transformCanceled,
            this, [this]() {
        m_occView->clearCadPreview();
        m_occView->clearTransformGizmo();
        updateCommandStates();
        });

    m_cadTaskPanelController->refreshPanelState();

    // Wire machine panel signals.
    connect(m_machinePanel, &WidgetMachinePanel::autoInstallWorkpieceChanged, this,
            [this](bool enabled) {
                m_appContext->camModule()->setAutoInstallWorkpiece(enabled);
            });
    // 注：旧的"轴原点 / AC 中心 / 切割头"独立设置 UI 已被移除（向导唯一入口），
    // 这里也相应移除了对应的信号-槽接线。仅保留向导入口。
    connect(m_machinePanel, &WidgetMachinePanel::axisCalibrationWizardRequested, this,
            [this]() {
                using lcnc::cam::ui::DialogAxisCalibrationWizard;
                m_appContext->camModule()->requestMachineView();
                if (!m_axisCalibWizard) {
                    m_axisCalibWizard = new DialogAxisCalibrationWizard(
                        m_appContext->camModule(), this);
                    connect(m_axisCalibWizard, &DialogAxisCalibrationWizard::pickRequested,
                            this, [this](DialogAxisCalibrationWizard::Stage stage) {
                                if (m_occView->isFacePickActive()) {
                                    m_occView->endFacePick();
                                }
                                switch (stage) {
                                case DialogAxisCalibrationWizard::Stage::AAxis:
                                    m_pendingCalibrationTarget = QStringLiteral("WIZARD_A");
                                    break;
                                case DialogAxisCalibrationWizard::Stage::CAxis:
                                    m_pendingCalibrationTarget = QStringLiteral("WIZARD_C");
                                    break;
                                case DialogAxisCalibrationWizard::Stage::CutterHead:
                                    m_pendingCalibrationTarget = QStringLiteral("WIZARD_HEAD");
                                    break;
                                }
                                m_appContext->camModule()->requestMachineView();
                                m_occView->beginFacePick();
                            });
                    // 关闭/接受时清理待拾取状态
                    connect(m_axisCalibWizard, &QDialog::finished, this, [this](int) {
                        if (m_pendingCalibrationTarget.startsWith(QStringLiteral("WIZARD_"))) {
                            if (m_occView->isFacePickActive())
                                m_occView->endFacePick();
                            m_pendingCalibrationTarget.clear();
                        }
                    });
                }
                m_axisCalibWizard->show();
                m_axisCalibWizard->raise();
                m_axisCalibWizard->activateWindow();
            });
        connect(m_machinePanel, &WidgetMachinePanel::workpieceSetupChanged, this,
            [this](double x, double y, double z, double rx, double ry, double rz) {
                lcnc::WorkpieceSetupTransform setup;
                setup.x = x;
                setup.y = y;
                setup.z = z;
                setup.rotationXDeg = rx;
                setup.rotationYDeg = ry;
                setup.rotationZDeg = rz;
                m_appContext->camModule()->setWorkpieceSetupTransform(setup);
            });
        connect(m_machinePanel, &WidgetMachinePanel::alignWorkpieceSetupToRotationCenterRequested, this,
            [this]() {
            if (m_appContext->camModule()->alignWorkpieceSetupToRotationCenter())
                m_machinePanel->setDocument(m_appContext->camModule()->machineDocument());
            });

        connect(m_appContext->camModule(), &CamModule::toolpathGenerated, this,
            [this]() {
            m_occView->endLeadInPick();
            m_appContext->camModule()->cancelLeadInPreview();
            m_toolpathPanel->setToolpath(&m_appContext->camModule()->toolpathRef());
            rebuildProjectExplorer();
            const auto activeId = m_appContext->camModule()->activeContourId();
            selectProjectExplorerContourById(activeId != 0
                ? activeId : m_appContext->camModule()->contourIdAt(0), 0);
            });
        connect(m_appContext->camModule(), &CamModule::activeToolpathContourChanged, this,
            [this](std::uint64_t contourId, int contourIndex) {
                m_toolpathPanel->setActiveContour(contourIndex);
                if (contourId != 0)
                    selectProjectExplorerContourById(contourId, contourIndex);
                updateCommandStates();
            });
        connect(m_appContext->camModule(), &CamModule::activeContourParametersChanged, this,
            [this]() {
                const int index = m_appContext->camModule()->activeContourIndex();
                m_toolpathPanel->showContourCoordinates(index);
            });
        connect(m_appContext->camModule(), &CamModule::toolpathCleared, this,
            [this]() {
            m_occView->endLeadInPick();
            m_appContext->camModule()->cancelLeadInPreview();
            m_toolpathPanel->setToolpath(nullptr);
            rebuildProjectExplorer();
            });

    // ── Toolpath panel signals ──────────────────────────────────────────
    connect(m_toolpathPanel, &WidgetToolpathPanel::generateRequested, this,
            [this]{ m_cmdContainer->findCommand(CmdGenerateToolpath::Name)->execute(); });
    connect(m_toolpathPanel, &WidgetToolpathPanel::separateFacesRequested, this,
            [this]{ m_appContext->camModule()->separateMachiningFacesAsync(); });
    connect(m_toolpathPanel, &WidgetToolpathPanel::pickMachiningFacesRequested, this,
            [this]{ m_cmdContainer->findCommand(CmdSelectMachiningFace::Name)->execute(); });
    connect(m_toolpathPanel, &WidgetToolpathPanel::applyMachiningFacesRequested, this,
            [this]{ m_appContext->camModule()->applyMachiningFaces(); });
    connect(m_toolpathPanel, &WidgetToolpathPanel::extractContoursRequested, this,
            [this]{ m_appContext->camModule()->extractContoursFromMachiningFacesAsync(); });
    connect(m_toolpathPanel, &WidgetToolpathPanel::discretizePointsRequested, this,
            [this]{ m_appContext->camModule()->discretizeCurrentContoursAsync(); });
    connect(m_toolpathPanel, &WidgetToolpathPanel::buildToolpathRequested, this,
            [this]{ m_appContext->camModule()->buildCurrentGeometricToolpathAsync(); });
    connect(m_toolpathPanel, &WidgetToolpathPanel::solveMachinePathRequested, this,
            [this]{ m_appContext->camModule()->solveCurrentGeometricToolpathAsync(); });
    connect(m_toolpathPanel, &WidgetToolpathPanel::machiningModeChanged, this,
            [this](lcnc::MachiningMode mode) {
                CamModule* cam = m_appContext->camModule();
                ProcessModule* process = m_appContext->processModule();
                if (process && process->state() != ProcessModule::State::Idle
                    && process->state() != ProcessModule::State::Stopped
                    && process->state() != ProcessModule::State::Error) {
                    m_toolpathPanel->setMachiningModes(cam->supportedMachiningModes(), cam->machiningMode());
                    return;
                }
                if (cam->setMachiningMode(mode) && cam->camData())
                    m_toolpathPanel->setMachineAxisLayout(cam->camData()->machineAxisLayout());
            });
    connect(cam, &CamModule::machiningModeChanged, this,
            [this, cam](lcnc::MachiningMode) {
                m_toolpathPanel->setMachiningModes(cam->supportedMachiningModes(), cam->machiningMode());
                if (cam->camData()) m_toolpathPanel->setMachineAxisLayout(cam->camData()->machineAxisLayout());
                if (m_laserControl) {
                    if (auto* machineConfig = lcnc::Kernel::current().service<lcnc::MachineConfigurationService>()) {
                        const auto definition = machineConfig->modeDefinition(cam->machiningMode());
                        m_laserControl->setTaskAxisStates(
                            definition.interpolatedAxes, definition.lockedAxisTargets);
                    }
                }
            });
    connect(m_toolpathPanel, &WidgetToolpathPanel::leadInLengthChanged, this,
            [this](double v) {
            if (m_toolpathPanel->parameterScope() == WidgetToolpathPanel::ParameterScope::CurrentContour)
                m_appContext->camModule()->setActiveContourLeadInLength(v);
            else
                m_appContext->camModule()->setLeadInLength(v);
            });
    connect(m_toolpathPanel, &WidgetToolpathPanel::discretizationIntervalChanged, this,
            [this](double v) {
            if (m_toolpathPanel->parameterScope() == WidgetToolpathPanel::ParameterScope::CurrentContour)
                m_appContext->camModule()->setActiveContourDeflection(v);
            else
                m_appContext->camModule()->setDeflection(v);
            });
    connect(m_toolpathPanel, &WidgetToolpathPanel::parameterScopeChanged, this,
            [this](bool currentContour) {
            CamModule* cam = m_appContext->camModule();
            if (currentContour) {
                m_toolpathPanel->refreshParameterEditors();
            } else {
                m_toolpathPanel->setLeadInLength(cam->leadInLength());
                m_toolpathPanel->setDiscretizationInterval(cam->deflection());
            }
            });
    connect(m_toolpathPanel, &WidgetToolpathPanel::smoothAngleChanged, this,
            [this](double v) { m_appContext->camModule()->setSmoothAngle(v); });

        connect(m_toolpathPanel, &WidgetToolpathPanel::extractionStrategyChanged, this,
            [this](int strategy) { m_appContext->camModule()->setExtractionStrategy(strategy); });

        // 法线显示参数信号
        connect(m_toolpathPanel, &WidgetToolpathPanel::showNormalsToggled, this,
            [this](bool on) { m_appContext->camModule()->setShowNormals(on); });
        connect(m_toolpathPanel, &WidgetToolpathPanel::normalSampleStepChanged, this,
            [this](double step) { m_appContext->camModule()->setNormalSampleStep(step); });

        // ── Process module ↔ execution page ─────────────────────────────────
        ProcessModule* process = m_appContext->processModule();
        connect(m_laserControl, &WidgetLaserControl::startRequested,
            process, &ProcessModule::runStart);
        connect(m_laserControl, &WidgetLaserControl::pauseRequested,
            process, &ProcessModule::runPause);
        connect(m_laserControl, &WidgetLaserControl::resumeRequested,
            process, &ProcessModule::runStart);
        connect(m_laserControl, &WidgetLaserControl::stopRequested,
            process, &ProcessModule::runStop);
        connect(m_laserControl, &WidgetLaserControl::jogRequested, this,
            [process](const QString& axis, int direction, int speedLevel, double distance) {
            process->jog(axis, direction, speedLevel, distance);
            });
        connect(m_laserControl, &WidgetLaserControl::absoluteMoveRequested, this,
            [process](const QString& axis, double position, int speedLevel) {
            process->moveAxisAbsolute(axis, position, speedLevel);
            });
        connect(m_laserControl, &WidgetLaserControl::continuousJogStarted, this,
            [process](const QString& axis, int direction, int speedLevel) {
            process->startContinuousJog(axis, direction, speedLevel);
            });
        connect(m_laserControl, &WidgetLaserControl::continuousJogStopped, this,
            [process](const QString& axis) {
            process->stopContinuousJog(axis);
            });
        connect(m_laserControl, &WidgetLaserControl::axisEnableToggled,
            process, &ProcessModule::setAxisEnabled);
        connect(m_laserControl, &WidgetLaserControl::digitalOutputToggled,
            process, &ProcessModule::setDigitalOutput);

        connect(process, &ProcessModule::connectionChanged,
            m_laserControl, &WidgetLaserControl::updateConnectionStatus);
        connect(process, &ProcessModule::stateChanged,
            m_laserControl, &WidgetLaserControl::updateRunState);
        connect(process, &ProcessModule::processingRunStarted,
            m_laserControl, &WidgetLaserControl::beginProcessingRun);
        connect(process, &ProcessModule::processingProgressChanged,
            m_laserControl, &WidgetLaserControl::updateProcessingProgress);
        connect(process, &ProcessModule::simulationModeChanged,
            m_laserControl, &WidgetLaserControl::updateSimulationMode);
        connect(process, &ProcessModule::statusMessageChanged,
            m_laserControl, &WidgetLaserControl::updateSystemStatus);
        connect(process, &ProcessModule::processLogMessage,
            m_laserControl, &WidgetLaserControl::appendLogMessage);
        connect(process, &ProcessModule::axisEnabledChanged,
            m_laserControl, &WidgetLaserControl::updateAxisEnabled);
        connect(process, &ProcessModule::digitalOutputChanged,
            m_laserControl, &WidgetLaserControl::updateDigitalOutput);
        connect(process, &ProcessModule::digitalOutputDescriptorsChanged,
            m_laserControl, &WidgetLaserControl::setDigitalOutputDescriptors);
        // 同步当前已知的描述符（init 期间已 emit 一次，但此 connect 可能晚于
        // 那次 emit —— 这里补一次推送，保证视图初始化）。
        m_laserControl->setDigitalOutputDescriptors(process->mainPanelDigitalOutputs());
        connect(process, &ProcessModule::axisPositionChanged, this,
            [this](const QString& axis, double value) {
            m_laserControl->updateAxisPosition(axis, value);
            m_appContext->camModule()->setAxisPosition(axis, value, false);
            });

        if (auto* machineConfig = lcnc::Kernel::current().service<lcnc::MachineConfigurationService>()) {
            connect(machineConfig, &lcnc::MachineConfigurationService::machineConfigurationChanged,
                    this, [this, process, machineConfig] {
                        const QList<MachineAxisDef> axes = machineConfig->axisDefinitions();
                        process->setAxisDefinitions(axes);
                        m_laserControl->setAxisDefinitions(axes);
                        const auto definition = machineConfig->modeDefinition(
                            m_appContext->camModule()->machiningMode());
                        m_laserControl->setTaskAxisStates(
                            definition.interpolatedAxes, definition.lockedAxisTargets);
                        if (auto* guiApp = lcnc::Kernel::current().guiApp())
                            guiApp->setMachineCoordinateFrame(axes);
                        lcnc::view::WorldAxesRenderer::instance().setMachineAxisDirections(axes);
                        const auto enabledStates = process->axisEnabledStates();
                        for (auto it = enabledStates.cbegin(); it != enabledStates.cend(); ++it)
                            m_laserControl->updateAxisEnabled(it.key(), it.value());
                    });
            const QList<MachineAxisDef> axes = machineConfig->axisDefinitions();
            m_laserControl->setAxisDefinitions(axes);
            const auto definition = machineConfig->modeDefinition(
                m_appContext->camModule()->machiningMode());
            m_laserControl->setTaskAxisStates(
                definition.interpolatedAxes, definition.lockedAxisTargets);
            if (auto* guiApp = lcnc::Kernel::current().guiApp())
                guiApp->setMachineCoordinateFrame(axes);
            lcnc::view::WorldAxesRenderer::instance().setMachineAxisDirections(axes);
        }

        m_laserControl->updateConnectionStatus(process->isConnected());
        m_laserControl->updateRunState(process->state());
        m_laserControl->updateSimulationMode(process->simulationMode());
        m_laserControl->updateSystemStatus(process->statusMessage());
        const auto axisPositions = process->currentAxisPositions();
        for (auto it = axisPositions.cbegin(); it != axisPositions.cend(); ++it)
        m_laserControl->updateAxisPosition(it.key(), it.value());
        const auto axisEnabledStates = process->axisEnabledStates();
        for (auto it = axisEnabledStates.cbegin(); it != axisEnabledStates.cend(); ++it)
            m_laserControl->updateAxisEnabled(it.key(), it.value());
        const auto outputStates = process->digitalOutputStates();
        for (auto it = outputStates.cbegin(); it != outputStates.cend(); ++it)
            m_laserControl->updateDigitalOutput(it.key(), QString(), it.value());
}

void MainWindow::updateCadPrimitivePreview()
{
    if (m_cadTaskPanelController)
        m_cadTaskPanelController->updatePrimitivePreview();
}

void MainWindow::updateCadFeaturePreview()
{
    if (m_cadTaskPanelController)
        m_cadTaskPanelController->updateFeaturePreview();
}

void MainWindow::updateCadTransformPreview()
{
    if (m_cadTaskPanelController)
        m_cadTaskPanelController->updateTransformPreview();
}

void MainWindow::updateCadTaskPanelState()
{
    if (m_cadTaskPanelController)
        m_cadTaskPanelController->refreshPanelState();
}

void MainWindow::refreshSketchElementsView()
{
    if (m_cadTaskPanelController)
        m_cadTaskPanelController->refreshSketchElements();
}

void MainWindow::refreshFinishedSketchesView()
{
    if (m_cadTaskPanelController)
        m_cadTaskPanelController->refreshFinishedSketches();
}

void MainWindow::updateCadSketchOverlay()
{
    if (m_cadTaskPanelController)
        m_cadTaskPanelController->updateSketchOverlay();
}

void MainWindow::handleCadSketchOverlayPicked(const QString& key)
{
    if (m_cadTaskPanelController)
        m_cadTaskPanelController->handleSketchOverlayPicked(key);
}

void MainWindow::handleCadSketchOverlayDrag(const QString& key, double deltaX, double deltaY)
{
    if (m_cadTaskPanelController)
        m_cadTaskPanelController->handleSketchOverlayDrag(key, deltaX, deltaY);
}

// ── Ribbon ─────────────────────────────────────────────────────────────────────
void MainWindow::createRibbon()
{
    SARibbonBar* ribbon = ribbonBar();
    ribbon->setRibbonStyle(SARibbonBar::RibbonStyleLooseThreeRow);
    // The application menu is unused: remove its blank launcher rather than
    // reserving a dead button at the far left of the tab strip.
    ribbon->setApplicationButton(nullptr);

    // 中文翻译：文件
    buildFileTab(ribbon->addCategoryPage(tr("File")));
    // 中文翻译：视图
    buildViewTab(ribbon->addCategoryPage(tr("view")));
    buildCadTab(ribbon->addCategoryPage(tr("CAD")));
    buildCamTab(ribbon->addCategoryPage(tr("CAM")));
    // 中文翻译：激光加工
    buildLaserTab(ribbon->addCategoryPage(tr("Laser processing")));

    connect(ribbon, &SARibbonBar::currentRibbonTabChanged,
            this, &MainWindow::syncRightPanelForRibbonIndex);
    syncRightPanelForRibbonIndex(ribbon->currentIndex());
}

void MainWindow::buildFileTab(SARibbonCategory* cat)
{
    // 中文翻译：文档
    SARibbonPanel* panelDoc = cat->addPanel(tr("Documentation"));
    panelDoc->addLargeAction(m_cmdContainer->findAction(CmdNewDocument::Name));
    panelDoc->addLargeAction(m_cmdContainer->findAction(CmdOpenDocument::Name));
    panelDoc->addLargeAction(m_cmdContainer->findAction(CmdSaveDocument::Name));

    // 中文翻译：导入/导出
    SARibbonPanel* panelIO = cat->addPanel(tr("Import/Export"));
    panelIO->addLargeAction(m_cmdContainer->findAction(CmdImportStep::Name));
    panelIO->addLargeAction(m_cmdContainer->findAction(CmdImportStl::Name));
    panelIO->addLargeAction(m_cmdContainer->findAction(CmdExportStep::Name));
    panelIO->addLargeAction(m_cmdContainer->findAction(CmdCloseDocument::Name));

    // ── 应用 — 选项按钮 ─────────────────────────────────────────────────
    // 中文翻译：应用
    SARibbonPanel* panelApp = cat->addPanel(tr("Application"));
    panelApp->addLargeAction(m_cmdContainer->findAction(CmdShowOptions::Name));
}

void MainWindow::buildViewTab(SARibbonCategory* cat)
{
    // 中文翻译：视图
    SARibbonPanel* panelView = cat->addPanel(tr("view"));
    panelView->addLargeAction(m_cmdContainer->findAction(CmdFitAll::Name));

    // View orientation quick actions
    struct OrientInfo { QString label; QString key; QString iconPath; V3d_TypeOfOrientation orient; };
    const QList<OrientInfo> orients = {
        // 中文翻译：正视
        { tr("Front"),   "1", QStringLiteral("themeicons:view_front.svg"), V3d_Xpos              },
        // 中文翻译：俯视
        { tr("Top"),   "2", QStringLiteral("themeicons:view_top.svg"),   V3d_Zpos              },
        // 中文翻译：侧视
        { tr("Side"),   "3", QStringLiteral("themeicons:view_side.svg"),  V3d_Ypos              },
        // 中文翻译：等轴测
        { tr("Isometric"), "0", QStringLiteral("themeicons:view_iso.svg"),   V3d_XposYnegZpos      },
    };
    for (auto& info : orients) {
        auto* act = new QAction(QIcon(info.iconPath), info.label + " [" + info.key + "]", this);
        connect(act, &QAction::triggered, this,
                [this, o = info.orient] {
                    if (auto* view = occView())
                        view->setOrientation(o);
                });
        panelView->addLargeAction(act);
    }

    // 抓取是视图拾取过滤器，而不是 CAD 建模命令：放在视图页，且作用于当前工作区。
    // 中文翻译：抓取
    SARibbonPanel* panelSnap = cat->addPanel(tr("crawl"));
    auto* snapGroup = new QActionGroup(this);
    snapGroup->setExclusive(true);
    QAction* snapNone = m_cmdContainer->findAction(CmdSnapNone::Name);
    QAction* snapVertex = m_cmdContainer->findAction(CmdSnapVertex::Name);
    QAction* snapEdge = m_cmdContainer->findAction(CmdSnapEdge::Name);
    QAction* snapFace = m_cmdContainer->findAction(CmdSnapFace::Name);
    snapGroup->addAction(snapNone);
    snapGroup->addAction(snapVertex);
    snapGroup->addAction(snapEdge);
    snapGroup->addAction(snapFace);
    auto* menuSnap = new QMenu(tr("crawl"), cat);
    menuSnap->setIcon(QIcon("themeicons:snap.svg"));
    menuSnap->addAction(snapNone);
    menuSnap->addAction(snapVertex);
    menuSnap->addAction(snapEdge);
    menuSnap->addAction(snapFace);
    panelSnap->addLargeMenu(menuSnap);

    // 中文翻译：显示
    SARibbonPanel* panelDisplay = cat->addPanel(tr("show"));
    panelDisplay->addLargeAction(m_cmdContainer->findAction(CmdToggleShaded::Name));
    panelDisplay->addLargeAction(m_cmdContainer->findAction(CmdToggleWireframe::Name));
    panelDisplay->addLargeAction(m_cmdContainer->findAction(CmdToggleShadedWithEdges::Name));
    panelDisplay->addLargeAction(m_cmdContainer->findAction(CmdToggleWorldAxes::Name));

    QAction* aWire = m_cmdContainer->findAction(CmdToggleWireframe::Name);
    QAction* aShade = m_cmdContainer->findAction(CmdToggleShaded::Name);
    QAction* aEdges = m_cmdContainer->findAction(CmdToggleShadedWithEdges::Name);
    if (aWire) {
        connect(aWire, &QAction::triggered, this, [this] {
            persistViewDisplayMode(AIS_WireFrame, false);
        });
    }
    if (aShade) {
        connect(aShade, &QAction::triggered, this, [this] {
            persistViewDisplayMode(AIS_Shaded, false);
        });
    }
    if (aEdges) {
        connect(aEdges, &QAction::triggered, this, [this] {
            persistViewDisplayMode(AIS_Shaded, true);
        });
    }
    if (QAction* worldAxes = m_cmdContainer->findAction(CmdToggleWorldAxes::Name)) {
        connect(worldAxes, &QAction::triggered, this, [this] {
            persistViewToggleState();
        });
    }

    // 中文翻译：机台显示
    SARibbonPanel* panelMachineView = cat->addPanel(tr("Machine display"));
    // 中文翻译：旋转轴线
    m_actRotaryAxisGuides = new QAction(QIcon("themeicons:machine.svg"), tr("axis of rotation"), this);
    m_actRotaryAxisGuides->setCheckable(true);
    // 中文翻译：显示/隐藏机台 A/C 旋转轴辅助线
    m_actRotaryAxisGuides->setStatusTip(tr("Show/hide machine A/C rotation axis auxiliary line"));
    panelMachineView->addLargeAction(m_actRotaryAxisGuides);

    // 中文翻译：模拟刀头
    m_actCutterHeadGuide = new QAction(QIcon("themeicons:machine.svg"), tr("Simulated cutter head"), this);
    m_actCutterHeadGuide->setCheckable(true);
    // 中文翻译：显示/隐藏模拟刀头辅助线和锥形指示
    m_actCutterHeadGuide->setStatusTip(tr("Show/hide simulated tool head guide lines and taper indicators"));
    panelMachineView->addLargeAction(m_actCutterHeadGuide);

    // 中文翻译：机台模型
    m_actMachineModelVisible = new QAction(QIcon("themeicons:machine.svg"), tr("Machine model"), this);
    m_actMachineModelVisible->setCheckable(true);
    // 中文翻译：显示/隐藏机台模型；开启后可在机台节点树中局部显示轴系
    m_actMachineModelVisible->setStatusTip(tr("Show/hide the machine model; after turning it on, the axis system can be partially displayed in the machine node tree"));
    panelMachineView->addLargeAction(m_actMachineModelVisible);

    connect(m_actRotaryAxisGuides, &QAction::toggled, this, [this](bool checked) {
        if (auto* cam = m_appContext ? m_appContext->camModule() : nullptr)
            cam->setRotaryAxisGuidesVisible(checked);
        persistViewToggleState();
    });
    connect(m_actCutterHeadGuide, &QAction::toggled, this, [this](bool checked) {
        if (auto* cam = m_appContext ? m_appContext->camModule() : nullptr)
            cam->setCutterHeadGuideVisible(checked);
        persistViewToggleState();
    });
    connect(m_actMachineModelVisible, &QAction::toggled, this, [this](bool checked) {
        if (auto* cam = m_appContext ? m_appContext->camModule() : nullptr)
            cam->setMachineModelVisible(checked);
        syncMachineTreeVisibilityState();
        persistViewToggleState();
    });
}

void MainWindow::buildCadTab(SARibbonCategory* cat)
{
    lcnc::cad::buildRibbonTab(cat, m_cmdContainer, this);
}

void MainWindow::buildCamTab(SARibbonCategory* cat)
{
    lcnc::cam::buildRibbonTab(cat, m_cmdContainer, this);
}

void MainWindow::buildLaserTab(SARibbonCategory* cat)
{
    lcnc::process::buildRibbonTab(cat, m_cmdContainer, this);
}
// ── Status bar ────────────────────────────────────────────────────────────────
void MainWindow::createStatusBar()
{
    // 中文翻译：无文档
    m_sbDocName = new QLabel(tr("No documentation"), this);
    m_sbCoords  = new QLabel("X: 0.000  Y: 0.000  Z: 0.000", this);
    const QString statusText = m_appContext
        ? m_appContext->processModule()->statusMessage()
        // 中文翻译：就绪
        : tr("ready");
    m_sbStatus  = new QLabel(statusText, this);
    m_sbDeviceProgress = new QProgressBar(this);

    m_sbDocName->setMinimumWidth(200);
    m_sbCoords->setMinimumWidth(280);
    m_sbDeviceProgress->setRange(0, 100);
    // 连接阶段格式为“设备名: 阶段说明 (百分比)”，260px 会截断控制器
    // 连接等常见阶段文字。槽位必须从启动起保留该宽度，不能在连接时扩张。
    // 中文翻译：预留足够宽度完整显示连接进度文字。
    m_sbDeviceProgress->setMinimumWidth(420);
    m_sbDeviceProgress->setTextVisible(true);
    m_sbDeviceProgress->setVisible(false);

    // QStatusBar ignores a hidden direct child when calculating its height.
    // Showing the connection progress bar would therefore recalculate the
    // status bar and resize the central viewport. Reserve its geometry in a
    // permanent slot so connection progress only changes painted content.
    // 中文翻译：为连接进度条预留固定槽位，避免显示/隐藏时改变状态栏和视窗尺寸。
    auto* progressSlot = new QWidget(statusBar());
    auto* progressSlotLayout = new QHBoxLayout(progressSlot);
    progressSlotLayout->setContentsMargins(0, 0, 0, 0);
    progressSlotLayout->addWidget(m_sbDeviceProgress);
    QSize progressSlotSize = m_sbDeviceProgress->sizeHint();
    progressSlotSize.setWidth(qMax(progressSlotSize.width(), m_sbDeviceProgress->minimumWidth()));
    progressSlot->setFixedSize(progressSlotSize);

    statusBar()->addWidget(m_sbDocName);
    statusBar()->addPermanentWidget(m_sbCoords);
    statusBar()->addPermanentWidget(progressSlot);
    statusBar()->addPermanentWidget(m_sbStatus);
}

// ── Slots ──────────────────────────────────────────────────────────────────────
void MainWindow::onProjectReset()
{
    // Display resources are owned by each workspace GuiDocument/WidgetOccView.
    // A reset only means no active project remains; workspace switching uses
    // activeWorkspaceChanged and must not clear/rebuild the active view.
    // 清空跨模块的轮廓选择顺序（属当前工程的瞬态状态）。
    if (auto svc = lcnc::Kernel::current().services().getService<lcnc::core::SelectionService>())
        svc->clear();
    if (m_appContext && m_appContext->cadModule()) {
        if (LcncDocument* doc = m_appContext->cadModule()->workpieceDocument())
            m_sbDocName->setText(doc->name());
        showWorkpieceView(m_appContext->cadModule()->workpieceDocumentId());
    }
    updateCommandStates();
    updateCadSketchOverlay();
}

void MainWindow::onProjectDomainChanged(lcnc::ProjectDomain domain)
{
    if (domain == lcnc::ProjectDomain::Workpiece && m_appContext && m_appContext->camModule()) {
        if (LcncDocument* doc = m_appContext->cadModule()->workpieceDocument())
            m_sbDocName->setText(doc->name());
        m_appContext->camModule()->autoInstallCurrentWorkpiece();
        if (auto* gd = m_appContext->camModule()->activeGuiDocument()) {
            auto* project = lcnc::Kernel::current().projectManager();
            activateWorkspaceOccView(project ? project->activeWorkspaceId() : kInvalidProjectWorkspaceId, gd);
            // autoInstallCurrentWorkpiece mounts the workpiece and refreshes
            // transforms, but that refresh runs before the OCC view is attached
            // to this document, so the workpiece AIS can be left at the home
            // pose until the next axis-motion tick. Re-apply the current
            // kinematic posture now that the view is live, mirroring the
            // motion-tick path (updateMachineWorkspaceTransforms) so the
            // workpiece - and any machining-face highlights - coincide with
            // the model immediately after opening a file.
            m_appContext->camModule()->refreshMachineTransforms();
        }

        const QString sourcePath = lcnc::Kernel::current()
            .projectManager()
            ->session()
            .workpiece()
            .sourceFilePath;
        if (m_skipNextSourceRecent) {
            m_skipNextSourceRecent = false;
        } else if (isStartGuideSupportedFile(sourcePath)) {
            addRecentFile(sourcePath);
            showViewTab();
            scheduleRecentThumbnailCapture(sourcePath);
        } else if (!m_pendingRecentThumbnailPath.isEmpty()) {
            scheduleRecentThumbnailCapture(m_pendingRecentThumbnailPath);
        }
    }
    updateCommandStates();
    updateCadSketchOverlay();
}

void MainWindow::onProjectExplorerCurrentItemChanged(QTreeWidgetItem* current,
                                                     QTreeWidgetItem* /*previous*/)
{
    if (m_blockProjectExplorerSignals || !current)
        return;

    const auto kind = projectNodeKind(current);
    const DocumentId docId = current->data(0, kRoleDocId).toInt();
    const QString entry = current->data(0, kRoleEntry).toString();
    const QString nodeKey = current->data(0, kRoleNodeKey).toString();
    const QStringList leafEntries = current->data(0, kRoleLeafEntries).toStringList();
    const auto contourId = static_cast<lcnc::cam::ContourId>(current->data(0, kRoleContourId).toULongLong());
    int contourIndex = m_appContext->camModule()->contourIndexById(contourId);
    if (contourIndex < 0)
        contourIndex = current->data(0, kRoleContourIndex).toInt();

    if (lcnc::app::isCadProjectNode(kind)) {
        showWorkpieceView(docId);

        if (docId != kInvalidDocumentId) {
            auto context = lcnc::cad::selection::CadSelectionResolver::fromProjectExplorerNode(
                docId,
                nodeKey,
                entry,
                leafEntries,
                true,
                m_appContext->cadModule()->isSketchEditing());
            m_appContext->cadModule()->setSelectionContext(context);
            QStringList sourceEntries = leafEntries;
            if (sourceEntries.isEmpty() && !entry.isEmpty())
                sourceEntries.append(entry);
            m_appContext->cadModule()->setSelectedEntries(docId, sourceEntries);
            updateCadSketchOverlay();
            updateCadTaskPanelState();
        }
    } else if (lcnc::app::isToolpathProjectNode(kind)) {
        m_appContext->camModule()->requestMachineView();
        if (kind == lcnc::app::ProjectExplorerNodeKind::ToolpathContour) {
            m_appContext->camModule()->setActiveContourId(contourId);
            m_toolpathPanel->showContourCoordinates(contourIndex);
            highlightContourInView(contourIndex);
        } else {
            m_appContext->camModule()->setActiveContourId(0);
            m_toolpathPanel->showContourCoordinates(-1);
        }
    }

    updateCommandStates();
}

void MainWindow::onProjectExplorerItemChanged(QTreeWidgetItem* item, int /*column*/)
{
    if (m_blockProjectExplorerSignals || !item || !m_projectExplorerController)
        return;
    m_blockProjectExplorerSignals = true;
    const auto change = m_projectExplorerController->visibilityChange(item);
    m_blockProjectExplorerSignals = false;
    if (!change)
        return;

    for (const auto& cad : change->cadEntries)
        m_appContext->cadModule()->setEntriesVisible(cad.documentId, cad.entries, change->visible);
    for (const auto& sketch : change->sketches)
        m_appContext->cadModule()->setSketchVisible(sketch.documentId, sketch.sketchId, change->visible);

    switch (change->target) {
    case lcnc::app::ProjectExplorerController::VisibilityChange::Target::MachiningFaces:
        m_appContext->camModule()->setMachiningFacesVisible(change->visible);
        break;
    case lcnc::app::ProjectExplorerController::VisibilityChange::Target::AllContours:
        m_appContext->camModule()->setAllContoursEnabled(change->visible);
        break;
    case lcnc::app::ProjectExplorerController::VisibilityChange::Target::Layer:
        m_appContext->camModule()->setToolpathLayerEnabled(change->layerId, change->visible);
        break;
    case lcnc::app::ProjectExplorerController::VisibilityChange::Target::Contour: {
        int contourIndex = m_appContext->camModule()->contourIndexById(
            static_cast<lcnc::cam::ContourId>(change->contourId));
        if (contourIndex < 0)
            contourIndex = change->contourIndex;
        m_appContext->camModule()->setContourEnabled(contourIndex, change->visible);
        if (item == m_projectExplorerTree->currentItem())
            highlightContourInView(contourIndex);
        break;
    }
    case lcnc::app::ProjectExplorerController::VisibilityChange::Target::None:
    case lcnc::app::ProjectExplorerController::VisibilityChange::Target::Cad:
        break;
    }
}

void MainWindow::onProjectExplorerItemDoubleClicked(QTreeWidgetItem* item, int /*column*/)
{
    if (!item || projectNodeKind(item) != lcnc::app::ProjectExplorerNodeKind::ToolpathLayer)
        return;

    const std::uint64_t layerId = item->data(0, kRoleLayerId).toULongLong();
    CamModule* cam = m_appContext ? m_appContext->camModule() : nullptr;
    if (!cam || layerId == 0)
        return;

    const ToolpathLayer* sourceLayer = nullptr;
    for (const ToolpathLayer& layer : cam->toolpathLayers()) {
        if (layer.layerId == layerId) {
            sourceLayer = &layer;
            break;
        }
    }
    if (!sourceLayer)
        return;

    QDialog dialog(this);
    // 中文翻译：图层配置
    dialog.setWindowTitle(tr("Layer configuration"));
    auto* layout = new QVBoxLayout(&dialog);
    auto* form = new QFormLayout();
    auto* nameEdit = new QLineEdit(sourceLayer->name, &dialog);
    auto* colorButton = new QPushButton(&dialog);
    auto* toolCombo = new QComboBox(&dialog);
    QColor selectedColor = sourceLayer->color.isValid() ? sourceLayer->color : QColor(80, 190, 150);

    auto refreshColorButton = [&]() {
        colorButton->setText(selectedColor.name(QColor::HexRgb).toUpper());
        colorButton->setStyleSheet(QStringLiteral("QPushButton { background: %1; color: %2; }")
            .arg(selectedColor.name(QColor::HexRgb), selectedColor.lightness() < 128 ? QStringLiteral("white") : QStringLiteral("black")));
    };
    refreshColorButton();

    connect(colorButton, &QPushButton::clicked, &dialog, [&]() {
        // 中文翻译：选择图层颜色
        const QColor color = QColorDialog::getColor(selectedColor, &dialog, tr("Select layer color"));
        if (!color.isValid())
            return;
        selectedColor = color;
        refreshColorButton();
    });

    // 中文翻译：名称
    form->addRow(tr("Name"), nameEdit);
    // 中文翻译：颜色
    form->addRow(tr("color"), colorButton);
    // 中文翻译：未指定工具
    toolCombo->addItem(tr("No tool specified"), QString());
    if (auto* planService = lcnc::Kernel::current().service<lcnc::process::ProcessCuttingPlanService>()) {
        const QStringList tools = planService->availableToolNames();
        for (const QString& toolName : tools) {
            const QString trimmed = toolName.trimmed();
            if (trimmed.isEmpty())
                continue;
            toolCombo->addItem(trimmed, trimmed);
        }
    }
    const QString currentTool = sourceLayer->toolName.trimmed();
    if (!currentTool.isEmpty() && toolCombo->findData(currentTool) < 0)
        toolCombo->addItem(currentTool, currentTool);
    const int currentToolIndex = toolCombo->findData(currentTool);
    toolCombo->setCurrentIndex(currentToolIndex >= 0 ? currentToolIndex : 0);
    // 中文翻译：工具
    form->addRow(tr("Tools"), toolCombo);
    layout->addLayout(form);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    layout->addWidget(buttons);

    if (dialog.exec() != QDialog::Accepted)
        return;

    const QString selectedTool = toolCombo->currentData().toString();
    if (cam->updateToolpathLayer(layerId, nameEdit->text(), selectedColor, selectedTool))
        rebuildProjectExplorer();
}

void MainWindow::onProjectExplorerContextMenuRequested(const QPoint& pos)
{
    if (!m_projectExplorerTree)
        return;

    QTreeWidgetItem* item = m_projectExplorerTree->itemAt(pos);
    if (!item)
        return;

    const auto kind = projectNodeKind(item);

    if (kind == lcnc::app::ProjectExplorerNodeKind::MachiningFace) {
        const std::uint64_t faceId = item->data(
            0, lcnc::app::ProjectExplorerRoles::MachiningFaceId).toULongLong();
        QMenu menu(this);
        // 中文翻译：设置面角色
        QMenu* roleMenu = menu.addMenu(tr("Set up the character"));
        // 中文翻译：加工面
        QAction* machiningAction = roleMenu->addAction(tr("Processing surface"));
        // 中文翻译：横截面
        QAction* crossSectionAction = roleMenu->addAction(tr("cross section"));
        // 中文翻译：删除加工面
        QAction* removeAction = menu.addAction(tr("Delete machining surface"));
        QAction* chosen = menu.exec(m_projectExplorerTree->viewport()->mapToGlobal(pos));
        CamModule* cam = m_appContext->camModule();
        if (chosen == machiningAction)
            cam->setMachiningFaceRole(faceId, lcnc::cam::MachiningFaceRole::MachiningSurface);
        else if (chosen == crossSectionAction)
            cam->setMachiningFaceRole(faceId, lcnc::cam::MachiningFaceRole::CrossSection);
        else if (chosen == removeAction)
            m_appContext->camModule()->removeMachiningFace(faceId);
        return;
    }
    if (kind == lcnc::app::ProjectExplorerNodeKind::MachiningFaceRoot) {
        QMenu menu(this);
        // 中文翻译：清除所有加工面
        QAction* clearAction = menu.addAction(tr("Clear all work surfaces"));
        QAction* chosen = menu.exec(m_projectExplorerTree->viewport()->mapToGlobal(pos));
        if (chosen == clearAction)
            m_appContext->camModule()->clearMachiningFaces();
        return;
    }

    if (kind == lcnc::app::ProjectExplorerNodeKind::ToolpathRoot) {
        CamModule* cam = m_appContext->camModule();
        if (!cam)
            return;
        QMenu menu(this);
        // 中文翻译：新建图层
        QAction* newLayerAction = menu.addAction(tr("New layer"));
        QAction* chosen = menu.exec(m_projectExplorerTree->viewport()->mapToGlobal(pos));
        if (chosen == newLayerAction) {
            cam->addToolpathLayer(QString(), QColor());
            rebuildProjectExplorer();
        }
        return;
    }

    if (kind == lcnc::app::ProjectExplorerNodeKind::ToolpathLayer) {
        CamModule* cam = m_appContext->camModule();
        if (!cam)
            return;
        const std::uint64_t layerId = item->data(0, kRoleLayerId).toULongLong();
        QMenu menu(this);
        // 中文翻译：移动到图层
        QAction* moveAction = menu.addAction(tr("Move to layer"));
        // 中文翻译：删除图层
        QAction* deleteAction = menu.addAction(tr("Delete layer"));
        QAction* chosen = menu.exec(m_projectExplorerTree->viewport()->mapToGlobal(pos));
        if (chosen == deleteAction) {
            // 中文翻译：删除图层；将删除该图层及其下所有轮廓，且不可撤销，是否继续？
            const QMessageBox::StandardButton btn = QMessageBox::question(
                this, tr("Delete layer"),
                tr("This will delete the layer and all contours under it, and cannot be undone. Continue?"),
                QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
            if (btn == QMessageBox::Yes) {
                cam->removeToolpathLayerWithContours(layerId);
                rebuildProjectExplorer();
            }
        } else if (chosen == moveAction) {
            const QList<lcnc::cam::ContourId> contourIds = gatherSelectedContourIds();
            if (contourIds.isEmpty()) {
                // 中文翻译：移动到图层；请先在视图或工程树中选择要移动的轮廓
                QMessageBox::information(
                    this, tr("Move to layer"),
                    tr("Please select the contours to move in the view or project tree first."));
                return;
            }
            if (cam->assignContoursToLayer(contourIds, layerId))
                rebuildProjectExplorer();
        }
        return;
    }

}

void MainWindow::rebuildProjectExplorer()
{
    if (!m_projectExplorerTree)
        return;

    m_projectExplorerSnapshot = lcnc::app::ProjectExplorerModel::build(
        m_appContext->cadProjectExplorerProjection(),
        m_appContext->camProjectExplorerProjection());

    m_blockProjectExplorerSignals = true;
    if (m_projectExplorerController)
        m_projectExplorerController->rebuild(m_projectExplorerSnapshot);
    m_blockProjectExplorerSignals = false;

    if (!isMachineViewActive())
        m_appContext->cadModule()->syncSelectionFromView();
}

void MainWindow::selectProjectExplorerContour(int contourIndex)
{
    const auto contourId = m_appContext && m_appContext->camModule()
        ? m_appContext->camModule()->contourIdAt(contourIndex)
        : 0;
    selectProjectExplorerContourById(contourId, contourIndex);
}

void MainWindow::selectProjectExplorerContourById(lcnc::cam::ContourId contourId, int fallbackIndex)
{
    m_blockProjectExplorerSignals = true;
    const auto selected = m_projectExplorerController
        ? m_projectExplorerController->selectContour(contourId, fallbackIndex)
        : std::nullopt;
    m_blockProjectExplorerSignals = false;

    if (!selected)
        return;
    int contourIndex = m_appContext && m_appContext->camModule()
        ? m_appContext->camModule()->contourIndexById(
              static_cast<lcnc::cam::ContourId>(selected->contourId))
        : -1;
    if (contourIndex < 0)
        contourIndex = selected->contourIndex;

    m_toolpathPanel->showContourCoordinates(contourIndex);
    highlightContourInView(contourIndex);
}

void MainWindow::selectProjectExplorerContours(const QList<int>& contourIndexes)
{
    if (!m_projectExplorerTree || contourIndexes.isEmpty() || !m_projectExplorerController)
        return;

    QList<std::uint64_t> contourIds;
    QList<int> fallbackIndexes;
    for (int contourIndex : contourIndexes) {
        if (contourIndex < 0)
            continue;
        const auto contourId = m_appContext && m_appContext->camModule()
            ? m_appContext->camModule()->contourIdAt(contourIndex)
            : 0;
        if (contourId != 0)
            contourIds.append(contourId);
        fallbackIndexes.append(contourIndex);
    }

    if (contourIds.isEmpty() && fallbackIndexes.isEmpty())
        return;

    m_blockProjectExplorerSignals = true;
    const auto selected = m_projectExplorerController->selectContours(contourIds, fallbackIndexes);
    m_blockProjectExplorerSignals = false;

    if (selected) {
        int contourIndex = m_appContext && m_appContext->camModule()
            ? m_appContext->camModule()->contourIndexById(
                  static_cast<lcnc::cam::ContourId>(selected->contourId))
            : -1;
        if (contourIndex < 0)
            contourIndex = selected->contourIndex;
        m_toolpathPanel->showContourCoordinates(contourIndex);
    }
}

void MainWindow::selectProjectExplorerEntries(DocumentId docId, const QStringList& entries)
{
    if (!m_projectExplorerTree || !m_projectExplorerController)
        return;

    m_blockProjectExplorerSignals = true;
    m_projectExplorerController->selectEntries(docId, entries);
    m_blockProjectExplorerSignals = false;
}

QList<lcnc::cam::ContourId> MainWindow::gatherSelectedContourIds() const
{
    QList<lcnc::cam::ContourId> ids;
    QSet<lcnc::cam::ContourId> seen;
    CamModule* cam = m_appContext ? m_appContext->camModule() : nullptr;

    // 视图拾取的轮廓（右键工程树图层不会影响 3D 视图选择，是最稳定的来源）。
    if (cam) {
        for (lcnc::cam::ContourId id : cam->selectedContourIds()) {
            if (id != 0 && !seen.contains(id)) {
                seen.insert(id);
                ids.append(id);
            }
        }
    }
    // 工程树中仍被选中的轮廓节点（例如未因右键被清除的扩展选区）。
    if (m_projectExplorerTree) {
        for (QTreeWidgetItem* item : m_projectExplorerTree->selectedItems()) {
            if (projectNodeKind(item) != lcnc::app::ProjectExplorerNodeKind::ToolpathContour)
                continue;
            const auto id = static_cast<lcnc::cam::ContourId>(
                item->data(0, kRoleContourId).toULongLong());
            if (id != 0 && !seen.contains(id)) {
                seen.insert(id);
                ids.append(id);
            }
        }
    }
    return ids;
}

void MainWindow::highlightContourInView(int contourIndex)
{
    CamModule* cam = m_appContext ? m_appContext->camModule() : nullptr;
    GuiDocument* gd = cam ? cam->activeGuiDocument() : nullptr;
    if (!gd || gd->context().IsNull())
        return;

    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    ctx->ClearSelected(Standard_False);

    const QList<Handle(AIS_Shape)>& contourAis = cam->contourAis();
    if (contourIndex >= 0
        && contourIndex < contourAis.size()
        && !contourAis.at(contourIndex).IsNull()) {
        ctx->AddOrRemoveSelected(contourAis.at(contourIndex), Standard_False);
    }

    if (gd->hasView())
        gd->view()->Redraw();
}

void MainWindow::restorePersistedCamState()
{
    CamModule* cam = m_appContext->camModule();
    if (!cam)
        return;

    const CamConfig& config = cam->config();
    const QString presetName = config.machinePreset();
    if (!lcnc::Kernel::current().service<lcnc::MachineConfigurationService>() && !presetName.isEmpty())
        cam->configureMachine(presetName);

    m_toolpathPanel->setLeadInLength(cam->leadInLength());
    m_toolpathPanel->setDiscretizationInterval(cam->deflection());
    m_toolpathPanel->setSmoothAngle(cam->smoothAngle());
    m_toolpathPanel->setExtractionStrategy(cam->extractionStrategy());
    m_toolpathPanel->setShowNormals(cam->showNormals());
    m_toolpathPanel->setNormalSampleStep(cam->normalSampleStep());

    const QString machinePath = cam->machineModelPath();
    if (config.autoLoadMachineModel() && !machinePath.isEmpty() && QFileInfo::exists(machinePath))
        cam->loadMachine(machinePath);
}

void MainWindow::syncMachineWorkspaceUi()
{
    syncMachineWorkspaceUiInternal(true);
}

void MainWindow::syncMachineWorkspaceUiInternal(bool rebuildTree)
{
    ProcessModule* process = m_appContext->processModule();
    LcncDocument* machineDoc = m_appContext->camModule()->machineDocument();
    auto* machineConfig = lcnc::Kernel::current().service<lcnc::MachineConfigurationService>();
    const QList<MachineAxisDef> configuredAxes = machineConfig
        ? machineConfig->axisDefinitions()
        : QList<MachineAxisDef>{};
    if (!machineDoc) {
        if (rebuildTree)
            rebuildProjectExplorer();
        m_machinePanel->setDocument(nullptr);
        syncMachineTreeVisibilityState();
        if (process)
            process->setAxisDefinitions(configuredAxes);
        if (m_laserControl)
            m_laserControl->setAxisDefinitions(configuredAxes);
        return;
    }

    if (rebuildTree)
        rebuildProjectExplorer();

    m_machinePanel->setDocument(machineDoc);
    syncMachineTreeVisibilityState();

    const QList<MachineAxisDef> axes = configuredAxes.isEmpty()
        ? machineDoc->machineKinematics()->axes()
        : configuredAxes;
    if (process)
        process->setAxisDefinitions(axes);
    if (m_laserControl)
        m_laserControl->setAxisDefinitions(axes);

    if (process && m_laserControl) {
        const auto axisPositions = process->currentAxisPositions();
        for (auto it = axisPositions.cbegin(); it != axisPositions.cend(); ++it)
            m_laserControl->updateAxisPosition(it.key(), it.value());
    }
}

void MainWindow::syncMachineTreeVisibilityState()
{
    if (!m_machineTree || !m_appContext || !m_appContext->camModule())
        return;

    CamModule* cam = m_appContext->camModule();
    m_machineTree->setVisibilityState(cam->isMachineModelVisible(), cam->visibleMachineEntries());
    if (m_actMachineModelVisible) {
        QSignalBlocker blocker(m_actMachineModelVisible);
        m_actMachineModelVisible->setChecked(cam->isMachineModelVisible());
    }
}

void MainWindow::persistViewDisplayMode(int displayMode, bool faceBoundary)
{
    if (m_viewStateController)
        (void)m_viewStateController->persistDisplayMode(displayMode, faceBoundary);
}

void MainWindow::persistViewToggleState()
{
    if (!m_viewStateController)
        return;

    bool worldAxesVisible = false;
    if (auto* worldAxes = m_cmdContainer ? m_cmdContainer->findAction(CmdToggleWorldAxes::Name) : nullptr)
        worldAxesVisible = worldAxes->isChecked();
    (void)m_viewStateController->persistToggles(
        worldAxesVisible,
        m_actRotaryAxisGuides && m_actRotaryAxisGuides->isChecked(),
        m_actCutterHeadGuide && m_actCutterHeadGuide->isChecked());
}

void MainWindow::applyPersistedViewState()
{
    if (!m_viewStateController)
        return;

    const auto state = m_viewStateController->state();
    QAction* aWire = m_cmdContainer ? m_cmdContainer->findAction(CmdToggleWireframe::Name) : nullptr;
    QAction* aShade = m_cmdContainer ? m_cmdContainer->findAction(CmdToggleShaded::Name) : nullptr;
    QAction* aEdges = m_cmdContainer ? m_cmdContainer->findAction(CmdToggleShadedWithEdges::Name) : nullptr;
    if (state.displayMode == AIS_WireFrame) {
        if (aWire)
            aWire->setChecked(true);
    } else if (state.faceBoundary) {
        if (aEdges)
            aEdges->setChecked(true);
    } else if (aShade) {
        aShade->setChecked(true);
    }
    if (auto* gd = m_appContext && m_appContext->camModule()
            ? m_appContext->camModule()->activeGuiDocument()
            : nullptr) {
        if (gd->renderingManager())
            gd->renderingManager()->setRuntimeDisplayMode(state.displayMode, state.faceBoundary);
    }

    if (QAction* worldAxes = m_cmdContainer ? m_cmdContainer->findAction(CmdToggleWorldAxes::Name) : nullptr) {
        worldAxes->setChecked(state.worldAxesVisible);
        auto& renderer = lcnc::view::WorldAxesRenderer::instance();
        if (auto* guiApp = lcnc::Kernel::current().guiApp()) {
            if (auto* workspace = guiApp->activeGuiDocument(); workspace && workspace->scene())
                renderer.attach(workspace->scene());
        }
        renderer.setGloballyVisible(state.worldAxesVisible);
    }

    if (m_actRotaryAxisGuides)
        m_actRotaryAxisGuides->setChecked(state.rotaryAxisGuidesVisible);
    if (m_actCutterHeadGuide)
        m_actCutterHeadGuide->setChecked(state.cutterHeadGuideVisible);
    if (m_actMachineModelVisible)
        m_actMachineModelVisible->setChecked(false);

    if (auto* cam = m_appContext ? m_appContext->camModule() : nullptr) {
        cam->setRotaryAxisGuidesVisible(state.rotaryAxisGuidesVisible);
        cam->setCutterHeadGuideVisible(state.cutterHeadGuideVisible);
        cam->setMachineModelVisible(false);
    }
    syncMachineTreeVisibilityState();
}

void MainWindow::showStartGuide()
{
    if (m_centerTabs && m_startGuide)
        m_centerTabs->setCurrentWidget(m_startGuide);
}

void MainWindow::refreshDocumentTabs()
{
    if (!m_documentTabs)
        return;

    auto* project = lcnc::Kernel::current().projectManager();
    QSignalBlocker blocker(m_documentTabs);
    while (m_documentTabs->count() > 0)
        m_documentTabs->removeTab(0);
    if (!project) {
        m_documentTabs->setVisible(false);
        return;
    }

    const ProjectWorkspaceId activeId = project->activeWorkspaceId();
    int activeIndex = -1;
    for (ProjectWorkspaceId id : project->workspaceIds()) {
        auto* workspace = project->workspace(id);
        QString title = workspace ? workspace->session().projectName() : QString();
        if (title.trimmed().isEmpty() && workspace && workspace->workpieceDocument())
            title = workspace->workpieceDocument()->name();
        if (title.trimmed().isEmpty())
            // 中文翻译：未命名
            title = tr("Unnamed");
        const int index = m_documentTabs->addTab(title);
        m_documentTabs->setTabData(index, id);
        if (id == activeId)
            activeIndex = index;
    }

    m_documentTabs->setVisible(m_documentTabs->count() > 0);
    if (activeIndex >= 0)
        m_documentTabs->setCurrentIndex(activeIndex);
}

void MainWindow::showViewTab()
{
    if (m_centerTabs && m_viewStack)
        m_centerTabs->setCurrentIndex(1);
}

bool MainWindow::isStartGuideSupportedFile(const QString& filePath) const
{
    if (filePath.trimmed().isEmpty())
        return false;
    const QFileInfo info(filePath);
    if (!info.exists())
        return false;
    return lcnc::LcncProjectPackage::isProjectPath(filePath) || isStepFile(filePath);
}

QString MainWindow::recentThumbnailPath(const QString& filePath) const
{
    QString root = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    if (root.isEmpty())
        root = QDir::tempPath() + QStringLiteral("/LaserCNC");
    const QString dirPath = root + QStringLiteral("/recent_thumbnails");
    QDir().mkpath(dirPath);

    const QString normalized = QFileInfo(filePath).absoluteFilePath();
    const QByteArray hash = QCryptographicHash::hash(normalized.toUtf8(),
                                                     QCryptographicHash::Sha1).toHex();
    return dirPath + QLatin1Char('/') + QString::fromLatin1(hash) + QStringLiteral(".png");
}

void MainWindow::refreshStartGuide()
{
    if (!m_startGuide)
        return;

    auto* settings = lcnc::Kernel::current().appSettings();
    QStringList files;
    QHash<QString, QString> thumbnails;
    if (settings) {
        for (const QString& path : settings->recentFiles) {
            if (!isStartGuideSupportedFile(path))
                continue;
            const QString normalized = lcnc::LcncProjectPackage::isProjectPath(path)
                ? QFileInfo(lcnc::LcncProjectPackage::packageDirectory(path)).absoluteFilePath()
                : QFileInfo(path).absoluteFilePath();
            if (files.contains(normalized, Qt::CaseInsensitive))
                continue;
            files.append(normalized);
            thumbnails.insert(normalized, recentThumbnailPath(normalized));
        }
    }
    m_startGuide->setRecentFiles(files, thumbnails);
}

void MainWindow::addRecentFile(const QString& filePath)
{
    if (!isStartGuideSupportedFile(filePath))
        return;

    auto* settings = lcnc::Kernel::current().appSettings();
    if (!settings)
        return;

    const QString normalized = lcnc::LcncProjectPackage::isProjectPath(filePath)
        ? QFileInfo(lcnc::LcncProjectPackage::packageDirectory(filePath)).absoluteFilePath()
        : QFileInfo(filePath).absoluteFilePath();

    QStringList next;
    next.append(normalized);
    for (const QString& existing : settings->recentFiles) {
        const QString existingNormalized = lcnc::LcncProjectPackage::isProjectPath(existing)
            ? QFileInfo(lcnc::LcncProjectPackage::packageDirectory(existing)).absoluteFilePath()
            : QFileInfo(existing).absoluteFilePath();
        if (existingNormalized.compare(normalized, Qt::CaseInsensitive) == 0)
            continue;
        if (!isStartGuideSupportedFile(existingNormalized))
            continue;
        next.append(existingNormalized);
        if (next.size() >= settings->recentLimit)
            break;
    }

    settings->recentFiles = next;
    settings->saveDefault();
    refreshStartGuide();
}

void MainWindow::openStartGuideFile(const QString& filePath)
{
    if (!isStartGuideSupportedFile(filePath)) {
        // 中文翻译：打开文件；文件不存在或格式不支持: %1
        QMessageBox::warning(this, tr("open file"), tr("File does not exist or format is not supported: %1").arg(filePath));
        refreshStartGuide();
        return;
    }

    showViewTab();
    addRecentFile(filePath);

    const DocumentId docId = m_appContext->cadModule()->openDocument(filePath);
    if (docId != kInvalidDocumentId)
        m_appContext->cadModule()->requestWorkpieceView(docId);
    if (isStepFile(filePath))
        m_pendingRecentThumbnailPath = QFileInfo(filePath).absoluteFilePath();
    else
        scheduleRecentThumbnailCapture(filePath);
    updateCommandStates();
}

void MainWindow::scheduleRecentThumbnailCapture(const QString& filePath)
{
    if (!isStartGuideSupportedFile(filePath))
        return;

    const QString normalized = lcnc::LcncProjectPackage::isProjectPath(filePath)
        ? QFileInfo(lcnc::LcncProjectPackage::packageDirectory(filePath)).absoluteFilePath()
        : QFileInfo(filePath).absoluteFilePath();
    m_pendingRecentThumbnailPath = normalized;

    QTimer::singleShot(350, this, [this, normalized]() {
        if (m_pendingRecentThumbnailPath.compare(normalized, Qt::CaseInsensitive) != 0)
            return;
        captureRecentThumbnail(normalized);
    });
}

void MainWindow::captureRecentThumbnail(const QString& filePath)
{
    if (!m_occView || !isStartGuideSupportedFile(filePath))
        return;

    const QString path = recentThumbnailPath(filePath);
    auto* gd = m_appContext && m_appContext->camModule()
        ? m_appContext->camModule()->activeGuiDocument()
        : nullptr;
    const bool saved = gd && gd->dumpWorkpiecePreview(path, 336, 236);
    if (saved) {
        m_pendingRecentThumbnailPath.clear();
        refreshStartGuide();
    }
}

// ── View routing helpers ────────────────────────────────────────────────────────────────
void MainWindow::showMachineView()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "MainWindow::showMachineView");
    m_machineWorkspaceActive = true;
    auto* project = lcnc::Kernel::current().projectManager();
    if (auto* gd = m_appContext->camModule()->activeGuiDocument())
        activateWorkspaceOccView(project ? project->activeWorkspaceId() : kInvalidProjectWorkspaceId, gd);
    else
        showDefaultOccView();
    if (m_occView)
        m_occView->clearSketchOverlay();

    syncMachineWorkspaceUiInternal(false);
}

void MainWindow::showWorkpieceView(DocumentId id)
{
    if (id == kInvalidDocumentId)
        id = m_appContext->cadModule()->workpieceDocumentId();
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "MainWindow::showWorkpieceView unified docId={}", id);

    m_machineWorkspaceActive = false;

    auto* project = lcnc::Kernel::current().projectManager();
    if (auto* gd = m_appContext->camModule()->activeGuiDocument())
        activateWorkspaceOccView(project ? project->activeWorkspaceId() : kInvalidProjectWorkspaceId, gd);
    else
        showDefaultOccView();
    updateCadSketchOverlay();
}

void MainWindow::syncRightPanelForRibbonIndex(int index)
{
    if (!m_rightStack)
        return;

    switch (index) {
    case kRibbonCadIndex:
        m_rightStack->setCurrentWidget(m_cadTaskPanel);
        showWorkpieceView();
        break;
    case kRibbonCamIndex:
        m_rightStack->setCurrentWidget(m_camRightTabs);
        showMachineView();
        break;
    case kRibbonLaserIndex:
        m_rightStack->setCurrentWidget(m_laserControl);
        showMachineView();
        break;
    case kRibbonFileIndex:
    default:
        break;
    }

    updateCadSketchOverlay();
    updateCommandStates();
}

void MainWindow::updateCommandStates()
{
    if (m_cmdContainer)
        m_cmdContainer->updateAllStates();
    updateCadTaskPanelState();
}

bool MainWindow::isMachineViewActive() const
{
    return m_machineWorkspaceActive;
}

void MainWindow::closeEvent(QCloseEvent* e)
{
    // Disconnect all signals before shutdown to prevent re-entrant callbacks
    disconnect(this, nullptr, nullptr, nullptr);

    // Detach rendering: stop the active ViewCube animation and unhook the
    // OCC view from the widget so no Redraw() fires during Qt teardown.
    if (m_defaultOccView)
        m_defaultOccView->attachDefaultScene(nullptr);
    if (m_workspacePresenter) {
        for (QWidget* widget : m_workspacePresenter->views()) {
            if (auto* view = qobject_cast<WidgetOccView*>(widget))
                view->attachDefaultScene(nullptr);
        }
    }

    // Accept; Qt's parent-child destructor chain cleans up all OCC resources.
    e->accept();
}
