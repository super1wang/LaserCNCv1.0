#include "app/main_window.h"
#include "core/kernel/kernel.h"
#include "app/app_context.h"
#include "app/command_registry.h"
#include "app/project_explorer_tree_utils.h"
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
#include "modules/process/Process/qg_processeswidget.h"
#include "view/widget_occ_view.h"
#include "modules/cam/ui/widget_machine_panel.h"
#include "modules/cam/ui/widget_toolpath_panel.h"
#include "modules/cam/ui/dialog_axis_calibration_wizard.h"
#include "modules/process/ui/widget_laser_control.h"
#include "app/dialog/dialog_task_manager.h"
#include "modules/cam/settings/cam_config.h"
#include "core/algorithms/cam/laser_toolpath.h"
#include "core/document/lcnc_document.h"
#include "core/project/lcnc_project_manager.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/document/xcaf_utils.h"
#include "view/graphics_scene.h"
#include "view/gui_application.h"
#include "view/gui_document.h"
#include "view/sketch_overlay_renderer.h"
#include "view/world_axes_renderer.h"
#include "modules/cad/cad_module.h"
#include "modules/cad/selection/cad_selection_resolver.h"
#include "modules/cam/cam_module.h"
#include "modules/process/process_module.h"

#include <SARibbonBar.h>
#include <SARibbonCategory.h>
#include <SARibbonPanel.h>

#include <QSplitter>
#include <QStackedWidget>
#include <QTreeWidget>
#include <QTreeWidgetItemIterator>
#include <QTabWidget>
#include <QHeaderView>
#include <QStatusBar>
#include <QLabel>
#include <QMenu>
#include <QAction>

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
#include <QTimer>
#include <QStyle>
#include <QSignalBlocker>
#include <QSet>
#include <QVariantMap>
#include <functional>
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
constexpr int kRoleAxisName = lcnc::app::ProjectExplorerRoles::AxisName;
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
    QVariantMap params;
    params.insert(QStringLiteral("sizeX"), sizeX);
    params.insert(QStringLiteral("sizeY"), sizeY);
    params.insert(QStringLiteral("sizeZ"), sizeZ);
    params.insert(QStringLiteral("radius1"), radius1);
    params.insert(QStringLiteral("radius2"), radius2);
    return params;
}

QVariantMap featureParams(double length, double angleDeg)
{
    QVariantMap params;
    params.insert(QStringLiteral("length"), length);
    params.insert(QStringLiteral("angle"), angleDeg);
    return params;
}

CadModule::TransformParameters transformParams(double translateX,
                                               double translateY,
                                               double translateZ,
                                               double rotateX,
                                               double rotateY,
                                               double rotateZ,
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

bool isActiveSketchOverlayKey(const QString& key)
{
    return key.startsWith(QStringLiteral("__sketch_active_element_"))
        || key.startsWith(QStringLiteral("__sketch_active_handle_"));
}

constexpr int kRibbonFileIndex = 0;
constexpr int kRibbonCadIndex = 1;
constexpr int kRibbonCamIndex = 2;
constexpr int kRibbonLaserIndex = 3;

}

MainWindow::MainWindow(QWidget* parent)
    : SARibbonMainWindow(parent)
{
    setWindowTitle(tr("LaserCNC — 五轴激光加工CAM软件"));
    setWindowIcon(QIcon(":/icons/app_icon.svg"));
    resize(1440, 900);

    // Initialise in dependency order
    createContext();        // AppContext (needs singletons)
    createCentralLayout();  // Central splitter — creates m_occView first
    createCommands();       // Commands connect to m_occView (must exist)
    createRibbon();         // Ribbon uses m_cmdContainer (must exist)
    createStatusBar();

        auto* project = lcnc::Kernel::current().projectManager();
        connect(project, &lcnc::LcncProjectManager::projectReset,
            this, &MainWindow::onProjectReset);
        connect(project, &lcnc::LcncProjectManager::domainDataChanged,
            this, &MainWindow::onProjectDomainChanged);

    // 世界坐标系渲染器：新建/关闭文档时自动 attach/detach 到该文档场景，
    // 全局可见性由 ribbon 上的 CmdToggleWorldAxes 控制。
    if (auto* guiApp = lcnc::Kernel::current().guiApp()) {
        if (auto* gd = guiApp->workspaceGuiDocument(); gd && gd->scene()) {
            LCNC_DEBUG(lcnc::LogCode::Generic,
                       "WorldAxes auto-attach for workspace");
            lcnc::view::WorldAxesRenderer::instance().attach(gd->scene());
        }
    }

    restorePersistedCamState();
    showMachineView();

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

    connect(m_appContext->cadModule(), &CadModule::operationFailed,
            this, [this](const QString& title, const QString& message) {
                QMessageBox::critical(this, title, message);
            });

    connect(m_appContext->camModule(), &CamModule::operationFailed,
            this, [this](const QString& title, const QString& message) {
                QMessageBox::critical(this, title, message);
            });

    connect(m_appContext->processModule(), &ProcessModule::statusMessageChanged,
            this, [this](const QString& status) {
                if (m_sbStatus)
                    m_sbStatus->setText(status);
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

    // ── Left panel ─────────────────────────────────────────────────────────
    createLeftPanel();

    // ── Right panel ────────────────────────────────────────────────────────
    createRightPanel();

    // ── Splitter ───────────────────────────────────────────────────────────
    m_splitter = new QSplitter(Qt::Horizontal, this);
    m_splitter->addWidget(m_leftTabs);
    m_splitter->addWidget(m_occView);
    m_splitter->addWidget(m_rightStack);
    m_splitter->setStretchFactor(0, 1);
    m_splitter->setStretchFactor(1, 5);
    m_splitter->setStretchFactor(2, 2);
    m_splitter->setSizes({250, 900, 290});

    setCentralWidget(m_splitter);

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

    // ── 3D selection → module coordination ────────────────────────────────
    connect(m_occView, &WidgetOccView::selectionChanged, this, [this] {
        if (isMachineViewActive()) {
            m_appContext->camModule()->syncSelectionFromView();
            return;
        }

        m_appContext->cadModule()->syncSelectionFromView(
            m_appContext->cadModule()->workpieceDocumentId());
    });

            connect(m_occView, &WidgetOccView::sketchOverlayPicked,
                this, &MainWindow::handleCadSketchOverlayPicked);
            connect(m_occView, &WidgetOccView::sketchOverlayDragMoved,
                this, &MainWindow::handleCadSketchOverlayDrag);
            connect(m_occView, &WidgetOccView::sketchOverlayDragCanceled,
                this, &MainWindow::handleCadSketchOverlayDrag);
            connect(m_occView, &WidgetOccView::transformGizmoDragMoved,
                this, [this](int operation, int axis, double delta) {
                    if (!m_cadTaskPanel || !m_cadTaskPanel->isTransformPageActive())
                        return;
                    m_cadTaskPanel->addTransformDragDelta(operation, axis, delta);
                });

    connect(m_occView, &WidgetOccView::leadInPickMoved, this,
            [this](const QPoint& pos) {
                m_appContext->camModule()->updateLeadInPreview(m_occView, pos);
            });
    connect(m_occView, &WidgetOccView::leadInPickConfirmed, this,
            [this](const QPoint& pos) {
                if (m_appContext->camModule()->commitLeadInPreview(m_occView, pos))
                    m_occView->endLeadInPick();
            });
    connect(m_occView, &WidgetOccView::leadInPickCanceled, this,
            [this]() {
                m_occView->endLeadInPick();
                m_appContext->camModule()->cancelLeadInPreview();
            });

    connect(m_occView, &WidgetOccView::facePickConfirmed, this,
            [this](const QPoint& pos) {
                if (m_pendingCalibrationTarget.isEmpty())
                    return;

                bool handled = false;
                // 标定向导是唯一的拾取入口；旧的 A/C/CUTTER_HEAD 直传分支已移除。
                if (m_pendingCalibrationTarget.startsWith(QStringLiteral("WIZARD_"))
                    && m_axisCalibWizard) {
                    using lcnc::cam::ui::DialogAxisCalibrationWizard;
                    gp_Pnt center;
                    QString errMsg;
                    if (m_appContext->camModule()->pickReferenceFaceCenter(
                            m_occView, pos, center, &errMsg)) {
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

                m_occView->endFacePick();
                m_pendingCalibrationTarget.clear();
                m_machinePanel->setCalibrationPickAxis(QString());
            });
    connect(m_occView, &WidgetOccView::facePickCanceled, this,
            [this]() {
                m_occView->endFacePick();
                if (m_pendingCalibrationTarget.startsWith(QStringLiteral("WIZARD_"))
                    && m_axisCalibWizard) {
                    m_axisCalibWizard->cancelPickInProgress();
                    m_axisCalibWizard->raise();
                    m_axisCalibWizard->activateWindow();
                }
                m_pendingCalibrationTarget.clear();
                m_machinePanel->setCalibrationPickAxis(QString());
            });

    connect(m_appContext->camModule(), &CamModule::selectionChanged, this,
            [this](const QStringList& entries) {
                const QStringList sourceEntries = m_appContext->camModule()
                    ->sourceWorkpieceEntriesForMountedEntries(entries);
                if (!sourceEntries.isEmpty())
                    selectProjectExplorerEntries(
                        m_appContext->workpieceDocumentId(), sourceEntries, true);
                else
                    selectProjectExplorerEntries(kInvalidDocumentId, entries, false);
                m_machinePanel->setSelectedEntries(entries);
            });
    connect(m_appContext->camModule(), &CamModule::toolpathContourSelected, this,
            [this](int contourIndex) {
                const auto contourId = m_appContext->camModule()->contourIdAt(contourIndex);
                selectProjectExplorerContourById(contourId, contourIndex);
                m_toolpathPanel->showContourCoordinates(contourIndex);
            });
    connect(m_appContext->camModule(), &CamModule::toolpathContoursSelected, this,
            [this](const QList<int>& contourIndexes) {
                selectProjectExplorerContours(contourIndexes);
            });

    connect(m_appContext->cadModule(), &CadModule::selectionChanged, this,
            [this](DocumentId docId, const QStringList& entries) {
                selectProjectExplorerEntries(docId, entries, true);
                updateCommandStates();
            });

    rebuildProjectExplorer();
    syncMachineWorkspaceUiInternal(true);
}

void MainWindow::create3DView()
{
    m_occView = new WidgetOccView(this);
    m_defaultScene = new GraphicsScene(this);
    m_occView->attachDefaultScene(m_defaultScene);
}

void MainWindow::createLeftPanel()
{
    m_leftTabs = new QTabWidget(this);
    m_leftTabs->setDocumentMode(true);
    m_leftTabs->setMinimumWidth(240);
    m_leftTabs->setMaximumWidth(380);

    m_projectExplorerTree = new QTreeWidget(this);
    m_projectExplorerTree->setColumnCount(2);
    m_projectExplorerTree->setHeaderLabels({tr("项目"), tr("信息")});
    m_projectExplorerTree->header()->setStretchLastSection(false);
    m_projectExplorerTree->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    m_projectExplorerTree->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    m_projectExplorerTree->setAnimated(true);
    m_projectExplorerTree->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_projectExplorerTree->setDragDropMode(QAbstractItemView::InternalMove);
    m_projectExplorerTree->setDefaultDropAction(Qt::MoveAction);
    m_projectExplorerTree->setDropIndicatorShown(true);
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
    connect(m_projectExplorerTree->model(), &QAbstractItemModel::rowsMoved,
            this, [this]() { handleProjectExplorerRowsMoved(); });
    connect(m_projectExplorerTree, &QTreeWidget::itemSelectionChanged, this,
            [this]() {
                if (m_blockProjectExplorerSignals || !m_projectExplorerTree)
                    return;
                const auto currentKind = projectNodeKind(m_projectExplorerTree->currentItem());
                if (!lcnc::app::isMachineProjectNode(currentKind))
                    return;

                QStringList entries;
                for (QTreeWidgetItem* item : m_projectExplorerTree->selectedItems()) {
                    const auto kind = projectNodeKind(item);
                    if (kind == lcnc::app::ProjectExplorerNodeKind::MachineShape) {
                        const QString entry = item->data(0, kRoleEntry).toString();
                        if (!entry.isEmpty())
                            entries.append(entry);
                    }
                }
                m_appContext->camModule()->setSelectedEntries(entries);
            });

    auto* processWidget = new QG_ProcessesWidget(m_leftTabs);
    m_processLeftPanel = processWidget;
    if (auto* process = m_appContext->processModule()) {
        processWidget->setFlowDocument(&process->processFlowDocument());
        connect(process, &ProcessModule::processFlowChanged,
                processWidget, &QG_ProcessesWidget::reloadFlowModel);
    }
    m_leftTabs->addTab(m_projectExplorerTree, tr("项目"));
    m_leftTabs->addTab(m_processLeftPanel, tr("执行"));
    connect(m_leftTabs, &QTabWidget::currentChanged, this, [this](int index) {
        if (index == 1) {
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
    m_machinePanel->setMachineModelPath(cam->machineModelPath());
    m_toolpathPanel->setLeadInLength(cam->leadInLength());
    m_toolpathPanel->setNormalAngle(cam->normalAngle());
    m_toolpathPanel->setDiscretizationInterval(cam->deflection());
    m_toolpathPanel->setSmoothAngle(cam->smoothAngle());
    m_toolpathPanel->setUseFaceClassification(cam->useFaceClassification());
    m_toolpathPanel->setShowNormals(cam->showNormals());
    m_toolpathPanel->setNormalSampleStep(cam->normalSampleStep());

    m_machineRefreshTimer = new QTimer(this);
    m_machineRefreshTimer->setSingleShot(true);
    m_machineRefreshTimer->setInterval(0);
    connect(m_machineRefreshTimer, &QTimer::timeout, this, [this] {
        m_appContext->camModule()->refreshMachineTransforms();
    });

    m_rightStack = new QStackedWidget(this);
    m_rightStack->addWidget(m_machinePanel);   // index 0 — CAM ribbon page
    m_rightStack->addWidget(m_toolpathPanel);  // index 1 — data detail page, updated by selection
    m_rightStack->addWidget(m_laserControl);   // index 2 — laser/process ribbon page
    m_rightStack->addWidget(m_cadTaskPanel);   // index 3 — CAD ribbon page
    m_rightStack->setMinimumWidth(320);
    m_rightStack->setMaximumWidth(420);
    m_rightStack->setCurrentIndex(3);

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
        updateCadPrimitivePreview();
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
        refreshSketchElementsView();
        rebuildProjectExplorer();
        updateCadSketchOverlay();
        updateCadTaskPanelState();
        });

    connect(cad, &CadModule::finishedSketchesChanged,
            this, [this](DocumentId) {
        refreshFinishedSketchesView();
        rebuildProjectExplorer();
        updateCadSketchOverlay();
        updateCadTaskPanelState();
        });

    connect(cad, &CadModule::sketchSelectionChanged,
            this, [this](int) {
        refreshFinishedSketchesView();
        updateCadSketchOverlay();
        updateCadTaskPanelState();
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
            updateCadPrimitivePreview();
        });

    connect(m_cadTaskPanel, &lcnc::cad::ui::WidgetCadTaskPanel::featureParametersChanged,
            this, [this](int, double, double) {
        if (m_cadTaskPanel->isFeaturePageActive())
            updateCadFeaturePreview();
        });

    connect(m_cadTaskPanel, &lcnc::cad::ui::WidgetCadTaskPanel::transformParametersChanged,
            this, [this](double, double, double, double, double, double, int) {
        if (m_cadTaskPanel->isTransformPageActive())
            updateCadTransformPreview();
        });

    connect(m_cadTaskPanel, &lcnc::cad::ui::WidgetCadTaskPanel::previewToggled,
            this, [this](bool enabled) {
        if (enabled && m_cadTaskPanel->isPrimitivePageActive())
            updateCadPrimitivePreview();
        else if (enabled && m_cadTaskPanel->isFeaturePageActive())
            updateCadFeaturePreview();
        else if (enabled && m_cadTaskPanel->isTransformPageActive())
            updateCadTransformPreview();
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

    updateCadTaskPanelState();

    // Wire machine panel signals to commands
        connect(m_machinePanel, &WidgetMachinePanel::machinePresetChanged, this,
            [this](const QString& presetName) {
            m_appContext->camModule()->configureMachine(presetName);
            });
    connect(m_machinePanel, &WidgetMachinePanel::machineModelPathChanged, this,
            [this](const QString& path) {
                m_appContext->camModule()->setMachineModelPath(path);
            });
    connect(m_machinePanel, &WidgetMachinePanel::loadMachineRequested, this,
            [this]{ m_cmdContainer->findCommand(CmdLoadMachine::Name)->execute(); });
        connect(m_machinePanel, &WidgetMachinePanel::compressMachineRequested, this,
            [this]{ m_cmdContainer->findCommand(CmdCompressMachine::Name)->execute(); });
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
        connect(m_machinePanel, &WidgetMachinePanel::workpieceInstallPositionChanged, this,
            [this](double x, double y, double z) {
            // spinbox 频繁回调走轻量路径：仅 SetLocation，不重建机台 view。
            m_appContext->camModule()->updateWorkpieceInstallLocation(gp_Pnt(x, y, z));
            });
        connect(m_machinePanel, &WidgetMachinePanel::alignWorkpieceRotationCenterRequested, this,
            [this]() {
            m_appContext->camModule()->alignWorkpieceInstallPositionToRotationCenter();
            });

    // New machine panel signals
    connect(m_machinePanel, &WidgetMachinePanel::unloadMachineRequested, this,
            [this]{ m_cmdContainer->findCommand(CmdUnloadMachine::Name)->execute(); });
    connect(m_machinePanel, &WidgetMachinePanel::exportMachineRequested, this,
            [this]{ m_cmdContainer->findCommand(CmdExportMachine::Name)->execute(); });

        connect(m_appContext->camModule(), &CamModule::toolpathGenerated, this,
            [this]() {
            m_occView->endLeadInPick();
            m_appContext->camModule()->cancelLeadInPreview();
            m_toolpathPanel->setToolpath(&m_appContext->camModule()->toolpathRef());
            rebuildProjectExplorer();
            selectProjectExplorerContourById(m_appContext->camModule()->contourIdAt(0), 0);
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
    connect(m_toolpathPanel, &WidgetToolpathPanel::pickLeadInRequested, this,
            [this]{ m_cmdContainer->findCommand(CmdSetLeadIn::Name)->execute(); });
    connect(m_toolpathPanel, &WidgetToolpathPanel::recalcRequested, this,
            [this]{ m_cmdContainer->findCommand(CmdRecalcToolpath::Name)->execute(); });
    connect(m_toolpathPanel, &WidgetToolpathPanel::previewToggled, this,
            [this](bool) { m_cmdContainer->findCommand(CmdToolpathPreview::Name)->execute(); });
    connect(m_toolpathPanel, &WidgetToolpathPanel::leadInLengthChanged, this,
            [this](double v) {
            m_appContext->camModule()->setLeadInLength(v);
            });
    connect(m_toolpathPanel, &WidgetToolpathPanel::normalAngleChanged, this,
            [this](double v) {
            m_appContext->camModule()->setNormalAngle(v);
            });
        connect(m_toolpathPanel, &WidgetToolpathPanel::discretizationIntervalChanged, this,
            [this](double v) { m_appContext->camModule()->setDeflection(v); });
    connect(m_toolpathPanel, &WidgetToolpathPanel::smoothAngleChanged, this,
            [this](double v) { m_appContext->camModule()->setSmoothAngle(v); });

        connect(m_toolpathPanel, &WidgetToolpathPanel::classificationModeChanged, this,
            [this](int mode) { m_appContext->camModule()->setUseFaceClassification(mode == 1); });

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
        connect(m_laserControl, &WidgetLaserControl::stopRequested,
            process, &ProcessModule::runStop);
        connect(m_laserControl, &WidgetLaserControl::eStopRequested,
            process, &ProcessModule::emergencyStop);
        connect(m_laserControl, &WidgetLaserControl::jogRequested, this,
            [process](const QString& axis, int direction, int speedLevel) {
            process->jog(axis, direction, speedLevel);
            });
        connect(m_laserControl, &WidgetLaserControl::homeRequested,
            process, &ProcessModule::home);
        connect(m_laserControl, &WidgetLaserControl::feedOverrideChanged,
            process, &ProcessModule::setFeedOverride);

        connect(process, &ProcessModule::connectionChanged,
            m_laserControl, &WidgetLaserControl::updateConnectionStatus);
        connect(process, &ProcessModule::simulationModeChanged,
            m_laserControl, &WidgetLaserControl::updateSimulationMode);
        connect(process, &ProcessModule::statusMessageChanged,
            m_laserControl, &WidgetLaserControl::updateSystemStatus);
        connect(process, &ProcessModule::axisPositionChanged, this,
            [this](const QString& axis, double value) {
            m_laserControl->updateAxisPosition(axis, value);
            m_appContext->camModule()->setAxisPosition(axis, value, false);

            if (m_machineRefreshTimer && !m_machineRefreshTimer->isActive())
                m_machineRefreshTimer->start();

            const auto positions = m_appContext->processModule()->currentAxisPositions();
            m_sbCoords->setText(
                tr("X: %1  Y: %2  Z: %3")
                .arg(positions.value(QStringLiteral("X"), 0.0), 0, 'f', 3)
                .arg(positions.value(QStringLiteral("Y"), 0.0), 0, 'f', 3)
                .arg(positions.value(QStringLiteral("Z"), 0.0), 0, 'f', 3));
            });

        m_laserControl->updateConnectionStatus(process->isConnected());
        m_laserControl->updateSimulationMode(process->simulationMode());
        m_laserControl->updateSystemStatus(process->statusMessage());
        const auto axisPositions = process->currentAxisPositions();
        for (auto it = axisPositions.cbegin(); it != axisPositions.cend(); ++it)
        m_laserControl->updateAxisPosition(it.key(), it.value());
}

void MainWindow::updateCadPrimitivePreview()
{
    if (!m_cadTaskPanel || !m_occView || !m_cadTaskPanel->isPrimitivePageActive()
        || !m_cadTaskPanel->isPreviewEnabled())
        return;

    const int primitiveIndex = m_cadTaskPanel->primitiveIndex();
    const QVariantMap params = primitiveParams(m_cadTaskPanel->primitiveSizeX(),
                                              m_cadTaskPanel->primitiveSizeY(),
                                              m_cadTaskPanel->primitiveSizeZ(),
                                              m_cadTaskPanel->primitiveRadius1(),
                                              m_cadTaskPanel->primitiveRadius2());

    TopoDS_Shape previewShape;
    QString errMsg;
        if (!m_appContext->cadModule()->previewTool(
            primitiveToolId(primitiveIndex), params, &previewShape, &errMsg)) {
        m_occView->clearCadPreview();
        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "MainWindow::updateCadPrimitivePreview skipped: {}",
                   errMsg.toStdString());
        return;
    }

    m_occView->setCadPreviewShape(previewShape);
}

void MainWindow::updateCadFeaturePreview()
{
    if (!m_cadTaskPanel || !m_occView || !m_cadTaskPanel->isFeaturePageActive()
        || !m_cadTaskPanel->isPreviewEnabled())
        return;

    TopoDS_Shape previewShape;
    QString errMsg;
        const int featureIndex = m_cadTaskPanel->featureIndex();
        if (!m_appContext->cadModule()->previewTool(
            featureToolId(featureIndex),
            featureParams(m_cadTaskPanel->featureLength(), m_cadTaskPanel->featureAngle()),
            &previewShape,
            &errMsg)) {
        m_occView->clearCadPreview();
        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "MainWindow::updateCadFeaturePreview skipped: {}",
                   errMsg.toStdString());
        return;
    }

    m_occView->setCadPreviewShape(previewShape);
}

void MainWindow::updateCadTransformPreview()
{
    if (!m_cadTaskPanel || !m_occView || !m_cadTaskPanel->isTransformPageActive())
        return;

    const CadModule::TransformParameters params = transformParams(
        m_cadTaskPanel->transformTranslateX(),
        m_cadTaskPanel->transformTranslateY(),
        m_cadTaskPanel->transformTranslateZ(),
        m_cadTaskPanel->transformRotateX(),
        m_cadTaskPanel->transformRotateY(),
        m_cadTaskPanel->transformRotateZ(),
        m_cadTaskPanel->transformReferenceMode());

    TopoDS_Shape previewShape;
    QString errMsg;
    double refX = 0.0;
    double refY = 0.0;
    double refZ = 0.0;
    if (!m_appContext->cadModule()->buildTransformPreview(
            params, &previewShape, &refX, &refY, &refZ, &errMsg)) {
        m_occView->clearCadPreview();
        m_occView->clearTransformGizmo();
        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "MainWindow::updateCadTransformPreview skipped: {}",
                   errMsg.toStdString());
        return;
    }

    m_occView->setTransformGizmo(refX, refY, refZ);
    if (m_cadTaskPanel->isPreviewEnabled())
        m_occView->setCadPreviewShape(previewShape);
    else
        m_occView->clearCadPreview();
}

void MainWindow::updateCadTaskPanelState()
{
    if (!m_cadTaskPanel || !m_appContext || !m_appContext->cadModule())
        return;

    CadModule* cad = m_appContext->cadModule();
    const DocumentId docId = cad->workpieceDocumentId();
    m_cadTaskPanel->setSelectionContext(cad->selectionContext(docId));
    if (m_cadTaskPanel->isTransformPageActive())
        updateCadTransformPreview();
}

void MainWindow::refreshSketchElementsView()
{
    if (!m_cadTaskPanel || !m_appContext || !m_appContext->cadModule())
        return;
    CadModule* cad = m_appContext->cadModule();
    QVector<lcnc::cad::ui::WidgetCadTaskPanel::SketchElementEntry> entries;
    if (cad->isSketchEditing()) {
        for (const auto& snap : cad->sketchElementSnapshots()) {
            lcnc::cad::ui::WidgetCadTaskPanel::SketchElementEntry entry;
            entry.id = snap.id;
            entry.kind = snap.kind;
            entry.label = snap.label;
            entries.append(entry);
        }
    }
    m_cadTaskPanel->setSketchElements(entries);
    m_cadTaskPanel->setActiveSketchTool(cad->sketchTool());
}

void MainWindow::refreshFinishedSketchesView()
{
    if (!m_cadTaskPanel || !m_appContext || !m_appContext->cadModule())
        return;
    CadModule* cad = m_appContext->cadModule();
    QVector<lcnc::cad::ui::WidgetCadTaskPanel::FinishedSketchEntry> entries;
    for (const auto& snap : cad->finishedSketchSnapshots()) {
        lcnc::cad::ui::WidgetCadTaskPanel::FinishedSketchEntry entry;
        entry.sketchId = snap.sketchId;
        entry.label = snap.name;
        entry.visible = snap.visible;
        entry.usedByFeature = snap.usedByFeature;
        entries.append(entry);
    }
    m_cadTaskPanel->setFinishedSketches(entries, cad->selectedSketchId());
}

void MainWindow::updateCadSketchOverlay()
{
    if (!m_occView || !m_appContext || !m_appContext->cadModule())
        return;

    const bool cadContextActive = m_rightStack && m_rightStack->currentWidget() == m_cadTaskPanel;
    if (!cadContextActive) {
        m_occView->clearSketchOverlay();
        return;
    }

    CadModule* cad = m_appContext->cadModule();
    QVector<lcnc::view::SketchOverlayItem> items;
    for (const auto& snap : cad->sketchOverlaySnapshots()) {
        lcnc::view::SketchOverlayItem item;
        item.key = snap.key;
        item.kind = snap.kind;
        item.plane = snap.plane;
        item.params = snap.params;
        item.visible = snap.visible;
        item.draggable = snap.draggable;
        if (snap.selected)
            item.color = QColor(255, 196, 40);
        else if (snap.activeSession)
            item.color = QColor(60, 220, 255);
        else if (snap.usedByFeature)
            item.color = QColor(120, 135, 145);
        else
            item.color = QColor(85, 210, 150);
        items.append(std::move(item));
    }
    m_occView->setSketchOverlayItems(items);
}

void MainWindow::handleCadSketchOverlayPicked(const QString& key)
{
    const bool cadContextActive = m_rightStack && m_rightStack->currentWidget() == m_cadTaskPanel;
    if (key.isEmpty() || !cadContextActive || !m_appContext || !m_appContext->cadModule())
        return;

    CadModule* cad = m_appContext->cadModule();
    auto context = lcnc::cad::selection::CadSelectionResolver::fromOverlayKey(
        cad->workpieceDocumentId(), key, true, cad->isSketchEditing());
    cad->setSelectionContext(context);
    updateCadSketchOverlay();
    updateCadTaskPanelState();
}

void MainWindow::handleCadSketchOverlayDrag(const QString& key, double deltaX, double deltaY)
{
    if (!isActiveSketchOverlayKey(key)
        || !m_appContext
        || !m_appContext->cadModule()
        || !m_rightStack
        || m_rightStack->currentWidget() != m_cadTaskPanel) {
        return;
    }

    const auto context = lcnc::cad::selection::CadSelectionResolver::fromOverlayKey(
        m_appContext->cadModule()->workpieceDocumentId(), key, true, true);
    if (context.items.isEmpty())
        return;

    QString errMsg;
    const auto item = context.items.first();
    const bool ok = item.sketchHandleIndex >= 0
        ? m_appContext->cadModule()->moveSketchElementHandle(
              item.sketchElementId, item.sketchHandleIndex, deltaX, deltaY, &errMsg)
        : m_appContext->cadModule()->moveSketchElement(item.sketchElementId, deltaX, deltaY, &errMsg);
    if (!ok) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "MainWindow: moveSketchElement failed: {}",
                  errMsg.toStdString());
    }
}

// ── Ribbon ─────────────────────────────────────────────────────────────────────
void MainWindow::createRibbon()
{
    SARibbonBar* ribbon = ribbonBar();
    ribbon->setRibbonStyle(SARibbonBar::RibbonStyleLooseThreeRow);

    buildFileTab(ribbon->addCategoryPage(tr("文件")));
    buildCadTab(ribbon->addCategoryPage(tr("CAD")));
    buildCamTab(ribbon->addCategoryPage(tr("CAM")));
    buildLaserTab(ribbon->addCategoryPage(tr("激光加工")));

    connect(ribbon, &SARibbonBar::currentRibbonTabChanged,
            this, &MainWindow::syncRightPanelForRibbonIndex);
    syncRightPanelForRibbonIndex(ribbon->currentIndex());
}

void MainWindow::buildFileTab(SARibbonCategory* cat)
{
    SARibbonPanel* panelDoc = cat->addPanel(tr("文档"));
    panelDoc->addLargeAction(m_cmdContainer->findAction(CmdNewDocument::Name));
    panelDoc->addLargeAction(m_cmdContainer->findAction(CmdOpenDocument::Name));
    panelDoc->addLargeAction(m_cmdContainer->findAction(CmdSaveDocument::Name));

    SARibbonPanel* panelIO = cat->addPanel(tr("导入/导出"));
    panelIO->addLargeAction(m_cmdContainer->findAction(CmdImportStep::Name));
    panelIO->addLargeAction(m_cmdContainer->findAction(CmdImportStl::Name));
    panelIO->addSmallAction(m_cmdContainer->findAction(CmdExportStep::Name));
    panelIO->addSmallAction(m_cmdContainer->findAction(CmdCloseDocument::Name));

    SARibbonPanel* panelView = cat->addPanel(tr("视图"));
    panelView->addLargeAction(m_cmdContainer->findAction(CmdFitAll::Name));

    // View orientation quick actions
    struct OrientInfo { QString label; QString key; V3d_TypeOfOrientation orient; };
    const QList<OrientInfo> orients = {
        { tr("正视"),   "1", V3d_Xpos              },
        { tr("俯视"),   "2", V3d_Zpos              },
        { tr("侧视"),   "3", V3d_Ypos              },
        { tr("等轴测"), "0", V3d_XposYnegZpos      },
    };
    for (auto& info : orients) {
        auto* act = new QAction(info.label + " [" + info.key + "]", this);
        connect(act, &QAction::triggered, m_occView,
                [this, o = info.orient]{ m_occView->setOrientation(o); });
        panelView->addSmallAction(act);
    }

    SARibbonPanel* panelDisplay = cat->addPanel(tr("显示"));
    panelDisplay->addSmallAction(m_cmdContainer->findAction(CmdToggleShaded::Name));
    panelDisplay->addSmallAction(m_cmdContainer->findAction(CmdToggleWireframe::Name));
    panelDisplay->addSmallAction(m_cmdContainer->findAction(CmdToggleShadedWithEdges::Name));
    panelDisplay->addLargeAction(m_cmdContainer->findAction(CmdToggleWorldAxes::Name));

    // ── 应用 — 选项按钮 ─────────────────────────────────────────────────
    SARibbonPanel* panelApp = cat->addPanel(tr("应用"));
    panelApp->addLargeAction(m_cmdContainer->findAction(CmdShowOptions::Name));
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
    m_sbDocName = new QLabel(tr("无文档"), this);
    m_sbCoords  = new QLabel("X: 0.000  Y: 0.000  Z: 0.000", this);
    const QString statusText = m_appContext
        ? m_appContext->processModule()->statusMessage()
        : tr("就绪");
    m_sbStatus  = new QLabel(statusText, this);

    m_sbDocName->setMinimumWidth(200);
    m_sbCoords->setMinimumWidth(280);

    statusBar()->addWidget(m_sbDocName);
    statusBar()->addPermanentWidget(m_sbCoords);
    statusBar()->addPermanentWidget(m_sbStatus);
}

// ── Slots ──────────────────────────────────────────────────────────────────────
void MainWindow::onProjectReset()
{
    if (m_appContext && m_appContext->camModule()) {
        if (auto* gd = m_appContext->camModule()->workspaceGuiDocument()) {
            gd->eraseDomain(lcnc::ProjectDomain::Workpiece);
            gd->eraseDomain(lcnc::ProjectDomain::Machine);
            gd->eraseDomain(lcnc::ProjectDomain::Cam);
        }
    }
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
        if (m_occView && m_appContext->camModule()->workspaceGuiDocument())
            m_occView->attachDocument(m_appContext->camModule()->workspaceGuiDocument());
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
    } else if (lcnc::app::isMachineProjectNode(kind)) {
        m_appContext->camModule()->requestMachineView();

        QStringList entries;
        for (QTreeWidgetItem* item : m_projectExplorerTree->selectedItems()) {
            const auto itemKind = projectNodeKind(item);
            if (itemKind == lcnc::app::ProjectExplorerNodeKind::MachineShape) {
                const QString selectedEntry = item->data(0, kRoleEntry).toString();
                if (!selectedEntry.isEmpty())
                    entries.append(selectedEntry);
            }
        }
        if (entries.isEmpty() && !entry.isEmpty())
            entries.append(entry);
        m_appContext->camModule()->setSelectedEntries(entries);
    } else if (lcnc::app::isToolpathProjectNode(kind)) {
        m_appContext->camModule()->requestMachineView();
        m_toolpathPanel->showContourCoordinates(contourIndex);
        highlightContourInView(contourIndex);
    }

    updateCommandStates();
}

void MainWindow::onProjectExplorerItemChanged(QTreeWidgetItem* item, int /*column*/)
{
    if (m_blockProjectExplorerSignals || !item)
        return;

    const auto kind = projectNodeKind(item);
    const bool visible = item->checkState(0) == Qt::Checked;

    auto cascadeCheckState = [this, visible](QTreeWidgetItem* root,
                                             const std::function<bool(QTreeWidgetItem*)>& shouldChange) {
        m_blockProjectExplorerSignals = true;
        QSignalBlocker blocker(m_projectExplorerTree);
        const bool updatesEnabled = m_projectExplorerTree->updatesEnabled();
        m_projectExplorerTree->setUpdatesEnabled(false);
        for (int index = 0; index < root->childCount(); ++index) {
            QTreeWidgetItem* child = root->child(index);
            std::function<void(QTreeWidgetItem*)> cascade = [&](QTreeWidgetItem* node) {
                if ((node->flags() & Qt::ItemIsUserCheckable) && shouldChange(node))
                    node->setCheckState(0, visible ? Qt::Checked : Qt::Unchecked);
                for (int childIndex = 0; childIndex < node->childCount(); ++childIndex)
                    cascade(node->child(childIndex));
            };
            cascade(child);
        }
        m_projectExplorerTree->setUpdatesEnabled(updatesEnabled);
        m_blockProjectExplorerSignals = false;
    };

    auto applyCadVisibility = [this, visible](QTreeWidgetItem* root) {
        QMap<DocumentId, QSet<QString>> entriesByDocument;
        QList<QPair<DocumentId, int>> sketches;

        std::function<void(QTreeWidgetItem*)> collect = [&](QTreeWidgetItem* node) {
            if (!node || !lcnc::app::isCadProjectNode(projectNodeKind(node)))
                return;

            const DocumentId nodeDocId = node->data(0, kRoleDocId).toInt();
            const QString nodeKey = node->data(0, kRoleNodeKey).toString();
            if (nodeDocId != kInvalidDocumentId
                && lcnc::cad::selection::CadSelectionResolver::isFinishedSketchNode(nodeKey)) {
                sketches.append({nodeDocId,
                    lcnc::cad::selection::CadSelectionResolver::sketchIdFromNodeKey(nodeKey)});
            }

            if (nodeDocId != kInvalidDocumentId) {
                const QStringList leafEntries = node->data(0, kRoleLeafEntries).toStringList();
                for (const QString& leafEntry : leafEntries) {
                    if (!leafEntry.isEmpty())
                        entriesByDocument[nodeDocId].insert(leafEntry);
                }
            }

            for (int childIndex = 0; childIndex < node->childCount(); ++childIndex)
                collect(node->child(childIndex));
        };

        collect(root);

        for (auto it = entriesByDocument.cbegin(); it != entriesByDocument.cend(); ++it) {
            QStringList entries;
            for (const QString& entry : it.value())
                entries.append(entry);
            m_appContext->cadModule()->setEntriesVisible(it.key(), entries, visible);
        }

        for (const auto& sketch : sketches)
            m_appContext->cadModule()->setSketchVisible(sketch.first, sketch.second, visible);
    };

    if (lcnc::app::isCadProjectNode(kind)) {
        const DocumentId docId = item->data(0, kRoleDocId).toInt();
        const QString entry = item->data(0, kRoleEntry).toString();
        const QString nodeKey = item->data(0, kRoleNodeKey).toString();
        const QStringList leafEntries = item->data(0, kRoleLeafEntries).toStringList();

        if (docId != kInvalidDocumentId
            && lcnc::cad::selection::CadSelectionResolver::isFinishedSketchNode(nodeKey)) {
            const int sketchId = lcnc::cad::selection::CadSelectionResolver::sketchIdFromNodeKey(nodeKey);
            m_appContext->cadModule()->setSketchVisible(docId, sketchId, visible);
            return;
        }

        if (entry.isEmpty()) {
            cascadeCheckState(item, [](QTreeWidgetItem* child) {
                return lcnc::app::isCadProjectNode(projectNodeKind(child));
            });
            applyCadVisibility(item);
            return;
        }

        if (docId == kInvalidDocumentId)
            return;

        m_appContext->cadModule()->setEntriesVisible(docId, leafEntries, visible);
        return;
    }

    if (lcnc::app::isMachineProjectNode(kind)) {
        const QString entry = item->data(0, kRoleEntry).toString();

        if (entry.isEmpty()) {
            cascadeCheckState(item, [](QTreeWidgetItem* child) {
                return lcnc::app::isMachineProjectNode(projectNodeKind(child));
            });

            std::function<void(QTreeWidgetItem*)> applyMachineVisibility = [&](QTreeWidgetItem* node) {
                const auto childKind = projectNodeKind(node);
                if (childKind == lcnc::app::ProjectExplorerNodeKind::MachineShape) {
                    const QString childEntry = node->data(0, kRoleEntry).toString();
                    if (!childEntry.isEmpty())
                        m_appContext->camModule()->setEntityVisible(childEntry, visible);
                }
                for (int childIndex = 0; childIndex < node->childCount(); ++childIndex)
                    applyMachineVisibility(node->child(childIndex));
            };
            for (int childIndex = 0; childIndex < item->childCount(); ++childIndex)
                applyMachineVisibility(item->child(childIndex));
            return;
        }

        m_appContext->camModule()->setEntityVisible(entry, visible);
        return;
    }

    if (kind == lcnc::app::ProjectExplorerNodeKind::ToolpathRoot) {
        cascadeCheckState(item, [](QTreeWidgetItem* child) {
            return lcnc::app::isToolpathProjectNode(projectNodeKind(child));
        });

        m_appContext->camModule()->setAllContoursEnabled(visible);
        return;
    }

    if (kind == lcnc::app::ProjectExplorerNodeKind::ToolpathLayer) {
        cascadeCheckState(item, [](QTreeWidgetItem* child) {
            return projectNodeKind(child) == lcnc::app::ProjectExplorerNodeKind::ToolpathContour;
        });
        const std::uint64_t layerId = item->data(0, kRoleLayerId).toULongLong();
        m_appContext->camModule()->setToolpathLayerEnabled(layerId, visible);
        return;
    }

    if (kind == lcnc::app::ProjectExplorerNodeKind::ToolpathContour) {
        const auto contourId = static_cast<lcnc::cam::ContourId>(item->data(0, kRoleContourId).toULongLong());
        int contourIndex = m_appContext->camModule()->contourIndexById(contourId);
        if (contourIndex < 0)
            contourIndex = item->data(0, kRoleContourIndex).toInt();
        m_appContext->camModule()->setContourEnabled(contourIndex, visible);
        if (item == m_projectExplorerTree->currentItem())
            highlightContourInView(contourIndex);
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
    dialog.setWindowTitle(tr("图层配置"));
    auto* layout = new QVBoxLayout(&dialog);
    auto* form = new QFormLayout();
    auto* nameEdit = new QLineEdit(sourceLayer->name, &dialog);
    auto* toolEdit = new QLineEdit(sourceLayer->toolName, &dialog);
    auto* colorButton = new QPushButton(&dialog);
    QColor selectedColor = sourceLayer->color.isValid() ? sourceLayer->color : QColor(80, 190, 150);

    auto refreshColorButton = [&]() {
        colorButton->setText(selectedColor.name(QColor::HexRgb).toUpper());
        colorButton->setStyleSheet(QStringLiteral("QPushButton { background: %1; color: %2; }")
            .arg(selectedColor.name(QColor::HexRgb), selectedColor.lightness() < 128 ? QStringLiteral("white") : QStringLiteral("black")));
    };
    refreshColorButton();

    connect(colorButton, &QPushButton::clicked, &dialog, [&]() {
        const QColor color = QColorDialog::getColor(selectedColor, &dialog, tr("选择图层颜色"));
        if (!color.isValid())
            return;
        selectedColor = color;
        refreshColorButton();
    });

    form->addRow(tr("名称"), nameEdit);
    form->addRow(tr("颜色"), colorButton);
    form->addRow(tr("工具"), toolEdit);
    layout->addLayout(form);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    layout->addWidget(buttons);

    if (dialog.exec() != QDialog::Accepted)
        return;

    if (cam->updateToolpathLayer(layerId, nameEdit->text(), selectedColor, toolEdit->text()))
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
    QString axisName = item->data(0, kRoleAxisName).toString();
    QString shapeEntry;

    if (kind == lcnc::app::ProjectExplorerNodeKind::MachineShape) {
        shapeEntry = item->data(0, kRoleEntry).toString();
        for (QTreeWidgetItem* parent = item->parent(); parent; parent = parent->parent()) {
            if (projectNodeKind(parent) == lcnc::app::ProjectExplorerNodeKind::MachineAxis) {
                axisName = parent->data(0, kRoleAxisName).toString();
                break;
            }
        }
    }

    if (axisName.isEmpty())
        return;

    QMenu menu(this);
    QAction* removeAction = nullptr;
    if (!shapeEntry.isEmpty())
        removeAction = menu.addAction(tr("删除所选节点"));
    QAction* clearAction = menu.addAction(tr("清除该轴系所有标记节点"));

    QAction* chosen = menu.exec(m_projectExplorerTree->viewport()->mapToGlobal(pos));
    if (!chosen)
        return;

    if (chosen == removeAction)
        m_appContext->camModule()->unassignShape(shapeEntry);
    else if (chosen == clearAction)
        m_appContext->camModule()->clearAxisAssignments(axisName);
}

void MainWindow::rebuildProjectExplorer()
{
    if (!m_projectExplorerTree)
        return;

    m_projectExplorerSnapshot = lcnc::app::ProjectExplorerModel::build(
        m_appContext->cadModule(),
        m_appContext->camModule());

    m_blockProjectExplorerSignals = true;
    QSignalBlocker blocker(m_projectExplorerTree);
    lcnc::app::populateProjectExplorerTree(m_projectExplorerTree, m_projectExplorerSnapshot);
    m_blockProjectExplorerSignals = false;

    if (!isMachineViewActive())
        m_appContext->cadModule()->syncSelectionFromView();
}

void MainWindow::handleProjectExplorerRowsMoved()
{
    if (m_blockProjectExplorerSignals || !m_projectExplorerTree)
        return;

    QTreeWidgetItem* toolpathRoot = nullptr;
    for (int index = 0; index < m_projectExplorerTree->topLevelItemCount(); ++index) {
        QTreeWidgetItem* item = m_projectExplorerTree->topLevelItem(index);
        if (projectNodeKind(item) == lcnc::app::ProjectExplorerNodeKind::ToolpathRoot) {
            toolpathRoot = item;
            break;
        }
    }
    if (!toolpathRoot)
        return;

    QList<int> order;
    QList<lcnc::cam::ContourId> idOrder;
    bool hasStableIds = true;
    int selectedRow = -1;
    lcnc::cam::ContourId selectedContourId = 0;
    std::function<void(QTreeWidgetItem*)> collectContours = [&](QTreeWidgetItem* node) {
        if (!node)
            return;
        if (projectNodeKind(node) == lcnc::app::ProjectExplorerNodeKind::ToolpathContour) {
            order.append(node->data(0, kRoleContourIndex).toInt());
            const auto contourId = static_cast<lcnc::cam::ContourId>(node->data(0, kRoleContourId).toULongLong());
            idOrder.append(contourId);
            hasStableIds = hasStableIds && contourId != 0;
            if (node == m_projectExplorerTree->currentItem())
                selectedRow = order.size() - 1;
            if (node == m_projectExplorerTree->currentItem())
                selectedContourId = contourId;
        }
        for (int index = 0; index < node->childCount(); ++index)
            collectContours(node->child(index));
    };
    collectContours(toolpathRoot);

    if (order.size() != m_appContext->camModule()->toolpath().contourCount()) {
        rebuildProjectExplorer();
        return;
    }

    if (hasStableIds)
        m_appContext->camModule()->reorderContoursById(idOrder);
    else
        m_appContext->camModule()->reorderContours(order);
    rebuildProjectExplorer();
    selectProjectExplorerContourById(selectedContourId, selectedRow >= 0 ? selectedRow : 0);
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
    if (!m_projectExplorerTree || (contourId == 0 && fallbackIndex < 0))
        return;

    QTreeWidgetItem* target = nullptr;
    QTreeWidgetItemIterator iterator(m_projectExplorerTree);
    while (*iterator) {
        const auto itemContourId =
            static_cast<lcnc::cam::ContourId>((*iterator)->data(0, kRoleContourId).toULongLong());
        if (projectNodeKind(*iterator) == lcnc::app::ProjectExplorerNodeKind::ToolpathContour
            && ((contourId != 0 && itemContourId == contourId)
                || (contourId == 0 && (*iterator)->data(0, kRoleContourIndex).toInt() == fallbackIndex))) {
            target = *iterator;
            break;
        }
        ++iterator;
    }

    if (!target)
        return;

    m_blockProjectExplorerSignals = true;
    QSignalBlocker blocker(m_projectExplorerTree);
    m_projectExplorerTree->clearSelection();
    m_projectExplorerTree->setCurrentItem(target);
    target->setSelected(true);
    m_projectExplorerTree->scrollToItem(target);
    m_blockProjectExplorerSignals = false;

    int contourIndex = m_appContext && m_appContext->camModule()
        ? m_appContext->camModule()->contourIndexById(contourId)
        : -1;
    if (contourIndex < 0)
        contourIndex = fallbackIndex;

    m_toolpathPanel->showContourCoordinates(contourIndex);
    highlightContourInView(contourIndex);
}

void MainWindow::selectProjectExplorerContours(const QList<int>& contourIndexes)
{
    if (!m_projectExplorerTree || contourIndexes.isEmpty())
        return;

    QSet<lcnc::cam::ContourId> contourIds;
    QSet<int> fallbackIndexes;
    for (int contourIndex : contourIndexes) {
        if (contourIndex < 0)
            continue;
        const auto contourId = m_appContext && m_appContext->camModule()
            ? m_appContext->camModule()->contourIdAt(contourIndex)
            : 0;
        if (contourId != 0)
            contourIds.insert(contourId);
        fallbackIndexes.insert(contourIndex);
    }

    if (contourIds.isEmpty() && fallbackIndexes.isEmpty())
        return;

    QTreeWidgetItem* firstSelected = nullptr;
    QTreeWidgetItem* lastSelected = nullptr;

    m_blockProjectExplorerSignals = true;
    QSignalBlocker blocker(m_projectExplorerTree);
    m_projectExplorerTree->clearSelection();

    QTreeWidgetItemIterator iterator(m_projectExplorerTree);
    while (*iterator) {
        if (projectNodeKind(*iterator) == lcnc::app::ProjectExplorerNodeKind::ToolpathContour) {
            const auto itemContourId =
                static_cast<lcnc::cam::ContourId>((*iterator)->data(0, kRoleContourId).toULongLong());
            const int itemIndex = (*iterator)->data(0, kRoleContourIndex).toInt();
            if ((itemContourId != 0 && contourIds.contains(itemContourId))
                || fallbackIndexes.contains(itemIndex)) {
                (*iterator)->setSelected(true);
                if (!firstSelected)
                    firstSelected = *iterator;
                lastSelected = *iterator;
            }
        }
        ++iterator;
    }

    if (lastSelected)
        m_projectExplorerTree->setCurrentItem(lastSelected);
    if (firstSelected)
        m_projectExplorerTree->scrollToItem(firstSelected);

    m_blockProjectExplorerSignals = false;

    if (lastSelected) {
        const auto contourId = static_cast<lcnc::cam::ContourId>(
            lastSelected->data(0, kRoleContourId).toULongLong());
        int contourIndex = m_appContext && m_appContext->camModule()
            ? m_appContext->camModule()->contourIndexById(contourId)
            : -1;
        if (contourIndex < 0)
            contourIndex = lastSelected->data(0, kRoleContourIndex).toInt();
        m_toolpathPanel->showContourCoordinates(contourIndex);
    }
}

void MainWindow::selectProjectExplorerEntries(DocumentId docId, const QStringList& entries, bool cadOnly)
{
    if (!m_projectExplorerTree)
        return;

    m_blockProjectExplorerSignals = true;
    QSignalBlocker blocker(m_projectExplorerTree);
    m_projectExplorerTree->clearSelection();

    if (!entries.isEmpty()) {
        QTreeWidgetItemIterator iterator(m_projectExplorerTree);
        while (*iterator) {
            const auto kind = projectNodeKind(*iterator);
            const bool kindMatches = cadOnly
                ? lcnc::app::isCadProjectNode(kind)
                : lcnc::app::isMachineProjectNode(kind);
            const bool docMatches = !cadOnly
                || docId == kInvalidDocumentId
                || (*iterator)->data(0, kRoleDocId).toInt() == docId;
            const QString entry = (*iterator)->data(0, kRoleEntry).toString();
            if (kindMatches && docMatches && !entry.isEmpty() && entries.contains(entry)) {
                (*iterator)->setSelected(true);
            }
            ++iterator;
        }
    }

    m_blockProjectExplorerSignals = false;
}

void MainWindow::highlightContourInView(int contourIndex)
{
    CamModule* cam = m_appContext ? m_appContext->camModule() : nullptr;
    GuiDocument* gd = cam ? cam->workspaceGuiDocument() : nullptr;
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
    if (!presetName.isEmpty())
        cam->configureMachine(presetName);

    m_machinePanel->setMachineModelPath(cam->machineModelPath());
    m_toolpathPanel->setLeadInLength(cam->leadInLength());
    m_toolpathPanel->setNormalAngle(cam->normalAngle());
    m_toolpathPanel->setDiscretizationInterval(cam->deflection());
    m_toolpathPanel->setSmoothAngle(cam->smoothAngle());
    m_toolpathPanel->setUseFaceClassification(cam->useFaceClassification());
    m_toolpathPanel->setShowNormals(cam->showNormals());
    m_toolpathPanel->setNormalSampleStep(cam->normalSampleStep());

    const QString machinePath = cam->machineModelPath();
    if (!machinePath.isEmpty() && QFileInfo::exists(machinePath))
        cam->loadMachine(machinePath);
}

void MainWindow::syncMachineWorkspaceUi()
{
    syncMachineWorkspaceUiInternal(true);
}

void MainWindow::syncMachineWorkspaceUiInternal(bool rebuildTree)
{
    ProcessModule* process = m_appContext->processModule();
    CamModule* cam = m_appContext->camModule();
    LcncDocument* machineDoc = m_appContext->camModule()->machineDocument();
    if (!machineDoc) {
        if (rebuildTree)
            rebuildProjectExplorer();
        m_machinePanel->setDocument(nullptr);
        m_machinePanel->setMachineModelPath(cam->machineModelPath());
        if (process)
            process->setAxisDefinitions({});
        if (m_laserControl)
            m_laserControl->setAxisDefinitions({});
        return;
    }

    if (rebuildTree)
        rebuildProjectExplorer();

    m_machinePanel->setDocument(machineDoc);
    m_machinePanel->setMachineModelPath(cam->machineModelPath());

    const QList<MachineAxisDef> axes = machineDoc->machineKinematics()->axes();
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

// ── View routing helpers ────────────────────────────────────────────────────────────────
void MainWindow::showMachineView()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "MainWindow::showMachineView");
    m_machineWorkspaceActive = true;
    if (auto* gd = m_appContext->camModule()->workspaceGuiDocument())
        m_occView->attachDocument(gd);
    else
        m_occView->attachDefaultScene(m_defaultScene);
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

    if (auto* gd = m_appContext->camModule()->workspaceGuiDocument())
        m_occView->attachDocument(gd);
    else
        m_occView->attachDefaultScene(m_defaultScene);
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
        m_rightStack->setCurrentWidget(m_machinePanel);
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
    if (m_occView)
        m_occView->attachDefaultScene(nullptr);

    // Accept; Qt's parent-child destructor chain cleans up all OCC resources.
    e->accept();
}
