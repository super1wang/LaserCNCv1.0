#include "app/mainwindow.h"
#include "app/app_context.h"
#include "app/commands_api.h"
#include "app/commands_file.h"
#include "app/commands_edit.h"
#include "app/commands_display.h"
#include "app/commands_cad.h"
#include "app/commands_machine.h"
#include "app/commands_cam.h"
#include "app/widget_occ_view.h"
#include "app/widget_model_tree.h"
#include "app/widget_machine_panel.h"
#include "app/widget_toolpath_panel.h"
#include "app/widget_laser_control.h"
#include "app/dialog_task_manager.h"
#include "base/cam_config.h"
#include "base/laser_toolpath.h"
#include "base/lcnc_application.h"
#include "base/lcnc_document.h"
#include "base/machine_kinematics.h"
#include "base/xcaf_utils.h"
#include "graphics/graphics_scene.h"
#include "gui/gui_application.h"
#include "gui/gui_document.h"
#include "modules/cad_module.h"
#include "modules/cam_module.h"
#include "modules/process_module.h"

#include <SARibbonBar.h>
#include <SARibbonCategory.h>
#include <SARibbonPanel.h>

#include <QSplitter>
#include <QTabWidget>
#include <QStackedWidget>
#include <QTreeWidget>
#include <QTreeWidgetItemIterator>
#include <QHeaderView>
#include <QStatusBar>
#include <QLabel>

#include <QVBoxLayout>
#include <QComboBox>
#include <QCloseEvent>
#include <QMessageBox>
#include <QInputDialog>
#include <QLineEdit>
#include <QApplication>
#include <QFileInfo>
#include <QIcon>
#include <QTimer>
#include <QStyle>
#include <QSignalBlocker>
#include <functional>
#include <V3d_TypeOfOrientation.hxx>
#include <TDF_LabelSequence.hxx>
#include <XCAFDoc_ShapeTool.hxx>

// ─────────────────────────────────────────────────────────────────────────────

namespace {
constexpr int kRoleDocId = Qt::UserRole + 1;
constexpr int kRoleEntry = Qt::UserRole + 2;
constexpr int kRoleNodeKey = Qt::UserRole + 3;
constexpr int kRoleLeafEntries = Qt::UserRole + 4;

bool usesMachineWorkspace(int tabIndex)
{
    return tabIndex >= 1;
}
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

    // Wire document lifecycle signals
    LcncApplication* lcnc = LcncApplication::instance();
    connect(lcnc, &LcncApplication::documentAdded,
            this, &MainWindow::onDocumentAdded);
    connect(lcnc, &LcncApplication::documentClosed,
            this, &MainWindow::onDocumentClosed);
    connect(lcnc, &LcncApplication::activeDocumentChanged,
            this, &MainWindow::onActiveDocumentChanged);
        connect(lcnc, &LcncApplication::documentModified,
            this, &MainWindow::onDocumentModified);

    restorePersistedCamState();

    updateCommandStates();
}

MainWindow::~MainWindow() = default;

// ── Initialization ─────────────────────────────────────────────────────────────
void MainWindow::createContext()
{
    m_appContext = new AppContext(this, this);

    // Ensure GUI application singleton is alive
    GuiApplication::instance();
    // Task manager
    TaskManager::instance();
    // Create the permanent machine workspace document.
    // Must be called AFTER GuiApplication::instance() so the documentAdded
    // signal is received and a GuiDocument is created for the machine doc.
    LcncApplication::instance()->ensureMachineDocument();

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
    m_cmdContainer = new CommandContainer(m_appContext, this);

    // File
    m_cmdContainer->addCommand<CmdNewDocument>(CmdNewDocument::Name);
    m_cmdContainer->addCommand<CmdOpenDocument>(CmdOpenDocument::Name);
    m_cmdContainer->addCommand<CmdSaveDocument>(CmdSaveDocument::Name);
    m_cmdContainer->addCommand<CmdSaveDocumentAs>(CmdSaveDocumentAs::Name);
    m_cmdContainer->addCommand<CmdImportStep>(CmdImportStep::Name);
    m_cmdContainer->addCommand<CmdImportStl>(CmdImportStl::Name);
    m_cmdContainer->addCommand<CmdExportStep>(CmdExportStep::Name);
    m_cmdContainer->addCommand<CmdCloseDocument>(CmdCloseDocument::Name);

    // Edit
    m_cmdContainer->addCommand<CmdUndo>(CmdUndo::Name);
    m_cmdContainer->addCommand<CmdRedo>(CmdRedo::Name);

    // CAD — Primitives
    m_cmdContainer->addCommand<CmdCreateBox>(CmdCreateBox::Name);
    m_cmdContainer->addCommand<CmdCreateCylinder>(CmdCreateCylinder::Name);
    m_cmdContainer->addCommand<CmdCreateSphere>(CmdCreateSphere::Name);
    m_cmdContainer->addCommand<CmdCreateCone>(CmdCreateCone::Name);
    m_cmdContainer->addCommand<CmdCreateTorus>(CmdCreateTorus::Name);

    // CAD — Transforms
    m_cmdContainer->addCommand<CmdMoveShape>(CmdMoveShape::Name);
    m_cmdContainer->addCommand<CmdRotateShape>(CmdRotateShape::Name);

    // CAD — Boolean
    m_cmdContainer->addCommand<CmdBoolUnion>(CmdBoolUnion::Name);
    m_cmdContainer->addCommand<CmdBoolCut>(CmdBoolCut::Name);
    m_cmdContainer->addCommand<CmdBoolCommon>(CmdBoolCommon::Name);

    // CAD — Measurement
    m_cmdContainer->addCommand<CmdMeasureDistance>(CmdMeasureDistance::Name);
    m_cmdContainer->addCommand<CmdMeasureAngle>(CmdMeasureAngle::Name);
    m_cmdContainer->addCommand<CmdMeasureArea>(CmdMeasureArea::Name);

    // CAD — Delete
    m_cmdContainer->addCommand<CmdDeleteShape>(CmdDeleteShape::Name);
    m_cmdContainer->addCommand<CmdExplodeShape>(CmdExplodeShape::Name);

    // Machine commands
    m_cmdContainer->addCommand<CmdLoadMachine>(CmdLoadMachine::Name);
    m_cmdContainer->addCommand<CmdCompressMachine>(CmdCompressMachine::Name);
    m_cmdContainer->addCommand<CmdMountWorkpiece>(CmdMountWorkpiece::Name);
    m_cmdContainer->addCommand<CmdUnloadMachine>(CmdUnloadMachine::Name);
    m_cmdContainer->addCommand<CmdExportMachine>(CmdExportMachine::Name);

    // CAM commands
    m_cmdContainer->addCommand<CmdGenerateToolpath>(CmdGenerateToolpath::Name);
    m_cmdContainer->addCommand<CmdSetLeadIn>(CmdSetLeadIn::Name);
    m_cmdContainer->addCommand<CmdToolpathPreview>(CmdToolpathPreview::Name);
    m_cmdContainer->addCommand<CmdRecalcToolpath>(CmdRecalcToolpath::Name);
    m_cmdContainer->addCommand<CmdSimulate>(CmdSimulate::Name);

    // Display — view orientation
    auto addOrient = [this](const QString& name,
                             V3d_TypeOfOrientation o,
                             const QString& label,
                             const QIcon& icon) {
        auto* cmd = new CmdViewOrient(m_appContext, o, label, icon);
        cmd->setParent(m_cmdContainer);
        // register by connecting action to fitAll/orient in OccView
        connect(cmd->action(), &QAction::triggered, m_occView,
                [this, o]{ m_occView->setOrientation(o); });
    };

    m_cmdContainer->addCommand<CmdFitAll>(CmdFitAll::Name);
    // Wire fit-all directly
    connect(m_cmdContainer->findAction(CmdFitAll::Name), &QAction::triggered,
            m_occView, &WidgetOccView::fitAll);

    m_cmdContainer->addCommand<CmdToggleShaded>(CmdToggleShaded::Name);
    m_cmdContainer->addCommand<CmdToggleWireframe>(CmdToggleWireframe::Name);
    m_cmdContainer->addCommand<CmdToggleShadedWithEdges>(CmdToggleShadedWithEdges::Name);

    // Wire display mode actions to OccView
    connect(m_cmdContainer->findAction(CmdToggleShaded::Name),
            &QAction::triggered, m_occView, [this]{ m_occView->setDisplayMode(1); });
    connect(m_cmdContainer->findAction(CmdToggleWireframe::Name),
            &QAction::triggered, m_occView, [this]{ m_occView->setDisplayMode(0); });
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

        connect(m_appContext->cadModule(), &CadModule::documentTreeChanged,
            this, &MainWindow::rebuildDocumentTree);
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
            m_appContext->cadModule()->activeDocumentId());
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
                if (m_pendingCalibrationTarget == QStringLiteral("A")
                    || m_pendingCalibrationTarget == QStringLiteral("C")) {
                    handled = m_appContext->camModule()->fillAxisOriginFromReferenceFace(
                        m_occView, pos, m_pendingCalibrationTarget);
                } else if (m_pendingCalibrationTarget == QStringLiteral("CUTTER_HEAD")) {
                    handled = m_appContext->camModule()->setCutterHeadModelPositionFromReferenceFace(
                        m_occView, pos);
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
                m_pendingCalibrationTarget.clear();
                m_machinePanel->setCalibrationPickAxis(QString());
            });

    connect(m_appContext->camModule(), &CamModule::selectionChanged, this,
            [this](const QStringList& entries) {
                m_modelTree->highlightEntries(entries);
                m_machinePanel->setSelectedEntries(entries);
            });

    connect(m_appContext->cadModule(), &CadModule::selectionChanged, this,
            [this](DocumentId docId, const QStringList& entries) {
                if (!m_documentTree)
                    return;

                QSignalBlocker blocker(m_documentTree);
                m_documentTree->clearSelection();
                QTreeWidgetItemIterator it(m_documentTree);
                while (*it) {
                    const DocumentId itemDocId = (*it)->data(0, kRoleDocId).toInt();
                    const QString entry = (*it)->data(0, kRoleEntry).toString();
                    if (itemDocId == docId && !entry.isEmpty() && entries.contains(entry)) {
                        (*it)->setSelected(true);
                        for (QTreeWidgetItem* p = (*it)->parent(); p; p = p->parent())
                            p->setExpanded(true);
                    }
                    ++it;
                }
            });

    // Tree selection → module selection and machine panel context
    connect(m_modelTree, &WidgetModelTree::selectionChanged, this,
            [this](const QStringList& entries) {
                m_appContext->camModule()->setSelectedEntries(entries);
            });

    // Checkbox visibility toggle → CAM module owns machine-side visibility
    connect(m_modelTree, &WidgetModelTree::visibilityChanged, this,
            [this](const QString& entry, bool visible) {
                m_appContext->camModule()->setEntityVisible(entry, visible);
            });

    rebuildDocumentTree();
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
    m_leftTabs->setMinimumWidth(220);
    m_leftTabs->setMaximumWidth(350);

    // ── 文档 tab (all opened files + assembly tree) ──────────────────────
    m_documentTree = new QTreeWidget(m_leftTabs);
    m_documentTree->setHeaderLabel(tr("文档结构"));
    m_documentTree->setColumnCount(1);
    m_documentTree->header()->setVisible(false);
    m_documentTree->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_leftTabs->addTab(m_documentTree, tr("文档"));

    // ── 准备 tab ───────────────────────────────────────────────────────────
    m_modelTree = new WidgetModelTree(m_leftTabs);
    m_leftTabs->addTab(m_modelTree, tr("准备"));

    // ── 刀路 tab ───────────────────────────────────────────────────────────
    m_contourListWidget = new QTreeWidget(m_leftTabs);
    m_contourListWidget->setHeaderLabels({tr("名称"), tr("点数")});
    m_contourListWidget->setColumnCount(2);
    m_contourListWidget->header()->setStretchLastSection(false);
    m_contourListWidget->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    m_contourListWidget->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    m_contourListWidget->setDragDropMode(QAbstractItemView::InternalMove);
    m_contourListWidget->setDefaultDropAction(Qt::MoveAction);
    m_contourListWidget->setSelectionMode(QAbstractItemView::SingleSelection);
    m_contourListWidget->setRootIsDecorated(false);
    m_contourListWidget->setItemsExpandable(false);
    m_contourListWidget->setExpandsOnDoubleClick(false);
    m_contourListWidget->setDropIndicatorShown(true);
    m_leftTabs->addTab(m_contourListWidget, tr("刀路"));

    // ── 执行 tab ───────────────────────────────────────────────────────────
    m_processTree = new QTreeWidget(m_leftTabs);
    m_processTree->setHeaderLabel(tr("加工流程"));
    m_processTree->setColumnCount(2);
    m_processTree->setHeaderLabels({tr("步骤"), tr("信息")});
    m_processTree->header()->setStretchLastSection(true);
    // Placeholder items
    auto* startItem = new QTreeWidgetItem(m_processTree);
    startItem->setText(0, tr("开始"));
    startItem->setBackground(0, QColor(60, 160, 60));
    startItem->setForeground(0, Qt::white);
    auto* stopItem = new QTreeWidgetItem(m_processTree);
    stopItem->setText(0, tr("结束"));
    stopItem->setBackground(0, QColor(160, 60, 60));
    stopItem->setForeground(0, Qt::white);

    m_leftTabs->addTab(m_processTree, tr("执行"));

    connect(m_leftTabs, &QTabWidget::currentChanged,
            this, &MainWindow::onLeftTabChanged);
    connect(m_documentTree, &QTreeWidget::itemClicked,
            this, &MainWindow::onDocumentTreeItemClicked);

    // Document tree checkbox visibility toggle
    connect(m_documentTree, &QTreeWidget::itemChanged, this,
            [this](QTreeWidgetItem* item, int /*column*/) {
                if (!item) return;
                const DocumentId docId = item->data(0, kRoleDocId).toInt();
                const QString entry = item->data(0, kRoleEntry).toString();
                const QStringList leafEntries = item->data(0, kRoleLeafEntries).toStringList();
                if (docId == kInvalidDocumentId) return;
                const bool visible = (item->checkState(0) == Qt::Checked);

                // Group / doc item: cascade then apply to all leaves
                if (entry.isEmpty()) {
                    QSignalBlocker sb(m_documentTree);
                    std::function<void(QTreeWidgetItem*)> cascade = [&](QTreeWidgetItem* node) {
                        for (int i = 0; i < node->childCount(); ++i) {
                            QTreeWidgetItem* child = node->child(i);
                            if (child->flags() & Qt::ItemIsUserCheckable)
                                child->setCheckState(0, visible ? Qt::Checked : Qt::Unchecked);
                            cascade(child);
                        }
                    };
                    cascade(item);
                    m_appContext->cadModule()->setEntriesVisible(docId, leafEntries, visible);
                    return;
                }

                // Leaf item
                m_appContext->cadModule()->setEntriesVisible(docId, leafEntries, visible);
            });

}

void MainWindow::createRightPanel()
{
    m_machinePanel   = new WidgetMachinePanel(this);
    m_toolpathPanel  = new WidgetToolpathPanel(this);
    m_laserControl   = new WidgetLaserControl(this);

    CamModule* cam = m_appContext->camModule();
    m_machinePanel->setMachineModelPath(cam->machineModelPath());
    m_machinePanel->setMachineRenderQuality(cam->machineRenderQuality());
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
    m_rightStack->addWidget(m_machinePanel);   // index 0 — shown when "准备"
    m_rightStack->addWidget(m_toolpathPanel);  // index 1 — shown when CAM tab
    m_rightStack->addWidget(m_laserControl);   // index 2 — shown when "执行"
    m_rightStack->setMinimumWidth(320);
    m_rightStack->setMaximumWidth(420);
    m_rightStack->setCurrentIndex(0);

    // Wire machine panel signals to commands
        connect(m_machinePanel, &WidgetMachinePanel::machinePresetChanged, this,
            [this](const QString& presetName) {
            m_appContext->camModule()->configureMachine(presetName);
            });
    connect(m_machinePanel, &WidgetMachinePanel::machineModelPathChanged, this,
            [this](const QString& path) {
                m_appContext->camModule()->setMachineModelPath(path);
            });
    connect(m_machinePanel, &WidgetMachinePanel::machineRenderQualityChanged, this,
            [this](MachineRenderQuality quality) {
                m_appContext->camModule()->setMachineRenderQuality(quality);
            });
    connect(m_machinePanel, &WidgetMachinePanel::loadMachineRequested, this,
            [this]{ m_cmdContainer->findCommand(CmdLoadMachine::Name)->execute(); });
        connect(m_machinePanel, &WidgetMachinePanel::compressMachineRequested, this,
            [this]{ m_cmdContainer->findCommand(CmdCompressMachine::Name)->execute(); });
    connect(m_machinePanel, &WidgetMachinePanel::mountWorkpieceRequested, this,
            [this]{ m_cmdContainer->findCommand(CmdMountWorkpiece::Name)->execute(); });
        connect(m_machinePanel, &WidgetMachinePanel::axisOriginChanged, this,
            [this](const QString& axisName, double x, double y, double z) {
            m_appContext->camModule()->setAxisOrigin(axisName, gp_Pnt(x, y, z));
            });
        connect(m_machinePanel, &WidgetMachinePanel::calibrationFacePickRequested, this,
            [this](const QString& targetName) {
                m_appContext->camModule()->requestMachineView();
            m_pendingCalibrationTarget = targetName;
            m_machinePanel->setCalibrationPickAxis(targetName);
                if (!m_occView->isFacePickActive())
                    m_occView->beginFacePick();
            });
    connect(m_machinePanel, &WidgetMachinePanel::alignToPhysicalCenterRequested, this,
            [this](double x, double y, double z) {
                if (m_occView->isFacePickActive()) {
                    m_occView->endFacePick();
                    m_pendingCalibrationTarget.clear();
                    m_machinePanel->setCalibrationPickAxis(QString());
                }

                m_appContext->camModule()->requestMachineView();
                m_appContext->camModule()->alignMachineToPhysicalCenter(gp_Pnt(x, y, z));
            });
    connect(m_machinePanel, &WidgetMachinePanel::cutterHeadModelPositionChanged, this,
            [this](double x, double y, double z) {
                m_appContext->camModule()->setCutterHeadModelPosition(gp_Pnt(x, y, z));
            });
    connect(m_machinePanel, &WidgetMachinePanel::cutterHeadPhysicalPositionChanged, this,
            [this](double x, double y, double z) {
                m_appContext->camModule()->setCutterHeadPhysicalPosition(gp_Pnt(x, y, z));
            });
    connect(m_machinePanel, &WidgetMachinePanel::alignToPhysicalCutterHeadRequested, this,
            [this]() {
                m_appContext->camModule()->requestMachineView();
                m_appContext->camModule()->alignMachineToPhysicalCutterHead();
            });
        connect(m_machinePanel, &WidgetMachinePanel::workpieceInstallPositionChanged, this,
            [this](double x, double y, double z) {
            m_appContext->camModule()->setWorkpieceInstallPosition(gp_Pnt(x, y, z));
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
            rebuildContourListWidget();
            });
        connect(m_appContext->camModule(), &CamModule::toolpathCleared, this,
            [this]() {
            m_occView->endLeadInPick();
            m_appContext->camModule()->cancelLeadInPreview();
            m_toolpathPanel->setToolpath(nullptr);
            rebuildContourListWidget();
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

    // ── Left contour list (刀路 tab) ────────────────────────────────────
    connect(m_contourListWidget, &QTreeWidget::itemChanged, this,
            [this](QTreeWidgetItem* item, int /*column*/) {
                if (!item) return;
                int row = m_contourListWidget->indexOfTopLevelItem(item);
                if (row < 0) return;
                bool checked = (item->checkState(0) == Qt::Checked);
                m_appContext->camModule()->setContourEnabled(row, checked);
            });

    // Reorder contours when drag-drop finishes
    connect(m_contourListWidget->model(), &QAbstractItemModel::rowsMoved, this,
            [this]() {
                struct ContourRowState {
                    QString name;
                    QString pointCount;
                    Qt::CheckState checkState{Qt::Unchecked};
                    int originalIndex{-1};
                    QString toolTip;
                };

                QList<ContourRowState> rows;
                rows.reserve(m_appContext->camModule()->toolpath().contourCount());

                const QTreeWidgetItem* currentItem = m_contourListWidget->currentItem();
                int currentRow = -1;
                QTreeWidgetItemIterator it(m_contourListWidget);
                while (*it) {
                    QTreeWidgetItem* item = *it;
                    ContourRowState row;
                    row.name = item->text(0);
                    row.pointCount = item->text(1);
                    row.checkState = item->checkState(0);
                    row.originalIndex = item->data(0, Qt::UserRole).toInt();
                    row.toolTip = item->toolTip(0);
                    if (item == currentItem)
                        currentRow = rows.size();
                    rows.append(row);
                    ++it;
                }

                if (rows.isEmpty())
                    return;

                {
                    QSignalBlocker blocker(m_contourListWidget);
                    m_contourListWidget->clear();
                    for (int i = 0; i < rows.size(); ++i) {
                        const auto& row = rows.at(i);
                        auto* item = new QTreeWidgetItem(m_contourListWidget);
                        item->setText(0, row.name);
                        item->setText(1, row.pointCount);
                        item->setFlags((item->flags() | Qt::ItemIsUserCheckable | Qt::ItemIsDragEnabled)
                                       & ~Qt::ItemIsDropEnabled);
                        item->setCheckState(0, row.checkState);
                        item->setData(0, Qt::UserRole, row.originalIndex);
                        if (!row.toolTip.isEmpty())
                            item->setToolTip(0, row.toolTip);
                    }
                    if (currentRow >= 0 && currentRow < m_contourListWidget->topLevelItemCount())
                        m_contourListWidget->setCurrentItem(m_contourListWidget->topLevelItem(currentRow));
                }

                QList<int> order;
                order.reserve(rows.size());
                for (const auto& row : rows)
                    order.append(row.originalIndex);

                m_appContext->camModule()->reorderContours(order);

                QSignalBlocker blocker(m_contourListWidget);
                for (int i = 0; i < m_contourListWidget->topLevelItemCount(); ++i) {
                    auto* item = m_contourListWidget->topLevelItem(i);
                    if (item)
                        item->setData(0, Qt::UserRole, i);
                }

                m_toolpathPanel->showContourCoordinates(currentRow);
            });

    // When user clicks a contour in the left list, show its coordinates in the right panel
    connect(m_contourListWidget, &QTreeWidget::currentItemChanged, this,
            [this](QTreeWidgetItem* current, QTreeWidgetItem* /*previous*/) {
                if (!current) return;
                int idx = current->data(0, Qt::UserRole).toInt();
                m_toolpathPanel->showContourCoordinates(idx);
            });

        // ── Process module ↔ execution page ─────────────────────────────────
        ProcessModule* process = m_appContext->processModule();
        connect(m_laserControl, &WidgetLaserControl::startRequested,
            process, &ProcessModule::start);
        connect(m_laserControl, &WidgetLaserControl::pauseRequested,
            process, &ProcessModule::pause);
        connect(m_laserControl, &WidgetLaserControl::stopRequested,
            process, &ProcessModule::stop);
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

// ── Ribbon ─────────────────────────────────────────────────────────────────────
void MainWindow::createRibbon()
{
    SARibbonBar* ribbon = ribbonBar();
    ribbon->setRibbonStyle(SARibbonBar::RibbonStyleLooseThreeRow);

    buildFileTab(ribbon->addCategoryPage(tr("文件")));
    buildCadTab(ribbon->addCategoryPage(tr("CAD")));
    buildCamTab(ribbon->addCategoryPage(tr("CAM")));
    buildLaserTab(ribbon->addCategoryPage(tr("激光加工")));
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
}

void MainWindow::buildCadTab(SARibbonCategory* cat)
{
    // Helper for unimplemented placeholder actions (sketch etc.)
    auto makeAct = [this](const QString& label, const QString& iconPath) -> QAction* {
        auto* a = new QAction(QIcon(iconPath), label, this);
        a->setStatusTip(tr("创建 ") + label);
        return a;
    };

    // ── 历史 ─────────────────────────────────────────────────────────────
    SARibbonPanel* panelHist = cat->addPanel(tr("历史"));
    panelHist->addLargeAction(m_cmdContainer->findAction(CmdUndo::Name));
    panelHist->addLargeAction(m_cmdContainer->findAction(CmdRedo::Name));

    // ── 基本体 ─────────────────────────────────────────────────────────────
    SARibbonPanel* panelPrim = cat->addPanel(tr("基本体"));
    panelPrim->addLargeAction(m_cmdContainer->findAction(CmdCreateBox::Name));
    panelPrim->addLargeAction(m_cmdContainer->findAction(CmdCreateCylinder::Name));
    panelPrim->addLargeAction(m_cmdContainer->findAction(CmdCreateSphere::Name));
    panelPrim->addSmallAction(m_cmdContainer->findAction(CmdCreateCone::Name));
    panelPrim->addSmallAction(m_cmdContainer->findAction(CmdCreateTorus::Name));

    // ── 操作 ───────────────────────────────────────────────────────────────
    SARibbonPanel* panelOps = cat->addPanel(tr("操作"));
    panelOps->addLargeAction(m_cmdContainer->findAction(CmdMoveShape::Name));
    panelOps->addLargeAction(m_cmdContainer->findAction(CmdRotateShape::Name));
    panelOps->addSmallAction(makeAct(tr("缩放"),   ":/icons/scale.svg"));
    panelOps->addSmallAction(m_cmdContainer->findAction(CmdBoolUnion::Name));
    panelOps->addSmallAction(m_cmdContainer->findAction(CmdBoolCut::Name));
    panelOps->addSmallAction(m_cmdContainer->findAction(CmdBoolCommon::Name));    panelOps->addSmallAction(m_cmdContainer->findAction(CmdDeleteShape::Name));
    panelOps->addSmallAction(m_cmdContainer->findAction(CmdExplodeShape::Name));
    // ── 测量 ───────────────────────────────────────────────────────────────
    SARibbonPanel* panelMeas = cat->addPanel(tr("测量"));
    panelMeas->addLargeAction(m_cmdContainer->findAction(CmdMeasureDistance::Name));
    panelMeas->addSmallAction(m_cmdContainer->findAction(CmdMeasureAngle::Name));
    panelMeas->addSmallAction(m_cmdContainer->findAction(CmdMeasureArea::Name));

    // ── 草图 (预留) ────────────────────────────────────────────────────────
    SARibbonPanel* panelSketch = cat->addPanel(tr("草图"));
    panelSketch->addLargeAction(makeAct(tr("新建草图"), ":/icons/sketch.svg"));
    panelSketch->addSmallAction(makeAct(tr("直线"),     ":/icons/line.svg"));
    panelSketch->addSmallAction(makeAct(tr("圆"),       ":/icons/circle.svg"));
    panelSketch->addSmallAction(makeAct(tr("圆弧"),     ":/icons/arc.svg"));
    panelSketch->addSmallAction(makeAct(tr("退出草图"), ":/icons/exit_sketch.svg"));
}

void MainWindow::buildCamTab(SARibbonCategory* cat)
{
    auto makeAct = [this](const QString& label, const QString& icon) -> QAction* {
        return new QAction(QIcon(icon), label, this);
    };

    // ── 机台 ───────────────────────────────────────────────────────────────
    SARibbonPanel* panelMach = cat->addPanel(tr("机台"));
    panelMach->addLargeAction(m_cmdContainer->findAction(CmdLoadMachine::Name));
    panelMach->addSmallAction(m_cmdContainer->findAction(CmdCompressMachine::Name));
    panelMach->addSmallAction(m_cmdContainer->findAction(CmdMountWorkpiece::Name));
    panelMach->addSmallAction(m_cmdContainer->findAction(CmdUnloadMachine::Name));
    panelMach->addSmallAction(m_cmdContainer->findAction(CmdExportMachine::Name));

    // ── 刀路 ───────────────────────────────────────────────────────────────
    SARibbonPanel* panelPath = cat->addPanel(tr("刀路"));
    panelPath->addLargeAction(m_cmdContainer->findAction(CmdGenerateToolpath::Name));
    panelPath->addSmallAction(m_cmdContainer->findAction(CmdSetLeadIn::Name));
    panelPath->addSmallAction(m_cmdContainer->findAction(CmdRecalcToolpath::Name));
    panelPath->addSmallAction(m_cmdContainer->findAction(CmdToolpathPreview::Name));

    // ── G代码 ─────────────────────────────────────────────────────────────
    SARibbonPanel* panelNC = cat->addPanel(tr("G代码"));
    panelNC->addLargeAction(makeAct(tr("生成G代码"), ":/icons/gcode.svg"));
    panelNC->addSmallAction(makeAct(tr("导入G代码"), ":/icons/import.svg"));
    panelNC->addSmallAction(makeAct(tr("导出G代码"), ":/icons/export.svg"));
    panelNC->addSmallAction(makeAct(tr("代码查看"),  ":/icons/code.svg"));

    // ── 仿真 ─────────────────────────────────────────────────────────────
    SARibbonPanel* panelSim = cat->addPanel(tr("仿真"));
    panelSim->addLargeAction(m_cmdContainer->findAction(CmdSimulate::Name));

    auto* actPause = makeAct(tr("暂停"), ":/icons/pause.svg");
    connect(actPause, &QAction::triggered, this, [this] {
        auto* cmd = static_cast<CmdSimulate*>(m_cmdContainer->findCommand(CmdSimulate::Name));
        cmd->pause();
    });
    panelSim->addSmallAction(actPause);

    auto* actStop = makeAct(tr("停止"), ":/icons/stop.svg");
    connect(actStop, &QAction::triggered, this, [this] {
        auto* cmd = static_cast<CmdSimulate*>(m_cmdContainer->findCommand(CmdSimulate::Name));
        cmd->stop();
    });
    panelSim->addSmallAction(actStop);

    // Speed combo (0.5x, 1x, 2x, 5x, 10x)
    auto* speedCombo = new QComboBox(this);
    speedCombo->addItem(tr("0.5x"), 0.5);
    speedCombo->addItem(tr("1x"),   1.0);
    speedCombo->addItem(tr("2x"),   2.0);
    speedCombo->addItem(tr("5x"),   5.0);
    speedCombo->addItem(tr("10x"),  10.0);
    speedCombo->setCurrentIndex(1);  // default 1x
    connect(speedCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            [this, speedCombo](int idx) {
                double factor = speedCombo->itemData(idx).toDouble();
                auto* cmd = static_cast<CmdSimulate*>(m_cmdContainer->findCommand(CmdSimulate::Name));
                cmd->setSpeed(factor);
            });
    panelSim->addSmallWidget(speedCombo);
}

void MainWindow::buildLaserTab(SARibbonCategory* cat)
{
    auto makeAct = [this](const QString& label, const QString& icon) -> QAction* {
        return new QAction(QIcon(icon), label, this);
    };

    // ── 连接 ───────────────────────────────────────────────────────────────
    SARibbonPanel* panelConn = cat->addPanel(tr("连接"));
    QAction* actConnect = makeAct(tr("连接控制器"), ":/icons/connect.svg");
    QAction* actSimulation = makeAct(tr("仿真模式"), ":/icons/simulate.svg");
    QAction* actDisconnect = makeAct(tr("断开连接"), ":/icons/disconnect.svg");
    actSimulation->setCheckable(true);
    actSimulation->setChecked(m_appContext->processModule()->simulationMode());
    panelConn->addLargeAction(actConnect);
    panelConn->addLargeAction(actSimulation);
    panelConn->addSmallAction(actDisconnect);

    connect(actConnect, &QAction::triggered, this, [this] {
        bool ok = false;
        const QString endpoint = QInputDialog::getText(
            this,
            tr("连接控制器"),
            tr("控制器地址:"),
            QLineEdit::Normal,
            QStringLiteral("tcp://127.0.0.1:5000"),
            &ok);
        if (!ok)
            return;

        m_appContext->processModule()->connectController(endpoint);
    });
    connect(actDisconnect, &QAction::triggered,
            m_appContext->processModule(), &ProcessModule::disconnectController);
    connect(actSimulation, &QAction::toggled,
            m_appContext->processModule(), &ProcessModule::setSimulationMode);
    connect(m_appContext->processModule(), &ProcessModule::simulationModeChanged,
            actSimulation, &QAction::setChecked);

    // ── 流程 ───────────────────────────────────────────────────────────────
    SARibbonPanel* panelProc = cat->addPanel(tr("流程"));
    panelProc->addLargeAction(makeAct(tr("新建流程"), ":/icons/new_process.svg"));
    panelProc->addLargeAction(makeAct(tr("加载流程"), ":/icons/open_process.svg"));
    panelProc->addSmallAction(makeAct(tr("保存流程"), ":/icons/save_process.svg"));

    // ── 运行 ───────────────────────────────────────────────────────────────
    SARibbonPanel* panelRun = cat->addPanel(tr("运行"));
    auto* actStart = makeAct(tr("运行"), ":/icons/start.svg");
    auto* actPause = makeAct(tr("暂停"), ":/icons/pause.svg");
    auto* actStop  = makeAct(tr("停止"), ":/icons/stop.svg");
    actStart->setShortcut(QKeySequence(Qt::Key_F5));
    panelRun->addLargeAction(actStart);
    panelRun->addSmallAction(actPause);
    panelRun->addSmallAction(actStop);

        connect(actStart, &QAction::triggered,
            m_appContext->processModule(), &ProcessModule::start);
        connect(actPause, &QAction::triggered,
            m_appContext->processModule(), &ProcessModule::pause);
        connect(actStop,  &QAction::triggered,
            m_appContext->processModule(), &ProcessModule::stop);

    // ── 参数 ───────────────────────────────────────────────────────────────
    SARibbonPanel* panelParam = cat->addPanel(tr("参数"));
    panelParam->addSmallAction(makeAct(tr("激光参数"), ":/icons/laser_param.svg"));
    panelParam->addSmallAction(makeAct(tr("运动参数"), ":/icons/motion_param.svg"));
    panelParam->addSmallAction(makeAct(tr("加工设置"), ":/icons/process_param.svg"));
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
void MainWindow::onLeftTabChanged(int index)
{
    // index 0 = 文档 / 1 = 准备 / 2 = 刀路 / 3 = 执行
    // Right stack: 0=machine, 1=toolpath, 2=laser
    if (index == 3)
        m_rightStack->setCurrentIndex(2);   // 执行 → laser control
    else if (index == 2)
        m_rightStack->setCurrentIndex(1);   // 刀路 → toolpath params
    else if (index == 1)
        m_rightStack->setCurrentIndex(0);   // 准备 → machine panel
    else
        m_rightStack->setCurrentIndex(1);   // 文档 → toolpath panel

    if (usesMachineWorkspace(index))
        m_appContext->camModule()->requestMachineView();
    else
        m_appContext->cadModule()->requestWorkpieceView();

    updateCommandStates();
}

void MainWindow::onDocumentAdded(DocumentId id)
{
    // Machine workspace document is managed via showMachineView(); skip normal flow.
    if (LcncApplication::instance()->isMachineDocument(id)) {
        if (isMachineViewActive())
            m_appContext->camModule()->requestMachineView();
        updateCommandStates();
        return;
    }

    LcncDocument* doc = m_appContext->cadModule()->documentById(id);
    if (doc)
        m_sbDocName->setText(doc->name());

    if (!isMachineViewActive())
        m_appContext->cadModule()->requestWorkpieceView(id);

    updateCommandStates();
}

void MainWindow::onDocumentClosed(DocumentId /*id*/)
{
    updateCommandStates();
}

void MainWindow::onActiveDocumentChanged(DocumentId id)
{
    LcncDocument* doc = m_appContext->cadModule()->documentById(id);
    if (doc) {
        m_sbDocName->setText(doc->name());
        if (!isMachineViewActive())
            m_appContext->cadModule()->requestWorkpieceView(id);
    } else {
        m_sbDocName->setText(tr("无文档"));
        if (!isMachineViewActive())
            m_occView->attachDefaultScene(m_defaultScene);
    }

    updateCommandStates();
}

void MainWindow::onDocumentModified(DocumentId id)
{
    Q_UNUSED(id);
    updateCommandStates();
}

void MainWindow::onDocumentTreeItemClicked(QTreeWidgetItem* item, int /*column*/)
{
    if (!item) return;

    // IMPORTANT: Read ALL data from item BEFORE any operation that may
    // trigger rebuildDocumentTree() and invalidate the item pointer.
    const DocumentId docId  = item->data(0, kRoleDocId).toInt();
    const QStringList leafEntries = item->data(0, kRoleLeafEntries).toStringList();

    if (docId == kInvalidDocumentId) return;

    // Switch document AFTER reading item data.
    // setActiveDocument emits activeDocumentChanged → onActiveDocumentChanged
    // → rebuildDocumentTree() → m_documentTree->clear() which deletes 'item'.
    if (m_appContext->cadModule()->activeDocumentId() != docId)
        m_appContext->cadModule()->setActiveDocument(docId);

    if (leafEntries.isEmpty()) return;
    m_appContext->cadModule()->setSelectedEntries(docId, leafEntries);
}

void MainWindow::rebuildDocumentTree()
{
    if (!m_documentTree) return;
    QSignalBlocker blocker(m_documentTree);
    m_documentTree->clear();

    const auto docs = m_appContext->cadModule()->documentTreeDocuments();
    std::function<void(QTreeWidgetItem*, DocumentId, const CadModule::DocumentTreeNode&)> addNode;
    addNode = [this, &addNode](QTreeWidgetItem* parent,
                               DocumentId docId,
                               const CadModule::DocumentTreeNode& node) {
        auto* item = new QTreeWidgetItem(parent);
        item->setText(0, node.displayName);
        item->setData(0, kRoleDocId, docId);
        item->setData(0, kRoleNodeKey, node.nodeKey);
        item->setData(0, kRoleEntry, node.entry);
        item->setData(0, kRoleLeafEntries, node.leafEntries);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(0, Qt::Checked);

        if (node.entry.isEmpty())
            item->setIcon(0, QIcon(":/icons/machine.svg"));
        else
            item->setIcon(0, QIcon(":/icons/shape.svg"));

        for (const auto& childNode : node.children)
            addNode(item, docId, childNode);
    };

    for (const auto& doc : docs) {
        auto* docItem = new QTreeWidgetItem(m_documentTree);
        docItem->setText(0, doc.displayName);
        docItem->setIcon(0, QIcon(":/icons/new_doc.svg"));
        docItem->setData(0, kRoleDocId, doc.documentId);
        docItem->setData(0, kRoleNodeKey, doc.nodeKey);
        docItem->setData(0, kRoleEntry, QString());
        docItem->setData(0, kRoleLeafEntries, doc.leafEntries);
        docItem->setFlags(docItem->flags() | Qt::ItemIsUserCheckable);
        docItem->setCheckState(0, Qt::Checked);

        for (const auto& node : doc.children)
            addNode(docItem, doc.documentId, node);
    }

    m_documentTree->expandToDepth(2);

    if (!isMachineViewActive())
        m_appContext->cadModule()->syncSelectionFromView();
}

void MainWindow::rebuildContourListWidget()
{
    if (!m_contourListWidget) return;

    const int previousIndex = m_contourListWidget->currentItem()
        ? m_contourListWidget->currentItem()->data(0, Qt::UserRole).toInt()
        : -1;

    QSignalBlocker blocker(m_contourListWidget);
    m_contourListWidget->clear();

    const auto& tp = m_appContext->camModule()->toolpath();
    int selectedRow = -1;
    for (int i = 0; i < tp.contourCount(); ++i) {
        const LaserContour& c = tp.contour(i);
        auto* item = new QTreeWidgetItem(m_contourListWidget);
        item->setText(0, c.name);
        item->setText(1, QString::number(c.points.size()));
        item->setFlags((item->flags() | Qt::ItemIsUserCheckable | Qt::ItemIsDragEnabled)
                       & ~Qt::ItemIsDropEnabled);
        item->setCheckState(0, c.enabled ? Qt::Checked : Qt::Unchecked);
        item->setData(0, Qt::UserRole, i);
        if (!c.sourceInfo.isEmpty())
            item->setToolTip(0, c.sourceInfo);
        if (i == previousIndex)
            selectedRow = i;
    }

    if (selectedRow < 0 && m_contourListWidget->topLevelItemCount() > 0)
        selectedRow = 0;

    if (selectedRow >= 0)
        m_contourListWidget->setCurrentItem(m_contourListWidget->topLevelItem(selectedRow));

    m_toolpathPanel->showContourCoordinates(selectedRow);
}

void MainWindow::restorePersistedCamState()
{
    CamModule* cam = m_appContext->camModule();
    if (!cam)
        return;

    const CamConfig& config = CamConfig::instance();
    const QString presetName = config.machinePreset();
    if (!presetName.isEmpty())
        cam->configureMachine(presetName);

    m_machinePanel->setMachineModelPath(cam->machineModelPath());
    m_machinePanel->setMachineRenderQuality(cam->machineRenderQuality());
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
        m_modelTree->clear();
        m_machinePanel->setDocument(nullptr);
        m_machinePanel->setMachineModelPath(cam->machineModelPath());
        m_machinePanel->setMachineRenderQuality(cam->machineRenderQuality());
        if (process)
            process->setAxisDefinitions({});
        if (m_laserControl)
            m_laserControl->setAxisDefinitions({});
        return;
    }

    if (rebuildTree || m_modelTree->currentDocument() != machineDoc)
        m_modelTree->rebuildForDocument(machineDoc);

    m_machinePanel->setDocument(machineDoc);
    m_machinePanel->setMachineModelPath(cam->machineModelPath());
    m_machinePanel->setMachineRenderQuality(cam->machineRenderQuality());

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
    if (auto* gd = m_appContext->camModule()->machineGuiDocument())
        m_occView->attachDocument(gd);
    else
        m_occView->attachDefaultScene(m_defaultScene);

    syncMachineWorkspaceUiInternal(false);
}

void MainWindow::showWorkpieceView(DocumentId id)
{
    if (id == kInvalidDocumentId)
        id = m_appContext->cadModule()->activeDocumentId();
    if (auto* gd = m_appContext->cadModule()->guiDocument(id))
        m_occView->attachDocument(gd);
    else
        m_occView->attachDefaultScene(m_defaultScene);
}

void MainWindow::updateCommandStates()
{
    m_cmdContainer->updateAllStates();
}

bool MainWindow::isMachineViewActive() const
{
    return m_leftTabs && usesMachineWorkspace(m_leftTabs->currentIndex());
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
