#include "app/mainwindow.h"
#include "app/app_context.h"
#include "app/commands_api.h"
#include "app/commands_file.h"
#include "app/commands_edit.h"
#include "app/commands_display.h"
#include "app/commands_cad.h"
#include "app/commands_machine.h"
#include "app/widget_occ_view.h"
#include "app/widget_model_tree.h"
#include "app/widget_machine_panel.h"
#include "app/widget_laser_control.h"
#include "app/dialog_task_manager.h"
#include "base/lcnc_application.h"
#include "base/lcnc_document.h"
#include "base/machine_kinematics.h"
#include "base/xcaf_utils.h"
#include "graphics/graphics_scene.h"
#include "gui/gui_application.h"
#include "gui/gui_document.h"

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
#include <QCloseEvent>
#include <QMessageBox>
#include <QApplication>
#include <QIcon>
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

    // Machine commands
    m_cmdContainer->addCommand<CmdLoadMachine>(CmdLoadMachine::Name);
    m_cmdContainer->addCommand<CmdMarkAxes>(CmdMarkAxes::Name);
    m_cmdContainer->addCommand<CmdMountWorkpiece>(CmdMountWorkpiece::Name);
    m_cmdContainer->addCommand<CmdUnloadMachine>(CmdUnloadMachine::Name);
    m_cmdContainer->addCommand<CmdExportMachine>(CmdExportMachine::Name);

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

    // ── 3D selection → tree highlight + machine panel ─────────────────────
    connect(m_occView, &WidgetOccView::selectionChanged, this, [this] {
        // Use machine doc when on 准备 tab, otherwise use active workpiece doc
        DocumentId id = (m_leftTabs && m_leftTabs->currentIndex() == 1)
            ? LcncApplication::instance()->machineDocumentId()
            : LcncApplication::instance()->activeDocumentId();
        if (auto* gd = GuiApplication::instance()->guiDocument(id)) {
            const QStringList entries = gd->selectedEntries();
            m_modelTree->highlightEntries(entries);
            m_machinePanel->setSelectedEntries(entries);
            // Highlight matching nodes in the 文档 tab tree
            if (m_documentTree && !entries.isEmpty()) {
                QSignalBlocker blocker(m_documentTree);
                m_documentTree->clearSelection();
                QTreeWidgetItemIterator it(m_documentTree);
                while (*it) {
                    const QString e = (*it)->data(0, kRoleEntry).toString();
                    if (!e.isEmpty() && entries.contains(e)) {
                        (*it)->setSelected(true);
                        for (QTreeWidgetItem* p = (*it)->parent(); p; p = p->parent())
                            p->setExpanded(true);
                    }
                    ++it;
                }
            }
        }
    });

    // Tree selection → machine panel mark buttons context
    connect(m_modelTree, &WidgetModelTree::selectionChanged, this,
            [this](const QStringList& entries) {
                m_machinePanel->setSelectedEntries(entries);
            });

    // Axis assignment in panel → notify machine document modified
    connect(m_machinePanel, &WidgetMachinePanel::axisAssignmentChanged, this, [this] {
        DocumentId machId = LcncApplication::instance()->machineDocumentId();
        if (machId != kInvalidDocumentId)
            LcncApplication::instance()->notifyDocumentModified(machId);
    });

    // Axis node removed via right-click context menu → refresh tree + panel
    connect(m_modelTree, &WidgetModelTree::axisNodeUnassigned, this, [this] {
        DocumentId machId = LcncApplication::instance()->machineDocumentId();
        if (machId != kInvalidDocumentId)
            LcncApplication::instance()->notifyDocumentModified(machId);
    });
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
}

void MainWindow::createRightPanel()
{
    m_machinePanel  = new WidgetMachinePanel(this);
    m_laserControl  = new WidgetLaserControl(this);

    m_rightStack = new QStackedWidget(this);
    m_rightStack->addWidget(m_machinePanel);   // index 0 — shown when "准备"
    m_rightStack->addWidget(m_laserControl);   // index 1 — shown when "执行"
    m_rightStack->setMinimumWidth(260);
    m_rightStack->setMaximumWidth(350);
    m_rightStack->setCurrentIndex(0);

    // Wire machine panel signals to commands
    connect(m_machinePanel, &WidgetMachinePanel::loadMachineRequested, this,
            [this]{ m_cmdContainer->findCommand(CmdLoadMachine::Name)->execute(); });
    connect(m_machinePanel, &WidgetMachinePanel::markAxesRequested, this,
            [this]{ m_cmdContainer->findCommand(CmdMarkAxes::Name)->execute(); });
    connect(m_machinePanel, &WidgetMachinePanel::mountWorkpieceRequested, this,
            [this]{ m_cmdContainer->findCommand(CmdMountWorkpiece::Name)->execute(); });

    // Axis position spinbox → kinematics + 3D update on machine document
    connect(m_machinePanel, &WidgetMachinePanel::axisPositionChanged, this,
            [this](const QString& axisName, double value) {
                LcncDocument* doc = LcncApplication::instance()->machineDocument();
                if (!doc) return;
                doc->machineKinematics()->setAxisPosition(axisName, value);
                DocumentId machId = LcncApplication::instance()->machineDocumentId();
                if (auto* gd = GuiApplication::instance()->guiDocument(machId))
                    gd->updateAxisTransforms();
            });

    // New machine panel signals
    connect(m_machinePanel, &WidgetMachinePanel::unloadMachineRequested, this,
            [this]{ m_cmdContainer->findCommand(CmdUnloadMachine::Name)->execute(); });
    connect(m_machinePanel, &WidgetMachinePanel::exportMachineRequested, this,
            [this]{ m_cmdContainer->findCommand(CmdExportMachine::Name)->execute(); });
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
    panelMach->addLargeAction(m_cmdContainer->findAction(CmdMarkAxes::Name));
    panelMach->addSmallAction(m_cmdContainer->findAction(CmdMountWorkpiece::Name));    panelMach->addSmallAction(m_cmdContainer->findAction(CmdUnloadMachine::Name));
    panelMach->addSmallAction(m_cmdContainer->findAction(CmdExportMachine::Name));    panelMach->addSmallAction(makeAct(tr("坐标系"), ":/icons/coordinate.svg"));

    // ── 刀路 ───────────────────────────────────────────────────────────────
    SARibbonPanel* panelPath = cat->addPanel(tr("刀路"));
    panelPath->addLargeAction(makeAct(tr("生成刀路"), ":/icons/toolpath.svg"));
    panelPath->addSmallAction(makeAct(tr("3+2 定位"), ":/icons/toolpath_32.svg"));
    panelPath->addSmallAction(makeAct(tr("5轴联动"),  ":/icons/toolpath_5x.svg"));
    panelPath->addSmallAction(makeAct(tr("刀路预览"), ":/icons/preview.svg"));

    // ── G代码 ─────────────────────────────────────────────────────────────
    SARibbonPanel* panelNC = cat->addPanel(tr("G代码"));
    panelNC->addLargeAction(makeAct(tr("生成G代码"), ":/icons/gcode.svg"));
    panelNC->addSmallAction(makeAct(tr("导入G代码"), ":/icons/import.svg"));
    panelNC->addSmallAction(makeAct(tr("导出G代码"), ":/icons/export.svg"));
    panelNC->addSmallAction(makeAct(tr("代码查看"),  ":/icons/code.svg"));
}

void MainWindow::buildLaserTab(SARibbonCategory* cat)
{
    auto makeAct = [this](const QString& label, const QString& icon) -> QAction* {
        return new QAction(QIcon(icon), label, this);
    };

    // ── 连接 ───────────────────────────────────────────────────────────────
    SARibbonPanel* panelConn = cat->addPanel(tr("连接"));
    panelConn->addLargeAction(makeAct(tr("连接控制器"), ":/icons/connect.svg"));
    panelConn->addLargeAction(makeAct(tr("仿真模式"),   ":/icons/simulate.svg"));
    panelConn->addSmallAction(makeAct(tr("断开连接"),   ":/icons/disconnect.svg"));

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

    // Wire to laser control widget
    connect(actStart, &QAction::triggered, m_laserControl, &WidgetLaserControl::startRequested);
    connect(actPause, &QAction::triggered, m_laserControl, &WidgetLaserControl::pauseRequested);
    connect(actStop,  &QAction::triggered, m_laserControl, &WidgetLaserControl::stopRequested);

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
    m_sbStatus  = new QLabel(tr("就绪"), this);

    m_sbDocName->setMinimumWidth(200);
    m_sbCoords->setMinimumWidth(280);

    statusBar()->addWidget(m_sbDocName);
    statusBar()->addPermanentWidget(m_sbCoords);
    statusBar()->addPermanentWidget(m_sbStatus);
}

// ── Slots ──────────────────────────────────────────────────────────────────────
void MainWindow::onLeftTabChanged(int index)
{
    // index 0 = 文档 / 1 = 准备 → show machine panel  /  index 2 = 执行 → laser
    m_rightStack->setCurrentIndex(index == 2 ? 1 : 0);
    if (index == 1)
        showMachineView();
    else
        showWorkpieceView();
}

void MainWindow::onDocumentAdded(DocumentId id)
{
    // Machine workspace document is managed via showMachineView(); skip normal flow.
    if (LcncApplication::instance()->isMachineDocument(id)) {
        if (m_leftTabs && m_leftTabs->currentIndex() == 1)
            showMachineView();
        updateCommandStates();
        return;
    }
    LcncDocument* doc = LcncApplication::instance()->documentById(id);
    if (doc) {
        m_sbDocName->setText(doc->name());
        // Only switch view if not on 准备 tab (which always shows machine view)
        if (!m_leftTabs || m_leftTabs->currentIndex() != 1) {
            if (auto* gd = GuiApplication::instance()->guiDocument(id))
                m_occView->attachDocument(gd);
        }
    }
    rebuildDocumentTree();
    updateCommandStates();
}

void MainWindow::onDocumentClosed(DocumentId /*id*/)
{
    // The "no active document" detach has already been done in
    // onActiveDocumentChanged when activeDocumentChanged(kInvalid) fired
    // (which precedes this signal).  Just refresh the tree and command states.
    rebuildDocumentTree();
    updateCommandStates();
}

void MainWindow::onActiveDocumentChanged(DocumentId id)
{
    LcncDocument* doc = LcncApplication::instance()->documentById(id);
    if (doc) {
        m_sbDocName->setText(doc->name());
        // Only switch view if not on 准备 tab (which always shows machine view)
        if (!m_leftTabs || m_leftTabs->currentIndex() != 1) {
            if (auto* gd = GuiApplication::instance()->guiDocument(id))
                m_occView->attachDocument(gd);
        }
    } else {
        // No active document — detach to default scene NOW, BEFORE the
        // subsequent documentClosed signal causes GuiApplication to delete
        // the old GuiDocument.  Without this, m_activeDoc would become a
        // dangling pointer by the time onDocumentClosed runs.
        m_sbDocName->setText(tr("无文档"));
        if (!m_leftTabs || m_leftTabs->currentIndex() != 1)
            m_occView->attachDefaultScene(m_defaultScene);
    }
    // Model tree and machine panel always reflect the machine workspace
    LcncDocument* machDoc = LcncApplication::instance()->machineDocument();
    if (machDoc) {
        m_modelTree->rebuildForDocument(machDoc);
        m_machinePanel->setDocument(machDoc);
    } else {
        m_modelTree->clear();
        m_machinePanel->setDocument(nullptr);
    }
    rebuildDocumentTree();
    updateCommandStates();
}

void MainWindow::onDocumentModified(DocumentId id)
{
    // Machine document modification: update model tree and machine panel
    if (LcncApplication::instance()->isMachineDocument(id)) {
        if (LcncDocument* doc = LcncApplication::instance()->machineDocument()) {
            m_modelTree->rebuildForDocument(doc);
            m_machinePanel->setDocument(doc);
        }
        return;
    }
    rebuildDocumentTree();
}

void MainWindow::onDocumentTreeItemClicked(QTreeWidgetItem* item, int /*column*/)
{
    if (!item) return;

    // IMPORTANT: Read ALL data from item BEFORE any operation that may
    // trigger rebuildDocumentTree() and invalidate the item pointer.
    const DocumentId docId  = item->data(0, kRoleDocId).toInt();
    const QString    entry  = item->data(0, kRoleEntry).toString();

    if (docId == kInvalidDocumentId) return;

    // Switch document AFTER reading item data.
    // setActiveDocument emits activeDocumentChanged → onActiveDocumentChanged
    // → rebuildDocumentTree() → m_documentTree->clear() which deletes 'item'.
    if (LcncApplication::instance()->activeDocumentId() != docId)
        LcncApplication::instance()->setActiveDocument(docId);

    // Safely get GUI document (scene may have been switched above)
    GuiDocument* gd = GuiApplication::instance()->guiDocument(docId);
    if (!gd) return;

    Handle(AIS_InteractiveContext) ctx = m_occView->context();
    if (ctx.IsNull()) return;

    Handle(V3d_View) view = m_occView->view();
    if (view.IsNull()) return;

    if (entry.isEmpty()) {
        // Group node: cascade-select all descendant leaf entries in 3D.
        // Collect leaf entries by recursively walking the tree item children.
        std::function<void(QTreeWidgetItem*, QStringList&)> collectLeaves;
        collectLeaves = [&](QTreeWidgetItem* node, QStringList& out) {
            for (int i = 0; i < node->childCount(); ++i) {
                QTreeWidgetItem* ch = node->child(i);
                const QString e = ch->data(0, kRoleEntry).toString();
                if (!e.isEmpty())
                    out << e;
                else
                    collectLeaves(ch, out);
            }
        };
        QStringList leaves;
        collectLeaves(item, leaves);
        if (leaves.isEmpty()) return;
        try {
            ctx->ClearSelected(false);
            for (const QString& e : leaves) {
                Handle(AIS_Shape) ais = gd->aisShape(e);
                if (!ais.IsNull())
                    ctx->AddOrRemoveSelected(ais, false);
            }
            view->Redraw();
        } catch (const std::exception& ex) {
            qWarning() << "Exception during group selection:" << ex.what();
        }
        return;
    }

    // Leaf node: select the single shape.
    // The aisMap stores only root/free shapes; sub-component labels are not
    // individually tracked, so gracefully skip if not found.
    Handle(AIS_Shape) ais = gd->aisShape(entry);
    if (ais.IsNull()) return;

    try {
        ctx->ClearSelected(false);
        ctx->SetSelected(ais, true);
        view->Redraw();
    } catch (const std::exception& e) {
        qWarning() << "Exception during selection:" << e.what();
    }
}

void MainWindow::rebuildDocumentTree()
{
    if (!m_documentTree) return;
    QSignalBlocker blocker(m_documentTree);
    m_documentTree->clear();

    const QList<LcncDocument*> docs = LcncApplication::instance()->documents();
    for (LcncDocument* doc : docs) {
        if (!doc) continue;

        // Machine workspace is managed via the "准备" tab; skip it in the doc tree.
        if (LcncApplication::instance()->isMachineDocument(doc->id())) continue;

        auto* docItem = new QTreeWidgetItem(m_documentTree);
        docItem->setText(0, doc->name());
        docItem->setIcon(0, QIcon(":/icons/new_doc.svg"));
        docItem->setData(0, kRoleDocId, doc->id());
        docItem->setData(0, kRoleEntry, QString());

        const auto& workpieceTree = doc->entityTree(LcncDocument::EntityKind::Workpiece);

        if (!workpieceTree.isEmpty()) {
            // ── Workpiece hierarchy from STEP/IGES import ───────────────
            std::function<void(QTreeWidgetItem*, const LcncDocument::ShapeTreeNode&)> addNode;
            addNode = [&](QTreeWidgetItem* parent, const LcncDocument::ShapeTreeNode& node) {
                auto* item = new QTreeWidgetItem(parent);
                item->setText(0, node.displayName.isEmpty() ? node.entry : node.displayName);
                item->setData(0, kRoleDocId, doc->id());
                item->setData(0, kRoleEntry, node.entry);
                if (node.entry.isEmpty()) {
                    // Virtual assembly/group node: use folder icon, keep selectable
                    item->setIcon(0, QIcon(":/icons/machine.svg"));
                } else {
                    item->setIcon(0, QIcon(":/icons/shape.svg"));
                }
                for (const auto& child : node.children)
                    addNode(item, child);
            };
            for (const auto& node : workpieceTree) addNode(docItem, node);
        } else {
            // ── Fallback: only show Workpiece-tagged free shapes ───────────
            TDF_LabelSequence wpcLabels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
            for (int i = 1; i <= wpcLabels.Length(); ++i) {
                TDF_Label lbl = wpcLabels.Value(i);
                const QString entry = XcafUtils::entry(lbl);
                QString text = XcafUtils::name(lbl);
                if (text.isEmpty()) text = entry;
                auto* item = new QTreeWidgetItem(docItem);
                item->setText(0, text);
                item->setIcon(0, QIcon(":/icons/shape.svg"));
                item->setData(0, kRoleDocId, doc->id());
                item->setData(0, kRoleEntry, entry);
            }
        }
    }

    m_documentTree->expandToDepth(2);
}

// ── View routing helpers ────────────────────────────────────────────────────────────────
void MainWindow::showMachineView()
{
    DocumentId machId = LcncApplication::instance()->machineDocumentId();
    if (auto* gd = GuiApplication::instance()->guiDocument(machId))
        m_occView->attachDocument(gd);
    else
        m_occView->attachDefaultScene(m_defaultScene);

    if (LcncDocument* machDoc = LcncApplication::instance()->machineDocument()) {
        m_modelTree->rebuildForDocument(machDoc);
        m_machinePanel->setDocument(machDoc);
    }
}

void MainWindow::showWorkpieceView(DocumentId id)
{
    if (id == kInvalidDocumentId)
        id = LcncApplication::instance()->activeDocumentId();
    if (auto* gd = GuiApplication::instance()->guiDocument(id))
        m_occView->attachDocument(gd);
    else
        m_occView->attachDefaultScene(m_defaultScene);
}

void MainWindow::updateCommandStates()
{
    m_cmdContainer->updateAllStates();
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
