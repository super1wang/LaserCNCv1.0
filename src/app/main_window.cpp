#include "app/main_window.h"
#include "core/kernel/kernel.h"
#include "app/app_context.h"
#include "app/command_registry.h"
#include "core/command/commands_api.h"
#include "core/logging/logger.h"
#include "modules/cad/commands/commands_file.h"
#include "modules/cad/commands/commands_edit.h"
#include "modules/cad/commands/commands_cad.h"
#include "modules/cam/commands/commands_machine.h"
#include "modules/cam/commands/commands_cam.h"
#include "app/commands/commands_display.h"
#include "modules/cad/ui/ribbon_cad_tab.h"
#include "modules/cam/ui/ribbon_cam_tab.h"
#include "modules/process/ui/ribbon_process_tab.h"
#include "view/widget_occ_view.h"
#include "modules/cad/ui/widget_model_tree.h"
#include "modules/cam/ui/widget_machine_panel.h"
#include "modules/cam/ui/widget_toolpath_panel.h"
#include "modules/cam/ui/dialog_axis_calibration_wizard.h"
#include "modules/process/ui/widget_laser_control.h"
#include "app/dialog/dialog_task_manager.h"
#include "modules/cam/settings/cam_config.h"
#include "core/algorithms/cam/laser_toolpath.h"
#include "core/document/lcnc_application.h"
#include "core/document/lcnc_document.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/document/xcaf_utils.h"
#include "view/graphics_scene.h"
#include "view/gui_application.h"
#include "view/gui_document.h"
#include "view/world_axes_renderer.h"
#include "modules/cad/cad_module.h"
#include "modules/cam/cam_module.h"
#include "modules/process/process_module.h"

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
    LcncApplication* lcnc = lcnc::Kernel::current().app();
    connect(lcnc, &LcncApplication::documentAdded,
            this, &MainWindow::onDocumentAdded);
    connect(lcnc, &LcncApplication::documentClosed,
            this, &MainWindow::onDocumentClosed);
    connect(lcnc, &LcncApplication::activeDocumentChanged,
            this, &MainWindow::onActiveDocumentChanged);
        connect(lcnc, &LcncApplication::documentModified,
            this, &MainWindow::onDocumentModified);

    // 世界坐标系渲染器：新建/关闭文档时自动 attach/detach 到该文档场景，
    // 全局可见性由 ribbon 上的 CmdToggleWorldAxes 控制。
    if (auto* guiApp = lcnc::Kernel::current().guiApp()) {
        connect(guiApp, &GuiApplication::guiDocumentAdded, this,
                [guiApp](DocumentId id) {
            auto* gd = guiApp->guiDocument(id);
            if (!gd || !gd->scene()) {
                LCNC_WARN(lcnc::LogCode::Generic,
                          "WorldAxes auto-attach: missing scene for doc {}",
                          static_cast<int>(id));
                return;
            }
            LCNC_DEBUG(lcnc::LogCode::Generic,
                       "WorldAxes auto-attach for new doc {}",
                       static_cast<int>(id));
            lcnc::view::WorldAxesRenderer::instance().attach(gd->scene());
        });
        connect(guiApp, &GuiApplication::guiDocumentClosed, this,
                [guiApp](DocumentId id) {
            auto* gd = guiApp->guiDocument(id);
            if (gd && gd->scene())
                lcnc::view::WorldAxesRenderer::instance().detach(gd->scene());
        });
        // 启动时机台 GuiDocument 通常已经存在，直接 attach。
        if (auto* mgd = guiApp->machineGuiDocument(); mgd && mgd->scene()) {
            LCNC_DEBUG(lcnc::LogCode::Generic,
                       "WorldAxes auto-attach for machine doc");
            lcnc::view::WorldAxesRenderer::instance().attach(mgd->scene());
        }
    }

    restorePersistedCamState();

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
    // Create the permanent machine workspace document.
    // Must be called AFTER lcnc::Kernel::current().guiApp() so the documentAdded
    // signal is received and a GuiDocument is created for the machine doc.
    lcnc::Kernel::current().app()->ensureMachineDocument();

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

    // Context menu → CAM module owns axis assignment storage
    connect(m_modelTree, &WidgetModelTree::axisShapeUnassignRequested, this,
            [this](const QString& shapeEntry) {
                m_appContext->camModule()->unassignShape(shapeEntry);
            });
    connect(m_modelTree, &WidgetModelTree::axisAssignmentsClearRequested, this,
            [this](const QString& axisName) {
                m_appContext->camModule()->clearAxisAssignments(axisName);
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
    // 选中节点蓝底白字，与左侧"准备"模型树保持一致
    m_documentTree->setStyleSheet(
        "QTreeWidget::item:selected { background-color: #2A6FDB; color: white; }"
        "QTreeWidget::item:selected:!active { background-color: #2A6FDB; color: white; }");
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
    connect(m_machinePanel, &WidgetMachinePanel::loadMachineRequested, this,
            [this]{ m_cmdContainer->findCommand(CmdLoadMachine::Name)->execute(); });
        connect(m_machinePanel, &WidgetMachinePanel::compressMachineRequested, this,
            [this]{ m_cmdContainer->findCommand(CmdCompressMachine::Name)->execute(); });
    connect(m_machinePanel, &WidgetMachinePanel::mountWorkpieceRequested, this,
            [this]{ m_cmdContainer->findCommand(CmdMountWorkpiece::Name)->execute(); });
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
    if (lcnc::Kernel::current().app()->isMachineDocument(id)) {
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
        m_modelTree->clear();
        m_machinePanel->setDocument(nullptr);
        m_machinePanel->setMachineModelPath(cam->machineModelPath());
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
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "MainWindow::showWorkpieceView docId={}", id);

    if (m_leftTabs && m_leftTabs->currentIndex() != 0) {
        QSignalBlocker blocker(m_leftTabs);
        m_leftTabs->setCurrentIndex(0);
    }

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
