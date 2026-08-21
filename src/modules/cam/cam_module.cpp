
#include "modules/cam/cam_module.h"
#include "modules/cam/internal/cam_module_support.h"
#include "view/toolpath_renderer.h"
#include "view/travel_path_renderer.h"
#include "view/contour_order_label_renderer.h"
#include "view/machine_guide_renderer.h"
#include "core/project/cam/cam_data_manager.h"
#include "core/project/cam/layer_container.h"
#include "core/project/cam/layer_manager.h"
#include "modules/cam/machine/machine_axis_detector.h"
#include "modules/cam/display/cam_display_projection_service.h"
#include "modules/cam/toolpath/toolpath_generation_service.h"
#include "modules/cam/machine/machine_io.h"
#include "modules/cam/interaction/reference_pick.h"
#include "modules/cam/integration/cam_service_adapters.h"
#include "modules/cam/collision/collision_geometry_cache.h"
#include "modules/cam/collision/cutter_collision_geometry.h"
#include "modules/cam/toolpath/toolpath_sequence_service.h"
#include "modules/cam/toolpath/toolpath_solve_service.h"
#include "core/machine/machine_workspace.h"
#include "modules/cam/contracts/cam_events.h"
#include "core/kinematics/machine_configuration_service.h"
#include "core/kernel/kernel.h"
#include "core/settings/app_settings.h"
#include "modules/cad/services/shape_service.h"

#include "modules/cam/settings/cam_config.h"
#include "core/algorithms/cam/face_classifier.h"
#include "core/algorithms/cam/travel_path_planner.h"
#include "core/document/lcnc_document.h"
#include "core/algorithms/cam/laser_toolpath.h"
#include "core/algorithms/cam/collision_scan_policy.h"
#include "core/algorithms/cam/collision_safety_domain.h"
#include "core/algorithms/cam/initial_approach_axis_planner.h"
#include "core/algorithms/cam/surface_collision_prefilter.h"
#include "core/algorithms/occt_exact_operation_lock.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/kinematics/machine_pose.h"
#include "core/kernel/i_kernel.h"
#include "core/kernel/service_registry.h"
#include "core/services/selection_service.h"
#include "core/logging/logger.h"
#include "core/project/lcnc_project_manager.h"
#include "core/task/task_manager.h"
#include "core/document/xcaf_utils.h"
#include "view/gui_application.h"
#include "view/gui_document.h"
#include "view/widget_occ_view.h"
#include "view/graphics_scene.h"

#include <QElapsedTimer>
#include <QCoreApplication>
#include <QDateTime>
#include <QFile>
#include <QFileInfo>
#include <QMutex>
#include <QMutexLocker>
#include <QPoint>
#include <QPointer>
#include <QScopeGuard>
#include <QSignalBlocker>
#include <QByteArray>
#include <QThread>
#include <QTimer>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepTools.hxx>
#include <STEPControl_Reader.hxx>
#include <StlAPI_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <TopLoc_Location.hxx>
#include <gp_Trsf.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <AIS_DisplayMode.hxx>
#include <Aspect_PolygonOffsetMode.hxx>
#include <Aspect_TypeOfLine.hxx>
#include <Graphic3d_ZLayerId.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <TDF_LabelSequence.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <Quantity_Color.hxx>
#include <Quantity_NameOfColor.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <Bnd_OBB.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Precision.hxx>
#include <SelectMgr_EntityOwner.hxx>
#include <SelectMgr_SelectableObject.hxx>
#include <StdSelect_BRepOwner.hxx>
#include <gp_Vec.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <OSD_Parallel.hxx>
#include <OSD_ThreadPool.hxx>
#include <TopTools_MapOfShape.hxx>

namespace {

using lcnc::cam::TravelCollisionBody;
using lcnc::cam::TravelCollisionGeometryCache;
using CamTravelCollisionBody = lcnc::cam::TravelCollisionBody;
using CamTravelCollisionLeaf = lcnc::cam::TravelCollisionLeaf;
using CamTravelCollisionGeometryCache = lcnc::cam::TravelCollisionGeometryCache;
using lcnc::cam::buildCollisionGeometry;
using lcnc::cam::collisionAxisSourceId;
using lcnc::cam::collisionGeometryKey;
using lcnc::cam::transformCollisionAabb;
using lcnc::cam::transformCollisionObb;

using lcnc::cam::detail::entityEntries;
using lcnc::cam::detail::faceBelongsToSource;
using lcnc::cam::detail::shapeCenter;
using lcnc::cam::detail::translatedShapeCopy;
using lcnc::cam::detail::watchTask;

} // namespace

// Out-of-line dtor for unique_ptr<前置声明类型>。
CamModule::~CamModule() = default;

// ── IModule ───────────────────────────────────────────────────────────────────────
lcnc::ModuleInfo CamModule::info() const
{
    return {
        QStringLiteral("cam"),
        // 中文翻译：CAM模块
        QStringLiteral("CAM module"),
        QStringLiteral(LCNC_VERSION_STRING),
        { QStringLiteral("cad") }
    };
}

bool CamModule::init(lcnc::IKernel& kernel)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CamModule::init begin");
    kernel.services().registerBorrowedService<CamModule>(*this);
    kernel.services().registerBorrowedService<lcnc::ICamFacade>(*this);
    lcnc::cam::registerCamServiceAdapters(kernel, *this);
    if (m_pose) {
        // 把 MachinePose 也作为共享 IService 暴露，跨模块（process/UI）可读写姿态。
        kernel.services().registerBorrowedService<lcnc::MachinePose>(*m_pose);
    }

    // 工程核心 CAM 数据的加载/保存已下沉到 core（LcncProjectManager + cam_toolpath_io）。
    // CAM 模块不再做项目文件 IO：仅在 core 完成加载后刷新视图，并在工程重置时清空展示。
    if (auto* pm = lcnc::Kernel::current().projectManager()) {
        connect(pm, &lcnc::LcncProjectManager::projectOpened,
                this, [this](const QString&) {
                    resetProjectViewState();
                    onCamDataLoaded();
                });
        connect(pm, &lcnc::LcncProjectManager::projectReset,
                this, [this]() {
                    resetProjectViewState();
                    if (m_camData && m_camData->hasToolpath())
                        onCamDataLoaded();
                    else
                        clearToolpathViewState(/*emitSignals=*/true);
                });
    }

    m_initialized = true;
    LCNC_INFO(lcnc::LogCode::Generic, "CamModule init done");

    // 中文翻译：切割路径显示
    // 订阅 CAM 顺序/显示事件，驱动视图覆盖层。
    m_eventSubscriptions.push_back(kernel.events().subscribe<lcnc::cam::events::TravelPathVisibilityToggled>(
        [this](const lcnc::cam::events::TravelPathVisibilityToggled& e) {
            setTravelPathVisible(e.visible);
        }));
    // 中文翻译：切割链表序号显示
    m_eventSubscriptions.push_back(kernel.events().subscribe<lcnc::cam::events::ContourOrderLabelVisibilityToggled>(
        [this](const lcnc::cam::events::ContourOrderLabelVisibilityToggled& e) {
            setContourOrderLabelVisible(e.visible);
        }));
    m_eventSubscriptions.push_back(kernel.events().subscribe<lcnc::cam::events::ContourSequenceChanged>(
        [this](const lcnc::cam::events::ContourSequenceChanged&) {
            refreshCuttingOrderOverlays();
        }));

    connect(this, &CamModule::cutterCollisionConfigurationChanged, this, [] {
        lcnc::Kernel::current().events().publish(lcnc::cam::events::ExecutionPlanChanged{
            lcnc::cam::events::ExecutionPlanChangeKind::Configuration});
    });
    connect(this, &CamModule::contourOrderTravelPlanRebuilt, this,
            [](const QVector<std::uint64_t>&) {
                auto& events = lcnc::Kernel::current().events();
                events.publish(lcnc::cam::events::ContourSequenceChanged{});
                events.publish(lcnc::cam::events::ExecutionPlanChanged{
                    lcnc::cam::events::ExecutionPlanChangeKind::Sequence});
            });
    connect(this, &CamModule::toolpathLayersChanged, this, [] {
        lcnc::Kernel::current().events().publish(lcnc::cam::events::ExecutionPlanChanged{
            lcnc::cam::events::ExecutionPlanChangeKind::Layers});
    });
    const auto publishSimulationInvalidation = [] {
        lcnc::Kernel::current().events().publish(
            lcnc::cam::events::OfflineSimulationSourceChanged{});
    };
    connect(this, &CamModule::machineWorkspaceChanged,
            this, publishSimulationInvalidation);
    connect(this, &CamModule::axisAssignmentsChanged,
            this, publishSimulationInvalidation);
    connect(this, &CamModule::toolpathGenerated,
            this, publishSimulationInvalidation);
    connect(this, &CamModule::toolpathCleared,
            this, publishSimulationInvalidation);
    connect(this, &CamModule::toolpathLayersChanged,
            this, publishSimulationInvalidation);
    connect(this, &CamModule::cutterCollisionConfigurationChanged,
            this, publishSimulationInvalidation);
    connect(this, &CamModule::collisionConfigurationChanged,
            this, publishSimulationInvalidation);
    connect(this, &CamModule::contourOrderTravelPlanRebuilt, this,
            [publishSimulationInvalidation](const QVector<std::uint64_t>&) {
                publishSimulationInvalidation();
            });

    return true;
}

bool CamModule::start()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CamModule::start (no-op)");
    return true;
}

void CamModule::stop()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CamModule::stop begin");
    if (!m_initialized) return;
    for (const lcnc::SubscriptionId id : m_eventSubscriptions)
        lcnc::Kernel::current().events().unsubscribe(id);
    m_eventSubscriptions.clear();
    if (!cancelOwnedTasks(10000)) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "CamModule::stop: machine/CAM task cancellation timed out; retaining core-owned workspace state");
    }
    m_travelCollisionGeometryCache.reset();
    m_initialized = false;
    LCNC_INFO(lcnc::LogCode::Generic, "CamModule stop done");
}

bool CamModule::cancelOwnedTasks(int timeoutMs)
{
    auto* taskMgr = lcnc::Kernel::current().taskManager();
    if (!taskMgr || m_taskScope.empty())
        return true;
    return m_taskScope.cancelAndWait(*taskMgr, timeoutMs);
}

CamModule::CamModule(QObject* parent)
    : QObject(parent)
    , m_toolpathRenderer(std::make_unique<lcnc::view::ToolpathRenderer>())
    , m_guideRenderer(std::make_unique<lcnc::view::MachineGuideRenderer>())
    , m_travelPathRenderer(std::make_unique<lcnc::view::TravelPathRenderer>())
    , m_contourOrderLabelRenderer(std::make_unique<lcnc::view::ContourOrderLabelRenderer>())
    , m_displayProjectionService(std::make_unique<lcnc::cam::CamDisplayProjectionService>())
    , m_machiningFacePipeline(std::make_unique<lcnc::cam::MachiningFacePipelineService>())
    , m_camData(lcnc::Kernel::current().projectManager()->camData())
{
    // 加载当前持久化 TOML 配置。
    m_config.loadDefault();

    auto* project = lcnc::Kernel::current().projectManager();
    project->ensureProject();

    // 机台工作台是独立参考资产，由 Kernel(core) 拥有并已 attach 到 pm 做视图域路由；
    // CAM 仅借用它执行加载/标定等业务，不负责其生命周期。
    m_machineWorkspace = lcnc::Kernel::current().machineWorkspace();

    m_machineConfig = lcnc::Kernel::current().service<lcnc::MachineConfigurationService>();
    if (m_machineConfig) {
        connect(m_machineConfig, &lcnc::MachineConfigurationService::machineConfigurationChanged,
                this, [this] {
                    applyConfiguredMachineAxes(true);
                });
    }

    CamConfig& config = m_config;
    m_machineModelPath = config.machineModelPath();
    m_machineRenderQualityPreset = config.machineRenderQualityPreset();
    toolpathRef().setGlobalLeadInLength(config.leadInLength());
    toolpathRef().setGlobalCuttingOffsetMm(config.cuttingOffsetMm());
    toolpathRef().setGlobalRapidOffsetMm(config.rapidOffsetMm());
    m_deflection = config.deflection();
    m_smoothAngle = config.smoothAngle();
    m_useFaceClassification = config.useFaceClassification();
    m_extractionStrategy = config.extractionStrategy();
    m_toolpathRenderer->setShowNormals(config.showNormals());
    m_toolpathRenderer->setNormalSampleStep(config.normalSampleStep());
    // 用全局默认值播种初始（空）工程的工程级生成参数。
    pushGenerationParamsToCamData();
    if (m_camData && !m_camData->hasToolpath()) {
        m_camData->appliedGenerationParams() = m_camData->generationParams();
        m_camData->setGenerationParamsDirty(false);
    }

    connect(project, &lcnc::LcncProjectManager::activeWorkspaceChanged,
            this, [this] {
                auto* project = lcnc::Kernel::current().projectManager();
                // Machining-face TopoDS handles belong to the workspace that
                // supplied them.  Drop them before exposing a replacement (or
                // no) workspace; otherwise the explorer can show stale faces
                // after the last file is closed.
                m_machiningFacePipeline->reset();
                refreshMachiningFaceDisplay();
                m_camData = project->camData();
                if (m_camData && !m_camData->hasToolpath()) {
                    toolpathRef().setGlobalLeadInLength(m_config.leadInLength());
                    toolpathRef().setGlobalCuttingOffsetMm(m_config.cuttingOffsetMm());
                    toolpathRef().setGlobalRapidOffsetMm(m_config.rapidOffsetMm());
                    m_deflection = m_config.deflection();
                    pushGenerationParamsToCamData();
                    m_camData->appliedGenerationParams() = m_camData->generationParams();
                    m_camData->setGenerationParamsDirty(false);
                }
                // A new workspace owns a fresh CamDataManager.  Seed its
                // project-level mode and axis layout from the active machine
                // before any global generation can capture the default
                // Planar3Axis enum value.  A persisted v5 layout is valid and
                // is therefore deliberately left untouched.
                // 中文翻译：切换到新工程时，按当前机床初始化其加工模式与轴布局。
                applyConfiguredMachineAxes(false);
                const auto activeId = project->activeWorkspaceId();
                m_machineModelVisible = m_machineVisibleWorkspaceIds.contains(activeId);
                if (auto* gd = activeGuiDocument()) {
                    gd->updateMachineWorkspaceTransforms(machineDocument(), workpieceDocument());
                    if (gd->hasView())
                        gd->view()->Redraw();
                }
                displayAxisGuides();
                if (m_camData && !m_camData->machiningFaceRecords().empty())
                    rebindMachiningFacesFromRecords();
                else
                    emit machiningFacesChanged();
                if (m_camData) {
                    emit machiningModeChanged(m_camData->machiningMode());
                    emit workpieceSetupTransformChanged();
                }
                emit machineVisibilityChanged();
            });

    connect(project, &lcnc::LcncProjectManager::domainDataChanged,
            this, [this](lcnc::ProjectDomain domain) {
                if (domain == lcnc::ProjectDomain::Machine) {
                    // 项目工作区变更（构型切换/重新装载）后同步 pose 的轴集合
                    if (m_pose && m_pose->kinematics() != kinematics())
                        m_pose->setKinematics(kinematics());
                    emit machineWorkspaceChanged();
                } else if (domain == lcnc::ProjectDomain::Cam
                           && !m_clearingToolpath
                           && (!m_camData || !m_camData->hasToolpath())) {
                    clearToolpathViewState(/*emitSignals=*/true);
                }
            });

    if (auto* guiApp = lcnc::Kernel::current().guiApp()) {
        connect(guiApp, &GuiApplication::guiDocumentAboutToClose,
                this, [this](ProjectWorkspaceId, GuiDocument* gd) {
                    if (m_guideRenderer)
                        m_guideRenderer->erase(gd);
                });
    }

    // 姿态状态对象 + 16ms 单次定时合并器（dirty-axis 局部刷新）。
    m_pose = std::make_unique<lcnc::MachinePose>(this);
    m_refreshCoalescer = new QTimer(this);
    m_refreshCoalescer->setSingleShot(true);
    m_refreshCoalescer->setInterval(16); // ≈ 60Hz 上限
    connect(m_refreshCoalescer, &QTimer::timeout, this, [this]() {
        QStringList dirty(m_pendingDirtyAxes.cbegin(), m_pendingDirtyAxes.cend());
        m_pendingDirtyAxes.clear();
        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "CamModule coalescer flush n={}", dirty.size());
        refreshMachineTransforms(dirty);
    });
    connect(m_pose.get(), &lcnc::MachinePose::poseChanged, this,
            [this](const QStringList& dirtyAxes) {
        if (m_inPoseSelfUpdate)
            return; // 由 setAxisPosition 触发的自更新已自行处理刷新
        for (const QString& a : dirtyAxes)
            m_pendingDirtyAxes.insert(a);
        if (!m_refreshCoalescer->isActive())
            m_refreshCoalescer->start();
    });

    // Every toolpath-generation boundary publishes an explicitly empty
    // selection before UI subscribers rebuild their trees and panels.
    connect(this, &CamModule::toolpathGenerated,
            this, &CamModule::clearToolpathSelectionState,
            Qt::DirectConnection);

    applyConfiguredMachineAxes(false);
}

// ── Domain documents ──────────────────────────────────────────────────────────

LcncDocument* CamModule::machineDocument() const
{
    return lcnc::Kernel::current().projectManager()->machineDocument();
}

DocumentId CamModule::machineDocumentId() const
{
    return lcnc::Kernel::current().projectManager()->machineDocumentId();
}

LcncDocument* CamModule::camDocument() const
{
    return lcnc::Kernel::current().projectManager()->camDocument();
}

DocumentId CamModule::camDocumentId() const
{
    return lcnc::Kernel::current().projectManager()->camDocumentId();
}

LcncDocument* CamModule::workpieceDocument() const
{
    return lcnc::Kernel::current().projectManager()->workpieceDocument();
}

DocumentId CamModule::workpieceDocumentId() const
{
    return lcnc::Kernel::current().projectManager()->workpieceDocumentId();
}

GuiDocument* CamModule::activeGuiDocument() const
{
    return lcnc::Kernel::current().guiApp()->activeGuiDocument();
}

MachineKinematics* CamModule::kinematics() const
{
    if (auto* doc = machineDocument())
        return doc->machineKinematics();
    return nullptr;
}

void CamModule::requestMachineView()
{
    emit machineViewRequested();
}

QString CamModule::machineModelPath() const
{
    return m_machineModelPath;
}

void CamModule::setMachineModelPath(const QString& filePath)
{
    const QString normalized = filePath.trimmed().isEmpty()
        ? QString()
        : QFileInfo(filePath).absoluteFilePath();
    if (m_machineModelPath == normalized)
        return;

    m_machineModelPath = normalized;
    m_config.setMachineModelPath(m_machineModelPath);
}

lcnc::RenderQualityPreset CamModule::machineRenderQualityPreset() const
{
    return m_machineRenderQualityPreset;
}

void CamModule::setMachineRenderQualityPreset(lcnc::RenderQualityPreset quality)
{
    if (m_machineRenderQualityPreset == quality)
        return;

    m_machineRenderQualityPreset = quality;
    m_config.setMachineRenderQualityPreset(quality);
}
