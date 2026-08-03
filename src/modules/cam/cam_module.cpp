
#include "modules/cam/cam_module.h"
#include "view/toolpath_renderer.h"
#include "view/travel_path_renderer.h"
#include "view/machine_guide_renderer.h"
#include "core/project/cam/cam_data_manager.h"
#include "core/project/cam/layer_container.h"
#include "core/project/cam/layer_manager.h"
#include "modules/cam/services/machine_axis_detector.h"
#include "modules/cam/services/toolpath_generation_service.h"
#include "modules/cam/services/machine_io.h"
#include "modules/cam/services/reference_pick.h"
#include "core/machine/machine_workspace.h"
#include "modules/cam/i_cam_layer_provider.h"
#include "core/kinematics/machine_configuration_service.h"
#include "core/kernel/kernel.h"
#include "modules/cad/services/shape_service.h"

#include "modules/cam/settings/cam_config.h"
#include "core/algorithms/cam/face_classifier.h"
#include "core/document/lcnc_document.h"
#include "core/algorithms/cam/laser_toolpath.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/kinematics/machine_pose.h"
#include "core/kernel/i_kernel.h"
#include "core/kernel/service_registry.h"
#include "core/services/selection_service.h"
#include "modules/process/cutting/i_process_cutting_plan_provider.h"
#include "modules/process/runtime/process_events.h"
#include "core/logging/logger.h"
#include "core/project/lcnc_project_manager.h"
#include "core/task/task_manager.h"
#include "core/document/xcaf_utils.h"
#include "view/gui_application.h"
#include "view/gui_document.h"
#include "view/widget_occ_view.h"
#include "view/graphics_scene.h"

#include <QElapsedTimer>
#include <QFileInfo>
#include <QMutex>
#include <QMutexLocker>
#include <QPoint>
#include <QPointer>
#include <QSignalBlocker>
#include <QByteArray>
#include <QThread>
#include <QTimer>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Builder.hxx>
#include <BRepGProp.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <TopLoc_Location.hxx>
#include <gp_Trsf.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
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
#include <GeomAbs_SurfaceType.hxx>
#include <GProp_GProps.hxx>
#include <Precision.hxx>
#include <SelectMgr_EntityOwner.hxx>
#include <SelectMgr_SelectableObject.hxx>
#include <StdSelect_BRepOwner.hxx>
#include <gp_Vec.hxx>
#include <gp_Ax2.hxx>

namespace {

void watchTask(QObject* owner, TaskId taskId, std::function<void(bool)> onFinished)
{
    auto connection = std::make_shared<QMetaObject::Connection>();
    *connection = QObject::connect(
        lcnc::Kernel::current().taskManager(),
        &TaskManager::taskFinished,
        owner,
        [taskId, onFinished = std::move(onFinished), connection](TaskId finishedId, bool success) mutable {
            if (finishedId != taskId)
                return;
            QObject::disconnect(*connection);
            onFinished(success);
        });
}

QStringList entityEntries(LcncDocument* doc, LcncDocument::EntityKind kind)
{
    QStringList result;
    if (!doc)
        return result;

    const TDF_LabelSequence labels = doc->entityLabels(kind);
    for (int i = 1; i <= labels.Length(); ++i)
        result << XcafUtils::entry(labels.Value(i));
    return result;
}

class CamToolpathProviderAdapter final
    : public QObject
    , public lcnc::cam::ICamToolpathProvider
{
public:
    explicit CamToolpathProviderAdapter(CamModule* module)
        : QObject(nullptr)
        , m_module(module)
    {
        if (m_module) {
            QObject::connect(m_module, &CamModule::toolpathGenerated,
                             this, [this] { refreshCache(); });
            QObject::connect(m_module, &CamModule::toolpathCleared,
                             this, [this] { refreshCache(); });
            QObject::connect(m_module, &CamModule::toolpathLayersChanged,
                             this, [this] { refreshCache(); });
            QObject::connect(m_module, &CamModule::activeContourParametersChanged,
                             this, [this] { refreshCache(); });
            refreshCache();
        }
    }

    bool hasToolpath() const override
    {
        QMutexLocker lock(&m_cacheMutex);
        return m_snapshot.hasEnabledContours();
    }

    std::uint64_t toolpathRevision() const override
    {
        QMutexLocker lock(&m_cacheMutex);
        return m_snapshot.revision;
    }

    bool solveToolpathForOrder(
        const QVector<std::uint64_t>& orderedContourIds) override
    {
        if (!m_module || QThread::currentThread() != m_module->thread()) {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "CamToolpathProviderAdapter: solveToolpathForOrder must run on the CAM thread");
            return false;
        }
        const bool solved = m_module->solveToolpathForOrder(orderedContourIds);
        if (solved)
            refreshCache();
        return solved;
    }

    lcnc::cam::ToolpathExportSnapshot exportToolpathSnapshot() const override
    {
        QMutexLocker lock(&m_cacheMutex);
        return m_snapshot;
    }

    lcnc::cam::ToolpathExportSnapshot exportToolpathSnapshotForOrder(
        const QVector<std::uint64_t>& orderedContourIds) const override
    {
        QMutexLocker lock(&m_cacheMutex);
        if (orderedContourIds.isEmpty())
            return m_snapshot;

        lcnc::cam::ToolpathExportSnapshot ordered;
        ordered.revision = m_snapshot.revision;
        ordered.description = m_snapshot.description;
        QHash<std::uint64_t, lcnc::cam::ToolpathExportContour> byId;
        for (const auto& contour : m_snapshot.contours)
            byId.insert(contour.contourId, contour);
        for (const std::uint64_t id : orderedContourIds) {
            const auto it = byId.constFind(id);
            if (it == byId.constEnd())
                continue;
            ordered.contours.append(it.value());
            ordered.pointsByContourId.insert(id, m_snapshot.pointsByContourId.value(id));
        }
        return ordered;
    }

private:
    void refreshCache()
    {
        if (!m_module || QThread::currentThread() != m_module->thread())
            return;
        auto snapshot = m_module->exportToolpathSnapshot();
        QMutexLocker lock(&m_cacheMutex);
        m_snapshot = std::move(snapshot);
    }

    CamModule* m_module{nullptr};
    mutable QMutex m_cacheMutex;
    lcnc::cam::ToolpathExportSnapshot m_snapshot;
};

/**
 * @brief 把 CamDataManager 的 LayerContainer/LayerManager 适配到 ICamLayerProvider。
 *
 * 这一层只做"取容器 + 取信号源 + 计数 revision"，所有真正的存储在 LayerContainer。
 * revision 通过订阅 LayerManager 的全部 mutation 信号自增，让 Process 端可以拉式刷新。
 */
class CamLayerProviderAdapter final : public QObject, public lcnc::cam::ICamLayerProvider
{
public:
    explicit CamLayerProviderAdapter(CamModule* module)
        : QObject(nullptr)
        , m_module(module)
    {
        if (m_module) {
            // LayerManager belongs to the active project CamDataManager and can be
            // replaced when a project is opened or switched.  Module-level signals
            // make that transition refresh the immutable Process-side cache too.
            QObject::connect(m_module, &CamModule::toolpathGenerated,
                             this, [this] { refreshCache(); });
            QObject::connect(m_module, &CamModule::toolpathCleared,
                             this, [this] { refreshCache(); });
            QObject::connect(m_module, &CamModule::toolpathLayersChanged,
                             this, [this] { refreshCache(); });
        }
        refreshCache();
    }

    // ── 只读 ─────────────────────────────────────────────────────────────
    QVector<lcnc::cam::LayerSnapshot> layers() const override
    {
        QMutexLocker lock(&m_cacheMutex);
        return m_layers;
    }

    QVector<lcnc::cam::ContourId> manualContourOrder() const override
    {
        QMutexLocker lock(&m_cacheMutex);
        return m_manualContourOrder;
    }

    lcnc::cam::CuttingPlanSortStrategy sortStrategy() const override
    {
        QMutexLocker lock(&m_cacheMutex);
        return m_sortStrategy;
    }

    lcnc::cam::AutoSortAxis lastAutoSortAxis() const override
    {
        QMutexLocker lock(&m_cacheMutex);
        return m_lastAutoSortAxis;
    }

    std::uint64_t revision() const override
    {
        QMutexLocker lock(&m_cacheMutex);
        return m_revision;
    }

    // ── 可变（全部走 LayerManager 触发细粒度信号）────────────────────────
    void setLayerToolName(std::uint64_t layerId, const QString& toolName) override
    {
        if (auto* mgr = layerMgr()) mgr->setLayerToolName(layerId, toolName);
    }
    void setLayerEnabled(std::uint64_t layerId, bool enabled) override
    {
        if (auto* mgr = layerMgr()) mgr->setLayerEnabled(layerId, enabled);
    }
    void setLayerCompensationIndex(std::uint64_t layerId, const QString& index) override
    {
        if (auto* mgr = layerMgr()) mgr->setLayerCompensationIndex(layerId, index);
    }
    void setLayerIncludedContours(std::uint64_t layerId, const QSet<lcnc::cam::ContourId>& included) override
    {
        if (auto* mgr = layerMgr()) mgr->setLayerIncludedContours(layerId, included);
    }
    void setManualContourOrder(const QVector<lcnc::cam::ContourId>& ids) override
    {
        if (auto* mgr = layerMgr()) mgr->setManualContourOrder(ids);
    }
    int appendToManualOrder(const QVector<lcnc::cam::ContourId>& ids) override
    {
        if (auto* mgr = layerMgr()) return mgr->appendToManualOrder(ids);
        return 0;
    }
    void removeFromManualOrder(const QVector<lcnc::cam::ContourId>& ids) override
    {
        if (auto* mgr = layerMgr()) mgr->removeFromManualOrder(ids);
    }
    void clearManualOrder() override
    {
        if (auto* mgr = layerMgr()) mgr->clearManualOrder();
    }
    void setSortStrategy(lcnc::cam::CuttingPlanSortStrategy s) override
    {
        if (auto* mgr = layerMgr()) mgr->setSortStrategy(s);
    }
    void setLastAutoSortAxis(lcnc::cam::AutoSortAxis a) override
    {
        if (auto* mgr = layerMgr()) mgr->setLastAutoSortAxis(a);
    }

    QObject* notifier() const override
    {
        return layerMgr();
    }

private:
    void bindLayerManager()
    {
        auto* const manager = layerMgr();
        if (m_layerManager == manager)
            return;

        if (m_layerManager)
            QObject::disconnect(m_layerManager, nullptr, this, nullptr);
        m_layerManager = manager;
        if (!m_layerManager)
            return;

        auto refresh = [this] { refreshCache(); };
        QObject::connect(m_layerManager, &lcnc::cam::LayerManager::layersReset,
                         this, refresh);
        QObject::connect(m_layerManager, &lcnc::cam::LayerManager::layerAdded,
                         this, [this](std::uint64_t) { refreshCache(); });
        QObject::connect(m_layerManager, &lcnc::cam::LayerManager::layerRemoved,
                         this, [this](std::uint64_t) { refreshCache(); });
        QObject::connect(m_layerManager, &lcnc::cam::LayerManager::layersReordered,
                         this, refresh);
        QObject::connect(m_layerManager, &lcnc::cam::LayerManager::layerPropertyChanged,
                         this, [this](std::uint64_t, lcnc::cam::LayerProperty) { refreshCache(); });
        QObject::connect(m_layerManager, &lcnc::cam::LayerManager::contourMembershipChanged,
                         this, refresh);
        QObject::connect(m_layerManager, &lcnc::cam::LayerManager::manualContourOrderChanged,
                         this, refresh);
        QObject::connect(m_layerManager, &lcnc::cam::LayerManager::sortStrategyChanged,
                         this, [this](lcnc::cam::CuttingPlanSortStrategy) { refreshCache(); });
        QObject::connect(m_layerManager, &lcnc::cam::LayerManager::lastAutoSortAxisChanged,
                         this, [this](lcnc::cam::AutoSortAxis) { refreshCache(); });
    }

    void refreshCache()
    {
        if (!m_module || QThread::currentThread() != m_module->thread())
            return;
        bindLayerManager();
        QVector<lcnc::cam::LayerSnapshot> layers;
        const auto* container = layerContainer();
        const auto* toolpath = container ? container->toolpath() : nullptr;
        if (toolpath) {
            layers.reserve(static_cast<int>(toolpath->layers().size()));
            for (const ToolpathLayer& layer : toolpath->layers()) {
                lcnc::cam::LayerSnapshot snapshot;
                snapshot.layerId = layer.layerId;
                snapshot.name = layer.name;
                snapshot.toolName = layer.toolName;
                snapshot.compensationIndex = layer.compensationIndex;
                snapshot.enabled = layer.enabled;
                snapshot.includedContours = layer.includedContours;
                snapshot.contourIds.reserve(static_cast<int>(layer.contourIds.size()));
                for (const std::uint64_t contourId : layer.contourIds)
                    snapshot.contourIds.push_back(static_cast<lcnc::cam::ContourId>(contourId));
                layers.append(std::move(snapshot));
            }
        }
        QMutexLocker lock(&m_cacheMutex);
        m_layers = std::move(layers);
        m_manualContourOrder = container
            ? container->manualContourOrder() : QVector<lcnc::cam::ContourId>{};
        m_sortStrategy = container
            ? container->sortStrategy() : lcnc::cam::CuttingPlanSortStrategy::LayerThenContour;
        m_lastAutoSortAxis = container
            ? container->lastAutoSortAxis() : lcnc::cam::AutoSortAxis::XPos;
        ++m_revision;
    }

    lcnc::cam::LayerContainer*       layerContainer() const
    {
        auto* data = m_module ? m_module->camData() : nullptr;
        return data ? &data->layerContainer() : nullptr;
    }
    lcnc::cam::LayerManager*         layerMgr() const
    {
        auto* data = m_module ? m_module->camData() : nullptr;
        return data ? data->layerManager() : nullptr;
    }

    CamModule*           m_module{nullptr};
    QPointer<lcnc::cam::LayerManager> m_layerManager;
    mutable QMutex       m_cacheMutex;
    QVector<lcnc::cam::LayerSnapshot> m_layers;
    QVector<lcnc::cam::ContourId> m_manualContourOrder;
    lcnc::cam::CuttingPlanSortStrategy m_sortStrategy{
        lcnc::cam::CuttingPlanSortStrategy::LayerThenContour};
    lcnc::cam::AutoSortAxis m_lastAutoSortAxis{lcnc::cam::AutoSortAxis::XPos};
    std::uint64_t        m_revision{1};
};

TopoDS_Shape translatedShapeCopy(const TopoDS_Shape& shape, const gp_Vec& translation)
{
    if (shape.IsNull() || translation.SquareMagnitude() < 1e-12)
        return shape;

    gp_Trsf trsf;
    trsf.SetTranslation(translation);
    BRepBuilderAPI_Transform xform(shape, trsf, Standard_True);
    return xform.IsDone() ? xform.Shape() : shape;
}

gp_Pnt bboxCenter(const Bnd_Box& box)
{
    if (box.IsVoid())
        return gp_Pnt(0.0, 0.0, 0.0);

    Standard_Real xmin = 0.0;
    Standard_Real ymin = 0.0;
    Standard_Real zmin = 0.0;
    Standard_Real xmax = 0.0;
    Standard_Real ymax = 0.0;
    Standard_Real zmax = 0.0;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    return gp_Pnt(0.5 * (xmin + xmax),
                  0.5 * (ymin + ymax),
                  0.5 * (zmin + zmax));
}

gp_Pnt shapeCenter(const TopoDS_Shape& shape)
{
    if (shape.IsNull())
        return gp_Pnt(0.0, 0.0, 0.0);

    Bnd_Box box;
    BRepBndLib::Add(shape, box);
    return bboxCenter(box);
}

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
        QStringLiteral("1.0.0"),
        { QStringLiteral("cad") }
    };
}

bool CamModule::init(lcnc::IKernel& kernel)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CamModule::init begin");
    auto svc = std::shared_ptr<CamModule>(this, [](CamModule*) {});
    kernel.services().registerService<CamModule>(svc);
    auto facade = std::shared_ptr<lcnc::ICamFacade>(svc, static_cast<lcnc::ICamFacade*>(this));
    kernel.services().registerService<lcnc::ICamFacade>(facade);
    auto toolpathProvider = std::make_shared<CamToolpathProviderAdapter>(this);
    kernel.services().registerService<lcnc::cam::ICamToolpathProvider>(toolpathProvider);

    // CAM 图层视图（OCC-free，Process 端读写工艺映射 / 排序策略 / 人工顺序）。
    auto layerProvider = std::make_shared<CamLayerProviderAdapter>(this);
    kernel.services().registerService<lcnc::cam::ICamLayerProvider>(layerProvider);
    if (m_pose) {
        // 把 MachinePose 也作为共享 IService 暴露，跨模块（process/UI）可读写姿态。
        auto poseSvc = std::shared_ptr<lcnc::MachinePose>(m_pose.get(), [](lcnc::MachinePose*) {});
        kernel.services().registerService<lcnc::MachinePose>(poseSvc);
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
    // 订阅 Process 模块的"Cutting path display"开关与切割链表变化，驱动 TravelPathRenderer。
    kernel.events().subscribe<lcnc::process::events::TravelPathVisibilityToggled>(
        [this](const lcnc::process::events::TravelPathVisibilityToggled& e) {
            setTravelPathVisible(e.visible);
        });
    kernel.events().subscribe<lcnc::process::events::CuttingPlanChanged>(
        [this](const lcnc::process::events::CuttingPlanChanged&) {
            if (m_travelPathRenderer && m_travelPathRenderer->isVisible())
                refreshTravelPath();
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
    if (!cancelOwnedTasks(10000)) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "CamModule::stop: machine/CAM task cancellation timed out; retaining core-owned workspace state");
    }
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
                m_machiningFaces.clear();
                m_nextMachiningFaceId = 1;
                refreshMachiningFaceDisplay();
                m_camData = project->camData();
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

// ── Machine Management ────────────────────────────────────────────────────────

void CamModule::configureMachine(const QString& presetName)
{
    LcncDocument* doc = machineDocument();
    if (!doc || presetName.isEmpty())
        return;

    MachineKinematics* kin = doc->machineKinematics();
    if (!kin)
        return;

    const bool sameConfig = (kin->configType() == presetName);
    kin->loadPreset(presetName);
    m_config.setMachinePreset(presetName);
    if (m_machineConfig)
        m_machineConfig->syncFromKinematics(kin);
    if (m_machineModelPath.isEmpty())
        m_workpieceInstallPosition = defaultWorkpieceInstallPosition();
    else
        applyStoredMachineProfile(m_machineModelPath);

    if (!sameConfig)
        clearToolpath();

    displayAxisGuides();
    refreshMachineTransforms();
    emit axisAssignmentsChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Machine);
}

void CamModule::loadMachine(const QString& filePath)
{
    LcncDocument* doc = machineDocument();
    if (!doc || filePath.isEmpty()) return;

    QFileInfo fi(filePath);
    if (!fi.exists())
        return;

    const QString normalizedPath = fi.absoluteFilePath();

    // Clear previous machine entities
    {
        TDF_LabelSequence existing = doc->entityLabels(LcncDocument::EntityKind::Machine);
        QStringList entriesToRemove;
        for (int i = 1; i <= existing.Length(); ++i)
            entriesToRemove << XcafUtils::entry(existing.Value(i));
        for (const QString& e : entriesToRemove)
            ShapeService::deleteShape(doc, e);
    }

    // 中文翻译：加载机台: %1
    TaskId taskId = lcnc::Kernel::current().taskManager()->run(tr("Loading machine: %1").arg(fi.fileName()),
        [filePath, doc](TaskProgress* prog) {
            if (prog->isAbortRequested())
                throw std::runtime_error("machine load cancelled");
            lcnc::cam::machine_io::loadMachineFromFile(doc, filePath, prog);
            if (prog->isAbortRequested())
                throw std::runtime_error("machine load cancelled");
        });

    m_taskScope.track(taskId);
    watchTask(this, taskId, [this, taskId, normalizedPath](bool ok) {
        m_taskScope.release(taskId);
        if (!ok)
            return;

        m_machineModelPath = normalizedPath;
        m_config.setMachineModelPath(m_machineModelPath);
        applyConfiguredMachineAxes(false);
        autoDetectAxes();
        applyStoredMachineProfile(m_machineModelPath);
        refreshMachineDisplay();
        emit machineLoaded();
    });
}

void CamModule::unloadMachine()
{
    LcncDocument* doc = machineDocument();
    if (!doc) return;

    // Remove machine entities
    TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Machine);
    QStringList entries;
    for (int i = 1; i <= labels.Length(); ++i)
        entries << XcafUtils::entry(labels.Value(i));
    for (const QString& e : entries)
        ShapeService::deleteShape(doc, e);

    m_machineModelPath.clear();
    m_config.setMachineModelPath(QString());
    refreshMachineDisplay();
    emit machineUnloaded();
}

void CamModule::exportMachine(const QString& filePath)
{
    LcncDocument* machDoc = machineDocument();
    if (!machDoc || filePath.isEmpty())
        return;

    lcnc::cam::machine_io::exportMachineToFile(machDoc, machDoc->machineKinematics(), filePath);
}

void CamModule::autoDetectAxes()
{
    LcncDocument* doc = machineDocument();
    if (!doc) return;

    lcnc::cam::machine_axis_detector::autoDetectAxisNames(doc, doc->machineKinematics());
    applyStoredMachineProfile(m_machineModelPath);
    if (m_machineConfig)
        m_machineConfig->syncFromKinematics(doc->machineKinematics());
    if (auto* gd = activeGuiDocument())
        gd->applyMachineDisplayStyle();
    refreshMachineTransforms();
    emit axisAssignmentsChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Machine);
}

void CamModule::applyAxisAssignments(const QMap<QString, QString>& entryToAxis)
{
    LcncDocument* doc = machineDocument();
    if (!doc || entryToAxis.isEmpty())
        return;

    MachineKinematics* kin = doc->machineKinematics();
    for (auto it = entryToAxis.cbegin(); it != entryToAxis.cend(); ++it) {
        if (it.value().isEmpty())
            kin->unassignShape(it.key());
        else
            kin->assignShape(it.key(), it.value());
    }

    if (auto* gd = activeGuiDocument())
        gd->applyMachineDisplayStyle();
    refreshMachineTransforms();
    emit axisAssignmentsChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Machine);
}

void CamModule::assignShapesToAxis(const QStringList& entries, const QString& axisName)
{
    if (entries.isEmpty() || axisName.isEmpty())
        return;

    QMap<QString, QString> entryToAxis;
    for (const QString& entry : entries)
        entryToAxis.insert(entry, axisName);
    applyAxisAssignments(entryToAxis);
}

void CamModule::unassignShape(const QString& entry)
{
    if (entry.isEmpty())
        return;

    QMap<QString, QString> entryToAxis;
    entryToAxis.insert(entry, QString());
    applyAxisAssignments(entryToAxis);
}

void CamModule::clearAxisAssignments(const QString& axisName)
{
    MachineKinematics* kin = kinematics();
    if (!kin || axisName.isEmpty())
        return;

    QMap<QString, QString> entryToAxis;
    for (const QString& entry : kin->shapesForAxis(axisName))
        entryToAxis.insert(entry, QString());
    applyAxisAssignments(entryToAxis);
}

QList<CamModule::AxisOption> CamModule::axisOptions(bool includeDetachOption) const
{
    QList<AxisOption> result;
    MachineKinematics* kin = kinematics();
    if (!kin)
        return result;

    if (includeDetachOption)
        // 中文翻译：— 解除已有挂载 —
        result.append({QString(), tr("— Uninstall existing mounts —")});

    for (const MachineAxisDef& axis : kin->axes()) {
        QString displayName;
        if (axis.name == QStringLiteral("BASE")) {
            // 中文翻译：BASE（固定基座）
            displayName = tr("BASE (fixed base)");
        } else if (axis.motionType == MachineAxisDef::Rotary) {
            // 中文翻译：%1 轴（旋转）
            displayName = tr("%1 axis (rotation)").arg(axis.name);
        } else {
            // 中文翻译：%1 轴（线性）
            displayName = tr("%1 axis (linear)").arg(axis.name);
        }
        result.append({axis.name, displayName});
    }

    return result;
}

QString CamModule::defaultWorkpieceMountAxis() const
{
    const MachineKinematics* kin = kinematics();
    if (!kin || kin->axes().isEmpty())
        return QString();

    const auto hasAxis = [kin](const QString& axisName) {
        return kin->findAxis(axisName) != nullptr;
    };

    const QString configType = kin->configType();
    if (configType == QStringLiteral("VERTICAL_AC_TABLE")
        || configType == QStringLiteral("VERTICAL_BC_TABLE")) {
        if (hasAxis(QStringLiteral("C")))
            return QStringLiteral("C");
    } else if (configType == QStringLiteral("XYZA")) {
        if (hasAxis(QStringLiteral("A")))
            return QStringLiteral("A");
    }

    if (hasAxis(QStringLiteral("BASE")))
        return QStringLiteral("BASE");

    return kin->axes().isEmpty() ? QString() : kin->axes().first().name;
}

gp_Pnt CamModule::axisOrigin(const QString& axisName) const
{
    if (MachineKinematics* kin = kinematics())
        return kin->axisOrigin(axisName);
    return gp_Pnt(0, 0, 0);
}

void CamModule::setAxisOrigin(const QString& axisName, const gp_Pnt& origin)
{
    MachineKinematics* kin = kinematics();
    if (!kin)
        return;

    const gp_Pnt current = kin->axisOrigin(axisName);
    if (current.SquareDistance(origin) < 1e-12)
        return;

    if (!kin->setAxisOrigin(axisName, origin))
        return;

    if (!m_machineModelPath.isEmpty())
        m_config.setAxisOriginForMachine(m_machineModelPath, axisName, origin);

    displayAxisGuides();
    refreshMachineTransforms();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Machine);
}

bool CamModule::setAxisLimits(const QString& axisName, double minVal, double maxVal)
{
    MachineKinematics* kin = kinematics();
    if (!kin)
        return false;

    if (!kin->setAxisLimits(axisName, minVal, maxVal))
        return false;

    displayAxisGuides();
    refreshMachineTransforms();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Machine);
    return true;
}

bool CamModule::supportsAcCenterCalibration() const
{
    return ensureAcCenterCalibrationAvailable();
}

bool CamModule::currentAcRotationCenter(gp_Pnt& center) const
{
    if (!ensureAcCenterCalibrationAvailable())
        return false;

    const MachineKinematics* kin = kinematics();
    if (!kin)
        return false;

    const gp_Pnt aOrigin = kin->axisOrigin(QStringLiteral("A"));
    const gp_Pnt cOrigin = kin->axisOrigin(QStringLiteral("C"));
    center = gp_Pnt(cOrigin.X(), aOrigin.Y(), aOrigin.Z());
    return true;
}

bool CamModule::currentWorkpieceRotationCenter(gp_Pnt& center) const
{
    const MachineKinematics* kin = kinematics();
    if (!kin)
        return false;

    const QString configType = kin->configType();
    if (configType == QStringLiteral("VERTICAL_AC_TABLE"))
        return currentAcRotationCenter(center);

    if (configType == QStringLiteral("VERTICAL_BC_TABLE")) {
        if (!kin->findAxis(QStringLiteral("B")) || !kin->findAxis(QStringLiteral("C")))
            return false;

        const gp_Pnt bOrigin = kin->axisOrigin(QStringLiteral("B"));
        const gp_Pnt cOrigin = kin->axisOrigin(QStringLiteral("C"));
        center = gp_Pnt(bOrigin.X(), cOrigin.Y(), bOrigin.Z());
        return true;
    }

    if (configType == QStringLiteral("XYZA")) {
        if (!kin->findAxis(QStringLiteral("A")))
            return false;

        center = kin->axisOrigin(QStringLiteral("A"));
        return true;
    }

    return false;
}

gp_Pnt CamModule::cutterHeadModelPosition() const
{
    return m_cutterHeadModelPosition;
}

gp_Pnt CamModule::cutterHeadPhysicalPosition() const
{
    return m_cutterHeadPhysicalPosition;
}

void CamModule::setCutterHeadModelPosition(const gp_Pnt& position)
{
    if (m_cutterHeadModelPosition.SquareDistance(position) < 1e-12)
        return;

    m_cutterHeadModelPosition = position;
    if (!m_machineModelPath.isEmpty()) {
        m_config.setCutterHeadModelPositionForMachine(
            m_machineModelPath,
            m_cutterHeadModelPosition);
    }
    displayAxisGuides();
    if (auto* gd = activeGuiDocument()) {
        if (gd->hasView())
            gd->view()->Redraw();
    }
    emit machineWorkspaceChanged();
}

void CamModule::setCutterHeadPhysicalPosition(const gp_Pnt& position)
{
    if (m_cutterHeadPhysicalPosition.SquareDistance(position) < 1e-12)
        return;

    m_cutterHeadPhysicalPosition = position;
    if (!m_machineModelPath.isEmpty()) {
        m_config.setCutterHeadPhysicalPositionForMachine(
            m_machineModelPath,
            m_cutterHeadPhysicalPosition);
    }
    emit machineWorkspaceChanged();
}

bool CamModule::fillAxisOriginFromReferenceFace(WidgetOccView* occView,
                                                const QPoint& screenPos,
                                                const QString& axisName)
{
    QString errorMessage;
    if (!ensureAcCenterCalibrationAvailable(&errorMessage)) {
        // 中文翻译：轴心快速填充
        emit operationFailed(tr("Fast filling of axis"), errorMessage);
        return false;
    }

    const QString normalizedAxis = axisName.trimmed().toUpper();
    if (normalizedAxis != QStringLiteral("A") && normalizedAxis != QStringLiteral("C")) {
        // 中文翻译：轴心快速填充
        emit operationFailed(tr("Fast filling of axis"),
                             // 中文翻译：当前快速填充仅支持 A 轴和 C 轴。
                             tr("Currently, quick filling only supports A-axis and C-axis."));
        return false;
    }

    gp_Pnt faceCenter;
    if (!resolveReferencePlaneCenter(occView, screenPos, faceCenter, &errorMessage)) {
        // 中文翻译：轴心快速填充
        emit operationFailed(tr("Fast filling of axis"), errorMessage);
        return false;
    }

    MachineKinematics* kin = kinematics();
    if (!kin) {
        // 中文翻译：轴心快速填充；找不到机台轴系配置。
        emit operationFailed(tr("Fast filling of axis"), tr("The machine axis system configuration cannot be found."));
        return false;
    }

    if (normalizedAxis == QStringLiteral("A")) {
        const gp_Pnt current = kin->axisOrigin(QStringLiteral("A"));
        setAxisOrigin(QStringLiteral("A"), gp_Pnt(current.X(), faceCenter.Y(), faceCenter.Z()));
    } else {
        const gp_Pnt current = kin->axisOrigin(QStringLiteral("C"));
        setAxisOrigin(QStringLiteral("C"), gp_Pnt(faceCenter.X(), current.Y(), current.Z()));
    }

    if (hasToolpath())
        updateToolpathMachineCoordinates();

    return true;
}

bool CamModule::setCutterHeadModelPositionFromReferenceFace(WidgetOccView* occView,
                                                            const QPoint& screenPos)
{
    gp_Pnt faceCenter;
    QString errorMessage;
    if (!resolveReferencePlaneCenter(occView, screenPos, faceCenter, &errorMessage)) {
        // 中文翻译：切割头对齐
        emit operationFailed(tr("Cutting head alignment"), errorMessage);
        return false;
    }

    setCutterHeadModelPosition(faceCenter);
    return true;
}

bool CamModule::pickReferenceFaceCenter(WidgetOccView* occView,
                                        const QPoint& screenPos,
                                        gp_Pnt& center,
                                        QString* errorMessage) const
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CamModule::pickReferenceFaceCenter begin");
    const bool ok = resolveReferencePlaneCenter(occView, screenPos, center, errorMessage);
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CamModule::pickReferenceFaceCenter end ok={}", ok);
    return ok;
}

bool CamModule::enterStandardCalibrationPose(const AxisCalibrationInputs& inputs,
                                             QString* errorMessage)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CamModule::enterStandardCalibrationPose begin");

    auto fail = [&](const QString& msg) {
        if (errorMessage)
            *errorMessage = msg;
        LCNC_ERR(lcnc::LogCode::Generic,
                 "CamModule::enterStandardCalibrationPose failed: {}",
                 msg.toStdString());
        // 中文翻译：机台标定位
        emit operationFailed(tr("Machine mark positioning"), msg);
        return false;
    };

    QString reason;
    if (!ensureAcCenterCalibrationAvailable(&reason))
        return fail(reason);

    MachineKinematics* kin = kinematics();
    if (!kin)
        // 中文翻译：找不到机台轴系配置。
        return fail(tr("The machine axis system configuration cannot be found."));

    try {
        gp_Pnt configuredCenter;
        if (!currentAcRotationCenter(configuredCenter))
            // 中文翻译：请先在应用程序选项的机台构型页填写 A/C 旋转中心。
            return fail(tr("Please fill in the A/C rotation center on the machine configuration page of the application options first."));

        // 切割头模型点（BASE 局部坐标），直接采用拾取面中心。
        m_cutterHeadModelPosition = inputs.cutterHeadFaceCenter;

        // 中文翻译：机台标定位
        // 进入"Machine mark positioning"：A=0, C=0；XY 调整为切割头世界 XY 与配置旋转中心 XY 对齐。
        kin->setAxisPosition(QStringLiteral("A"), 0.0);
        kin->setAxisPosition(QStringLiteral("C"), 0.0);
        if (kin->findAxis(QStringLiteral("X")))
            kin->setAxisPosition(QStringLiteral("X"),
                                 configuredCenter.X() - m_cutterHeadModelPosition.X());
        if (kin->findAxis(QStringLiteral("Y")))
            kin->setAxisPosition(QStringLiteral("Y"),
                                 configuredCenter.Y() - m_cutterHeadModelPosition.Y());

        LCNC_INFO(lcnc::LogCode::Generic,
                  "Standard pose entered: configuredCenter=({:.3f},{:.3f},{:.3f}) "
                  "head.model=({:.3f},{:.3f},{:.3f})",
                  configuredCenter.X(), configuredCenter.Y(), configuredCenter.Z(),
                  m_cutterHeadModelPosition.X(),
                  m_cutterHeadModelPosition.Y(),
                  m_cutterHeadModelPosition.Z());
    } catch (const Standard_Failure& f) {
        // 中文翻译：OCC 异常：%1
        return fail(tr("OCC exception: %1").arg(QString::fromUtf8(f.GetMessageString())));
    } catch (const std::exception& e) {
        // 中文翻译：异常：%1
        return fail(tr("Exception: %1").arg(QString::fromUtf8(e.what())));
    } catch (...) {
        // 中文翻译：发生未知异常。
        return fail(tr("An unknown exception occurred."));
    }

    displayAxisGuides();
    refreshMachineTransforms();
    return true;
}

gp_Pnt CamModule::cutterHeadWorldPosition() const
{
    const MachineKinematics* kin = kinematics();
    if (!kin)
        return m_cutterHeadModelPosition;
    // 切割头几何挂在 Z 轴链下（BASE→Y→X→Z），由该链变换 m_cutterHeadModelPosition。
    gp_Pnt pos = m_cutterHeadModelPosition;
    pos.Transform(kin->computeAxisTransform(QStringLiteral("Z")));
    return pos;
}

bool CamModule::acAngleOffset(double& outA, double& outC) const
{
    if (!m_hasAcAngleOffset)
        return false;
    outA = m_acAngleOffsetA;
    outC = m_acAngleOffsetC;
    return true;
}

bool CamModule::physicalAcCenter(gp_Pnt& outCenter) const
{
    if (!m_hasPhysicalAcCenter)
        return false;
    outCenter = m_physicalAcCenter;
    return true;
}

bool CamModule::isMachineCalibrated() const
{
    // 标定完整的判据：已记录物理中心 + AC 角度映射 + 切割头模型点。
    // 三者通常由 applyAxisCalibration 一次性写入。
    return m_hasPhysicalAcCenter && m_hasAcAngleOffset;
}

bool CamModule::applyAxisCalibration(const AxisCalibrationInputs& inputs,
                                     QString* errorMessage)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CamModule::applyAxisCalibration begin");

    auto fail = [&](const QString& msg) {
        if (errorMessage)
            *errorMessage = msg;
        LCNC_ERR(lcnc::LogCode::Generic,
                 "CamModule::applyAxisCalibration failed: {}",
                 msg.toStdString());
        // 中文翻译：机台坐标系标定
        emit operationFailed(tr("Machine coordinate system calibration"), msg);
        return false;
    };

    // ── 1) 进入标定位（写入轴心、切割头模型点、A=C=0、XY 对齐） ────────
    QString reason;
    if (!enterStandardCalibrationPose(inputs, &reason))
        return fail(reason);

    try {
        gp_Pnt configuredCenter;
        if (!currentAcRotationCenter(configuredCenter))
            // 中文翻译：请先在应用程序选项的机台构型页填写 A/C 旋转中心。
            return fail(tr("Please fill in the A/C rotation center on the machine configuration page of the application options first."));

        // A/C 拾取只用于推导“模型当前的 AC 交点”，不写入物理旋转中心。
        const gp_Pnt pickedModelCenter(inputs.cFaceCenter.X(),
                                       inputs.aFaceCenter.Y(),
                                       inputs.aFaceCenter.Z());
        const gp_Vec translation(pickedModelCenter, configuredCenter);
        if (translation.SquareMagnitude() >= 1e-12) {
            // 中文翻译：机台模型对齐
            if (!translateMachineGeometryOnly(translation, tr("Machine model alignment")))
                return false; // translateMachineGeometryOnly 已发 operationFailed
        } else if (!m_machineModelPath.isEmpty()) {
            m_config.setCutterHeadModelPositionForMachine(m_machineModelPath,
                                                          m_cutterHeadModelPosition);
        }

        // 自动 STEP 回写：对齐后的模型作为下次启动的初始模型。
        if (!m_machineModelPath.isEmpty()) {
            LCNC_INFO(lcnc::LogCode::Generic,
                      "Saving aligned machine model back to: {}",
                      m_machineModelPath.toStdString());
            LcncDocument* doc = machineDocument();
            if (doc && !lcnc::cam::machine_io::exportMachineToFile(
                            doc, doc->machineKinematics(), m_machineModelPath)) {
                LCNC_WARN(lcnc::LogCode::Generic,
                          "Auto-save of machine model failed: {}",
                          m_machineModelPath.toStdString());
                // 仅警告，不中断标定流程
            }
        }
    } catch (const Standard_Failure& f) {
        // 中文翻译：OCC 异常：%1
        return fail(tr("OCC exception: %1").arg(QString::fromUtf8(f.GetMessageString())));
    } catch (const std::exception& e) {
        // 中文翻译：异常：%1
        return fail(tr("Exception: %1").arg(QString::fromUtf8(e.what())));
    } catch (...) {
        // 中文翻译：发生未知异常。
        return fail(tr("An unknown exception occurred."));
    }

    displayAxisGuides();
    refreshMachineDisplay();
    if (hasToolpath())
        updateToolpathMachineCoordinates();

    LCNC_INFO(lcnc::LogCode::Generic,
              "CamModule::applyAxisCalibration done (model geometry aligned; rotation center kept from machine config)");
    emit machineWorkspaceChanged();
    return true;
}

bool CamModule::alignMachineToPhysicalCenter(const gp_Pnt& physicalCenter)
{
    QString errorMessage;
    if (!ensureAcCenterCalibrationAvailable(&errorMessage)) {
        // 中文翻译：机台坐标系转换
        emit operationFailed(tr("Machine coordinate system conversion"), errorMessage);
        return false;
    }

    gp_Pnt currentCenter;
    if (!currentAcRotationCenter(currentCenter)) {
        // 中文翻译：机台坐标系转换；无法计算当前模型 AC 中心。
        emit operationFailed(tr("Machine coordinate system conversion"), tr("Unable to calculate current model AC center."));
        return false;
    }

    const gp_Vec translation(currentCenter, physicalCenter);
    if (translation.SquareMagnitude() < 1e-12)
        return true;

    // 中文翻译：机台坐标系转换
    return translateMachineWorkspace(translation, tr("Machine coordinate system conversion"));
}

bool CamModule::alignMachineToPhysicalCutterHead()
{
    const gp_Vec translation(m_cutterHeadModelPosition, m_cutterHeadPhysicalPosition);
    if (translation.SquareMagnitude() < 1e-12)
        return true;

    // 中文翻译：切割头物理对齐
    return translateMachineWorkspace(translation, tr("Cutting head physical alignment"));
}

bool CamModule::translateMachineWorkspace(const gp_Vec& translation, const QString& operationTitle)
{
    if (translation.SquareMagnitude() < 1e-12)
        return true;

    LcncDocument* doc = machineDocument();
    MachineKinematics* kin = kinematics();
    if (!doc || !kin) {
        // 中文翻译：找不到项目文档或轴系配置。
        emit operationFailed(operationTitle, tr("Project document or axis configuration not found."));
        return false;
    }

    const TDF_LabelSequence machineLabels = doc->entityLabels(LcncDocument::EntityKind::Machine);
    const TDF_LabelSequence workpieceLabels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
    const QList<MachineAxisDef> axisSnapshot = kin->axes();
    const gp_Pnt cutterHeadSnapshot = m_cutterHeadModelPosition;
    QList<TDF_Label> movedLabels;

    auto moveLabels = [&](const TDF_LabelSequence& labels) {
        for (int i = 1; i <= labels.Length(); ++i) {
            const TDF_Label label = labels.Value(i);
            if (label.IsNull())
                continue;

            if (!ShapeService::moveShape(doc, label, translation))
                return false;

            movedLabels.append(label);
        }
        return true;
    };

    auto rollback = [&]() {
        const gp_Vec reverse(-translation.X(), -translation.Y(), -translation.Z());
        for (int i = movedLabels.size() - 1; i >= 0; --i)
            ShapeService::moveShape(doc, movedLabels.at(i), reverse);

        for (const MachineAxisDef& axis : axisSnapshot)
            kin->setAxisOrigin(axis.name, axis.origin);

        m_cutterHeadModelPosition = cutterHeadSnapshot;
    };

    if (!moveLabels(machineLabels) || !moveLabels(workpieceLabels)) {
        rollback();
        // 中文翻译：整机平移失败，当前模型已恢复原始位置。
        emit operationFailed(operationTitle, tr("The translation of the whole machine failed, and the current model has returned to its original position."));
        return false;
    }

    for (const MachineAxisDef& axis : axisSnapshot) {
        const gp_Pnt shiftedOrigin = axis.origin.Translated(translation);
        if (!kin->setAxisOrigin(axis.name, shiftedOrigin)) {
            rollback();
            // 中文翻译：轴心平移失败，当前模型已恢复原始位置。
            emit operationFailed(operationTitle, tr("Axis translation failed and the current model has returned to its original position."));
            return false;
        }
    }

    m_cutterHeadModelPosition.Translate(translation);
    m_workpieceInstallPosition.Translate(translation);
    if (!m_machineModelPath.isEmpty()) {
        CamConfig& config = m_config;
        for (const MachineAxisDef& axis : kin->axes())
            config.setAxisOriginForMachine(m_machineModelPath, axis.name, axis.origin);
        config.setCutterHeadModelPositionForMachine(m_machineModelPath, m_cutterHeadModelPosition);
        config.setWorkpieceInstallPositionForMachine(m_machineModelPath, m_workpieceInstallPosition);
    }
    translateToolpathWorldData(translation);
    refreshMachineDisplay();
    if (hasToolpath() || m_previewLeadInValid)
        refreshToolpathDisplay();

    emit machineWorkspaceChanged();
    return true;
}

bool CamModule::translateMachineGeometryOnly(const gp_Vec& translation, const QString& operationTitle)
{
    if (translation.SquareMagnitude() < 1e-12)
        return true;

    LcncDocument* doc = machineDocument();
    if (!doc) {
        // 中文翻译：找不到机台项目文档。
        emit operationFailed(operationTitle, tr("The machine project document cannot be found."));
        return false;
    }

    const TDF_LabelSequence machineLabels = doc->entityLabels(LcncDocument::EntityKind::Machine);
    const TDF_LabelSequence workpieceLabels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
    const gp_Pnt cutterHeadSnapshot = m_cutterHeadModelPosition;
    const gp_Pnt workpieceInstallSnapshot = m_workpieceInstallPosition;
    QList<TDF_Label> movedLabels;

    auto moveLabels = [&](const TDF_LabelSequence& labels) {
        for (int i = 1; i <= labels.Length(); ++i) {
            const TDF_Label label = labels.Value(i);
            if (label.IsNull())
                continue;

            if (!ShapeService::moveShape(doc, label, translation))
                return false;

            movedLabels.append(label);
        }
        return true;
    };

    auto rollback = [&]() {
        const gp_Vec reverse(-translation.X(), -translation.Y(), -translation.Z());
        for (int i = movedLabels.size() - 1; i >= 0; --i)
            ShapeService::moveShape(doc, movedLabels.at(i), reverse);
        m_cutterHeadModelPosition = cutterHeadSnapshot;
        m_workpieceInstallPosition = workpieceInstallSnapshot;
    };

    if (!moveLabels(machineLabels) || !moveLabels(workpieceLabels)) {
        rollback();
        // 中文翻译：机台几何平移失败，当前模型已恢复原始位置。
        emit operationFailed(operationTitle, tr("The machine geometric translation failed and the current model has been restored to its original position."));
        return false;
    }

    m_cutterHeadModelPosition.Translate(translation);
    m_workpieceInstallPosition.Translate(translation);
    if (!m_machineModelPath.isEmpty()) {
        m_config.setCutterHeadModelPositionForMachine(m_machineModelPath, m_cutterHeadModelPosition);
        m_config.setWorkpieceInstallPositionForMachine(m_machineModelPath, m_workpieceInstallPosition);
    }

    translateToolpathWorldData(translation);
    if (hasToolpath())
        updateToolpathMachineCoordinates();
    refreshMachineDisplay();
    displayAxisGuides();
    if (hasToolpath() || m_previewLeadInValid)
        refreshToolpathDisplay();

    emit machineWorkspaceChanged();
    return true;
}

QList<CamModule::WorkpieceMountCandidate> CamModule::mountableWorkpieces() const
{
    QList<WorkpieceMountCandidate> result;
    LcncDocument* doc = lcnc::Kernel::current().projectManager()->workpieceDocument();
    if (!doc)
        return result;

    const int workpieceCount = doc->entityLabels(LcncDocument::EntityKind::Workpiece).Length();
    if (workpieceCount <= 0)
        return result;

    const QString stateName = lcnc::Kernel::current().projectManager()->session().workpiece().displayName.trimmed();
    // 中文翻译：当前工件
    const QString displayName = stateName.isEmpty() ? tr("current workpiece") : stateName;

    // 中文翻译：%1  (%2 形体)
    result.append({doc->id(), tr("%1 (%2 shape)").arg(displayName).arg(workpieceCount), workpieceCount});

    return result;
}

gp_Pnt CamModule::workpieceInstallPosition() const
{
    return m_workpieceInstallPosition;
}

bool CamModule::autoInstallWorkpiece() const
{
    return m_config.autoInstallWorkpiece();
}

void CamModule::setAutoInstallWorkpiece(bool enabled)
{
    m_config.setAutoInstallWorkpiece(enabled);
    autoInstallCurrentWorkpieceInternal(enabled);
    emit machineWorkspaceChanged();
}

bool CamModule::autoInstallCurrentWorkpiece()
{
    return autoInstallCurrentWorkpieceInternal(m_config.autoInstallWorkpiece());
}

QStringList CamModule::mountedWorkpieceEntriesForSourceEntries(const QStringList& sourceEntries) const
{
    QStringList result;
    const auto appendEntry = [&result](const QString& entry) {
        if (!entry.isEmpty() && !result.contains(entry))
            result.append(entry);
    };

    if (sourceEntries.isEmpty()) {
        if (LcncDocument* doc = workpieceDocument()) {
            const TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
            for (int i = 1; i <= labels.Length(); ++i)
                appendEntry(XcafUtils::entry(labels.Value(i)));
        }
        return result;
    }

    for (const QString& sourceEntry : sourceEntries)
        appendEntry(sourceEntry);
    return result;
}

QStringList CamModule::sourceWorkpieceEntriesForMountedEntries(const QStringList& mountedEntries) const
{
    QStringList result;
    GuiDocument* gd = activeGuiDocument();
    LcncDocument* doc = workpieceDocument();
    if (!gd || !doc)
        return result;

    const QStringList selectedSourceEntries = gd->selectedEntries(doc->id());
    if (selectedSourceEntries.isEmpty())
        return result;

    for (const QString& mountedEntry : mountedEntries) {
        if (!mountedEntry.isEmpty()
            && selectedSourceEntries.contains(mountedEntry)
            && !result.contains(mountedEntry))
            result.append(mountedEntry);
    }
    return result;
}

void CamModule::setMountedWorkpieceEntriesVisible(const QStringList& sourceEntries, bool visible)
{
    GuiDocument* gd = activeGuiDocument();
    const DocumentId docId = workpieceDocumentId();
    if (!gd || docId == kInvalidDocumentId)
        return;

    const QStringList entries = mountedWorkpieceEntriesForSourceEntries(sourceEntries);
    for (const QString& entry : entries) {
        Handle(AIS_Shape) ais = gd->aisShape(docId, entry);
        if (ais.IsNull())
            continue;
        if (visible)
            gd->scene()->displayObject(ais, false);
        else
            gd->scene()->eraseObject(ais, false);
    }
    if (!gd->context().IsNull())
        gd->context()->UpdateCurrentViewer();
    if (gd->hasView())
        gd->view()->Redraw();
}

void CamModule::setSelectedMountedWorkpieceEntries(const QStringList& sourceEntries)
{
    GuiDocument* gd = activeGuiDocument();
    const DocumentId docId = workpieceDocumentId();
    if (!gd || docId == kInvalidDocumentId)
        return;

    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull())
        return;

    ctx->ClearSelected(false);
    for (const QString& entry : mountedWorkpieceEntriesForSourceEntries(sourceEntries)) {
        Handle(AIS_Shape) ais = gd->aisShape(docId, entry);
        if (!ais.IsNull())
            ctx->AddOrRemoveSelected(ais, false);
    }

    if (gd->hasView())
        gd->view()->Redraw();
}

void CamModule::setWorkpieceInstallPosition(const gp_Pnt& position)
{
    const bool currentUnchanged = m_workpieceInstallPosition.SquareDistance(position) < 1e-12;
    const bool bakedUnchanged = m_workpieceInstallPositionBaked.SquareDistance(position) < 1e-12;
    if (currentUnchanged && bakedUnchanged)
        return;

    const gp_Vec translation(m_workpieceInstallPositionBaked, position);
    const bool movedWorkpieces = translation.SquareMagnitude() > 1e-12;
    if (movedWorkpieces && !translateWorkpieceDocument(translation)) {
        // 中文翻译：工件安装位置；更新工件安装位置失败，当前安装位置未修改。
        emit operationFailed(tr("Workpiece installation position"), tr("Failed to update the workpiece installation location. The current installation location has not been modified."));
        return;
    }

    if (movedWorkpieces) {
        translateToolpathWorldData(translation);
        updateToolpathMachineCoordinates();
    }

    m_workpieceInstallPosition = position;
    m_workpieceInstallPositionBaked = position;
    if (!m_machineModelPath.isEmpty()) {
        m_config.setWorkpieceInstallPositionForMachine(
            m_machineModelPath,
            m_workpieceInstallPosition);
    }

    if (movedWorkpieces) {
        resetWorkpieceDisplayLocation();
        refreshWorkpieceDisplay();
        if (hasToolpath()) {
            syncCamDocumentContours(/*forceRebuild=*/true);
            lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
        }
        if (hasToolpath() || m_previewLeadInValid)
            refreshToolpathDisplay();
    }

    emit machineWorkspaceChanged();
}

void CamModule::updateWorkpieceInstallLocation(const gp_Pnt& position)
{
    if (m_workpieceInstallPosition.SquareDistance(position) < 1e-12)
        return;

    m_workpieceInstallPosition = position;
    if (!m_machineModelPath.isEmpty()) {
        m_config.setWorkpieceInstallPositionForMachine(
            m_machineModelPath, m_workpieceInstallPosition);
    }

    auto* gd = activeGuiDocument();
    LcncDocument* doc = workpieceDocument();
    if (!gd || !doc)
        return;

    // 计算 baked → 当前 的平移增量，对所有已展示的工件 AIS 调 SetLocation。
    const gp_Vec delta(m_workpieceInstallPositionBaked, m_workpieceInstallPosition);
    gp_Trsf trsf;
    trsf.SetTranslation(delta);
    const TopLoc_Location loc(trsf);
    const auto& ctx = gd->scene()->context();
    if (ctx.IsNull())
        return;

    const TDF_LabelSequence wpcLabels =
        doc->entityLabels(LcncDocument::EntityKind::Workpiece);
    int updated = 0;
    for (int i = 1; i <= wpcLabels.Length(); ++i) {
        const QString entry = XcafUtils::entry(wpcLabels.Value(i));
        Handle(AIS_Shape) ais = gd->aisShape(doc->id(), entry);
        if (ais.IsNull())
            continue;
        ctx->SetLocation(ais, loc);
        ++updated;
    }
    if (updated > 0)
        ctx->UpdateCurrentViewer();
}

bool CamModule::supportsWorkpieceRotationAlignment() const
{
    gp_Pnt center;
    return currentWorkpieceRotationCenter(center);
}

bool CamModule::alignWorkpieceInstallPositionToRotationCenter()
{
    gp_Pnt center;
    if (!currentWorkpieceRotationCenter(center)) {
        // 中文翻译：工件安装位置；当前构型没有可用于对齐的工件旋转中心。
        emit operationFailed(tr("Workpiece installation position"), tr("There is no workpiece rotation center available for alignment in the current configuration."));
        return false;
    }

    gp_Pnt alignedPosition = m_workpieceInstallPosition;
    alignedPosition.SetX(center.X());
    alignedPosition.SetY(center.Y());
    setWorkpieceInstallPosition(alignedPosition);
    autoInstallCurrentWorkpieceInternal(true);
    return true;
}

void CamModule::autoDetectAxisOrigins()
{
    lcnc::cam::machine_axis_detector::autoDetectAxisOrigins(machineDocument(), kinematics());
}

void CamModule::applyStoredMachineProfile(const QString& machinePath)
{
    MachineKinematics* kin = kinematics();
    if (!kin || machinePath.isEmpty())
        return;

    m_workpieceInstallPosition = defaultWorkpieceInstallPosition();

    const auto profile = lcnc::cam::machine_axis_detector::applyStoredMachineProfile(
        kin, m_config, machinePath);

    if (profile.hasCutterHeadModel)
        m_cutterHeadModelPosition = profile.cutterHeadModelPosition;
    if (profile.hasCutterHeadPhysical)
        m_cutterHeadPhysicalPosition = profile.cutterHeadPhysicalPosition;
    if (profile.hasWorkpieceInstall)
        m_workpieceInstallPosition = profile.workpieceInstallPosition;

    double aOff = 0.0;
    double cOff = 0.0;
    if (m_config.acAngleOffsetForMachine(machinePath, &aOff, &cOff)) {
        m_hasAcAngleOffset = true;
        m_acAngleOffsetA   = aOff;
        m_acAngleOffsetC   = cOff;
        LCNC_INFO(lcnc::LogCode::Generic,
                  "Loaded AC angle offset for machine: A={:.3f}° C={:.3f}°", aOff, cOff);
    } else {
        m_hasAcAngleOffset = false;
        m_acAngleOffsetA = 0.0;
        m_acAngleOffsetC = 0.0;
    }

    gp_Pnt physCenter;
    if (m_config.physicalAcCenterForMachine(machinePath, &physCenter)) {
        m_hasPhysicalAcCenter = true;
        m_physicalAcCenter    = physCenter;
        LCNC_INFO(lcnc::LogCode::Generic,
                  "Loaded physical AC center for machine: ({:.3f},{:.3f},{:.3f})",
                  physCenter.X(), physCenter.Y(), physCenter.Z());
    } else {
        m_hasPhysicalAcCenter = false;
        m_physicalAcCenter    = gp_Pnt(0.0, 0.0, 0.0);
    }
}

bool CamModule::applyConfiguredMachineAxes(bool updateView)
{
    MachineKinematics* kin = kinematics();
    if (!kin || !m_machineConfig)
        return false;

    const QList<MachineAxisDef> axes = m_machineConfig->axisDefinitions();
    if (axes.isEmpty())
        return false;

    kin->setAxes(axes, m_machineConfig->presetName());
    m_config.setMachinePreset(m_machineConfig->presetName());
    if (m_pose && m_pose->kinematics() != kin)
        m_pose->setKinematics(kin);
    if (hasToolpath())
        updateToolpathMachineCoordinates();

    if (!updateView)
        return true;

    displayAxisGuides();
    if (hasToolpath())
        refreshToolpathDisplay();
    refreshMachineTransforms();
    emit axisAssignmentsChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Machine);
    return true;
}

gp_Pnt CamModule::defaultWorkpieceInstallPosition() const
{
    gp_Pnt center;
    if (currentWorkpieceRotationCenter(center))
        return gp_Pnt(center.X(), center.Y(), 0.0);

    return gp_Pnt(0.0, 0.0, 0.0);
}

// ── Workpiece Installation ───────────────────────────────────────────────────

void CamModule::mountWorkpiece(DocumentId sourceDocId, const QString& axisName, bool alignToInstallPosition)
{
    auto* project = lcnc::Kernel::current().projectManager();
    LcncDocument* srcDoc = project->domainDocumentById(sourceDocId);
    if (!srcDoc || !project->isDomainDocument(sourceDocId, lcnc::ProjectDomain::Workpiece))
        return;

    MachineKinematics* kin = kinematics();
    const QString targetAxis = axisName.trimmed().isEmpty()
        ? defaultWorkpieceMountAxis()
        : axisName.trimmed();

    TDF_LabelSequence sourceLabels = srcDoc->entityLabels(LcncDocument::EntityKind::Workpiece);
    if (sourceLabels.Length() == 0)
        return;

    BRep_Builder bb;
    TopoDS_Compound compound;
    bb.MakeCompound(compound);
    bool hasShape = false;
    Handle(XCAFDoc_ShapeTool) sourceShapeTool = srcDoc->shapeTool();
    for (int i = 1; i <= sourceLabels.Length(); ++i) {
        TopoDS_Shape sh = sourceShapeTool->GetShape(sourceLabels.Value(i));
        if (!sh.IsNull()) {
            bb.Add(compound, sh);
            hasShape = true;
        }
    }
    if (!hasShape) return;

    const gp_Pnt currentCenter = shapeCenter(compound);
    const gp_Pnt targetPosition = alignToInstallPosition ? m_workpieceInstallPosition : currentCenter;
    const gp_Vec placement(currentCenter, targetPosition);
    bool mountChanged = false;
    QString firstEntry;
    QMap<QString, QString> mountedEntries;
    for (int i = 1; i <= sourceLabels.Length(); ++i) {
        const QString entry = XcafUtils::entry(sourceLabels.Value(i));
        if (entry.isEmpty())
            continue;
        if (firstEntry.isEmpty())
            firstEntry = entry;
        mountedEntries.insert(entry, entry);
        if (!kin)
            continue;

        const QString currentAxis = kin->mountedAxis(entry);
        if (currentAxis == targetAxis)
            continue;

        if (targetAxis.isEmpty())
            kin->unmountWorkpiece(entry);
        else
            kin->mountWorkpiece(entry, targetAxis);
        mountChanged = true;
    }

    const bool movedWorkpiece = placement.SquareMagnitude() > 1e-12;
    const bool positionUnchanged = m_workpieceInstallPosition.SquareDistance(targetPosition) < 1e-12
        && m_workpieceInstallPositionBaked.SquareDistance(targetPosition) < 1e-12;
    if (!movedWorkpiece && !mountChanged && positionUnchanged)
        return;

    if (movedWorkpiece) {
        clearToolpath();
        if (!translateWorkpieceDocument(placement)) {
            // 中文翻译：工件安装；移动工件到安装位置失败。
            emit operationFailed(tr("Workpiece installation"), tr("Failed to move workpiece to installation location."));
            return;
        }
    }

    m_workpieceInstallPosition = targetPosition;
    m_workpieceInstallPositionBaked = targetPosition;
    m_mountedWorkpieceEntryBySourceEntry = mountedEntries;
    if (!m_machineModelPath.isEmpty())
        m_config.setWorkpieceInstallPositionForMachine(m_machineModelPath, m_workpieceInstallPosition);

    resetWorkpieceDisplayLocation();
    if (movedWorkpiece)
        refreshWorkpieceDisplay();
    else
        refreshMachineTransforms();
    emit machineWorkspaceChanged();
    emit workpieceMounted(firstEntry);
}

bool CamModule::autoInstallCurrentWorkpieceInternal(bool alignToInstallPosition)
{
    if (!alignToInstallPosition)
        return false;

    auto* project = lcnc::Kernel::current().projectManager();
    const DocumentId sourceDocId = project->workpieceDocumentId();
    LcncDocument* sourceDoc = project->domainDocumentById(sourceDocId);
    if (!sourceDoc || sourceDoc->entityLabels(LcncDocument::EntityKind::Workpiece).Length() == 0) {
        clearMountedWorkpieceDisplay(false);
        return false;
    }

    mountWorkpiece(sourceDocId, QString(), true);
    return true;
}

bool CamModule::clearMountedWorkpieceDisplay(bool refreshView)
{
    LcncDocument* doc = machineDocument();
    if (!doc)
        return false;

    MachineKinematics* kin = doc->machineKinematics();
    const QStringList workpieceEntries = entityEntries(doc, LcncDocument::EntityKind::Workpiece);
    const bool hadEntries = !workpieceEntries.isEmpty() || !m_mountedWorkpieceEntryBySourceEntry.isEmpty();
    for (const QString& entry : workpieceEntries) {
        if (kin)
            kin->unmountWorkpiece(entry);
    }
    if (kin) {
        for (const QString& entry : m_mountedWorkpieceEntryBySourceEntry.keys())
            kin->unmountWorkpiece(entry);
        if (LcncDocument* srcDoc = workpieceDocument()) {
            const TDF_LabelSequence sourceLabels = srcDoc->entityLabels(LcncDocument::EntityKind::Workpiece);
            for (int i = 1; i <= sourceLabels.Length(); ++i)
                kin->unmountWorkpiece(XcafUtils::entry(sourceLabels.Value(i)));
        }
    }
    doc->clearEntityKind(LcncDocument::EntityKind::Workpiece);
    m_mountedWorkpieceEntryBySourceEntry.clear();

    if (!hadEntries)
        return false;

    if (refreshView)
        refreshMachineDisplay();
    emit workpieceUnmounted();
    emit machineWorkspaceChanged();
    return true;
}

void CamModule::refreshWorkpieceDisplay()
{
    if (GuiDocument* gd = activeGuiDocument()) {
        gd->rebuildDomain(lcnc::ProjectDomain::Workpiece, workpieceDocument());
        gd->updateMachineWorkspaceTransforms(machineDocument(), workpieceDocument());
        if (gd->hasView())
            gd->view()->Redraw();
    }
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Workpiece);
}

void CamModule::resetWorkpieceDisplayLocation()
{
    GuiDocument* gd = activeGuiDocument();
    LcncDocument* doc = workpieceDocument();
    if (!gd || !doc)
        return;

    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull())
        return;

    const TopLoc_Location identityLoc;
    const TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
    for (int i = 1; i <= labels.Length(); ++i) {
        const QString entry = XcafUtils::entry(labels.Value(i));
        Handle(AIS_Shape) ais = gd->aisShape(doc->id(), entry);
        if (!ais.IsNull())
            ctx->SetLocation(ais, identityLoc);
    }
}

bool CamModule::translateWorkpieceDocument(const gp_Vec& translation)
{
    LcncDocument* doc = workpieceDocument();
    if (!doc || translation.SquareMagnitude() <= 1e-12)
        return true;

    const TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
    QList<TDF_Label> movedLabels;
    for (int i = 1; i <= labels.Length(); ++i) {
        const TDF_Label label = labels.Value(i);
        if (label.IsNull())
            continue;

        if (!ShapeService::moveShape(doc, label, translation)) {
            const gp_Vec rollback(-translation.X(), -translation.Y(), -translation.Z());
            for (int index = movedLabels.size() - 1; index >= 0; --index)
                ShapeService::moveShape(doc, movedLabels.at(index), rollback);
            return false;
        }
        movedLabels.append(label);
    }
    return true;
}

void CamModule::unmountAllWorkpieces()
{
    LcncDocument* doc = machineDocument();
    if (!doc) return;

    MachineKinematics* kin = doc->machineKinematics();
    const QStringList workpieceEntries = entityEntries(doc, LcncDocument::EntityKind::Workpiece);
    bool hadMountedWorkpieces = false;
    for (const QString& entry : workpieceEntries) {
        if (!kin->mountedAxis(entry).isEmpty()) {
            kin->unmountWorkpiece(entry);
            hadMountedWorkpieces = true;
        }
    }
    for (const QString& entry : m_mountedWorkpieceEntryBySourceEntry.keys()) {
        if (!kin->mountedAxis(entry).isEmpty()) {
            kin->unmountWorkpiece(entry);
            hadMountedWorkpieces = true;
        }
    }
    if (LcncDocument* srcDoc = workpieceDocument()) {
        const TDF_LabelSequence sourceLabels = srcDoc->entityLabels(LcncDocument::EntityKind::Workpiece);
        for (int i = 1; i <= sourceLabels.Length(); ++i) {
            const QString entry = XcafUtils::entry(sourceLabels.Value(i));
            if (!kin->mountedAxis(entry).isEmpty()) {
                kin->unmountWorkpiece(entry);
                hadMountedWorkpieces = true;
            }
        }
    }
    m_mountedWorkpieceEntryBySourceEntry.clear();

    const bool hadToolpath = hasToolpath();
    clearToolpath();

    if (hadMountedWorkpieces || hadToolpath)
        refreshMachineTransforms();

    if (hadMountedWorkpieces || hadToolpath) {
        lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Machine);
        emit machineWorkspaceChanged();
    }

    if (hadMountedWorkpieces)
        emit workpieceUnmounted();
}

// ── Shape Operations ──────────────────────────────────────────────────────────

bool CamModule::moveShape(const QString& entry, const gp_Vec& translation)
{
    LcncDocument* doc = machineDocument();
    if (!doc || entry.isEmpty()) return false;

    TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Machine);
    for (int i = 1; i <= labels.Length(); ++i) {
        if (XcafUtils::entry(labels.Value(i)) == entry) {
            bool ok = ShapeService::moveShape(doc, labels.Value(i), translation);
            if (ok) refreshMachineDisplay();
            return ok;
        }
    }

    TDF_LabelSequence wpcLabels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
    for (int i = 1; i <= wpcLabels.Length(); ++i) {
        if (XcafUtils::entry(wpcLabels.Value(i)) == entry) {
            bool ok = ShapeService::moveShape(doc, wpcLabels.Value(i), translation);
            if (ok) refreshMachineDisplay();
            return ok;
        }
    }
    return false;
}

bool CamModule::rotateShape(const QString& entry, const gp_Ax1& axis, double angleDeg)
{
    LcncDocument* doc = machineDocument();
    if (!doc || entry.isEmpty()) return false;

    TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Machine);
    for (int i = 1; i <= labels.Length(); ++i) {
        if (XcafUtils::entry(labels.Value(i)) == entry) {
            bool ok = ShapeService::rotateShape(doc, labels.Value(i), axis, angleDeg);
            if (ok) refreshMachineDisplay();
            return ok;
        }
    }

    TDF_LabelSequence wpcLabels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
    for (int i = 1; i <= wpcLabels.Length(); ++i) {
        if (XcafUtils::entry(wpcLabels.Value(i)) == entry) {
            bool ok = ShapeService::rotateShape(doc, wpcLabels.Value(i), axis, angleDeg);
            if (ok) refreshMachineDisplay();
            return ok;
        }
    }
    return false;
}

void CamModule::deleteShape(const QString& entry)
{
    LcncDocument* doc = machineDocument();
    if (!doc || entry.isEmpty()) return;

    if (auto* gd = activeGuiDocument())
        gd->eraseEntity(doc->id(), entry);

    ShapeService::deleteShape(doc, entry);
    refreshMachineTransforms();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Machine);
}

// ── Toolpath ──────────────────────────────────────────────────────────────────

TopoDS_Shape CamModule::collectWorkpieceShape() const
{
    const QList<WorkpieceShapeSource> sources = collectWorkpieceShapes();
    if (sources.isEmpty())
        return {};

    if (sources.size() == 1)
        return sources.first().shape;

    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    for (const WorkpieceShapeSource& source : sources) {
        if (!source.shape.IsNull())
            builder.Add(compound, source.shape);
    }
    return compound;
}

QList<CamModule::WorkpieceShapeSource> CamModule::collectWorkpieceShapes() const
{
    LcncDocument* doc = workpieceDocument();
    QList<WorkpieceShapeSource> result;
    if (!doc)
        return result;

    TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
    if (labels.Length() == 0)
        return result;

    Handle(XCAFDoc_ShapeTool) st = doc->shapeTool();
    for (int i = 1; i <= labels.Length(); ++i) {
        const TDF_Label label = labels.Value(i);
        const QString workpieceEntry = XcafUtils::entry(label);
        const TopoDS_Shape shape = st->GetShape(label);
        if (shape.IsNull())
            continue;

        bool expanded = false;
        if (shape.ShapeType() == TopAbs_COMPOUND || shape.ShapeType() == TopAbs_COMPSOLID) {
            int componentIndex = 0;
            for (TopoDS_Iterator it(shape, Standard_True, Standard_True); it.More(); it.Next()) {
                const TopoDS_Shape child = it.Value();
                if (child.IsNull())
                    continue;

                result.append({workpieceEntry, child, componentIndex});
                ++componentIndex;
                expanded = true;
            }
        }

        if (!expanded)
            result.append({workpieceEntry, shape, 0});
    }

    return result;
}

bool CamModule::rejectConflictingPipelineOperation(const QString& operation)
{
    if (!property("camAutoPipelineRunning").toBool())
        return false;
    // 中文翻译：全自动加工流程正在运行，请先取消或等待其结束。
    emit operationFailed(operation, tr("The fully automatic processing process is running, please cancel or wait for it to end."));
    return true;
}

std::uint64_t CamModule::machiningFaceSetRevision() const
{
    std::uint64_t hash = 1469598103934665603ull;
    const auto mix = [&hash](std::uint64_t value) {
        hash ^= value + 0x9e3779b97f4a7c15ull + (hash << 6) + (hash >> 2);
    };
    for (const MachiningFaceEntry& entry : m_machiningFaces) {
        mix(entry.faceId);
        mix(entry.face.IsNull() ? 0 : LaserToolpathBuilder::computeFaceSignature(entry.face));
        mix(static_cast<std::uint64_t>(entry.role));
        mix(entry.manual ? 1 : 0);
        for (const QChar ch : entry.workpieceEntry)
            mix(ch.unicode());
    }
    return hash;
}

std::uint64_t CamModule::machineSetupRevision() const
{
    const MachineKinematics* machine = kinematics();
    if (!machine)
        return 0;
    std::uint64_t hash = 1469598103934665603ull;
    const auto mix = [&hash](std::uint64_t value) {
        hash ^= value + 0x9e3779b97f4a7c15ull + (hash << 6) + (hash >> 2);
    };
    const auto mixDouble = [&mix](double value) {
        mix(static_cast<std::uint64_t>(std::llround(value * 1000000.0)));
    };
    for (const QChar ch : machine->configType()) mix(ch.unicode());
    for (const MachineAxisDef& axis : machine->axes()) {
        for (const QChar ch : axis.name) mix(ch.unicode());
        mix(static_cast<std::uint64_t>(axis.motionType));
        mixDouble(axis.direction.X()); mixDouble(axis.direction.Y()); mixDouble(axis.direction.Z());
        mixDouble(axis.origin.X()); mixDouble(axis.origin.Y()); mixDouble(axis.origin.Z());
        mixDouble(axis.minVal); mixDouble(axis.maxVal);
        for (const QChar ch : axis.parentAxis) mix(ch.unicode());
    }
    for (auto it = machine->wpcMounts().cbegin(); it != machine->wpcMounts().cend(); ++it) {
        for (const QChar ch : it.key()) mix(ch.unicode());
        for (const QChar ch : it.value()) mix(ch.unicode());
    }
    return hash;
}

bool facesShareBoundaryEdge(const TopoDS_Face& first, const TopoDS_Face& second)
{
    if (first.IsNull() || second.IsNull())
        return false;
    for (TopExp_Explorer firstEdges(first, TopAbs_EDGE); firstEdges.More(); firstEdges.Next()) {
        const TopoDS_Shape edge = firstEdges.Current();
        for (TopExp_Explorer secondEdges(second, TopAbs_EDGE); secondEdges.More(); secondEdges.Next()) {
            if (edge.IsSame(secondEdges.Current()))
                return true;
        }
    }
    return false;
}

class AutoPipelineRunner final : public QObject
{
public:
    explicit AutoPipelineRunner(CamModule* cam)
        : QObject(cam)
        , m_cam(cam)
    {
        connect(m_cam, &CamModule::pipelineStageChanged, this,
                [this](lcnc::cam::CamPipelineStage stage) {
                    if (stage != m_waitingStage)
                        return;
                    advance();
                });
        connect(m_cam, &CamModule::operationFailed, this,
                [this](const QString& title, const QString&) {
                    if (title == stageTitle(m_waitingStage))
                        finish(false);
                });
    }

    TaskId start(lcnc::cam::CamPipelineStage firstStage =
                 lcnc::cam::CamPipelineStage::FaceSeparation)
    {
        return startStage(firstStage);
    }

private:
    static QString stageTitle(lcnc::cam::CamPipelineStage stage)
    {
        switch (stage) {
        // 中文翻译：分离加工面
        case lcnc::cam::CamPipelineStage::FaceSeparation: return QObject::tr("Separate processing surface");
        // 中文翻译：提取轮廓
        case lcnc::cam::CamPipelineStage::ContourExtraction: return QObject::tr("Extract contours");
        // 中文翻译：离散点
        case lcnc::cam::CamPipelineStage::PointDiscretization: return QObject::tr("discrete points");
        // 中文翻译：构造刀路
        case lcnc::cam::CamPipelineStage::GeometricToolpath: return QObject::tr("Construct toolpath");
        // 中文翻译：求解机床坐标
        case lcnc::cam::CamPipelineStage::MachineSolve: return QObject::tr("Solve for machine coordinates");
        default: return {};
        }
    }

    TaskId startStage(lcnc::cam::CamPipelineStage stage)
    {
        if (!m_cam) {
            finish(false);
            return kInvalidTaskId;
        }
        m_waitingStage = stage;
        TaskId taskId = kInvalidTaskId;
        switch (stage) {
        case lcnc::cam::CamPipelineStage::FaceSeparation:
            taskId = m_cam->separateMachiningFacesAsync(); break;
        case lcnc::cam::CamPipelineStage::ContourExtraction:
            taskId = m_cam->extractContoursFromMachiningFacesAsync(); break;
        case lcnc::cam::CamPipelineStage::PointDiscretization:
            taskId = m_cam->discretizeCurrentContoursAsync(); break;
        case lcnc::cam::CamPipelineStage::GeometricToolpath:
            taskId = m_cam->buildCurrentGeometricToolpathAsync(); break;
        case lcnc::cam::CamPipelineStage::MachineSolve:
            taskId = m_cam->solveCurrentGeometricToolpathAsync(); break;
        default:
            finish(false);
            return kInvalidTaskId;
        }
        if (taskId == kInvalidTaskId)
            finish(false);
        return taskId;
    }

    void advance()
    {
        switch (m_waitingStage) {
        case lcnc::cam::CamPipelineStage::FaceSeparation:
            QTimer::singleShot(0, this, [this] {
                startStage(lcnc::cam::CamPipelineStage::ContourExtraction);
            });
            break;
        case lcnc::cam::CamPipelineStage::ContourExtraction:
            QTimer::singleShot(0, this, [this] {
                startStage(lcnc::cam::CamPipelineStage::PointDiscretization);
            });
            break;
        case lcnc::cam::CamPipelineStage::PointDiscretization:
            QTimer::singleShot(0, this, [this] {
                startStage(lcnc::cam::CamPipelineStage::GeometricToolpath);
            });
            break;
        case lcnc::cam::CamPipelineStage::GeometricToolpath:
            QTimer::singleShot(0, this, [this] {
                startStage(lcnc::cam::CamPipelineStage::MachineSolve);
            });
            break;
        case lcnc::cam::CamPipelineStage::MachineSolve:
            finish(true);
            break;
        default:
            finish(false);
            break;
        }
    }

    void finish(bool success)
    {
        if (m_finished)
            return;
        m_finished = true;
        if (m_cam)
            m_cam->setProperty("camAutoPipelineRunning", false);
        if (success)
            LCNC_INFO(lcnc::LogCode::Generic, "CAM automatic pipeline completed");
        deleteLater();
    }

    QPointer<CamModule> m_cam;
    lcnc::cam::CamPipelineStage m_waitingStage{lcnc::cam::CamPipelineStage::Count};
    bool m_finished{false};
};

bool CamModule::separateMachiningFaces()
{
    const QList<WorkpieceShapeSource> sources = collectWorkpieceShapes();
    if (sources.isEmpty()) {
        // 中文翻译：分离加工面；项目工作区中未找到工件。
        emit operationFailed(tr("Separate processing surface"), tr("Workpiece not found in project workspace."));
        return false;
    }

    // Hand-picked faces are deliberate operator input.  An automatic refresh
    // may replace only auto results; it must never silently resurrect or erase
    // the manual set.
    std::vector<MachiningFaceEntry> result;
    for (const MachiningFaceEntry& entry : m_machiningFaces) {
        if (entry.manual)
            result.push_back(entry);
    }

    const ExtractionStrategy strategy = static_cast<ExtractionStrategy>(m_extractionStrategy);
    if (strategy == ExtractionStrategy::ManualFaceSelection) {
        if (result.empty()) {
            // 中文翻译：应用加工面；手动选面模式下至少需要保留一个加工面。
            emit operationFailed(tr("Application processing surface"), tr("At least one processing surface needs to be reserved in manual surface selection mode."));
            return false;
        }
        m_machiningFaces = std::move(result);
        return applyMachiningFaces();
    }

    auto appendFace = [this, &result](const TopoDS_Face& face,
                                      const QString& workpieceEntry,
                                      lcnc::cam::MachiningFaceRole role) {
        if (face.IsNull())
            return;
        const auto duplicate = std::find_if(result.cbegin(), result.cend(),
            [&face, role](const MachiningFaceEntry& entry) {
                return entry.role == role && !entry.face.IsNull() && entry.face.IsSame(face);
            });
        if (duplicate != result.cend())
            return;
        MachiningFaceEntry entry;
        entry.faceId = m_nextMachiningFaceId++;
        entry.face = face;
        entry.workpieceEntry = workpieceEntry;
        entry.manual = false;
        entry.role = role;
        result.push_back(std::move(entry));
    };

    for (const WorkpieceShapeSource& source : sources) {
        if (source.shape.IsNull())
            continue;

        if (strategy == ExtractionStrategy::TubeClassification
            || strategy == ExtractionStrategy::Auto) {
            const FaceClassification classification = FaceClassifier::classifyFaces(
                source.shape, m_smoothAngle);
            if (const auto* outer = classification.outerGroup()) {
                for (const TopoDS_Face& face : outer->faces)
                    appendFace(face, source.workpieceEntry,
                               lcnc::cam::MachiningFaceRole::MachiningSurface);
            }
            for (const auto* group : classification.crossSectionGroups()) {
                if (!group)
                    continue;
                for (const TopoDS_Face& face : group->faces)
                    appendFace(face, source.workpieceEntry,
                               lcnc::cam::MachiningFaceRole::CrossSection);
            }
            continue;
        }

        const TopoDS_Face face = LaserToolpathBuilder::selectMachiningFace(
            source.shape, beamDirectionWpc(source.workpieceEntry));
        appendFace(face, source.workpieceEntry,
                   lcnc::cam::MachiningFaceRole::MachiningSurface);
    }

    if (result.empty()) {
        // 中文翻译：分离加工面；未识别到可加工面，请改用手动选面。
        emit operationFailed(tr("Separate processing surface"), tr("No machinable surface is identified, please select manual surface instead."));
        return false;
    }

    m_machiningFaces = std::move(result);
    return applyMachiningFaces();
}

TaskId CamModule::separateMachiningFacesAsync()
{
    // 中文翻译：分离加工面
    if (rejectConflictingPipelineOperation(tr("Separate processing surface")))
        return kInvalidTaskId;
    const ExtractionStrategy strategy = static_cast<ExtractionStrategy>(m_extractionStrategy);
    if (strategy == ExtractionStrategy::ManualFaceSelection) {
        // 中文翻译：分离加工面；手动模式请通过拾取加工面后点击“应用加工面并继续”。
        emit operationFailed(tr("Separate processing surface"), tr("In manual mode, please select the processing surface and click \"Apply processing surface and continue\"."));
        return kInvalidTaskId;
    }
    if (property("camFaceSeparationRunning").toBool()) {
        // 中文翻译：分离加工面；加工面分离任务正在执行。
        emit operationFailed(tr("Separate processing surface"), tr("The processing surface separation task is being executed."));
        return kInvalidTaskId;
    }
    const QList<WorkpieceShapeSource> sources = collectWorkpieceShapes();
    auto* taskManager = lcnc::Kernel::current().taskManager();
    if (sources.isEmpty() || !taskManager) {
        // 中文翻译：分离加工面；项目工作区中未找到工件，或后台任务不可用。
        emit operationFailed(tr("Separate processing surface"), tr("The workpiece was not found in the project workspace, or the background task is not available."));
        return kInvalidTaskId;
    }

    struct FaceCandidate {
        TopoDS_Face face;
        QString workpieceEntry;
        lcnc::cam::MachiningFaceRole role{lcnc::cam::MachiningFaceRole::MachiningSurface};
    };
    struct Result { std::vector<FaceCandidate> faces; QString error; bool ok{false}; };
    const auto result = std::make_shared<Result>();
    const double smoothAngle = m_smoothAngle;
    QVector<gp_Dir> beamDirections;
    beamDirections.reserve(sources.size());
    for (const WorkpieceShapeSource& source : sources)
        beamDirections.push_back(beamDirectionWpc(source.workpieceEntry));

    // 中文翻译：分离加工面
    TaskSpec spec{tr("Separate processing surface"), QStringLiteral("cam.pipeline"), TaskPriority::Normal, true};
    setProperty("camFaceSeparationRunning", true);
    const TaskId taskId = taskManager->run(spec,
        [sources, beamDirections, strategy, smoothAngle, result](TaskProgress* progress) {
            progress->setRange(0, std::max(1, static_cast<int>(sources.size())));
            for (int index = 0; index < sources.size(); ++index) {
                if (progress->isAbortRequested())
                    // 中文翻译：加工面分离已取消
                    throw std::runtime_error("Machining surface separation canceled");
                const WorkpieceShapeSource& source = sources.at(index);
                if (source.shape.IsNull())
                    continue;
                if (strategy == ExtractionStrategy::TubeClassification
                    || strategy == ExtractionStrategy::Auto) {
                    const FaceClassification classification = FaceClassifier::classifyFaces(
                        source.shape, smoothAngle);
                    if (const auto* outer = classification.outerGroup()) {
                        for (const TopoDS_Face& face : outer->faces)
                            result->faces.push_back({face, source.workpieceEntry,
                                lcnc::cam::MachiningFaceRole::MachiningSurface});
                    }
                    for (const auto* group : classification.crossSectionGroups()) {
                        if (!group)
                            continue;
                        for (const TopoDS_Face& face : group->faces)
                            result->faces.push_back({face, source.workpieceEntry,
                                lcnc::cam::MachiningFaceRole::CrossSection});
                    }
                } else {
                    const TopoDS_Face face = LaserToolpathBuilder::selectMachiningFace(
                        source.shape, beamDirections.at(index));
                    if (!face.IsNull())
                        result->faces.push_back({face, source.workpieceEntry,
                            lcnc::cam::MachiningFaceRole::MachiningSurface});
                }
                progress->setValue(index + 1);
            }
            if (result->faces.empty()) {
                // 中文翻译：未识别到可加工面，请改用手动选面。
                result->error = QObject::tr("No machinable surface is identified, please select manual surface instead.");
                return;
            }
            result->ok = true;
        });
    m_taskScope.track(taskId);
    watchTask(this, taskId, [this, taskId, result, sources](bool success) {
        m_taskScope.release(taskId);
        setProperty("camFaceSeparationRunning", false);
        if (!success || !result->ok) {
            // 中文翻译：分离加工面
            emit operationFailed(tr("Separate processing surface"), result->error.isEmpty()
                // 中文翻译：加工面分离失败或已取消
                ? tr("Processing surface separation failed or canceled") : result->error);
            return;
        }
        const QList<WorkpieceShapeSource> currentSources = collectWorkpieceShapes();
        const bool sourceChanged = currentSources.size() != sources.size()
            || std::any_of(sources.cbegin(), sources.cend(), [&currentSources](const WorkpieceShapeSource& source) {
                const auto found = std::find_if(currentSources.cbegin(), currentSources.cend(),
                    [&source](const WorkpieceShapeSource& current) {
                        return current.workpieceEntry == source.workpieceEntry
                            && current.shape.IsSame(source.shape);
                    });
                return found == currentSources.cend();
            });
        if (sourceChanged) {
            // 中文翻译：分离加工面；工件在后台识别期间已变更，结果已丢弃。
            emit operationFailed(tr("Separate processing surface"), tr("The workpiece was changed during background identification and the results were discarded."));
            return;
        }

        // Keep any operator-picked faces added while the worker was running,
        // then atomically replace only the automatic portion.
        std::vector<MachiningFaceEntry> merged;
        for (const MachiningFaceEntry& entry : m_machiningFaces) {
            if (entry.manual)
                merged.push_back(entry);
        }
        for (const FaceCandidate& candidate : result->faces) {
            if (candidate.face.IsNull())
                continue;
            const auto duplicate = std::find_if(merged.cbegin(), merged.cend(), [&candidate](const MachiningFaceEntry& entry) {
                return entry.workpieceEntry == candidate.workpieceEntry
                    && entry.role == candidate.role && !entry.face.IsNull()
                    && entry.face.IsSame(candidate.face);
            });
            if (duplicate != merged.cend())
                continue;
            MachiningFaceEntry entry;
            entry.faceId = m_nextMachiningFaceId++;
            entry.face = candidate.face;
            entry.workpieceEntry = candidate.workpieceEntry;
            entry.role = candidate.role;
            merged.push_back(std::move(entry));
        }
        if (merged.empty()) {
            // 中文翻译：分离加工面；加工面识别结果为空。
            emit operationFailed(tr("Separate processing surface"), tr("The processing surface identification result is empty."));
            return;
        }
        m_machiningFaces = std::move(merged);
        applyMachiningFaces();
    });
    return taskId;
}

bool CamModule::applyMachiningFaces()
{
    // 中文翻译：应用加工面
    if (rejectConflictingPipelineOperation(tr("Application processing surface")))
        return false;
    if (!m_camData || m_machiningFaces.empty())
        return false;

    // A changed face set invalidates all derived data.  Preserve stale data for
    // inspection but make Process reject it until the explicit next stage is run.
    for (LaserContour& contour : toolpathRef().contours())
        contour.needsRecalculation = true;
    m_camData->setGenerationParamsDirty(true);
    m_camData->commitPipelineStage(lcnc::cam::CamPipelineStage::FaceSeparation);
    pushMachiningFaceRecordsToCamData();
    refreshMachiningFaceDisplay();
    emit machiningFacesChanged();
    emit pipelineStageChanged(lcnc::cam::CamPipelineStage::FaceSeparation);
    emit toolpathGenerated();
    return true;
}

lcnc::cam::CamPipelineStageState CamModule::pipelineStageState(
    lcnc::cam::CamPipelineStage stage) const
{
    // ProjectExplorer may rebuild during a workspace transition, before the
    // activeWorkspaceChanged slot refreshes m_camData.  Never dereference the
    // cached workspace-owned manager from that transient state.
    const auto* project = lcnc::Kernel::current().projectManager();
    const auto* activeCamData = project ? project->camData() : nullptr;
    return activeCamData ? activeCamData->pipelineStageState(stage)
                         : lcnc::cam::CamPipelineStageState{};
}

bool CamModule::extractContoursFromMachiningFaces()
{
    if (!m_camData || m_machiningFaces.empty()) {
        // 中文翻译：提取轮廓；请先分离或手动应用加工面。
        emit operationFailed(tr("Extract contours"), tr("Please separate or manually apply the machined surface first."));
        return false;
    }
    const auto& faceState = m_camData->pipelineStageState(
        lcnc::cam::CamPipelineStage::FaceSeparation);
    if (!faceState.available || faceState.dirty) {
        // 中文翻译：提取轮廓；加工面尚未应用，请先执行分离面或应用加工面。
        emit operationFailed(tr("Extract contours"), tr("The machined surface has not yet been applied. Please perform separation or application of the machined surface first."));
        return false;
    }

    std::vector<LaserContour> extracted;
    ContourExtractionParams params;
    params.smoothAngleThresholdDeg = m_smoothAngle;
    params.deflection = m_deflection;
    params.strategy = ExtractionStrategy::ManualFaceSelection;
    for (const WorkpieceShapeSource& source : collectWorkpieceShapes()) {
        std::vector<TopoDS_Face> machiningFaces;
        std::vector<TopoDS_Face> crossSectionFaces;
        for (const MachiningFaceEntry& entry : m_machiningFaces) {
            if (entry.workpieceEntry != source.workpieceEntry || entry.face.IsNull())
                continue;
            switch (entry.role) {
            case lcnc::cam::MachiningFaceRole::MachiningSurface: machiningFaces.push_back(entry.face); break;
            case lcnc::cam::MachiningFaceRole::CrossSection: crossSectionFaces.push_back(entry.face); break;
            }
        }
        if (machiningFaces.empty())
            continue;
        params.machiningBeamDirection = beamDirectionWpc(source.workpieceEntry);
        std::vector<LaserContour> contours = !crossSectionFaces.empty()
            ? LaserToolpathBuilder::extractTubeContoursFromFaceGroups(
                source.shape, machiningFaces, crossSectionFaces, params)
            : LaserToolpathBuilder::extractContoursFromFaces(
                source.shape, machiningFaces, params.machiningBeamDirection, params);
        for (LaserContour& contour : contours) {
            contour.workpieceEntry = source.workpieceEntry;
            contour.sourceShape = source.shape;
            contour.appliedParams = {m_config.leadInLength(), m_deflection};
            contour.pendingParams = contour.appliedParams;
            contour.needsRecalculation = true;
            extracted.push_back(std::move(contour));
        }
    }
    if (extracted.empty()) {
        // 中文翻译：提取轮廓；当前加工面中未提取到闭合轮廓。
        emit operationFailed(tr("Extract contours"), tr("No closed contour is extracted from the current processing surface."));
        return false;
    }

    eraseToolpathDisplay();
    toolpathRef().contours() = std::move(extracted);
    toolpathRef().setGlobalLeadInLength(m_config.leadInLength());
    m_camData->ensureContourIds();
    m_camData->ensureToolpathLayers();
    m_camData->commitPipelineStage(lcnc::cam::CamPipelineStage::ContourExtraction,
                                   faceState.revision);
    m_camData->setGenerationParamsDirty(true);
    m_camData->markDirty(true);
    writeContourGeometryToDocument();
    syncCamDocumentContours(/*forceRebuild=*/true);
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    refreshToolpathDisplay();
    setActiveContourId(static_cast<lcnc::cam::ContourId>(toolpathRef().contour(0).contourId));
    emit toolpathGenerated();
    emit toolpathLayersChanged();
    emit pipelineStageChanged(lcnc::cam::CamPipelineStage::ContourExtraction);
    return true;
}

bool CamModule::discretizeCurrentContours()
{
    if (!m_camData || toolpathRef().contourCount() == 0) {
        // 中文翻译：离散点；请先提取轮廓。
        emit operationFailed(tr("discrete points"), tr("Please extract the outline first."));
        return false;
    }
    const auto& contourState = m_camData->pipelineStageState(
        lcnc::cam::CamPipelineStage::ContourExtraction);
    if (!contourState.available || contourState.dirty) {
        // 中文翻译：离散点；轮廓数据已过期，请先重新提取轮廓。
        emit operationFailed(tr("discrete points"), tr("The contour data has expired, please re-extract the contours first."));
        return false;
    }
    for (LaserContour& contour : toolpathRef().contours()) {
        if (contour.sourceShape.IsNull()) {
            // 中文翻译：离散点；轮廓缺少所属工件几何。
            emit operationFailed(tr("discrete points"), tr("The contour lacks the associated workpiece geometry."));
            return false;
        }
        contour.points.clear();
        contour.leadIn = {};
        contour.leadInSolution = {};
        LaserToolpathBuilder::discretizeContour(contour, contour.sourceShape, m_deflection);
        if (contour.points.empty()) {
            // 中文翻译：离散点；轮廓 "%1" 离散失败。
            emit operationFailed(tr("discrete points"), tr("Contour \"%1\" discrete failure.").arg(contour.name));
            return false;
        }
        contour.appliedParams = {m_config.leadInLength(), m_deflection};
        contour.pendingParams = contour.appliedParams;
        contour.needsRecalculation = true;
    }
    m_camData->commitPipelineStage(lcnc::cam::CamPipelineStage::PointDiscretization,
                                   contourState.revision);
    m_camData->setGenerationParamsDirty(true);
    m_camData->markDirty(true);
    refreshToolpathDisplay();
    emit toolpathGenerated();
    emit pipelineStageChanged(lcnc::cam::CamPipelineStage::PointDiscretization);
    return true;
}

bool CamModule::buildCurrentGeometricToolpath()
{
    if (!m_camData || toolpathRef().contourCount() == 0) {
        // 中文翻译：构造刀路；请先完成轮廓离散。
        emit operationFailed(tr("Construct toolpath"), tr("Please complete the contour discretization first."));
        return false;
    }
    const auto& sampleState = m_camData->pipelineStageState(
        lcnc::cam::CamPipelineStage::PointDiscretization);
    if (!sampleState.available || sampleState.dirty) {
        // 中文翻译：构造刀路；离散点数据已过期，请先重新离散。
        emit operationFailed(tr("Construct toolpath"), tr("The discrete point data has expired, please re-discretize first."));
        return false;
    }
    for (LaserContour& contour : toolpathRef().contours()) {
        contour.leadIn.length = contour.pendingParams.leadInLength;
        QString error;
        if (!LaserToolpathBuilder::setAutomaticContourStart(contour, &error)
            || !contour.leadInSolution.valid) {
            if (error.isEmpty())
                error = contour.leadInSolution.error;
            // 中文翻译：构造刀路
            emit operationFailed(tr("Construct toolpath"),
                                 // 中文翻译：轮廓 "%1" 下刀线生成失败：%2
                                 tr("Contour \"%1\" lower cut line generation failed: %2").arg(contour.name, error));
            return false;
        }
        contour.needsRecalculation = true;
    }
    m_camData->commitPipelineStage(lcnc::cam::CamPipelineStage::GeometricToolpath,
                                   sampleState.revision);
    m_camData->setGenerationParamsDirty(true);
    m_camData->markDirty(true);
    refreshToolpathDisplay();
    emit toolpathGenerated();
    emit pipelineStageChanged(lcnc::cam::CamPipelineStage::GeometricToolpath);
    return true;
}

bool CamModule::solveCurrentGeometricToolpath()
{
    if (!m_camData || toolpathRef().contourCount() == 0) {
        // 中文翻译：求解机床坐标；请先构造几何刀路。
        emit operationFailed(tr("Solve for machine coordinates"), tr("Please construct the geometric tool path first."));
        return false;
    }
    const auto& pathState = m_camData->pipelineStageState(
        lcnc::cam::CamPipelineStage::GeometricToolpath);
    if (!pathState.available || pathState.dirty) {
        // 中文翻译：求解机床坐标；几何刀路已过期，请先重新构造刀路。
        emit operationFailed(tr("Solve for machine coordinates"), tr("The geometric tool path has expired, please reconstruct the tool path first."));
        return false;
    }
    if (!solveToolpathForOrder(defaultCuttingOrderByCAxis()))
        return false;
    for (LaserContour& contour : toolpathRef().contours())
        contour.needsRecalculation = false;
    m_camData->commitPipelineStage(lcnc::cam::CamPipelineStage::MachineSolve,
                                   pathState.revision);
    m_camData->setGenerationParamsDirty(false);
    m_camData->markDirty(true);
    m_camData->commitToolpathStates();
    refreshToolpathDisplay();
    refreshTravelPath();
    emit toolpathGenerated();
    emit pipelineStageChanged(lcnc::cam::CamPipelineStage::MachineSolve);
    return true;
}

TaskId CamModule::extractContoursFromMachiningFacesAsync()
{
    // 中文翻译：提取轮廓
    if (rejectConflictingPipelineOperation(tr("Extract contours")))
        return kInvalidTaskId;
    if (!m_camData || m_machiningFaces.empty()) {
        // 中文翻译：提取轮廓；请先分离或手动应用加工面。
        emit operationFailed(tr("Extract contours"), tr("Please separate or manually apply the machined surface first."));
        return kInvalidTaskId;
    }
    const auto faceState = m_camData->pipelineStageState(
        lcnc::cam::CamPipelineStage::FaceSeparation);
    auto* taskManager = lcnc::Kernel::current().taskManager();
    if (!faceState.available || faceState.dirty || !taskManager) {
        // 中文翻译：提取轮廓；加工面尚未应用，或后台任务不可用。
        emit operationFailed(tr("Extract contours"), tr("The machining surface has not yet been applied, or the background task is not available."));
        return kInvalidTaskId;
    }

    const QList<WorkpieceShapeSource> sources = collectWorkpieceShapes();
    const std::vector<MachiningFaceEntry> faces = m_machiningFaces;
    const double smoothAngle = m_smoothAngle;
    const double deflection = m_deflection;
    const double leadInLength = m_config.leadInLength();
    struct Result { std::vector<LaserContour> contours; QString error; bool ok{false}; };
    const auto result = std::make_shared<Result>();
    // 中文翻译：提取加工轮廓
    TaskSpec spec{tr("Extract machining contours"), QStringLiteral("cam.pipeline"), TaskPriority::Normal, true};
    const TaskId taskId = taskManager->run(spec,
        [sources, faces, smoothAngle, deflection, leadInLength, result](TaskProgress* progress) {
            progress->setRange(0, std::max(1, static_cast<int>(sources.size())));
            ContourExtractionParams params;
            params.smoothAngleThresholdDeg = smoothAngle;
            params.deflection = deflection;
            params.strategy = ExtractionStrategy::ManualFaceSelection;
            for (int sourceIndex = 0; sourceIndex < sources.size(); ++sourceIndex) {
                if (progress->isAbortRequested())
                    // 中文翻译：轮廓提取已取消
                    throw std::runtime_error("Contour extraction canceled");
                const WorkpieceShapeSource& source = sources.at(sourceIndex);
                std::vector<TopoDS_Face> machiningFaces;
                std::vector<TopoDS_Face> crossSectionFaces;
                for (const MachiningFaceEntry& entry : faces) {
                    if (entry.workpieceEntry != source.workpieceEntry || entry.face.IsNull())
                        continue;
                    switch (entry.role) {
                    case lcnc::cam::MachiningFaceRole::MachiningSurface: machiningFaces.push_back(entry.face); break;
                    case lcnc::cam::MachiningFaceRole::CrossSection: crossSectionFaces.push_back(entry.face); break;
                    }
                }
                if (!machiningFaces.empty()) {
                    auto contours = !crossSectionFaces.empty()
                        ? LaserToolpathBuilder::extractTubeContoursFromFaceGroups(
                            source.shape, machiningFaces, crossSectionFaces, params)
                        : LaserToolpathBuilder::extractContoursFromFaces(
                            source.shape, machiningFaces, gp_Dir(0.0, 0.0, -1.0), params);
                    for (LaserContour& contour : contours) {
                        contour.workpieceEntry = source.workpieceEntry;
                        contour.sourceShape = source.shape;
                        contour.appliedParams = {leadInLength, deflection};
                        contour.pendingParams = contour.appliedParams;
                        contour.needsRecalculation = true;
                        result->contours.push_back(std::move(contour));
                    }
                }
                progress->setValue(sourceIndex + 1);
            }
            if (result->contours.empty()) {
                // 中文翻译：当前加工面中未提取到闭合轮廓。
                result->error = QObject::tr("No closed contour is extracted from the current processing surface.");
                return;
            }
            result->ok = true;
        });
    m_taskScope.track(taskId);
    watchTask(this, taskId, [this, taskId, result, faceRevision = faceState.revision](bool success) {
        m_taskScope.release(taskId);
        if (!success || !result->ok) {
            // 中文翻译：提取轮廓
            emit operationFailed(tr("Extract contours"), result->error.isEmpty()
                // 中文翻译：轮廓提取失败或已取消
                ? tr("Contour extraction failed or canceled") : result->error);
            return;
        }
        const auto currentFace = m_camData->pipelineStageState(
            lcnc::cam::CamPipelineStage::FaceSeparation);
        if (currentFace.dirty || currentFace.revision != faceRevision) {
            // 中文翻译：提取轮廓；加工面在后台计算期间已变更，结果已丢弃。
            emit operationFailed(tr("Extract contours"), tr("The machining surface has changed during background calculation and the results have been discarded."));
            return;
        }
        eraseToolpathDisplay();
        toolpathRef().contours() = std::move(result->contours);
        toolpathRef().setGlobalLeadInLength(m_config.leadInLength());
        m_camData->ensureContourIds();
        m_camData->ensureToolpathLayers();
        m_camData->commitPipelineStage(lcnc::cam::CamPipelineStage::ContourExtraction,
                                       currentFace.revision);
        m_camData->setGenerationParamsDirty(true);
        m_camData->markDirty(true);
        writeContourGeometryToDocument();
        syncCamDocumentContours(true);
        lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
        refreshToolpathDisplay();
        setActiveContourId(static_cast<lcnc::cam::ContourId>(toolpathRef().contour(0).contourId));
        emit toolpathGenerated();
        emit toolpathLayersChanged();
        emit pipelineStageChanged(lcnc::cam::CamPipelineStage::ContourExtraction);
    });
    return taskId;
}

TaskId CamModule::discretizeCurrentContoursAsync()
{
    // 中文翻译：离散点
    if (rejectConflictingPipelineOperation(tr("discrete points")))
        return kInvalidTaskId;
    if (!m_camData || toolpathRef().contourCount() == 0) {
        // 中文翻译：离散点；请先提取轮廓。
        emit operationFailed(tr("discrete points"), tr("Please extract the outline first."));
        return kInvalidTaskId;
    }
    const auto contourState = m_camData->pipelineStageState(
        lcnc::cam::CamPipelineStage::ContourExtraction);
    auto* taskManager = lcnc::Kernel::current().taskManager();
    if (!contourState.available || contourState.dirty || !taskManager) {
        // 中文翻译：离散点；轮廓数据已过期，或后台任务不可用。
        emit operationFailed(tr("discrete points"), tr("The profile data has expired, or the background task is unavailable."));
        return kInvalidTaskId;
    }
    const std::vector<LaserContour> input = toolpathRef().contours();
    const double deflection = m_deflection;
    const double leadInLength = m_config.leadInLength();
    struct Result { std::vector<LaserContour> contours; QString error; bool ok{false}; };
    const auto result = std::make_shared<Result>();
    // 中文翻译：离散加工轮廓
    TaskSpec spec{tr("Discrete machining contours"), QStringLiteral("cam.pipeline"), TaskPriority::Normal, true};
    const TaskId taskId = taskManager->run(spec,
        [input, deflection, leadInLength, result](TaskProgress* progress) {
            progress->setRange(0, std::max(1, static_cast<int>(input.size())));
            result->contours = input;
            for (std::size_t index = 0; index < result->contours.size(); ++index) {
                if (progress->isAbortRequested())
                    // 中文翻译：轮廓离散已取消
                    throw std::runtime_error("Contour discretization canceled");
                LaserContour& contour = result->contours[index];
                if (contour.sourceShape.IsNull()) {
                    // 中文翻译：轮廓 "%1" 缺少工件几何。
                    result->error = QObject::tr("Contour \"%1\" is missing workpiece geometry.").arg(contour.name);
                    return;
                }
                contour.points.clear();
                contour.leadIn = {};
                contour.leadInSolution = {};
                LaserToolpathBuilder::discretizeContour(contour, contour.sourceShape, deflection);
                if (contour.points.empty()) {
                    // 中文翻译：轮廓 "%1" 离散失败。
                    result->error = QObject::tr("Contour \"%1\" discrete failure.").arg(contour.name);
                    return;
                }
                contour.appliedParams = {leadInLength, deflection};
                contour.pendingParams = contour.appliedParams;
                contour.needsRecalculation = true;
                progress->setValue(static_cast<int>(index + 1));
            }
            result->ok = true;
        });
    m_taskScope.track(taskId);
    watchTask(this, taskId, [this, taskId, result, contourRevision = contourState.revision](bool success) {
        m_taskScope.release(taskId);
        if (!success || !result->ok) {
            // 中文翻译：离散点
            emit operationFailed(tr("discrete points"), result->error.isEmpty()
                // 中文翻译：轮廓离散失败或已取消
                ? tr("Contour discretization failed or canceled") : result->error);
            return;
        }
        const auto currentContour = m_camData->pipelineStageState(
            lcnc::cam::CamPipelineStage::ContourExtraction);
        if (currentContour.dirty || currentContour.revision != contourRevision) {
            // 中文翻译：离散点；轮廓在后台计算期间已变更，结果已丢弃。
            emit operationFailed(tr("discrete points"), tr("The contour has changed during background calculation and the results have been discarded."));
            return;
        }
        toolpathRef().contours() = std::move(result->contours);
        m_camData->commitPipelineStage(lcnc::cam::CamPipelineStage::PointDiscretization,
                                       currentContour.revision);
        m_camData->setGenerationParamsDirty(true);
        m_camData->markDirty(true);
        refreshToolpathDisplay();
        emit toolpathGenerated();
        emit pipelineStageChanged(lcnc::cam::CamPipelineStage::PointDiscretization);
    });
    return taskId;
}

TaskId CamModule::buildCurrentGeometricToolpathAsync()
{
    // 中文翻译：构造刀路
    if (rejectConflictingPipelineOperation(tr("Construct toolpath")))
        return kInvalidTaskId;
    if (!m_camData || toolpathRef().contourCount() == 0) {
        // 中文翻译：构造刀路；请先完成轮廓离散。
        emit operationFailed(tr("Construct toolpath"), tr("Please complete the contour discretization first."));
        return kInvalidTaskId;
    }
    const auto sampleState = m_camData->pipelineStageState(
        lcnc::cam::CamPipelineStage::PointDiscretization);
    auto* taskManager = lcnc::Kernel::current().taskManager();
    if (!sampleState.available || sampleState.dirty || !taskManager) {
        // 中文翻译：构造刀路；离散点数据已过期，或后台任务不可用。
        emit operationFailed(tr("Construct toolpath"), tr("The discrete point data is out of date, or the background task is unavailable."));
        return kInvalidTaskId;
    }
    const std::vector<LaserContour> input = toolpathRef().contours();
    struct Result { std::vector<LaserContour> contours; QString error; bool ok{false}; };
    const auto result = std::make_shared<Result>();
    TaskSpec spec;
    // 中文翻译：构造几何刀路
    spec.label = tr("Construct geometry toolpath");
    spec.scope = QStringLiteral("cam.pipeline");
    spec.priority = TaskPriority::Normal;
    spec.cancellable = true;
    const TaskId taskId = taskManager->run(spec,
        [input, result](TaskProgress* progress) {
            progress->setRange(0, std::max(1, static_cast<int>(input.size())));
            result->contours = input;
            for (std::size_t index = 0; index < result->contours.size(); ++index) {
                if (progress->isAbortRequested())
                    // 中文翻译：几何刀路构造已取消
                    throw std::runtime_error("Geometric toolpath construction has been cancelled");
                LaserContour& contour = result->contours[index];
                contour.leadIn.length = contour.pendingParams.leadInLength;
                QString error;
                if (!LaserToolpathBuilder::setAutomaticContourStart(contour, &error)
                    || !contour.leadInSolution.valid) {
                    result->error = error.isEmpty() ? contour.leadInSolution.error : error;
                    return;
                }
                contour.needsRecalculation = true;
                progress->setValue(static_cast<int>(index + 1));
            }
            result->ok = true;
        });
    m_taskScope.track(taskId);
    watchTask(this, taskId, [this, taskId, result, sampleRevision = sampleState.revision](bool success) {
        m_taskScope.release(taskId);
        if (!success || !result->ok) {
            // 中文翻译：构造刀路
            emit operationFailed(tr("Construct toolpath"), result->error.isEmpty()
                // 中文翻译：几何刀路构造失败或已取消
                ? tr("Geometric toolpath construction failed or canceled") : result->error);
            return;
        }
        const auto currentSamples = m_camData->pipelineStageState(
            lcnc::cam::CamPipelineStage::PointDiscretization);
        if (currentSamples.dirty || currentSamples.revision != sampleRevision) {
            // 中文翻译：构造刀路；离散点在后台计算期间已变更，结果已丢弃。
            emit operationFailed(tr("Construct toolpath"), tr("The discrete points were changed during background calculation and the results were discarded."));
            return;
        }
        toolpathRef().contours() = std::move(result->contours);
        m_camData->commitPipelineStage(lcnc::cam::CamPipelineStage::GeometricToolpath,
                                       currentSamples.revision);
        m_camData->setGenerationParamsDirty(true);
        m_camData->markDirty(true);
        refreshToolpathDisplay();
        emit toolpathGenerated();
        emit pipelineStageChanged(lcnc::cam::CamPipelineStage::GeometricToolpath);
    });
    return taskId;
}

TaskId CamModule::solveCurrentGeometricToolpathAsync()
{
    // 中文翻译：求解机床坐标
    if (rejectConflictingPipelineOperation(tr("Solve for machine coordinates")))
        return kInvalidTaskId;
    if (!m_camData || toolpathRef().contourCount() == 0) {
        // 中文翻译：求解机床坐标；请先构造几何刀路。
        emit operationFailed(tr("Solve for machine coordinates"), tr("Please construct the geometric tool path first."));
        return kInvalidTaskId;
    }
    const auto pathState = m_camData->pipelineStageState(
        lcnc::cam::CamPipelineStage::GeometricToolpath);
    auto* taskManager = lcnc::Kernel::current().taskManager();
    MachineKinematics* machine = kinematics();
    if (!pathState.available || pathState.dirty || !taskManager || !machine) {
        // 中文翻译：求解机床坐标；几何刀路已过期，或机台后台服务不可用。
        emit operationFailed(tr("Solve for machine coordinates"), tr("The geometric tool path has expired, or the machine background service is unavailable."));
        return kInvalidTaskId;
    }
    const std::vector<LaserContour> input = toolpathRef().contours();
    const QVector<lcnc::cam::ContourId> order = defaultCuttingOrderByCAxis();
    const QList<MachineAxisDef> axes = machine->axes();
    const QString configType = machine->configType();
    struct Result { std::vector<LaserContour> contours; QString error; bool ok{false}; };
    const auto result = std::make_shared<Result>();
    TaskSpec spec;
    // 中文翻译：求解机床坐标
    spec.label = tr("Solve for machine coordinates");
    spec.scope = QStringLiteral("cam.pipeline");
    spec.priority = TaskPriority::Normal;
    spec.cancellable = true;
    const TaskId taskId = taskManager->run(spec,
        [input, order, axes, configType, result](TaskProgress* progress) {
            progress->setRange(0, 100);
            result->contours = input;
            QSet<std::uint64_t> seen;
            std::vector<LaserContour*> ordered;
            ordered.reserve(static_cast<std::size_t>(order.size()));
            for (std::uint64_t id : order) {
                if (id == 0 || seen.contains(id))
                    continue;
                const auto it = std::find_if(result->contours.begin(), result->contours.end(),
                    [id](const LaserContour& contour) { return contour.contourId == id; });
                if (it == result->contours.end())
                    continue;
                seen.insert(id);
                for (ToolpathPoint& point : it->points)
                    point.machineCoord = {};
                if (it->leadInSolution.valid)
                    it->leadInSolution.point.machineCoord = {};
                ordered.push_back(&*it);
            }
            if (ordered.empty()) {
                // 中文翻译：当前没有可求解的轮廓顺序。
                result->error = QObject::tr("There is currently no contour sequence to solve.");
                return;
            }
            MachineKinematics workerKinematics;
            workerKinematics.setAxes(axes, configType);
            LaserToolpathBuilder::computeMachineCoordinatesForOrder(
                ordered, &workerKinematics, gp_Trsf(), nullptr);
            for (const LaserContour& contour : result->contours) {
                if (!contour.leadInSolution.valid
                    || !contour.leadInSolution.point.machineCoord.valid
                    || !std::all_of(contour.points.begin(), contour.points.end(),
                        [](const ToolpathPoint& point) { return point.machineCoord.valid; })) {
                    // 中文翻译：轮廓 "%1" 五轴坐标求解失败。
                    result->error = QObject::tr("Contour \"%1\" five-axis coordinate solution failed.").arg(contour.name);
                    return;
                }
            }
            result->ok = true;
            progress->setValue(100);
        });
    m_taskScope.track(taskId);
    watchTask(this, taskId, [this, taskId, result, pathRevision = pathState.revision](bool success) {
        m_taskScope.release(taskId);
        if (!success || !result->ok) {
            // 中文翻译：求解机床坐标
            emit operationFailed(tr("Solve for machine coordinates"), result->error.isEmpty()
                // 中文翻译：机床坐标求解失败或已取消
                ? tr("Machine tool coordinate solution failed or canceled") : result->error);
            return;
        }
        const auto currentPath = m_camData->pipelineStageState(
            lcnc::cam::CamPipelineStage::GeometricToolpath);
        if (currentPath.dirty || currentPath.revision != pathRevision) {
            // 中文翻译：求解机床坐标；几何刀路在后台计算期间已变更，结果已丢弃。
            emit operationFailed(tr("Solve for machine coordinates"), tr("The geometry toolpath was changed during background calculation and the results were discarded."));
            return;
        }
        toolpathRef().contours() = std::move(result->contours);
        for (LaserContour& contour : toolpathRef().contours())
            contour.needsRecalculation = false;
        m_camData->commitPipelineStage(lcnc::cam::CamPipelineStage::MachineSolve,
                                       currentPath.revision);
        m_camData->setGenerationParamsDirty(false);
        m_camData->markDirty(true);
        m_camData->commitToolpathStates();
        refreshToolpathDisplay();
        refreshTravelPath();
        emit toolpathGenerated();
        emit pipelineStageChanged(lcnc::cam::CamPipelineStage::MachineSolve);
    });
    return taskId;
}

TaskId CamModule::runAutoPipelineAsync(AutoPipelineFaceMode mode)
{
    if (property("camAutoPipelineRunning").toBool()) {
        // 中文翻译：全自动执行；已有自动加工流程正在执行。
        emit operationFailed(tr("Fully automatic execution"), tr("There is already an automatic processing process being executed."));
        return kInvalidTaskId;
    }
    const ExtractionStrategy strategy = static_cast<ExtractionStrategy>(m_extractionStrategy);
    if (strategy == ExtractionStrategy::ManualFaceSelection) {
        if (!applyMachiningFaces())
            return kInvalidTaskId;
    }
    // A global generation is always one TaskManager job.  Manual-stage
    // buttons remain the only entry points that create separate stage tasks.
    setProperty("camAutoPipelineRunning", true);
    const TaskId automaticTask = generateToolpathAsync(m_smoothAngle,
                                                        m_useFaceClassification,
                                                        m_deflection,
                                                        mode);
    if (automaticTask == kInvalidTaskId) {
        setProperty("camAutoPipelineRunning", false);
        return automaticTask;
    }
    watchTask(this, automaticTask, [this](bool) {
        setProperty("camAutoPipelineRunning", false);
    });
    return automaticTask;
}

bool CamModule::runAutoPipeline(AutoPipelineFaceMode mode)
{
    return runAutoPipelineAsync(mode) != kInvalidTaskId;
}

bool CamModule::generateToolpath(double smoothAngle, bool useFaceClassification, double deflection)
{
    const QList<WorkpieceShapeSource> workpieceSources = collectWorkpieceShapes();
    if (workpieceSources.isEmpty())
        return false;

    // LaserToolpath::clear() 会复位运行时值；全局生成始终以程序配置为准。
    const double leadInLength = m_config.leadInLength();
    const LaserToolpath previousToolpath = toolpathRef();
    const QVector<lcnc::cam::ContourId> previousOrder =
        m_camData->layerContainer().manualContourOrder();
    const auto previousSortStrategy = m_camData->layerContainer().sortStrategy();
    const auto previousAutoSortAxis = m_camData->layerContainer().lastAutoSortAxis();
    QHash<std::uint64_t, LaserContour> previousBySignature;
    for (const LaserContour& contour : previousToolpath.contours())
        previousBySignature.insert(contour.signature, contour);
    // Phase C：CAM 已不再镜像到 XCAF，AIS 在 syncCamDocumentContours 中按 contourId 重建。
    m_workpieceShape = collectWorkpieceShape();
    bool effectiveUseFaceClassification = useFaceClassification;
    if (m_machineConfig) {
        switch (m_machineConfig->toolpathAlgorithm()) {
        case lcnc::MachineToolpathAlgorithm::ThreeAxis:
            effectiveUseFaceClassification = false;
            break;
        case lcnc::MachineToolpathAlgorithm::FiveAxisTable:
        case lcnc::MachineToolpathAlgorithm::FiveAxisHead:
            effectiveUseFaceClassification = true;
            break;
        }
        LCNC_INFO(lcnc::LogCode::Generic,
                  "cam.toolpath: machine algorithm='{}' faceClassification={}",
                  m_machineConfig->toolpathAlgorithmText().toStdString(),
                  effectiveUseFaceClassification);
    }
    m_smoothAngle = smoothAngle;
    m_useFaceClassification = effectiveUseFaceClassification;
    m_deflection = deflection;

    ContourExtractionParams params;
    params.smoothAngleThresholdDeg = smoothAngle;
    params.strategy = static_cast<ExtractionStrategy>(m_extractionStrategy);
    if (params.strategy == ExtractionStrategy::ManualFaceSelection)
        params.selectedMachiningFaces = manualMachiningFaces();
    params.deflection = deflection;
    std::vector<LaserContour> allContours;

    for (const WorkpieceShapeSource& source : workpieceSources) {
        if (source.shape.IsNull())
            continue;

        params.machiningBeamDirection = beamDirectionWpc(source.workpieceEntry);
        FaceClassification classification;
        auto contours = LaserToolpathBuilder::extractContours(source.shape, params, &classification);
        if (contours.empty())
            continue;

        std::vector<TopoDS_Face> outerFaces;
        std::vector<TopoDS_Face> crossFaces;
        if (effectiveUseFaceClassification) {
            if (classification.outerGroup())
                outerFaces = classification.outerGroup()->faces;
            for (const auto* group : classification.crossSectionGroups()) {
                if (group)
                    crossFaces.insert(crossFaces.end(), group->faces.begin(), group->faces.end());
            }
        }

        for (auto& contour : contours) {
            const auto oldIt = previousBySignature.constFind(contour.signature);
            const LaserContour* oldContour = oldIt == previousBySignature.constEnd()
                ? nullptr : &oldIt.value();
            if (oldContour) {
                contour.contourId = oldContour->contourId;
                contour.layerId = oldContour->layerId;
                contour.enabled = oldContour->enabled;
                contour.name = oldContour->name;
                contour.leadIn = oldContour->leadIn;
                if (contour.leadIn.entryEdgeIndex < 0 && !oldContour->points.empty())
                    contour.leadIn.entryEdgeIndex = oldContour->points.front().sourceEdgeIndex;
            }
            // 全局生成以当前程序级全局参数统一重建所有轮廓；轮廓级参数仅由
            // “重新计算当前轮廓”写入和随工程保存，下一次全局生成会被此处覆盖。
            contour.leadIn.length = leadInLength;
            contour.appliedParams = {leadInLength, deflection};
            contour.pendingParams = contour.appliedParams;
            contour.needsRecalculation = false;
            contour.workpieceEntry = source.workpieceEntry;
            contour.sourceShape = source.shape;
            if (workpieceSources.size() > 1) {
                contour.sourceInfo = contour.sourceInfo.isEmpty()
                    // 中文翻译：工件源 #%1
                    ? tr("Workpiece source #%1").arg(source.componentIndex + 1)
                    // 中文翻译：%1 · 工件源 #%2
                    : tr("%1 · Workpiece source #%2").arg(contour.sourceInfo).arg(source.componentIndex + 1);
            }

            if (oldContour && oldContour->leadIn.valid) {
                if (!outerFaces.empty() && !crossFaces.empty())
                    LaserToolpathBuilder::discretizeContourWithClassification(
                        contour, outerFaces, crossFaces, deflection);
                else
                    LaserToolpathBuilder::discretizeContour(contour, source.shape, deflection);
            } else if (contour.points.empty()) {
                LaserToolpathBuilder::discretizeContour(contour, source.shape, deflection);
            }

            if (!contour.points.empty()) {
                int startIndex = 0;
                const bool hasManualStart = oldContour && oldContour->leadIn.valid;
                if (oldContour && oldContour->leadIn.valid) {
                    auto selected = std::find_if(contour.points.begin(), contour.points.end(),
                        [&contour](const ToolpathPoint& point) {
                            return point.sourceEdgeIndex == contour.leadIn.entryEdgeIndex
                                && std::abs(point.param - contour.leadIn.entryParam) <= 1e-10;
                        });
                    if (selected == contour.points.end() && contour.leadIn.entryEdgeIndex < 0) {
                        selected = std::min_element(contour.points.begin(), contour.points.end(),
                            [&contour](const ToolpathPoint& a, const ToolpathPoint& b) {
                                return a.position.SquareDistance(contour.leadIn.entryPoint)
                                    < b.position.SquareDistance(contour.leadIn.entryPoint);
                            });
                    }
                    if (selected == contour.points.end()) {
                        // 中文翻译：全局生成刀路
                        emit operationFailed(tr("Generate toolpath globally"),
                                             // 中文翻译：轮廓 "%1" 无法恢复人工起点
                                             tr("Contour \"%1\" cannot restore artificial starting point").arg(contour.name));
                        return false;
                    }
                    if (oldContour->leadIn.entryEdgeIndex >= 0
                        && selected->position.Distance(oldContour->leadIn.entryPoint) > 1e-6) {
                        // 中文翻译：全局生成刀路
                        emit operationFailed(tr("Generate toolpath globally"),
                                             // 中文翻译：轮廓 "%1" 的起点拓扑锚点已变化
                                             tr("The starting topology anchor point of contour \"%1\" has changed").arg(contour.name));
                        return false;
                    }
                    startIndex = static_cast<int>(std::distance(contour.points.begin(), selected));
                }
                QString leadInError;
                const bool startSet = hasManualStart
                    ? LaserToolpathBuilder::setContourStart(
                          contour, startIndex, &leadInError)
                    : LaserToolpathBuilder::setAutomaticContourStart(
                          contour, &leadInError);
                if (!startSet
                    || !contour.leadInSolution.valid) {
                    if (leadInError.isEmpty())
                        leadInError = contour.leadInSolution.error;
                    // 中文翻译：全局生成刀路
                    emit operationFailed(tr("Generate toolpath globally"),
                                         // 中文翻译：轮廓 "%1" 下刀点生成失败：%2
                                         tr("Contour \"%1\" cutting point generation failed: %2")
                                             .arg(contour.name, leadInError));
                    return false;
                }
            }

            allContours.push_back(std::move(contour));
        }
    }

    if (allContours.empty())
        return false;

    eraseToolpathDisplay();
    toolpathRef().contours() = std::move(allContours);
    toolpathRef().setGlobalLeadInLength(leadInLength);
    m_camData->ensureContourIds();
    m_camData->ensureToolpathLayers();
    QVector<lcnc::cam::ContourId> solveOrder;
    QSet<lcnc::cam::ContourId> retainedIds;
    for (lcnc::cam::ContourId id : previousOrder) {
        if (contourIndexById(id) >= 0 && !retainedIds.contains(id)) {
            retainedIds.insert(id);
            solveOrder.append(id);
        }
    }
    for (const LaserContour& contour : toolpathRef().contours()) {
        const auto id = static_cast<lcnc::cam::ContourId>(contour.contourId);
        if (!retainedIds.contains(id)) {
            retainedIds.insert(id);
            solveOrder.append(id);
        }
    }
    if (auto* manager = m_camData->layerManager()) {
        manager->setManualContourOrder(solveOrder);
        manager->setSortStrategy(lcnc::cam::CuttingPlanSortStrategy::Manual);
    }
    if (!solveToolpathForOrder(solveOrder)) {
        toolpathRef() = previousToolpath;
        m_camData->layerContainer().setManualContourOrder(previousOrder);
        m_camData->layerContainer().setSortStrategy(previousSortStrategy);
        m_camData->layerContainer().setLastAutoSortAxis(previousAutoSortAxis);
        refreshToolpathDisplay();
        return false;
    }
    pushGenerationParamsToCamData();
    auto applied = m_camData->generationParams();
    applied.useFaceClassification = effectiveUseFaceClassification;
    m_camData->appliedGenerationParams() = applied;
    m_camData->setGenerationParamsDirty(false);
    m_camData->markDirty(true);
    m_camData->commitToolpathStates();    // 把 signature → id 映射固化下来，跨次稳定

    // Auto-capture machining faces for the tree/view (sync path).
    if (m_extractionStrategy == static_cast<int>(ExtractionStrategy::Auto)
        || m_extractionStrategy == static_cast<int>(ExtractionStrategy::PlanarFaceWires)) {
        std::vector<TopoDS_Face> autoFaces;
        for (const WorkpieceShapeSource& src : workpieceSources) {
            if (src.shape.IsNull()) continue;
            const TopoDS_Face f = LaserToolpathBuilder::selectMachiningFace(
                src.shape, beamDirectionWpc(src.workpieceEntry));
            if (!f.IsNull())
                autoFaces.push_back(f);
        }
        setAutoMachiningFaces(autoFaces, QString());
    } else {
        setAutoMachiningFaces({}, QString());
    }

    writeContourGeometryToDocument();      // 轮廓 wire 写入统一工程文档(EntityKind::Cam)
    relinkContourGeometryFromDocument();   // 与工程包加载路径一致：显示/拾取使用 XCAF 文档版 wire
    syncCamDocumentContours(/*forceRebuild=*/true);
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    m_toolpathRenderer->setVisible(activeGuiDocument(), true);

    refreshToolpathDisplay();
    if (contourIndexById(m_activeContourId) < 0)
        setActiveContourId(toolpathRef().contourCount() > 0
            ? static_cast<lcnc::cam::ContourId>(toolpathRef().contour(0).contourId) : 0);
    emit toolpathGenerated();
    emit toolpathLayersChanged();
    refreshTravelPath();
    return true;
}

TaskId CamModule::generateToolpathAsync(double smoothAngle, bool useFaceClassification, double deflection,
                                        AutoPipelineFaceMode mode)
{
    const bool reuseCurrentFaces = (mode == AutoPipelineFaceMode::ReuseCurrent);
    const QList<WorkpieceShapeSource> workpieceSources = collectWorkpieceShapes();
    auto* taskManager = lcnc::Kernel::current().taskManager();
    if (workpieceSources.isEmpty() || !taskManager)
        return kInvalidTaskId;

    bool effectiveUseFaceClassification = useFaceClassification;
    if (m_machineConfig) {
        switch (m_machineConfig->toolpathAlgorithm()) {
        case lcnc::MachineToolpathAlgorithm::ThreeAxis:
            effectiveUseFaceClassification = false;
            break;
        case lcnc::MachineToolpathAlgorithm::FiveAxisTable:
        case lcnc::MachineToolpathAlgorithm::FiveAxisHead:
            effectiveUseFaceClassification = true;
            break;
        }
    }
    const double leadInLength = m_config.leadInLength();
    const QVector<lcnc::cam::ContourId> previousOrder =
        m_camData->layerContainer().manualContourOrder();
    QHash<std::uint64_t, LaserContour> previousBySignature;
    for (const LaserContour& contour : toolpathRef().contours())
        previousBySignature.insert(contour.signature, contour);

    struct GenerationResult {
        std::vector<LaserContour> contours;
        QString error;
        bool ok{false};
    };
    const auto result = std::make_shared<GenerationResult>();
    const std::uint64_t sourceRevision = toolpathRevision();
    const std::uint64_t faceSetRevision = machiningFaceSetRevision();
    const std::uint64_t setupRevision = machineSetupRevision();
    const int extractionStrategy = m_extractionStrategy;
    MachineKinematics* machine = kinematics();
    if (!machine) {
        // 中文翻译：全局生成刀路；找不到机台运动学配置
        emit operationFailed(tr("Generate toolpath globally"), tr("Machine kinematics configuration not found"));
        return kInvalidTaskId;
    }
    const QList<MachineAxisDef> axes = machine->axes();
    const QString configType = machine->configType();
    ContourExtractionParams params;
    params.smoothAngleThresholdDeg = smoothAngle;
    params.strategy = static_cast<ExtractionStrategy>(m_extractionStrategy);
    // Reusing the current face set means the operator confirmed "use current
    // machining faces": drive the worker through the explicit-face path so the
    // current m_machiningFaces is the sole face input, instead of re-running
    // auto face selection and stacking detected faces on top of the manual picks.
    if (reuseCurrentFaces)
        params.strategy = ExtractionStrategy::ManualFaceSelection;
    if (params.strategy == ExtractionStrategy::ManualFaceSelection)
        params.selectedMachiningFaces = manualMachiningFaces();
    params.deflection = deflection;
    const std::vector<MachiningFaceEntry> selectedFaceEntries = m_machiningFaces;
    lcnc::cam::ToolpathGenerationStamp generationStamp;
    generationStamp.toolpathRevision = sourceRevision;
    generationStamp.machiningFaceRevision = faceSetRevision;
    generationStamp.machineSetupRevision = setupRevision;
    generationStamp.leadInLength = leadInLength;
    generationStamp.smoothAngle = smoothAngle;
    generationStamp.deflection = deflection;
    generationStamp.useFaceClassification = effectiveUseFaceClassification;
    generationStamp.extractionStrategy = extractionStrategy;
    generationStamp.contourIds.reserve(toolpathRef().contours().size());
    for (const auto& contour : toolpathRef().contours())
        generationStamp.contourIds.push_back(contour.contourId);
    generationStamp.sources.reserve(static_cast<std::size_t>(workpieceSources.size()));
    for (const WorkpieceShapeSource& source : workpieceSources) {
        generationStamp.sources.push_back(
            {source.workpieceEntry, source.componentIndex, source.shape});
    }

    // Beam direction is workpiece-mount-dependent, so compute it per source on
    // this (main) thread where the kinematics wpc mounts live; the worker only
    // gets a axes/configType copy without mounts.
    QVector<gp_Dir> beamDirs;
    beamDirs.reserve(workpieceSources.size());
    for (const WorkpieceShapeSource& s : workpieceSources)
        beamDirs.append(beamDirectionWpc(s.workpieceEntry));

    TaskSpec spec;
    // 中文翻译：全局生成刀路
    spec.label = tr("Generate toolpath globally");
    spec.scope = QStringLiteral("cam.toolpath");
    spec.priority = TaskPriority::Normal;
    const TaskId taskId = taskManager->run(spec,
        [workpieceSources, previousBySignature, previousOrder, leadInLength, params, selectedFaceEntries,
         beamDirs, effectiveUseFaceClassification, axes, configType, result](TaskProgress* progress) {
            progress->setRange(0, 100);
            // 中文翻译：1/5 正在分离加工面与横截面
            progress->setStepName(QObject::tr("1/5 Separating the machined surface and cross section"));
            progress->setValue(5);
            std::vector<LaserContour> allContours;
            for (int sourceIndex = 0; sourceIndex < workpieceSources.size(); ++sourceIndex) {
                if (progress->isAbortRequested())
                    // 中文翻译：全局刀路生成已取消
                    throw std::runtime_error("Global toolpath generation canceled");
                const WorkpieceShapeSource& source = workpieceSources.at(sourceIndex);
                // 中文翻译：2/5 正在提取工件轮廓 %1/%2
                progress->setStepName(QObject::tr("2/5 Extracting workpiece contour %1/%2")
                    .arg(sourceIndex + 1).arg(workpieceSources.size()));
                if (source.shape.IsNull())
                    continue;
                ContourExtractionParams perSourceParams = params;
                perSourceParams.machiningBeamDirection =
                    beamDirs.value(sourceIndex, gp_Dir(0.0, 0.0, -1.0));
                FaceClassification classification;
                std::vector<TopoDS_Face> outerFaces;
                std::vector<TopoDS_Face> crossFaces;
                std::vector<LaserContour> contours;
                if (perSourceParams.strategy == ExtractionStrategy::ManualFaceSelection) {
                    for (const MachiningFaceEntry& entry : selectedFaceEntries) {
                        if (entry.workpieceEntry != source.workpieceEntry || entry.face.IsNull())
                            continue;
                        if (entry.role == lcnc::cam::MachiningFaceRole::MachiningSurface)
                            outerFaces.push_back(entry.face);
                        else if (entry.role == lcnc::cam::MachiningFaceRole::CrossSection)
                            crossFaces.push_back(entry.face);
                    }
                    if (outerFaces.empty()) {
                        // 中文翻译：工件源 #%1 未提供加工面。
                        result->error = QObject::tr("Workpiece source #%1 does not provide a machining surface.")
                            .arg(source.componentIndex + 1);
                        return;
                    }
                    contours = crossFaces.empty()
                        ? LaserToolpathBuilder::extractContoursFromFaces(
                            source.shape, outerFaces, perSourceParams.machiningBeamDirection, perSourceParams)
                        : LaserToolpathBuilder::extractTubeContoursFromFaceGroups(
                            source.shape, outerFaces, crossFaces, perSourceParams);
                } else {
                    const FaceClassification autoClassification = FaceClassifier::classifyFaces(
                        source.shape, perSourceParams.smoothAngleThresholdDeg);
                    if ((perSourceParams.strategy == ExtractionStrategy::Auto
                         || perSourceParams.strategy == ExtractionStrategy::TubeClassification)
                        && (!autoClassification.hasOuter() || !autoClassification.hasCrossSection())) {
                        // 中文翻译：工件源 #%1 无法可靠识别加工面与横截面，请手动调整面组。
                        result->error = QObject::tr("Workpiece source #%1 cannot reliably identify the processing surface and cross section. Please adjust the quilt manually.")
                            .arg(source.componentIndex + 1);
                        return;
                    }
                    contours = LaserToolpathBuilder::extractContours(
                        source.shape, perSourceParams, &classification);
                    if (classification.groups.empty())
                        classification = autoClassification;
                }
                if (effectiveUseFaceClassification && outerFaces.empty()) {
                    if (classification.outerGroup())
                        outerFaces = classification.outerGroup()->faces;
                    for (const auto* group : classification.crossSectionGroups())
                        if (group)
                            crossFaces.insert(crossFaces.end(), group->faces.begin(), group->faces.end());
                }
                for (std::size_t contourIndex = 0; contourIndex < contours.size(); ++contourIndex) {
                    if ((contourIndex % 16u) == 0u && progress->isAbortRequested())
                        // 中文翻译：全局刀路生成已取消
                        throw std::runtime_error("Global toolpath generation canceled");
                    auto& contour = contours[contourIndex];
                    const auto oldIt = previousBySignature.constFind(contour.signature);
                    const LaserContour* oldContour = oldIt == previousBySignature.constEnd()
                        ? nullptr : &oldIt.value();
                    if (oldContour) {
                        contour.contourId = oldContour->contourId;
                        contour.layerId = oldContour->layerId;
                        contour.enabled = oldContour->enabled;
                        contour.name = oldContour->name;
                        contour.leadIn = oldContour->leadIn;
                        if (contour.leadIn.entryEdgeIndex < 0 && !oldContour->points.empty())
                            contour.leadIn.entryEdgeIndex = oldContour->points.front().sourceEdgeIndex;
                    }
                    contour.leadIn.length = leadInLength;
                    contour.appliedParams = {leadInLength, params.deflection};
                    contour.pendingParams = contour.appliedParams;
                    contour.needsRecalculation = false;
                    contour.workpieceEntry = source.workpieceEntry;
                    contour.sourceShape = source.shape;
                    if (workpieceSources.size() > 1) {
                        contour.sourceInfo = contour.sourceInfo.isEmpty()
                            // 中文翻译：工件源 #%1
                            ? QObject::tr("Workpiece source #%1").arg(source.componentIndex + 1)
                            // 中文翻译：%1 · 工件源 #%2
                            : QObject::tr("%1 · Workpiece source #%2").arg(contour.sourceInfo).arg(source.componentIndex + 1);
                    }
                    if (oldContour && oldContour->leadIn.valid) {
                        if (!outerFaces.empty() && !crossFaces.empty())
                            LaserToolpathBuilder::discretizeContourWithClassification(
                                contour, outerFaces, crossFaces, params.deflection);
                        else
                            LaserToolpathBuilder::discretizeContour(contour, source.shape, params.deflection);
                    } else if (contour.points.empty()) {
                        LaserToolpathBuilder::discretizeContour(contour, source.shape, params.deflection);
                    }
                    if (!contour.points.empty()) {
                        int startIndex = 0;
                        const bool hasManualStart = oldContour && oldContour->leadIn.valid;
                        if (oldContour && oldContour->leadIn.valid) {
                            auto selected = std::find_if(contour.points.begin(), contour.points.end(),
                                [&contour](const ToolpathPoint& point) {
                                    return point.sourceEdgeIndex == contour.leadIn.entryEdgeIndex
                                        && std::abs(point.param - contour.leadIn.entryParam) <= 1e-10;
                                });
                            if (selected == contour.points.end() && contour.leadIn.entryEdgeIndex < 0) {
                                selected = std::min_element(contour.points.begin(), contour.points.end(),
                                    [&contour](const ToolpathPoint& a, const ToolpathPoint& b) {
                                        return a.position.SquareDistance(contour.leadIn.entryPoint)
                                            < b.position.SquareDistance(contour.leadIn.entryPoint);
                                    });
                            }
                            if (selected == contour.points.end()
                                || (oldContour->leadIn.entryEdgeIndex >= 0
                                    && selected->position.Distance(oldContour->leadIn.entryPoint) > 1e-6)) {
                                // 中文翻译：轮廓 "%1" 的人工起点无法恢复
                                result->error = QObject::tr("Artificial starting point for contour \"%1\" cannot be restored").arg(contour.name);
                                return;
                            }
                            startIndex = static_cast<int>(std::distance(contour.points.begin(), selected));
                        }
                        QString leadInError;
                        const bool startSet = hasManualStart
                            ? LaserToolpathBuilder::setContourStart(
                                  contour, startIndex, &leadInError)
                            : LaserToolpathBuilder::setAutomaticContourStart(
                                  contour, &leadInError);
                        if (!startSet
                            || !contour.leadInSolution.valid) {
                            result->error = leadInError.isEmpty() ? contour.leadInSolution.error : leadInError;
                            return;
                        }
                    }
                    allContours.push_back(std::move(contour));
                }
                progress->setValue(10 + (40 * (sourceIndex + 1))
                    / std::max(1, static_cast<int>(workpieceSources.size())));
            }
            if (allContours.empty()) {
                // 中文翻译：未找到可用的轮廓边缘
                result->error = QObject::tr("No available contour edges found");
                return;
            }
            // 中文翻译：3/5 正在离散轮廓点
            progress->setStepName(QObject::tr("3/5 Discretizing contour points"));
            progress->setValue(60);
            // 中文翻译：4/5 正在构造下刀线与几何刀路
            progress->setStepName(QObject::tr("4/5 Constructing knife lines and geometric tool paths"));
            progress->setValue(70);
            // 中文翻译：5/5 正在求解机台坐标
            progress->setStepName(QObject::tr("5/5 Solving machine coordinates"));
            MachineKinematics workerKinematics;
            workerKinematics.setAxes(axes, configType);
            std::vector<LaserContour*> solveContours;
            solveContours.reserve(allContours.size());
            QSet<std::uint64_t> retainedIds;
            for (const lcnc::cam::ContourId id : previousOrder) {
                if (id == 0 || retainedIds.contains(id))
                    continue;
                const auto found = std::find_if(allContours.begin(), allContours.end(),
                    [id](const LaserContour& contour) { return contour.contourId == id; });
                if (found != allContours.end()) {
                    retainedIds.insert(id);
                    solveContours.push_back(&*found);
                }
            }
            for (LaserContour& contour : allContours) {
                if (contour.contourId != 0 && retainedIds.contains(contour.contourId))
                    continue;
                if (contour.contourId != 0)
                    retainedIds.insert(contour.contourId);
                solveContours.push_back(&contour);
            }
            LaserToolpathBuilder::computeMachineCoordinatesForOrder(
                solveContours, &workerKinematics, gp_Trsf(), nullptr);
            const bool coordinatesValid = std::all_of(allContours.begin(), allContours.end(),
                [](const LaserContour& contour) {
                    return contour.leadInSolution.valid
                        && contour.leadInSolution.point.machineCoord.valid
                        && std::all_of(contour.points.begin(), contour.points.end(),
                            [](const ToolpathPoint& point) { return point.machineCoord.valid; });
                });
            if (!coordinatesValid) {
                // 中文翻译：全局五轴刀路求解失败
                result->error = QObject::tr("Global five-axis tool path solution failed");
                return;
            }
            result->contours = std::move(allContours);
            result->ok = true;
            progress->setValue(100);
        });

    m_taskScope.track(taskId);
    watchTask(this, taskId,
        [this, taskId, result, leadInLength, previousOrder,
         effectiveUseFaceClassification, smoothAngle, deflection,
         generationStamp, reuseCurrentFaces](bool success) {
            m_taskScope.release(taskId);
            if (!success || !result->ok) {
                // 中文翻译：全局生成刀路
                emit operationFailed(tr("Generate toolpath globally"),
                    // 中文翻译：刀路生成失败或已取消
                    result->error.isEmpty() ? tr("Tool path generation failed or canceled") : result->error);
                return;
            }
            const QList<WorkpieceShapeSource> currentSources = collectWorkpieceShapes();
            bool currentEffectiveFaceClassification = m_useFaceClassification;
            if (m_machineConfig) {
                switch (m_machineConfig->toolpathAlgorithm()) {
                case lcnc::MachineToolpathAlgorithm::ThreeAxis:
                    currentEffectiveFaceClassification = false;
                    break;
                case lcnc::MachineToolpathAlgorithm::FiveAxisTable:
                case lcnc::MachineToolpathAlgorithm::FiveAxisHead:
                    currentEffectiveFaceClassification = true;
                    break;
                }
            }
            lcnc::cam::ToolpathGenerationStamp currentStamp;
            currentStamp.toolpathRevision = toolpathRevision();
            currentStamp.machiningFaceRevision = machiningFaceSetRevision();
            currentStamp.machineSetupRevision = machineSetupRevision();
            currentStamp.leadInLength = m_config.leadInLength();
            currentStamp.smoothAngle = m_smoothAngle;
            currentStamp.deflection = m_deflection;
            currentStamp.useFaceClassification = currentEffectiveFaceClassification;
            currentStamp.extractionStrategy = m_extractionStrategy;
            const auto& currentContours = toolpathRef().contours();
            currentStamp.contourIds.reserve(currentContours.size());
            for (const auto& contour : currentContours)
                currentStamp.contourIds.push_back(contour.contourId);
            currentStamp.sources.reserve(static_cast<std::size_t>(currentSources.size()));
            for (const WorkpieceShapeSource& source : currentSources) {
                currentStamp.sources.push_back(
                    {source.workpieceEntry, source.componentIndex, source.shape});
            }
            if (!lcnc::cam::ToolpathGenerationService::acceptsResult(
                    generationStamp, currentStamp, true, false)) {
                // 中文翻译：全局生成刀路；刀路在计算期间已变更，后台结果已丢弃
                emit operationFailed(tr("Generate toolpath globally"), tr("The tool path has changed during calculation and the background results have been discarded"));
                return;
            }
            eraseToolpathDisplay();
            toolpathRef().contours() = std::move(result->contours);
            toolpathRef().setGlobalLeadInLength(leadInLength);
            m_smoothAngle = smoothAngle;
            m_useFaceClassification = effectiveUseFaceClassification;
            m_deflection = deflection;
            m_workpieceShape = collectWorkpieceShape();
            if (!reuseCurrentFaces) {
                // Capture the exact two face groups used by Auto/Tube.  They are
                // project data, not a renderer-only side effect: later manual
                // stages continue from these groups without reclassifying.
                std::vector<MachiningFaceEntry> captured;
                for (const MachiningFaceEntry& entry : m_machiningFaces)
                    if (entry.manual)
                        captured.push_back(entry);
                const ExtractionStrategy currentStrategy =
                    static_cast<ExtractionStrategy>(m_extractionStrategy);
                for (const WorkpieceShapeSource& src : currentSources) {
                    if (src.shape.IsNull())
                        continue;
                    auto appendCaptured = [this, &captured, &src](const TopoDS_Face& face,
                                                                   lcnc::cam::MachiningFaceRole role) {
                        if (face.IsNull())
                            return;
                        const auto duplicate = std::find_if(captured.cbegin(), captured.cend(),
                            [&face, &src, role](const MachiningFaceEntry& entry) {
                                return entry.workpieceEntry == src.workpieceEntry
                                    && entry.role == role && !entry.face.IsNull()
                                    && entry.face.IsSame(face);
                            });
                        if (duplicate != captured.cend())
                            return;
                        MachiningFaceEntry entry;
                        entry.faceId = m_nextMachiningFaceId++;
                        entry.face = face;
                        entry.workpieceEntry = src.workpieceEntry;
                        entry.role = role;
                        captured.push_back(std::move(entry));
                    };
                    if (currentStrategy == ExtractionStrategy::Auto
                        || currentStrategy == ExtractionStrategy::TubeClassification) {
                        const FaceClassification classification = FaceClassifier::classifyFaces(
                            src.shape, m_smoothAngle);
                        if (const auto* group = classification.outerGroup())
                            for (const TopoDS_Face& face : group->faces)
                                appendCaptured(face, lcnc::cam::MachiningFaceRole::MachiningSurface);
                        for (const auto* group : classification.crossSectionGroups()) {
                            if (!group) continue;
                            for (const TopoDS_Face& face : group->faces)
                                appendCaptured(face, lcnc::cam::MachiningFaceRole::CrossSection);
                        }
                    } else if (currentStrategy == ExtractionStrategy::PlanarFaceWires) {
                        appendCaptured(LaserToolpathBuilder::selectMachiningFace(
                            src.shape, beamDirectionWpc(src.workpieceEntry)),
                            lcnc::cam::MachiningFaceRole::MachiningSurface);
                    }
                }
                m_machiningFaces = std::move(captured);
                pushMachiningFaceRecordsToCamData();
                refreshMachiningFaceDisplay();
                emit machiningFacesChanged();
            }
            // This worker completed all five stages as one atomic automatic
            // operation.  Commit the persisted stage chain only now, after
            // the frozen inputs have passed the stale-result checks above.
            m_camData->commitPipelineStage(lcnc::cam::CamPipelineStage::FaceSeparation);
            const auto faceStage = m_camData->pipelineStageState(
                lcnc::cam::CamPipelineStage::FaceSeparation);
            m_camData->commitPipelineStage(lcnc::cam::CamPipelineStage::ContourExtraction,
                                           faceStage.revision);
            const auto contourStage = m_camData->pipelineStageState(
                lcnc::cam::CamPipelineStage::ContourExtraction);
            m_camData->commitPipelineStage(lcnc::cam::CamPipelineStage::PointDiscretization,
                                           contourStage.revision);
            const auto pointStage = m_camData->pipelineStageState(
                lcnc::cam::CamPipelineStage::PointDiscretization);
            m_camData->commitPipelineStage(lcnc::cam::CamPipelineStage::GeometricToolpath,
                                           pointStage.revision);
            const auto pathStage = m_camData->pipelineStageState(
                lcnc::cam::CamPipelineStage::GeometricToolpath);
            m_camData->commitPipelineStage(lcnc::cam::CamPipelineStage::MachineSolve,
                                           pathStage.revision);
            m_camData->ensureContourIds();
            m_camData->ensureToolpathLayers();
            QVector<lcnc::cam::ContourId> solveOrder;
            QSet<lcnc::cam::ContourId> retainedIds;
            for (lcnc::cam::ContourId id : previousOrder) {
                if (contourIndexById(id) >= 0 && !retainedIds.contains(id)) {
                    retainedIds.insert(id);
                    solveOrder.append(id);
                }
            }
            for (const LaserContour& contour : toolpathRef().contours()) {
                const auto id = static_cast<lcnc::cam::ContourId>(contour.contourId);
                if (!retainedIds.contains(id)) {
                    retainedIds.insert(id);
                    solveOrder.append(id);
                }
            }
            if (auto* manager = m_camData->layerManager()) {
                manager->setManualContourOrder(solveOrder);
                manager->setSortStrategy(lcnc::cam::CuttingPlanSortStrategy::Manual);
            }
            pushGenerationParamsToCamData();
            auto applied = m_camData->generationParams();
            applied.useFaceClassification = effectiveUseFaceClassification;
            m_camData->appliedGenerationParams() = applied;
            m_camData->setGenerationParamsDirty(false);
            m_camData->markDirty(true);
            m_camData->commitToolpathStates();
            writeContourGeometryToDocument();
            relinkContourGeometryFromDocument();
            syncCamDocumentContours(/*forceRebuild=*/true);
            lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
            m_toolpathRenderer->setVisible(activeGuiDocument(), true);
            refreshToolpathDisplay();
            if (contourIndexById(m_activeContourId) < 0)
                setActiveContourId(toolpathRef().contourCount() > 0
                    ? static_cast<lcnc::cam::ContourId>(toolpathRef().contour(0).contourId) : 0);
            emit toolpathGenerated();
            emit toolpathLayersChanged();
            emit pipelineStageChanged(lcnc::cam::CamPipelineStage::MachineSolve);
            refreshTravelPath();
        });
    return taskId;
}

void CamModule::clearToolpath()
{
    clearToolpathViewState(/*emitSignals=*/false);
    m_camData->clearToolpath();
    // 加工面集合随刀路一并清空（避免上一工程/工件的高亮残留）。
    m_machiningFaces.clear();
    m_camData->setMachiningFaceRecords({});
    refreshMachiningFaceDisplay();
    // 统一工程文档：清掉轮廓几何(EntityKind::Cam)实体。
    if (LcncDocument* doc = workpieceDocument())
        doc->clearEntityKind(LcncDocument::EntityKind::Cam);
    m_clearingToolpath = true;
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    m_clearingToolpath = false;
    emit toolpathCleared();
    emit toolpathLayersChanged();
    emit machiningFacesChanged();
}

bool CamModule::resolveReferencePlaneCenter(WidgetOccView* occView,
                                            const QPoint& screenPos,
                                            gp_Pnt& center,
                                            QString* errorMessage) const
{
    return lcnc::cam::reference_pick::resolveReferencePlaneCenter(occView, screenPos, center, errorMessage);
}

bool CamModule::ensureAcCenterCalibrationAvailable(QString* errorMessage) const
{
    const MachineKinematics* kin = kinematics();
    if (!kin) {
        if (errorMessage)
            // 中文翻译：找不到机台轴系配置。请先选择 AC 转台构型。
            *errorMessage = tr("The machine axis system configuration cannot be found. Please select the AC turntable configuration first.");
        return false;
    }

    if (kin->configType() != QStringLiteral("VERTICAL_AC_TABLE")) {
        if (errorMessage)
            // 中文翻译：当前仅 VERTICAL_AC_TABLE 构型支持 A/C 模型对齐。
            *errorMessage = tr("A/C model alignment is currently supported only for the VERTICAL_AC_TABLE configuration.");
        return false;
    }

    if (!kin->findAxis(QStringLiteral("A")) || !kin->findAxis(QStringLiteral("C"))) {
        if (errorMessage)
            // 中文翻译：当前 AC 转台轴定义不完整，缺少 A 轴或 C 轴。\n请先在应用程序选项的机台构型页完成配置。
            *errorMessage = tr("The current AC rotary table axis definition is incomplete, either the A or C axis is missing.\nPlease complete the configuration on the Machine Configuration page of the application options first.");
        return false;
    }

    return true;
}

void CamModule::translateToolpathWorldData(const gp_Vec& translation)
{
    if (translation.SquareMagnitude() < 1e-12)
        return;

    for (LaserContour& contour : toolpathRef().contours()) {
        const TopoDS_Shape movedWire = translatedShapeCopy(contour.wire, translation);
        if (!movedWire.IsNull() && movedWire.ShapeType() == TopAbs_WIRE)
            contour.wire = TopoDS::Wire(movedWire);
        contour.sourceShape = translatedShapeCopy(contour.sourceShape, translation);

        for (ToolpathPoint& point : contour.points)
            point.position.Translate(translation);

        if (contour.leadIn.valid)
            contour.leadIn.entryPoint.Translate(translation);
        if (contour.leadInSolution.valid)
            contour.leadInSolution.point.position.Translate(translation);
    }

    m_workpieceShape = translatedShapeCopy(m_workpieceShape, translation);

    if (m_previewLeadInValid)
        m_previewLeadInPoint.Translate(translation);
}

void CamModule::updateToolpathMachineCoordinates()
{
    if (auto provider = lcnc::Kernel::current()
                            .services()
                            .getService<lcnc::process::IProcessCuttingPlanProvider>()) {
        const auto order = provider->orderedContourIds();
        solveToolpathForOrder(QVector<std::uint64_t>(order.cbegin(), order.cend()));
        return;
    }

    solveToolpathForOrder(defaultCuttingOrderByCAxis());
}

QVector<lcnc::cam::ContourId> CamModule::defaultCuttingOrderByCAxis() const
{
    struct OrderItem
    {
        lcnc::cam::ContourId id{0};
        double axial{0.0};
        double angle{0.0};
        int originalIndex{0};
    };

    QVector<OrderItem> items;
    items.reserve(toolpathRef().contourCount());

    const MachineKinematics* kin = kinematics();
    const MachineAxisDef* cAxis = kin ? kin->findAxis(QStringLiteral("C")) : nullptr;
    const bool hasCAxis = cAxis && cAxis->motionType == MachineAxisDef::Rotary;

    gp_Pnt origin(0.0, 0.0, 0.0);
    gp_Dir axisDir(0.0, 0.0, 1.0);
    gp_Dir zeroDir(1.0, 0.0, 0.0);
    if (hasCAxis) {
        const gp_Trsf axisTrsf = kin->computeAxisTransform(cAxis->name);
        origin = cAxis->origin.Transformed(axisTrsf);
        axisDir = cAxis->direction.Transformed(axisTrsf);

        const gp_Vec axisVec(axisDir);
        gp_Vec zero(gp_Dir(1, 0, 0).Transformed(axisTrsf));
        zero = zero - axisVec * zero.Dot(axisVec);
        if (zero.Magnitude() <= 1e-9) {
            zero = gp_Vec(gp_Dir(0, 1, 0).Transformed(axisTrsf));
            zero = zero - axisVec * zero.Dot(axisVec);
        }
        if (zero.Magnitude() > 1e-9)
            zeroDir = gp_Dir(zero);
    }

    auto appendStorageOrder = [&] {
        QVector<lcnc::cam::ContourId> order;
        order.reserve(toolpathRef().contourCount());
        for (const LaserContour& contour : toolpathRef().contours()) {
            if (contour.contourId != 0)
                order.append(static_cast<lcnc::cam::ContourId>(contour.contourId));
        }
        return order;
    };

    if (!hasCAxis)
        return appendStorageOrder();

    const gp_Vec axisVec(axisDir);
    const gp_Vec zeroVec(zeroDir);
    int originalIndex = 0;
    for (const LaserContour& contour : toolpathRef().contours()) {
        if (contour.contourId == 0 || contour.points.empty()) {
            ++originalIndex;
            continue;
        }

        const bool omitClosingPoint = contour.points.size() > 2
            && contour.points.front().position.SquareDistance(contour.points.back().position) < 1e-10;
        const int pointCount = static_cast<int>(contour.points.size()) - (omitClosingPoint ? 1 : 0);
        if (pointCount <= 0) {
            ++originalIndex;
            continue;
        }

        double sx = 0.0;
        double sy = 0.0;
        double sz = 0.0;
        for (int i = 0; i < pointCount; ++i) {
            const gp_Pnt& p = contour.points[static_cast<std::size_t>(i)].position;
            sx += p.X();
            sy += p.Y();
            sz += p.Z();
        }
        const gp_Pnt center(sx / pointCount, sy / pointCount, sz / pointCount);
        gp_Vec radial(origin, center);
        const double axial = radial.Dot(axisVec);
        radial = radial - axisVec * axial;

        double angle = 0.0;
        if (radial.Magnitude() > 1e-9) {
            radial.Normalize();
            angle = std::atan2(axisVec.Dot(zeroVec.Crossed(radial)),
                               zeroVec.Dot(radial));
        }

        items.append({static_cast<lcnc::cam::ContourId>(contour.contourId),
                      axial,
                      angle,
                      originalIndex});
        ++originalIndex;
    }

    if (items.size() != toolpathRef().contourCount())
        return appendStorageOrder();

    std::stable_sort(items.begin(), items.end(),
                     [](const OrderItem& a, const OrderItem& b) {
                         if (std::abs(a.axial - b.axial) > 1e-6)
                             return a.axial < b.axial;
                         if (std::abs(a.angle - b.angle) > 1e-9)
                             return a.angle < b.angle;
                         return a.originalIndex < b.originalIndex;
                     });

    QVector<lcnc::cam::ContourId> order;
    order.reserve(items.size());
    for (const OrderItem& item : items)
        order.append(item.id);
    return order;
}

void CamModule::applyDefaultCuttingOrder()
{
    if (!m_camData || toolpathRef().contourCount() <= 0)
        return;

    const QVector<lcnc::cam::ContourId> order = defaultCuttingOrderByCAxis();
    if (order.isEmpty())
        return;

    if (auto* mgr = m_camData->layerManager()) {
        mgr->setManualContourOrder(order);
        mgr->setSortStrategy(lcnc::cam::CuttingPlanSortStrategy::Manual);
    } else {
        auto& container = m_camData->layerContainer();
        container.setManualContourOrder(order);
        container.setSortStrategy(lcnc::cam::CuttingPlanSortStrategy::Manual);
    }

    m_camData->markDirty(true);
    LCNC_INFO(lcnc::LogCode::Generic,
              "cam.toolpath: default cutting order set by C-axis axial position, contours={}",
              order.size());
}

const LaserToolpath& CamModule::toolpath() const
{
    return toolpathRef();
}

lcnc::cam::CamDataManager* CamModule::camData()
{
    m_camData = lcnc::Kernel::current().projectManager()->camData();
    return m_camData;
}

const lcnc::cam::CamDataManager* CamModule::camData() const
{
    return lcnc::Kernel::current().projectManager()->camData();
}

LaserToolpath& CamModule::toolpathRef()
{
    static LaserToolpath emptyToolpath;
    if (auto* data = camData())
        return data->toolpath();
    emptyToolpath.clear();
    return emptyToolpath;
}

const LaserToolpath& CamModule::toolpathRef() const
{
    static const LaserToolpath emptyToolpath;
    if (auto* data = camData())
        return data->toolpath();
    return emptyToolpath;
}

bool CamModule::hasToolpath() const
{
    return toolpathRef().contourCount() > 0;
}

int CamModule::toolpathContourCount() const
{
    return toolpathRef().contourCount();
}

int CamModule::toolpathContourPointCount(int contourIndex) const
{
    if (contourIndex < 0 || contourIndex >= toolpathRef().contourCount())
        return 0;
    return static_cast<int>(toolpathRef().contour(contourIndex).points.size());
}

std::uint64_t CamModule::toolpathRevision() const
{
    std::uint64_t revision = 1469598103934665603ull;
    auto mix = [&revision](std::uint64_t value) {
        revision ^= value + 0x9e3779b97f4a7c15ull + (revision << 6) + (revision >> 2);
    };
    auto mixRounded = [&mix](double value) {
        mix(static_cast<std::uint64_t>(std::llround(value * 1000.0)));
    };
    auto mixString = [&mix](const QString& text) {
        const QByteArray bytes = text.toUtf8();
        mix(static_cast<std::uint64_t>(bytes.size()));
        for (const char ch : bytes)
            mix(static_cast<unsigned char>(ch));
    };

    mix(static_cast<std::uint64_t>(toolpathRef().contourCount()));
    mixRounded(toolpathRef().globalLeadInLength());
    mix(m_camData && m_camData->generationParamsDirty() ? 1ull : 0ull);
    for (const LaserContour& contour : toolpathRef().contours()) {
        mix(contour.contourId);
        mix(contour.layerId);
        mix(contour.enabled ? 1ull : 0ull);
        mix(contour.leadIn.valid ? 1ull : 0ull);
        mix(contour.leadInSolution.valid ? 1ull : 0ull);
        mix(contour.needsRecalculation ? 1ull : 0ull);
        mixRounded(contour.appliedParams.leadInLength);
        mixRounded(contour.appliedParams.deflection);
        mixRounded(contour.pendingParams.leadInLength);
        mixRounded(contour.pendingParams.deflection);
        if (contour.leadInSolution.valid) {
            const ToolpathPoint& lead = contour.leadInSolution.point;
            mixRounded(lead.position.X());
            mixRounded(lead.position.Y());
            mixRounded(lead.position.Z());
            mixRounded(lead.machineCoord.x);
            mixRounded(lead.machineCoord.y);
            mixRounded(lead.machineCoord.z);
            mixRounded(lead.machineCoord.r1);
            mixRounded(lead.machineCoord.r2);
        }
        mix(static_cast<std::uint64_t>(contour.points.size()));
        if (!contour.points.empty()) {
            const ToolpathPoint& first = contour.points.front();
            const ToolpathPoint& last = contour.points.back();
            mixRounded(first.position.X());
            mixRounded(first.position.Y());
            mixRounded(last.position.X());
            mixRounded(last.position.Y());
        }
        for (const ToolpathPoint& point : contour.points) {
            mixRounded(point.position.X());
            mixRounded(point.position.Y());
            mixRounded(point.position.Z());
            mixRounded(point.machineCoord.x);
            mixRounded(point.machineCoord.y);
            mixRounded(point.machineCoord.z);
            mixRounded(point.machineCoord.r1);
            mixRounded(point.machineCoord.r2);
            mixString(point.machineCoord.r1Name);
            mixString(point.machineCoord.r2Name);
            mix(point.machineCoord.valid ? 1ull : 0ull);
        }
    }
    for (const ToolpathLayer& layer : toolpathRef().layers()) {
        mix(layer.layerId);
        mix(layer.enabled ? 1ull : 0ull);
        mix(static_cast<std::uint64_t>(layer.contourIds.size()));
    }
    return revision;
}

bool CamModule::solveToolpathForOrder(
    const QVector<std::uint64_t>& orderedContourIds)
{
    MachineKinematics* kin = kinematics();
    if (!kin) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "cam.toolpath: cannot solve five-axis toolpath without machine kinematics");
        return false;
    }

    auto& contours = toolpathRef().contours();
    for (LaserContour& contour : contours) {
        for (ToolpathPoint& point : contour.points)
            point.machineCoord = {};
        if (contour.leadInSolution.valid)
            contour.leadInSolution.point.machineCoord = {};
    }

    QSet<std::uint64_t> seenIds;
    std::vector<LaserContour*> orderedContours;
    orderedContours.reserve(static_cast<std::size_t>(orderedContourIds.size()));
    for (std::uint64_t id : orderedContourIds) {
        if (id == 0 || seenIds.contains(id))
            continue;
        const int contourIdx = contourIndexById(static_cast<lcnc::cam::ContourId>(id));
        if (contourIdx < 0 || contourIdx >= static_cast<int>(contours.size())) {
            LCNC_WARN(lcnc::LogCode::Generic,
                      "cam.toolpath: cutting order references missing contour {}", id);
            continue;
        }
        seenIds.insert(id);
        orderedContours.push_back(&contours[static_cast<std::size_t>(contourIdx)]);
    }

    if (orderedContours.empty()) {
        LCNC_INFO(lcnc::LogCode::Generic,
                  "cam.toolpath: cleared five-axis coordinates because cutting order is empty");
        refreshToolpathDisplay();
        refreshTravelPath();
        return true;
    }

    LaserToolpathBuilder::computeMachineCoordinatesForOrder(
        orderedContours, kin, gp_Trsf(), nullptr);

    LCNC_INFO(lcnc::LogCode::Generic,
              "cam.toolpath: solved five-axis coordinates from cutting order, contours={}",
              orderedContours.size());
    refreshToolpathDisplay();
    refreshTravelPath();
    return true;
}

lcnc::cam::ToolpathExportSnapshot CamModule::exportToolpathSnapshot() const
{
    return buildToolpathExportSnapshot(
        toolpathRef().contours(),
        toolpathRevision(),
        // 中文翻译：CAM 刀路快照已导出；CAM 当前无刀路
        hasToolpath() ? tr("CAM tool path snapshot exported") : tr("CAM currently has no tool path"));
}

lcnc::cam::ToolpathExportSnapshot CamModule::exportToolpathSnapshotForOrder(
    const QVector<std::uint64_t>& orderedContourIds) const
{
    if (orderedContourIds.isEmpty())
        return exportToolpathSnapshot();

    std::vector<LaserContour> orderedContours;
    orderedContours.reserve(static_cast<std::size_t>(orderedContourIds.size()));
    for (std::uint64_t id : orderedContourIds) {
        const int contourIdx = contourIndexById(static_cast<lcnc::cam::ContourId>(id));
        if (contourIdx < 0 || contourIdx >= toolpathRef().contourCount())
            continue;
        orderedContours.push_back(toolpathRef().contour(contourIdx));
    }

    LCNC_INFO(lcnc::LogCode::Generic,
              "cam.toolpath: exported ordered snapshot with resolved machine coordinates, contours={}",
              orderedContours.size());

    return buildToolpathExportSnapshot(
        orderedContours,
        toolpathRevision(),
        // 中文翻译：CAM 当前无刀路；CAM 有序规划刀路快照已导出
        orderedContours.empty() ? tr("CAM currently has no tool path") : tr("CAM orderly planned tool path snapshot has been exported"));
}

lcnc::cam::ToolpathExportSnapshot CamModule::buildToolpathExportSnapshot(
    const std::vector<LaserContour>& contours,
    std::uint64_t revision,
    const QString& description) const
{
    lcnc::cam::ToolpathExportSnapshot snapshot;
    snapshot.revision = revision;
    snapshot.description = description;

    auto layerForId = [this](std::uint64_t layerId) -> const ToolpathLayer* {
        for (const ToolpathLayer& layer : toolpathRef().layers()) {
            if (layer.layerId == layerId)
                return &layer;
        }
        return nullptr;
    };

    MachineKinematics* kin = kinematics();

    for (const LaserContour& contour : contours) {
        const ToolpathLayer* layer = layerForId(contour.layerId);
        lcnc::cam::ToolpathExportContour exportedContour;
        exportedContour.contourId = contour.contourId;
        exportedContour.layerId = contour.layerId;
        exportedContour.contourName = contour.name;
        exportedContour.layerName = layer ? layer->name : QString();
        exportedContour.toolName = layer ? layer->toolName : QString();
        exportedContour.workpieceEntry = contour.workpieceEntry;
        exportedContour.enabled = contour.enabled;
        exportedContour.layerEnabled = layer ? layer->enabled : true;
        const bool machineSolveReady = m_camData && m_camData->hasCompletePipelineChain();
        exportedContour.needsRecalculation = contour.needsRecalculation
            || (m_camData && m_camData->generationParamsDirty())
            || !machineSolveReady;
        if (contour.needsRecalculation)
            // 中文翻译：轮廓参数或起点尚未重新计算
            exportedContour.recalculationReason = tr("Contour parameters or starting point have not been recalculated");
        else if (m_camData && m_camData->generationParamsDirty())
            // 中文翻译：全局生成参数尚未应用
            exportedContour.recalculationReason = tr("Global build parameters have not been applied yet");
        else if (!machineSolveReady)
            // 中文翻译：五阶段 CAM 流程未完成或上游版本链不一致
            exportedContour.recalculationReason = tr("The five-stage CAM process is incomplete or the upstream version chain is inconsistent");
        exportedContour.pointCount = static_cast<int>(contour.points.size());

        // 计算世界坐标系下的几何端点：cut start = points.front()；end = points.back()；
        // start = leadIn 起点（若 leadIn.valid）否则等于 cut start。
        if (!contour.points.empty()) {
            const gp_Pnt cutStartLocal = contour.points.front().position;
            const gp_Pnt endLocal      = contour.points.back().position;
            gp_Pnt cutStartWorld = cutStartLocal;
            gp_Pnt endWorld      = endLocal;
            gp_Pnt startWorld    = cutStartLocal;
            bool   hasLeadIn = false;

            gp_Trsf wpc;
            if (kin && !contour.workpieceEntry.isEmpty())
                wpc = kin->computeWpcTransform(contour.workpieceEntry);

            cutStartWorld.Transform(wpc);
            endWorld.Transform(wpc);

            if (contour.leadInSolution.valid) {
                gp_Pnt leadStartWorld = contour.leadInSolution.point.position;
                leadStartWorld.Transform(wpc);
                startWorld = leadStartWorld;
                hasLeadIn = contour.leadInSolution.point.machineCoord.valid;
            } else {
                startWorld = cutStartWorld;
            }

            exportedContour.cutStartX = cutStartWorld.X();
            exportedContour.cutStartY = cutStartWorld.Y();
            exportedContour.cutStartZ = cutStartWorld.Z();
            exportedContour.endX      = endWorld.X();
            exportedContour.endY      = endWorld.Y();
            exportedContour.endZ      = endWorld.Z();
            exportedContour.startX    = startWorld.X();
            exportedContour.startY    = startWorld.Y();
            exportedContour.startZ    = startWorld.Z();
            exportedContour.hasLeadIn = hasLeadIn;
            exportedContour.leadInError = contour.leadInSolution.error;
            if (contour.leadInSolution.valid && !hasLeadIn
                && exportedContour.leadInError.isEmpty()) {
                // 中文翻译：下刀点尚未重新计算五轴坐标
                exportedContour.leadInError = tr("The five-axis coordinates of the tool lowering point have not been recalculated.");
            }
            if (hasLeadIn) {
                const ToolpathPoint& lead = contour.leadInSolution.point;
                exportedContour.leadInPoint.x = lead.position.X();
                exportedContour.leadInPoint.y = lead.position.Y();
                exportedContour.leadInPoint.z = lead.position.Z();
                exportedContour.leadInPoint.normalX = lead.normal.X();
                exportedContour.leadInPoint.normalY = lead.normal.Y();
                exportedContour.leadInPoint.normalZ = lead.normal.Z();
                exportedContour.leadInPoint.tangentX = lead.tangent.X();
                exportedContour.leadInPoint.tangentY = lead.tangent.Y();
                exportedContour.leadInPoint.tangentZ = lead.tangent.Z();
                exportedContour.leadInPoint.machineX = lead.machineCoord.x;
                exportedContour.leadInPoint.machineY = lead.machineCoord.y;
                exportedContour.leadInPoint.machineZ = lead.machineCoord.z;
                exportedContour.leadInPoint.machineR1 = lead.machineCoord.r1;
                exportedContour.leadInPoint.machineR2 = lead.machineCoord.r2;
                exportedContour.leadInPoint.rotaryAxis1Name = lead.machineCoord.r1Name;
                exportedContour.leadInPoint.rotaryAxis2Name = lead.machineCoord.r2Name;
                exportedContour.leadInPoint.machineCoordValid = lead.machineCoord.valid;
            }
            exportedContour.endpointsValid = true;
        }

        snapshot.contours.append(exportedContour);

        QVector<lcnc::cam::ToolpathExportPoint> points;
        points.reserve(static_cast<int>(contour.points.size()));
        for (const ToolpathPoint& point : contour.points) {
            lcnc::cam::ToolpathExportPoint exportedPoint;
            exportedPoint.x = point.position.X();
            exportedPoint.y = point.position.Y();
            exportedPoint.z = point.position.Z();
            exportedPoint.normalX = point.normal.X();
            exportedPoint.normalY = point.normal.Y();
            exportedPoint.normalZ = point.normal.Z();
            exportedPoint.tangentX = point.tangent.X();
            exportedPoint.tangentY = point.tangent.Y();
            exportedPoint.tangentZ = point.tangent.Z();
            exportedPoint.curveParam = point.param;
            exportedPoint.machineX = point.machineCoord.x;
            exportedPoint.machineY = point.machineCoord.y;
            exportedPoint.machineZ = point.machineCoord.z;
            exportedPoint.machineR1 = point.machineCoord.r1;
            exportedPoint.machineR2 = point.machineCoord.r2;
            exportedPoint.rotaryAxis1Name = point.machineCoord.r1Name;
            exportedPoint.rotaryAxis2Name = point.machineCoord.r2Name;
            exportedPoint.machineCoordValid = point.machineCoord.valid;
            points.append(exportedPoint);
        }
        snapshot.pointsByContourId.insert(contour.contourId, points);
    }

    return snapshot;
}

void CamModule::setLeadInLength(double mm)
{
    if (mm <= 0.0)
        return;
    toolpathRef().setGlobalLeadInLength(mm);
    m_config.setLeadInLength(mm);
    // 参数只记录为待应用值；已持久化/显示的下刀点仅在“重新计算”时重建。
    LCNC_INFO(lcnc::LogCode::Generic,
              "cam.toolpath: lead-in length changed to {:.3f} mm; apply on explicit recalculation",
              mm);
    pushGenerationParamsToCamData();
    if (m_camData)
        m_camData->setGenerationParamsDirty(true);
    if (m_camData)
        m_camData->markDirty(true);
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
}

double CamModule::leadInLength() const
{
    return m_config.leadInLength();
}

void CamModule::setDeflection(double mm)
{
    if (mm <= 0.0)
        return;

    m_deflection = mm;
    m_config.setDeflection(mm);
    pushGenerationParamsToCamData();
    if (m_camData) {
        m_camData->setGenerationParamsDirty(true);
        m_camData->markDirty(true);
    }
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
}

double CamModule::deflection() const
{
    return m_deflection;
}

bool CamModule::showNormals() const
{
    return m_toolpathRenderer->showNormals();
}

void CamModule::setShowNormals(bool on)
{
    if (m_toolpathRenderer->showNormals() == on)
        return;
    m_toolpathRenderer->setShowNormals(on);
    m_config.setShowNormals(on);
    m_toolpathRenderer->refreshNormals(activeGuiDocument(), toolpathRef(), kinematics());
}

double CamModule::normalSampleStep() const
{
    return m_toolpathRenderer->normalSampleStep();
}

void CamModule::setNormalSampleStep(double mm)
{
    const double current = m_toolpathRenderer->normalSampleStep();
    if (mm <= 0.0)
        return;
    if (qFuzzyCompare(current + 1.0, mm + 1.0))
        return;
    m_toolpathRenderer->setNormalSampleStep(mm);
    m_config.setNormalSampleStep(mm);
    if (m_toolpathRenderer->showNormals())
        m_toolpathRenderer->refreshNormals(activeGuiDocument(), toolpathRef(), kinematics());
}

bool CamModule::resolveLeadInHit(WidgetOccView* occView,
                                 const QPoint& screenPos,
                                 int& contourIdx,
                                 int& pointIdx,
                                 gp_Pnt& entryPoint,
                                 double& entryParam) const
{
    return lcnc::cam::reference_pick::resolveLeadInHit(occView, screenPos, toolpathRef(),
                                                       contourIdx, pointIdx, entryPoint, entryParam);
}

void CamModule::setActiveContourId(lcnc::cam::ContourId contourId)
{
    if (contourId != 0 && contourIndexById(contourId) < 0)
        contourId = 0;
    if (m_activeContourId == contourId)
        return;
    m_activeContourId = contourId;
    emit activeToolpathContourChanged(m_activeContourId, activeContourIndex());
    emit activeContourParametersChanged();
}

int CamModule::activeContourIndex() const
{
    return contourIndexById(m_activeContourId);
}

ContourGenerationParams CamModule::activeContourPendingParams() const
{
    const int index = activeContourIndex();
    if (index < 0 || index >= toolpathRef().contourCount())
        return {};
    return toolpathRef().contour(index).pendingParams;
}

bool CamModule::activeContourNeedsRecalculation() const
{
    const int index = activeContourIndex();
    return index >= 0 && index < toolpathRef().contourCount()
        && toolpathRef().contour(index).needsRecalculation;
}

bool CamModule::setActiveContourLeadInLength(double mm)
{
    const int index = activeContourIndex();
    if (mm <= 0.0 || index < 0 || index >= toolpathRef().contourCount())
        return false;
    LaserContour& contour = toolpathRef().contour(index);
    contour.pendingParams.leadInLength = mm;
    contour.needsRecalculation = true;
    m_camData->markDirty(true);
    emit activeContourParametersChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    return true;
}

bool CamModule::setActiveContourDeflection(double mm)
{
    const int index = activeContourIndex();
    if (mm <= 0.0 || index < 0 || index >= toolpathRef().contourCount())
        return false;
    LaserContour& contour = toolpathRef().contour(index);
    contour.pendingParams.deflection = mm;
    contour.needsRecalculation = true;
    m_camData->markDirty(true);
    emit activeContourParametersChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    return true;
}

bool CamModule::updateLeadInPreview(WidgetOccView* occView, const QPoint& screenPos)
{
    int contourIdx = -1;
    int pointIdx = -1;
    gp_Pnt entryPoint;
    double entryParam = 0.0;

    if (!resolveLeadInHit(occView, screenPos, contourIdx, pointIdx, entryPoint, entryParam)) {
        if (m_previewLeadInValid)
            cancelLeadInPreview();
        return false;
    }

    if (m_previewLeadInValid
        && m_previewLeadInContour == contourIdx
        && m_previewLeadInPointIndex == pointIdx
        && m_previewLeadInPoint.Distance(entryPoint) < 1e-6
        && std::abs(m_previewLeadInParam - entryParam) < 1e-6) {
        return true;
    }

    m_previewLeadInContour = contourIdx;
    m_previewLeadInPointIndex = pointIdx;
    m_previewLeadInPoint = entryPoint;
    m_previewLeadInParam = entryParam;
    m_previewLeadInValid = true;
    lcnc::view::ToolpathRenderer::LeadInPreview preview{
        m_previewLeadInContour, m_previewLeadInPointIndex, m_previewLeadInPoint, m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refreshLeadIns(activeGuiDocument(), toolpathRef(), kinematics(), preview);
    return true;
}

bool CamModule::commitLeadInPreview(WidgetOccView* occView, const QPoint& screenPos)
{
    if (!updateLeadInPreview(occView, screenPos) || !m_previewLeadInValid)
        return false;

    if (m_previewLeadInContour < 0 || m_previewLeadInContour >= toolpathRef().contourCount())
        return false;

    LaserContour& contour = toolpathRef().contour(m_previewLeadInContour);
    LaserContour updated = contour;
    QString startError;
    if (!LaserToolpathBuilder::setContourStart(
            updated, m_previewLeadInPointIndex, &startError)) {
        // 中文翻译：设置轮廓起点失败
        emit operationFailed(tr("Failed to set outline start point"), startError);
        return false;
    }
    if (!updated.leadInSolution.valid) {
        // 中文翻译：设置轮廓起点失败
        emit operationFailed(tr("Failed to set outline start point"), updated.leadInSolution.error);
        return false;
    }
    contour = std::move(updated);
    contour.needsRecalculation = true;
    if (contour.leadInSolution.valid)
        contour.leadInSolution.point.machineCoord = {};
    for (ToolpathPoint& point : contour.points)
        point.machineCoord = {};
    setActiveContourId(static_cast<lcnc::cam::ContourId>(contour.contourId));

    m_previewLeadInContour = -1;
    m_previewLeadInPointIndex = -1;
    m_previewLeadInParam = 0.0;
    m_previewLeadInValid = false;
    setActiveContourId(0);
    // 仅更新当前轮廓的下刀几何；五轴解算留给用户显式点击“重新计算”。
    m_toolpathRenderer->refreshLeadIns(activeGuiDocument(), toolpathRef(), kinematics());
    refreshTravelPath();
    if (m_camData)
        m_camData->markDirty(true);
    emit activeContourParametersChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    return true;
}

void CamModule::cancelLeadInPreview()
{
    if (!m_previewLeadInValid)
        return;

    m_previewLeadInContour = -1;
    m_previewLeadInPointIndex = -1;
    m_previewLeadInParam = 0.0;
    m_previewLeadInValid = false;
    m_toolpathRenderer->refreshLeadIns(activeGuiDocument(), toolpathRef(), kinematics());
}

void CamModule::setContourEnabled(int contourIdx, bool enabled)
{
    if (contourIdx < 0 || contourIdx >= toolpathRef().contourCount())
        return;

    LaserContour& contour = toolpathRef().contour(contourIdx);
    if (contour.enabled == enabled)
        return;

    contour.enabled = enabled;
    setCamContourVisible(contourIdx, enabled && m_toolpathRenderer->isVisible(), false);
    lcnc::view::ToolpathRenderer::LeadInPreview preview{
        m_previewLeadInContour, m_previewLeadInPointIndex, m_previewLeadInPoint, m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refreshContour(activeGuiDocument(), toolpathRef(), kinematics(), contourIdx, preview);
    emit toolpathLayersChanged();
}

void CamModule::setAllContoursEnabled(bool enabled)
{
    bool changed = false;
    for (int index = 0; index < toolpathRef().contourCount(); ++index) {
        LaserContour& contour = toolpathRef().contour(index);
        if (contour.enabled == enabled)
            continue;
        contour.enabled = enabled;
        changed = true;
    }

    if (!changed)
        return;

    setCamContoursVisible(m_toolpathRenderer->isVisible(), false);
    lcnc::view::ToolpathRenderer::LeadInPreview preview{
        m_previewLeadInContour, m_previewLeadInPointIndex, m_previewLeadInPoint, m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refresh(activeGuiDocument(), toolpathRef(), kinematics(), preview);
    emit toolpathLayersChanged();
}

lcnc::cam::ContourId CamModule::contourIdAt(int contourIdx) const
{
    return m_camData ? m_camData->contourIdAt(contourIdx) : 0;
}

int CamModule::contourIndexById(lcnc::cam::ContourId contourId) const
{
    return m_camData ? m_camData->contourIndexById(contourId) : -1;
}

void CamModule::reorderContoursById(const QList<lcnc::cam::ContourId>& order)
{
    if (!m_camData || !m_camData->reorderContoursById(order))
        return;

    syncCamDocumentContours();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    if (m_toolpathRenderer->isVisible())
        refreshToolpathDisplay();
    emit toolpathLayersChanged();
}

void CamModule::reorderContours(const QList<int>& order)
{
    if (!m_camData || !m_camData->reorderContours(order))
        return;

    syncCamDocumentContours();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    if (m_toolpathRenderer->isVisible())
        refreshToolpathDisplay();
    emit toolpathLayersChanged();
}

const std::vector<ToolpathLayer>& CamModule::toolpathLayers() const
{
    static const std::vector<ToolpathLayer> empty;
    return m_camData ? m_camData->toolpathLayers() : empty;
}

std::uint64_t CamModule::addToolpathLayer(const QString& name, const QColor& color)
{
    if (!m_camData)
        return 0;
    const std::uint64_t id = m_camData->addLayer(name, color);
    emit toolpathLayersChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    return id;
}

bool CamModule::removeToolpathLayer(std::uint64_t layerId, std::uint64_t reassignTo)
{
    if (!m_camData || !m_camData->removeLayer(layerId, reassignTo))
        return false;
    applyToolpathLayerColors();
    if (m_toolpathRenderer->isVisible())
        refreshToolpathDisplay();
    emit toolpathLayersChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    return true;
}

QList<int> CamModule::contourIndexesInLayer(std::uint64_t layerId) const
{
    return m_camData ? m_camData->contourIndexesInLayer(layerId) : QList<int>{};
}

bool CamModule::updateToolpathLayer(std::uint64_t layerId,
                                    const QString& name,
                                    const QColor& color,
                                    const QString& toolName)
{
    if (!m_camData || !m_camData->updateToolpathLayer(layerId, name, color, toolName))
        return false;

    applyToolpathLayerColors();
    emit toolpathLayersChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    return true;
}

bool CamModule::updateToolpathLayer(std::uint64_t layerId,
                                    const QString& name,
                                    const QColor& color)
{
    // 不动 toolName 字段：透传当前值（兼容老项目读写）。
    const ToolpathLayer* layer = nullptr;
    if (m_camData) {
        layer = m_camData->toolpathLayer(layerId);
    }
    const QString preservedToolName = layer ? layer->toolName : QString();
    return updateToolpathLayer(layerId, name, color, preservedToolName);
}

bool CamModule::setToolpathLayerEnabled(std::uint64_t layerId, bool enabled)
{
    if (!m_camData || !m_camData->setToolpathLayerEnabled(layerId, enabled))
        return false;

    setCamContoursVisible(m_toolpathRenderer->isVisible(), false);
    lcnc::view::ToolpathRenderer::LeadInPreview preview{
        m_previewLeadInContour, m_previewLeadInPointIndex, m_previewLeadInPoint, m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refresh(activeGuiDocument(), toolpathRef(), kinematics(), preview);
    emit toolpathLayersChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    return true;
}

bool CamModule::recalcToolpath()
{
    const int contourIndex = activeContourIndex();
    if (contourIndex < 0 || contourIndex >= toolpathRef().contourCount()) {
        // 中文翻译：重新计算当前轮廓；请先在项目树中选择一条轮廓
        emit operationFailed(tr("Recalculate the current contour"), tr("Please select a profile in the project tree first"));
        return false;
    }

    const LaserContour& current = toolpathRef().contour(contourIndex);
    LaserContour updated = current;
    const TopoDS_Shape sourceShape = updated.sourceShape.IsNull()
        ? m_workpieceShape : updated.sourceShape;
    if (sourceShape.IsNull()) {
        // 中文翻译：重新计算当前轮廓；当前轮廓缺少工件几何
        emit operationFailed(tr("Recalculate the current contour"), tr("The current profile is missing workpiece geometry"));
        return false;
    }

    const auto& appliedGlobal = m_camData->appliedGenerationParams();
    if (appliedGlobal.useFaceClassification) {
        FaceClassification classification =
            FaceClassifier::classifyFaces(sourceShape, appliedGlobal.smoothAngle);
        std::vector<TopoDS_Face> outerFaces;
        std::vector<TopoDS_Face> crossFaces;
        if (classification.outerGroup())
            outerFaces = classification.outerGroup()->faces;
        for (const auto* group : classification.crossSectionGroups()) {
            if (group)
                crossFaces.insert(crossFaces.end(), group->faces.begin(), group->faces.end());
        }
        if (!outerFaces.empty() && !crossFaces.empty()) {
            LaserToolpathBuilder::bindLeadInSurfaceContext(
                updated, outerFaces, crossFaces);
            LaserToolpathBuilder::discretizeContourWithClassification(
                updated, outerFaces, crossFaces, updated.pendingParams.deflection);
        } else {
            LaserToolpathBuilder::discretizeContour(
                updated, sourceShape, updated.pendingParams.deflection);
        }
    } else {
        LaserToolpathBuilder::discretizeContour(
            updated, sourceShape, updated.pendingParams.deflection);
    }

    if (updated.points.empty()) {
        // 中文翻译：重新计算当前轮廓；轮廓 "%1" 离散后没有可用点
        emit operationFailed(tr("Recalculate the current contour"), tr("Contour \"%1\" has no available points after discretization").arg(updated.name));
        return false;
    }

    auto selected = std::find_if(updated.points.begin(), updated.points.end(),
        [&updated](const ToolpathPoint& point) {
            return point.sourceEdgeIndex == updated.leadIn.entryEdgeIndex
                && std::abs(point.param - updated.leadIn.entryParam) <= 1e-10;
        });
    if (selected == updated.points.end() && updated.leadIn.entryEdgeIndex < 0) {
        selected = std::min_element(updated.points.begin(), updated.points.end(),
            [&updated](const ToolpathPoint& a, const ToolpathPoint& b) {
                return a.position.SquareDistance(updated.leadIn.entryPoint)
                    < b.position.SquareDistance(updated.leadIn.entryPoint);
            });
    }
    if (selected == updated.points.end()) {
        // 中文翻译：重新计算当前轮廓
        emit operationFailed(tr("Recalculate the current contour"),
                             // 中文翻译：轮廓 "%1" 无法恢复人工起点
                             tr("Contour \"%1\" cannot restore artificial starting point").arg(updated.name));
        return false;
    }
    if (current.leadIn.entryEdgeIndex >= 0
        && selected->position.Distance(current.leadIn.entryPoint) > 1e-6) {
        // 中文翻译：重新计算当前轮廓
        emit operationFailed(tr("Recalculate the current contour"),
                             // 中文翻译：轮廓 "%1" 的起点拓扑锚点已变化
                             tr("The starting topology anchor point of contour \"%1\" has changed").arg(updated.name));
        return false;
    }

    updated.leadIn.length = updated.pendingParams.leadInLength;
    QString startError;
    if (!LaserToolpathBuilder::setContourStart(
            updated, static_cast<int>(std::distance(updated.points.begin(), selected)), &startError)
        || !updated.leadInSolution.valid) {
        if (startError.isEmpty())
            startError = updated.leadInSolution.error;
        // 中文翻译：重新计算当前轮廓
        emit operationFailed(tr("Recalculate the current contour"),
                             // 中文翻译：轮廓 "%1" 下刀点生成失败：%2
                             tr("Contour \"%1\" cutting point generation failed: %2").arg(updated.name, startError));
        return false;
    }

    QVector<lcnc::cam::ContourId> order;
    if (auto provider = lcnc::Kernel::current()
                            .services()
                            .getService<lcnc::process::IProcessCuttingPlanProvider>()) {
        const auto orderedIds = provider->orderedContourIds();
        order = QVector<lcnc::cam::ContourId>(orderedIds.cbegin(), orderedIds.cend());
    }
    if (order.isEmpty())
        order = defaultCuttingOrderByCAxis();

    MachineCoord continuity;
    const auto currentId = static_cast<lcnc::cam::ContourId>(updated.contourId);
    const int orderIndex = order.indexOf(currentId);
    for (int i = orderIndex - 1; i >= 0; --i) {
        const int previousIndex = contourIndexById(order.at(i));
        if (previousIndex < 0)
            continue;
        const LaserContour& previous = toolpathRef().contour(previousIndex);
        if (!previous.enabled || previous.points.empty())
            continue;
        const MachineCoord& candidate = previous.points.back().machineCoord;
        if (candidate.valid) {
            continuity = candidate;
            break;
        }
    }

    LaserToolpathBuilder::computeMachineCoordinates(
        updated, kinematics(), gp_Trsf(), continuity.valid ? &continuity : nullptr);
    const bool coordinatesValid = updated.leadInSolution.point.machineCoord.valid
        && std::all_of(updated.points.begin(), updated.points.end(), [](const ToolpathPoint& point) {
            return point.machineCoord.valid;
        });
    if (!coordinatesValid) {
        // 中文翻译：重新计算当前轮廓
        emit operationFailed(tr("Recalculate the current contour"),
                             // 中文翻译：轮廓 "%1" 五轴坐标求解失败
                             tr("Contour \"%1\" five-axis coordinate solution failed").arg(updated.name));
        return false;
    }

    updated.appliedParams = updated.pendingParams;
    updated.leadIn.length = updated.appliedParams.leadInLength;
    updated.needsRecalculation = false;
    toolpathRef().contour(contourIndex) = std::move(updated);
    m_camData->markDirty(true);
    lcnc::view::ToolpathRenderer::LeadInPreview preview{
        m_previewLeadInContour, m_previewLeadInPointIndex, m_previewLeadInPoint,
        m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refreshContour(
        lcnc::Kernel::current().guiApp()->activeGuiDocument(),
        toolpathRef(), kinematics(), contourIndex, preview);
    refreshTravelPath();
    emit activeContourParametersChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    return true;
}

TaskId CamModule::recalcToolpathAsync()
{
    const int contourIndex = activeContourIndex();
    if (contourIndex < 0 || contourIndex >= toolpathRef().contourCount()) {
        // 中文翻译：重新计算当前轮廓；请先在项目树中选择一条轮廓
        emit operationFailed(tr("Recalculate the current contour"), tr("Please select a profile in the project tree first"));
        return kInvalidTaskId;
    }
    auto* taskManager = lcnc::Kernel::current().taskManager();
    MachineKinematics* machine = kinematics();
    if (!taskManager || !machine) {
        // 中文翻译：重新计算当前轮廓；后台任务或机台运动学服务不可用
        emit operationFailed(tr("Recalculate the current contour"), tr("Background tasks or machine kinematics services are not available"));
        return kInvalidTaskId;
    }

    const LaserContour current = toolpathRef().contour(contourIndex);
    const TopoDS_Shape sourceShape = current.sourceShape.IsNull()
        ? m_workpieceShape : current.sourceShape;
    if (sourceShape.IsNull()) {
        // 中文翻译：重新计算当前轮廓；当前轮廓缺少工件几何
        emit operationFailed(tr("Recalculate the current contour"), tr("The current profile is missing workpiece geometry"));
        return kInvalidTaskId;
    }

    QVector<lcnc::cam::ContourId> order;
    if (auto provider = lcnc::Kernel::current().services()
                            .getService<lcnc::process::IProcessCuttingPlanProvider>()) {
        const auto orderedIds = provider->orderedContourIds();
        order = QVector<lcnc::cam::ContourId>(orderedIds.cbegin(), orderedIds.cend());
    }
    if (order.isEmpty())
        order = defaultCuttingOrderByCAxis();

    MachineCoord continuity;
    const auto currentId = static_cast<lcnc::cam::ContourId>(current.contourId);
    const int orderIndex = order.indexOf(currentId);
    for (int i = orderIndex - 1; i >= 0; --i) {
        const int previousIndex = contourIndexById(order.at(i));
        if (previousIndex < 0)
            continue;
        const LaserContour& previous = toolpathRef().contour(previousIndex);
        if (!previous.enabled || previous.points.empty())
            continue;
        if (previous.points.back().machineCoord.valid) {
            continuity = previous.points.back().machineCoord;
            break;
        }
    }

    struct RecalcResult {
        LaserContour contour;
        QString error;
        bool ok{false};
    };
    const auto result = std::make_shared<RecalcResult>();
    const auto appliedGlobal = m_camData->appliedGenerationParams();
    const QList<MachineAxisDef> axes = machine->axes();
    const QString configType = machine->configType();
    // Recalculation works from an immutable contour snapshot.  Do not apply its
    // result if any project-level toolpath, face-pipeline, or machine setup
    // input changed while the worker was running.
    const std::uint64_t capturedToolpathRevision = toolpathRevision();
    const std::uint64_t capturedFaceRevision = machiningFaceSetRevision();
    const std::uint64_t capturedSetupRevision = machineSetupRevision();
    const std::uint64_t originalSignature = current.signature;
    const ContourGenerationParams originalPendingParams = current.pendingParams;
    const lcnc::cam::ContourId targetId = currentId;

    TaskSpec spec;
    // 中文翻译：重新计算当前轮廓
    spec.label = tr("Recalculate the current contour");
    spec.scope = QStringLiteral("cam.toolpath");
    spec.priority = TaskPriority::Normal;
    spec.cancellable = true;
    const TaskId taskId = taskManager->run(spec,
        [current, sourceShape, appliedGlobal, continuity, axes, configType, result](TaskProgress* progress) {
            progress->setRange(0, 100);
            // 中文翻译：正在离散轮廓
            progress->setStepName(QObject::tr("discretizing contours"));
            LaserContour updated = current;
            if (appliedGlobal.useFaceClassification) {
                FaceClassification classification =
                    FaceClassifier::classifyFaces(sourceShape, appliedGlobal.smoothAngle);
                std::vector<TopoDS_Face> outerFaces;
                std::vector<TopoDS_Face> crossFaces;
                if (classification.outerGroup())
                    outerFaces = classification.outerGroup()->faces;
                for (const auto* group : classification.crossSectionGroups())
                    if (group)
                        crossFaces.insert(crossFaces.end(), group->faces.begin(), group->faces.end());
                if (!outerFaces.empty() && !crossFaces.empty()) {
                    LaserToolpathBuilder::bindLeadInSurfaceContext(
                        updated, outerFaces, crossFaces);
                    LaserToolpathBuilder::discretizeContourWithClassification(
                        updated, outerFaces, crossFaces, updated.pendingParams.deflection);
                } else {
                    LaserToolpathBuilder::discretizeContour(updated, sourceShape, updated.pendingParams.deflection);
                }
            } else {
                LaserToolpathBuilder::discretizeContour(updated, sourceShape, updated.pendingParams.deflection);
            }
            if (progress->isAbortRequested())
                // 中文翻译：轮廓重新计算已取消
                throw std::runtime_error("Contour recalculation canceled");
            if (updated.points.empty()) {
                // 中文翻译：轮廓 "%1" 离散后没有可用点
                result->error = QObject::tr("Contour \"%1\" has no available points after discretization").arg(updated.name);
                return;
            }
            auto selected = std::find_if(updated.points.begin(), updated.points.end(),
                [&updated](const ToolpathPoint& point) {
                    return point.sourceEdgeIndex == updated.leadIn.entryEdgeIndex
                        && std::abs(point.param - updated.leadIn.entryParam) <= 1e-10;
                });
            if (selected == updated.points.end() && updated.leadIn.entryEdgeIndex < 0) {
                selected = std::min_element(updated.points.begin(), updated.points.end(),
                    [&updated](const ToolpathPoint& a, const ToolpathPoint& b) {
                        return a.position.SquareDistance(updated.leadIn.entryPoint)
                            < b.position.SquareDistance(updated.leadIn.entryPoint);
                    });
            }
            if (selected == updated.points.end()) {
                // 中文翻译：轮廓 "%1" 无法恢复人工起点
                result->error = QObject::tr("Contour \"%1\" cannot restore artificial starting point").arg(updated.name);
                return;
            }
            if (current.leadIn.entryEdgeIndex >= 0
                && selected->position.Distance(current.leadIn.entryPoint) > 1e-6) {
                // 中文翻译：轮廓 "%1" 的起点拓扑锚点已变化
                result->error = QObject::tr("The starting topology anchor point of contour \"%1\" has changed").arg(updated.name);
                return;
            }
            updated.leadIn.length = updated.pendingParams.leadInLength;
            QString startError;
            if (!LaserToolpathBuilder::setContourStart(
                    updated, static_cast<int>(std::distance(updated.points.begin(), selected)), &startError)
                || !updated.leadInSolution.valid) {
                result->error = startError.isEmpty() ? updated.leadInSolution.error : startError;
                return;
            }
            progress->setValue(65);
            // 中文翻译：正在求解机台坐标
            progress->setStepName(QObject::tr("Solving for machine coordinates"));
            MachineKinematics workerKinematics;
            workerKinematics.setAxes(axes, configType);
            MachineCoord continuityState = continuity;
            LaserToolpathBuilder::computeMachineCoordinates(
                updated, &workerKinematics, gp_Trsf(), continuityState.valid ? &continuityState : nullptr);
            const bool coordinatesValid = updated.leadInSolution.point.machineCoord.valid
                && std::all_of(updated.points.begin(), updated.points.end(), [](const ToolpathPoint& point) {
                    return point.machineCoord.valid;
                });
            if (!coordinatesValid) {
                // 中文翻译：轮廓 "%1" 五轴坐标求解失败
                result->error = QObject::tr("Contour \"%1\" five-axis coordinate solution failed").arg(updated.name);
                return;
            }
            updated.appliedParams = updated.pendingParams;
            updated.leadIn.length = updated.appliedParams.leadInLength;
            updated.needsRecalculation = false;
            result->contour = std::move(updated);
            result->ok = true;
            progress->setValue(100);
        });

    watchTask(this, taskId,
        [this, result, targetId, capturedToolpathRevision, capturedFaceRevision,
         capturedSetupRevision, originalSignature, originalPendingParams, sourceShape](bool success) {
        const int latestIndex = contourIndexById(targetId);
        if (!success || !result->ok) {
            // 中文翻译：重新计算当前轮廓
            emit operationFailed(tr("Recalculate the current contour"),
                                 // 中文翻译：轮廓重新计算失败或已取消
                                 result->error.isEmpty() ? tr("Contour recalculation failed or was canceled") : result->error);
            return;
        }
        const TopoDS_Shape latestSourceShape = latestIndex < 0
            ? TopoDS_Shape{}
            : (toolpathRef().contour(latestIndex).sourceShape.IsNull()
                ? m_workpieceShape : toolpathRef().contour(latestIndex).sourceShape);
        if (latestIndex < 0 || latestSourceShape.IsNull()
            || toolpathRevision() != capturedToolpathRevision
            || machiningFaceSetRevision() != capturedFaceRevision
            || machineSetupRevision() != capturedSetupRevision
            || toolpathRef().contour(latestIndex).signature != originalSignature
            || !latestSourceShape.IsSame(sourceShape)
            || std::abs(toolpathRef().contour(latestIndex).pendingParams.leadInLength
                        - originalPendingParams.leadInLength) > 1e-12
            || std::abs(toolpathRef().contour(latestIndex).pendingParams.deflection
                        - originalPendingParams.deflection) > 1e-12) {
            // 中文翻译：重新计算当前轮廓；轮廓在计算期间已变更，后台结果已丢弃
            emit operationFailed(tr("Recalculate the current contour"), tr("The contour changed during calculation and the background results were discarded"));
            return;
        }
        toolpathRef().contour(latestIndex) = std::move(result->contour);
        m_camData->markDirty(true);
        lcnc::view::ToolpathRenderer::LeadInPreview preview{
            m_previewLeadInContour, m_previewLeadInPointIndex, m_previewLeadInPoint,
            m_previewLeadInParam, m_previewLeadInValid
        };
        m_toolpathRenderer->refreshContour(
            lcnc::Kernel::current().guiApp()->activeGuiDocument(),
            toolpathRef(), kinematics(), latestIndex, preview);
        refreshTravelPath();
        emit activeContourParametersChanged();
        lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    });
    return taskId;
}

void CamModule::setToolpathVisible(bool visible)
{
    if (m_toolpathRenderer->isVisible() == visible) {
        setCamContoursVisible(visible);
        return;
    }
    m_toolpathRenderer->setVisible(activeGuiDocument(), visible);
    setCamContoursVisible(visible);
    if (visible)
        refreshToolpathDisplay();
    emit toolpathVisibilityChanged(visible);
}

bool CamModule::isToolpathVisible() const
{
    return m_toolpathRenderer->isVisible();
}

double CamModule::smoothAngle() const
{
    return m_smoothAngle;
}

void CamModule::setSmoothAngle(double deg)
{
    if (qFuzzyCompare(m_smoothAngle + 1.0, deg + 1.0))
        return;
    m_smoothAngle = deg;
    m_config.setSmoothAngle(deg);
    pushGenerationParamsToCamData();
    if (m_camData) {
        m_camData->setGenerationParamsDirty(true);
        m_camData->markDirty(true);
    }
}

bool CamModule::useFaceClassification() const
{
    return m_useFaceClassification;
}

void CamModule::setUseFaceClassification(bool on)
{
    if (m_useFaceClassification == on)
        return;
    m_useFaceClassification = on;
    m_config.setUseFaceClassification(on);
    pushGenerationParamsToCamData();
    if (m_camData) {
        m_camData->setGenerationParamsDirty(true);
        m_camData->markDirty(true);
    }
}

gp_Dir CamModule::beamDirectionWpc(const QString& wpcEntry) const
{
    const MachineKinematics* kin = kinematics();
    if (!kin)
        return gp_Dir(0.0, 0.0, -1.0);
    // wpcHome maps workpiece -> machine at the home posture; the inverse maps
    // the machine-space beam direction into workpiece coordinates. Directions
    // are translation-invariant, so only the rotation matters.
    const gp_Trsf wpcHome = kin->computeWpcTransformHome(wpcEntry);
    return kin->nominalBeamDirectionMachine().Transformed(wpcHome.Inverted());
}

int CamModule::extractionStrategy() const
{
    return m_extractionStrategy;
}

void CamModule::setExtractionStrategy(int strategy)
{
    if (m_extractionStrategy == strategy)
        return;
    m_extractionStrategy = strategy;
    m_config.setExtractionStrategy(strategy);  // persist to cam.toml
    if (m_camData) {
        m_camData->setGenerationParamsDirty(true);
        m_camData->markDirty(true);
    }
}

int CamModule::machiningFaceCount() const
{
    return static_cast<int>(m_machiningFaces.size());
}

bool CamModule::hasManualMachiningFaces() const
{
    for (const auto& entry : m_machiningFaces)
        if (entry.manual)
            return true;
    return false;
}

QList<CamModule::MachiningFaceInfo> CamModule::machiningFacesForTree() const
{
    QList<MachiningFaceInfo> result;
    result.reserve(static_cast<int>(m_machiningFaces.size()));
    int autoIdx = 0, manualIdx = 0;
    for (const auto& entry : m_machiningFaces) {
        // Cross sections are algorithmic reference faces.  They intentionally
        // stay out of the operator-facing machining-face tree.
        if (entry.role == lcnc::cam::MachiningFaceRole::CrossSection)
            continue;
        MachiningFaceInfo info;
        info.faceId = entry.faceId;
        info.workpieceEntry = entry.workpieceEntry;
        info.manual = entry.manual;
        info.role = entry.role;
        const QString roleName = [this, &entry]() {
            switch (entry.role) {
            // 中文翻译：加工面
            case lcnc::cam::MachiningFaceRole::MachiningSurface: return tr("Processing surface");
            // 中文翻译：横截面
            case lcnc::cam::MachiningFaceRole::CrossSection: return tr("cross section");
            }
            // 中文翻译：加工面
            return tr("Processing surface");
        }();
        info.displayName = entry.manual
            // 中文翻译：手动%1 %2
            ? tr("Manual %1 %2").arg(roleName).arg(++manualIdx)
            : tr("%1 %2").arg(roleName).arg(++autoIdx);
        result.append(info);
    }
    return result;
}

std::vector<TopoDS_Face> CamModule::manualMachiningFaces() const
{
    std::vector<TopoDS_Face> faces;
    for (const auto& entry : m_machiningFaces)
        if (entry.manual && !entry.face.IsNull())
            faces.push_back(entry.face);
    return faces;
}

void CamModule::addMachiningFace(const TopoDS_Face& face)
{
    // 中文翻译：编辑加工面
    if (rejectConflictingPipelineOperation(tr("Edit machining surface")))
        return;
    if (face.IsNull())
        return;
    for (const auto& existing : m_machiningFaces)
        if (!existing.face.IsNull() && existing.face.IsSame(face))
            return;
    MachiningFaceEntry entry;
    entry.faceId = m_nextMachiningFaceId++;
    entry.face = face;
    entry.manual = true;
    entry.role = lcnc::cam::MachiningFaceRole::MachiningSurface;
    for (const WorkpieceShapeSource& source : collectWorkpieceShapes()) {
        bool found = false;
        for (TopExp_Explorer exp(source.shape, TopAbs_FACE); exp.More(); exp.Next()) {
            if (TopoDS::Face(exp.Current()).IsSame(face)) {
                found = true;
                break;
            }
        }
        if (found) {
            entry.workpieceEntry = source.workpieceEntry;
            break;
        }
    }
    m_machiningFaces.push_back(std::move(entry));
    // An explicit pick must always be visible so the operator can confirm the
    // growing face set, even when the machining-face tree was hidden earlier.
    m_machiningFacesVisible = true;
    refreshMachiningFaceDisplay();
    pushMachiningFaceRecordsToCamData();
    if (m_camData) {
        m_camData->failPipelineStage(lcnc::cam::CamPipelineStage::FaceSeparation,
                                     // 中文翻译：加工面编辑尚未应用
                                     tr("Machining surface editing has not been applied yet"));
        m_camData->setGenerationParamsDirty(true);
    }
    emit machiningFacesChanged();
}

bool CamModule::removeMachiningFace(std::uint64_t faceId)
{
    // 中文翻译：编辑加工面
    if (rejectConflictingPipelineOperation(tr("Edit machining surface")))
        return false;
    auto it = std::find_if(m_machiningFaces.begin(), m_machiningFaces.end(),
        [faceId](const MachiningFaceEntry& e) { return e.faceId == faceId; });
    if (it == m_machiningFaces.end())
        return false;
    m_machiningFaces.erase(it);
    pushMachiningFaceRecordsToCamData();
    if (m_camData) {
        m_camData->failPipelineStage(lcnc::cam::CamPipelineStage::FaceSeparation,
                                     // 中文翻译：加工面编辑尚未应用
                                     tr("Machining surface editing has not been applied yet"));
        m_camData->setGenerationParamsDirty(true);
    }
    refreshMachiningFaceDisplay();
    emit machiningFacesChanged();
    return true;
}

bool CamModule::setMachiningFaceRole(
    std::uint64_t faceId, lcnc::cam::MachiningFaceRole role)
{
    // 中文翻译：编辑加工面
    if (rejectConflictingPipelineOperation(tr("Edit machining surface")))
        return false;
    auto it = std::find_if(m_machiningFaces.begin(), m_machiningFaces.end(),
        [faceId](const MachiningFaceEntry& entry) { return entry.faceId == faceId; });
    if (it == m_machiningFaces.end() || it->role == role)
        return false;
    if (role == lcnc::cam::MachiningFaceRole::CrossSection) {
        const bool intersectsMachiningFace = std::any_of(
            m_machiningFaces.cbegin(), m_machiningFaces.cend(), [it](const MachiningFaceEntry& other) {
                return other.faceId != it->faceId
                    && other.workpieceEntry == it->workpieceEntry
                    && other.role == lcnc::cam::MachiningFaceRole::MachiningSurface
                    && facesShareBoundaryEdge(other.face, it->face);
            });
        if (!intersectsMachiningFace) {
            // 中文翻译：设置横截面
            emit operationFailed(tr("Set cross section"),
                                 // 中文翻译：横截面必须与同一工件的加工面相交或共享边。
                                 tr("The cross section must intersect or share an edge with a machined surface of the same workpiece."));
            return false;
        }
    }
    // Reclassifying an auto result is an operator decision; retain it across a
    // later automatic refresh just like an explicitly picked face.
    it->role = role;
    it->manual = true;
    pushMachiningFaceRecordsToCamData();
    if (m_camData) {
        m_camData->failPipelineStage(lcnc::cam::CamPipelineStage::FaceSeparation,
                                     // 中文翻译：加工面角色编辑尚未应用
                                     tr("Machining surface role editing has not been applied yet"));
        m_camData->setGenerationParamsDirty(true);
    }
    refreshMachiningFaceDisplay();
    emit machiningFacesChanged();
    return true;
}

void CamModule::clearMachiningFaces()
{
    // 中文翻译：编辑加工面
    if (rejectConflictingPipelineOperation(tr("Edit machining surface")))
        return;
    if (m_machiningFaces.empty())
        return;
    m_machiningFaces.clear();
    pushMachiningFaceRecordsToCamData();
    if (m_camData) {
        m_camData->failPipelineStage(lcnc::cam::CamPipelineStage::FaceSeparation,
                                     // 中文翻译：加工面编辑尚未应用
                                     tr("Machining surface editing has not been applied yet"));
        m_camData->setGenerationParamsDirty(true);
    }
    refreshMachiningFaceDisplay();
    emit machiningFacesChanged();
}

void CamModule::setAutoMachiningFaces(const std::vector<TopoDS_Face>& faces,
                                      const QString& workpieceEntry)
{
    // Replace auto-captured entries; keep manual picks.
    std::vector<MachiningFaceEntry> kept;
    for (auto& entry : m_machiningFaces)
        if (entry.manual)
            kept.push_back(std::move(entry));
    for (const TopoDS_Face& face : faces) {
        if (face.IsNull())
            continue;
        bool dup = false;
        for (const auto& e : kept)
            if (!e.face.IsNull() && e.face.IsSame(face)) { dup = true; break; }
        if (dup)
            continue;
        MachiningFaceEntry entry;
        entry.faceId = m_nextMachiningFaceId++;
        entry.face = face;
        entry.workpieceEntry = workpieceEntry;
        entry.manual = false;
        kept.push_back(std::move(entry));
    }
    m_machiningFaces = std::move(kept);
    pushMachiningFaceRecordsToCamData();
    refreshMachiningFaceDisplay();
    emit machiningFacesChanged();
}

void CamModule::refreshMachiningFaceDisplay()
{
    GuiDocument* gd = activeGuiDocument();
    if (!gd || gd->context().IsNull())
        return;
    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    for (auto it = m_machiningFaceAis.cbegin(); it != m_machiningFaceAis.cend(); ++it) {
        if (!it.value().IsNull())
            ctx->Remove(it.value(), Standard_False);
    }
    m_machiningFaceAis.clear();
    if (!m_machiningFacesVisible) {
        ctx->UpdateCurrentViewer();
        return;
    }
    for (const auto& entry : m_machiningFaces) {
        if (entry.face.IsNull()
            || entry.role != lcnc::cam::MachiningFaceRole::MachiningSurface)
            continue;
        const Quantity_Color highlightColor = entry.manual
            ? Quantity_Color(1.0, 0.82, 0.0, Quantity_TOC_RGB)
            : Quantity_Color(0.0, 0.85, 1.0, Quantity_TOC_RGB);
        Handle(AIS_Shape) ais = new AIS_Shape(entry.face);
        ais->SetDisplayMode(AIS_Shaded);
        ais->SetColor(highlightColor);
        ais->SetTransparency(entry.manual ? 0.25 : 0.45);
        ais->SetPolygonOffsets(Aspect_POM_Fill, -1.0f, -1.0f);
        if (!ais->Attributes().IsNull()) {
            ais->Attributes()->SetFaceBoundaryDraw(true);
            ais->Attributes()->SetFaceBoundaryAspect(
                new Prs3d_LineAspect(highlightColor, Aspect_TOL_SOLID,
                                     entry.manual ? 3.0 : 2.0));
        }
        ctx->Display(ais, AIS_Shaded, 0, Standard_False);
        // Draw after the workpiece while inheriting its depth buffer.  This
        // avoids coplanar Z-fighting without showing back-side faces through
        // the solid.
        ctx->SetZLayer(ais, Graphic3d_ZLayerId_Top);
        // The confirmation overlay is presentation-only; otherwise it can
        // intercept the next click in a multi-face pick session.
        ctx->Deactivate(ais);
        m_machiningFaceAis.insert(entry.faceId, ais);
    }
    ctx->UpdateCurrentViewer();
}

void CamModule::setMachiningFacesVisible(bool visible)
{
    if (m_machiningFacesVisible == visible)
        return;
    m_machiningFacesVisible = visible;
    refreshMachiningFaceDisplay();
}

bool CamModule::machiningFacesVisible() const
{
    return m_machiningFacesVisible;
}

void CamModule::pushMachiningFaceRecordsToCamData()
{
    if (!m_camData)
        return;
    std::vector<lcnc::cam::CamDataManager::MachiningFaceRecord> records;
    records.reserve(m_machiningFaces.size());
    for (const auto& entry : m_machiningFaces) {
        lcnc::cam::CamDataManager::MachiningFaceRecord rec;
        rec.faceId         = entry.faceId;
        rec.workpieceEntry = entry.workpieceEntry;
        rec.manual         = entry.manual;
        rec.role           = entry.role;
        rec.signature      = entry.face.IsNull()
            ? 0 : LaserToolpathBuilder::computeFaceSignature(entry.face);
        records.push_back(rec);
    }
    m_camData->setMachiningFaceRecords(std::move(records));
    m_camData->markDirty(true);
}

void CamModule::rebindMachiningFacesFromRecords()
{
    if (!m_camData)
        return;
    const auto& records = m_camData->machiningFaceRecords();
    if (records.empty())
        return;

    struct RebindCandidate {
        QString workpieceEntry;
        TopoDS_Face face;
        std::uint64_t signature{0};
    };
    std::vector<RebindCandidate> workpieceFaces;
    for (const WorkpieceShapeSource& source : collectWorkpieceShapes()) {
        for (TopExp_Explorer exp(source.shape, TopAbs_FACE); exp.More(); exp.Next()) {
            const TopoDS_Face face = TopoDS::Face(exp.Current());
            if (face.IsNull())
                continue;
            workpieceFaces.push_back({source.workpieceEntry, face,
                LaserToolpathBuilder::computeFaceSignature(face)});
        }
    }
    if (workpieceFaces.empty()) {
        m_camData->failPipelineStage(lcnc::cam::CamPipelineStage::FaceSeparation,
                                     // 中文翻译：加工面无法在当前工件中重绑
                                     tr("The machining surface cannot be re-bound in the current workpiece"));
        m_camData->setGenerationParamsDirty(true);
        return;
    }

    std::vector<MachiningFaceEntry> rebound;
    for (const auto& rec : records) {
        if (rec.signature == 0) continue;
        MachiningFaceEntry entry;
        entry.faceId = rec.faceId;
        entry.workpieceEntry = rec.workpieceEntry;
        entry.manual = rec.manual;
        entry.role = rec.role;

        // Find the face in the current workpiece by matching signature.
        for (const RebindCandidate& candidate : workpieceFaces) {
            if (candidate.signature == rec.signature
                && (rec.workpieceEntry.isEmpty()
                    || candidate.workpieceEntry == rec.workpieceEntry)) {
                entry.face = candidate.face;
                break;
            }
        }
        if (entry.face.IsNull()) {
            LCNC_WARN(lcnc::LogCode::Generic,
                      "cam.machiningFace: signature {:016x} not found in workpiece; "
                      "manual face {} dropped",
                      rec.signature, rec.faceId);
            continue;
        }
        rebound.push_back(std::move(entry));
    }

    m_machiningFaces = std::move(rebound);
    if (m_machiningFaces.size() != records.size()) {
        m_camData->failPipelineStage(lcnc::cam::CamPipelineStage::FaceSeparation,
                                     // 中文翻译：部分加工面或横截面无法在当前工件中重绑
                                     tr("Some machined surfaces or cross-sections cannot be re-bound in the current workpiece"));
        m_camData->setGenerationParamsDirty(true);
    }
    if (!m_machiningFaces.empty()) {
        std::uint64_t maxId = 0;
        for (const auto& e : m_machiningFaces)
            maxId = std::max(maxId, e.faceId);
        m_nextMachiningFaceId = std::max(m_nextMachiningFaceId, maxId + 1);
    }

    refreshMachiningFaceDisplay();
    emit machiningFacesChanged();

    LCNC_INFO(lcnc::LogCode::Generic,
              "cam.machiningFace: rebound {} faces from persisted signatures",
              m_machiningFaces.size());
}

bool CamModule::pickMachiningFace(WidgetOccView* view, const QPoint& pos, QString* error)
{
    if (!view || view->view().IsNull() || view->context().IsNull()) {
        // 中文翻译：没有可用的视图用于拾取加工面。
        if (error) *error = tr("There are no views available for picking work surfaces.");
        return false;
    }
    const Handle(AIS_InteractiveContext)& context = view->context();
    context->MoveTo(pos.x(), pos.y(), view->view(), Standard_False);
    const Handle(SelectMgr_EntityOwner) owner = context->DetectedOwner();
    const Handle(StdSelect_BRepOwner) brepOwner =
        Handle(StdSelect_BRepOwner)::DownCast(owner);
    if (brepOwner.IsNull() || !brepOwner->HasShape()) {
        // 中文翻译：未检测到面，请将光标放在工件表面上重试。
        if (error) *error = tr("No face detected, please place the cursor on the workpiece surface and try again.");
        return false;
    }
    const TopoDS_Shape picked = brepOwner->Shape();
    if (picked.IsNull() || picked.ShapeType() != TopAbs_FACE) {
        // 中文翻译：拾取到的不是面，请选择工件上的加工面。
        if (error) *error = tr("The picked up surface is not a surface, please select the processing surface on the workpiece.");
        return false;
    }
    bool belongsToWorkpiece = false;
    for (const WorkpieceShapeSource& source : collectWorkpieceShapes()) {
        for (TopExp_Explorer exp(source.shape, TopAbs_FACE); exp.More(); exp.Next()) {
            if (TopoDS::Face(exp.Current()).IsSame(picked)) {
                belongsToWorkpiece = true;
                break;
            }
        }
        if (belongsToWorkpiece)
            break;
    }
    if (!belongsToWorkpiece) {
        // 中文翻译：只能选择工件模型上的面，机台、刀路和辅助显示不可作为加工面。
        if (error) *error = tr("Only the surfaces on the workpiece model can be selected, and the machine table, tool path and auxiliary display cannot be used as processing surfaces.");
        return false;
    }
    addMachiningFace(TopoDS::Face(picked));
    return true;
}

void CamModule::eraseAxisGuideDisplay()
{
    m_guideRenderer->erase(activeGuiDocument());
}

void CamModule::displayAxisGuides()
{
    m_guideRenderer->refresh(activeGuiDocument(), kinematics(), cutterHeadWorldPosition());
}

void CamModule::updateAxisGuideTransforms()
{
    m_guideRenderer->updateTransforms(activeGuiDocument(), kinematics(), cutterHeadWorldPosition());
}

void CamModule::refreshToolpathDisplay()
{
    lcnc::view::ToolpathRenderer::LeadInPreview preview{
        m_previewLeadInContour, m_previewLeadInPointIndex, m_previewLeadInPoint, m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refresh(activeGuiDocument(), toolpathRef(), kinematics(), preview);
}

void CamModule::eraseToolpathDisplay()
{
    m_toolpathRenderer->erase(activeGuiDocument());
}

void CamModule::resetProjectViewState()
{
    const auto activeId = lcnc::Kernel::current().projectManager()->activeWorkspaceId();
    m_machineModelVisible = m_machineVisibleWorkspaceIds.contains(activeId);
    m_machineVisibilityInitialized = false;
    m_visibleMachineEntries.clear();
    m_lastCamSelectionContourIds.clear();
    if (m_machineModelVisible) {
        refreshMachineDisplay();
    } else {
        if (auto* gd = activeGuiDocument())
            gd->eraseDomain(lcnc::ProjectDomain::Machine);
    }
    emit machineVisibilityChanged();
}

void CamModule::clearToolpathViewState(bool emitSignals)
{
    eraseToolpathDisplay();
    // CAM AIS 按 ContourId 管理，eraseAllContours 删 GuiDocument 注册项。
    if (GuiDocument* gd = activeGuiDocument())
        gd->eraseAllContours();

    m_workpieceShape.Nullify();
    m_previewLeadInContour = -1;
    m_previewLeadInPointIndex = -1;
    m_previewLeadInParam = 0.0;
    m_previewLeadInValid = false;

    if (emitSignals) {
        emit toolpathCleared();
        emit toolpathLayersChanged();
        refreshTravelPath();
    }
}

const QList<Handle(AIS_Shape)>& CamModule::contourAis() const
{
    m_camContourAisCache.clear();
    GuiDocument* gd = activeGuiDocument();
    if (!gd)
        return m_camContourAisCache;

    // Phase C：AIS 直接按 ContourId 查找，不再读 CAM XCAF 标签。
    for (int i = 0; i < toolpathRef().contourCount(); ++i) {
        const LaserContour& contour = toolpathRef().contour(i);
        Handle(AIS_Shape) ais = gd->aisShapeForContour(contour.contourId);
        m_camContourAisCache.append(ais);
    }
    return m_camContourAisCache;
}

void CamModule::setAxisPosition(const QString& axisName, double value, bool refreshNow)
{
    const QString normalizedAxis = axisName.trimmed().toUpper();
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CamModule::setAxisPosition {}={} refreshNow={}",
               normalizedAxis.toStdString(), value, refreshNow);
    if (!m_pose || !kinematics()) {
        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "CamModule::setAxisPosition: no pose/kinematics, skip");
        return;
    }
    // 由 pose 统一写值；写入会同步回 kinematics 并 emit poseChanged。
    // 通过 m_inPoseSelfUpdate 抑制 coalescer，让本调用同步刷新（保留旧语义）。
    m_inPoseSelfUpdate = true;
    const bool changed = m_pose->setAxisValue(normalizedAxis, value, /*emitChanged*/ false);
    m_inPoseSelfUpdate = false;
    if (!changed)
        return;
    if (refreshNow) {
        refreshMachineTransforms(QStringList{normalizedAxis});
    } else {
        m_pendingDirtyAxes.insert(normalizedAxis);
        if (m_refreshCoalescer && !m_refreshCoalescer->isActive())
            m_refreshCoalescer->start();
    }
}

lcnc::MachinePose* CamModule::machinePose() const
{
    return m_pose.get();
}

void CamModule::setEntityVisible(const QString& entry, bool visible)
{
    if (entry.isEmpty())
        return;

    if (visible)
        m_visibleMachineEntries.insert(entry);
    else
        m_visibleMachineEntries.remove(entry);
    m_machineVisibilityInitialized = true;

    if (auto* gd = activeGuiDocument()) {
        Handle(AIS_Shape) ais = gd->aisShape(machineDocumentId(), entry);
        if (ais.IsNull()) {
            if (visible && m_machineModelVisible)
                refreshMachineDisplay();
            return;
        }

        if (visible && m_machineModelVisible)
            gd->scene()->displayObject(ais);
        else
            gd->scene()->eraseObject(ais);

        if (gd->hasView())
            gd->view()->Redraw();
    }
}

bool CamModule::isEntityVisible(const QString& entry) const
{
    return m_machineModelVisible && m_visibleMachineEntries.contains(entry);
}

QStringList CamModule::visibleMachineEntries() const
{
    return QStringList(m_visibleMachineEntries.cbegin(), m_visibleMachineEntries.cend());
}

bool CamModule::isMachineModelVisible() const
{
    return m_machineModelVisible;
}

void CamModule::setMachineModelVisible(bool visible)
{
    if (m_machineModelVisible == visible)
        return;

    const auto activeId = lcnc::Kernel::current().projectManager()->activeWorkspaceId();
    if (activeId != kInvalidProjectWorkspaceId) {
        if (visible)
            m_machineVisibleWorkspaceIds.insert(activeId);
        else
            m_machineVisibleWorkspaceIds.remove(activeId);
    }

    m_machineModelVisible = visible;
    if (visible) {
        refreshMachineDisplay();
        return;
    }

    if (auto* gd = activeGuiDocument()) {
        gd->eraseDomain(lcnc::ProjectDomain::Machine);
        if (gd->hasView())
            gd->view()->Redraw();
    }
    emit machineVisibilityChanged();
}

void CamModule::setRotaryAxisGuidesVisible(bool visible)
{
    m_guideRenderer->setRotaryAxisVisible(activeGuiDocument(), visible);
    displayAxisGuides();
}

bool CamModule::rotaryAxisGuidesVisible() const
{
    return m_guideRenderer->rotaryAxisVisible();
}

void CamModule::setCutterHeadGuideVisible(bool visible)
{
    m_guideRenderer->setCutterHeadVisible(activeGuiDocument(), visible);
    displayAxisGuides();
}

bool CamModule::cutterHeadGuideVisible() const
{
    return m_guideRenderer->cutterHeadVisible();
}

void CamModule::setSelectedEntries(const QStringList& entries)
{
    GuiDocument* gd = activeGuiDocument();
    if (!gd)
        return;

    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull())
        return;

    ctx->ClearSelected(false);
    for (const QString& entry : entries) {
        Handle(AIS_Shape) ais = gd->aisShape(machineDocumentId(), entry);
        if (!ais.IsNull())
            ctx->AddOrRemoveSelected(ais, false);
    }

    if (gd->hasView())
        gd->view()->Redraw();

    emit selectionChanged(gd->selectedEntries(machineDocumentId()));
}

QStringList CamModule::selectedEntries() const
{
    if (auto* gd = activeGuiDocument())
        return gd->selectedEntries(machineDocumentId());

    return {};
}

void CamModule::syncSelectionFromView()
{
    GuiDocument* gd = activeGuiDocument();
    QStringList selectedEntries;
    if (gd) {
        selectedEntries = gd->selectedEntries(workpieceDocumentId());
        if (selectedEntries.isEmpty())
            selectedEntries = gd->selectedEntries(machineDocumentId());
    }
    emit selectionChanged(selectedEntries);

    const QList<int> contourIndexes = selectedCamContourIndexes();
    if (!contourIndexes.isEmpty()) {
        emit toolpathContoursSelected(contourIndexes);
        if (contourIndexes.size() == 1)
            emit toolpathContourSelected(contourIndexes.first());
    }

    // ── 推送选择顺序到 SelectionService（差分式）─────────────────────────
    auto selSvc = lcnc::Kernel::current()
                      .services()
                      .getService<lcnc::core::SelectionService>();
    if (selSvc) {
        QSet<std::uint64_t> nowSelected;
        QVector<lcnc::core::SelectionEntry> newlyAdded;
        for (int idx : contourIndexes) {
            if (idx < 0 || idx >= toolpathRef().contourCount()) continue;
            const std::uint64_t cid = toolpathRef().contour(idx).contourId;
            if (cid == 0) continue;
            nowSelected.insert(cid);
            if (!m_lastCamSelectionContourIds.contains(cid)) {
                lcnc::core::SelectionEntry e;
                e.contourId = cid;
                e.source    = lcnc::core::SelectionEntry::OccView;
                newlyAdded.append(e);
            }
        }
        if (!newlyAdded.isEmpty())
            selSvc->recordSelectedBatch(newlyAdded);
        for (auto cid : m_lastCamSelectionContourIds) {
            if (!nowSelected.contains(cid))
                selSvc->removeContour(cid);
        }
        m_lastCamSelectionContourIds = nowSelected;
    }
}

void CamModule::refreshMachineTransforms()
{
    if (auto* gd = activeGuiDocument()) {
        gd->updateMachineWorkspaceTransforms(machineDocument(), workpieceDocument());
        if (m_toolpathRenderer && m_toolpathRenderer->isVisible()) {
            applyCamContourTransforms();
            m_toolpathRenderer->updateTransforms(gd, toolpathRef(), kinematics());
        }
        updateAxisGuideTransforms();
        if (m_travelPathRenderer && m_travelPathRenderer->isVisible())
            m_travelPathRenderer->updateTransforms(gd, kinematics());
        if (gd->hasView())
            gd->view()->Redraw();
    }
}

void CamModule::refreshMachineTransforms(const QStringList& dirtyAxes)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CamModule::refreshMachineTransforms(dirty) n={}", dirtyAxes.size());
    // 现阶段 GuiDocument::updateAxisTransforms 与 MachineGuideRenderer::updateTransforms
    // 内部已是就地 SetLocalTransformation，dirty 集合主要用于：
    //   1) 跳过 m_pose 与几何已一致的"无变化"刷新（上层早 return）；
    //   2) 为后续按子轴粒度的精细化刷新预留接入点。
    // dirty 为空时退化为全量。
    if (dirtyAxes.isEmpty()) {
        refreshMachineTransforms();
        return;
    }
    if (auto* gd = activeGuiDocument()) {
        gd->updateMachineWorkspaceTransforms(machineDocument(), workpieceDocument());
        if (m_toolpathRenderer && m_toolpathRenderer->isVisible()) {
            applyCamContourTransforms();
            m_toolpathRenderer->updateTransforms(gd, toolpathRef(), kinematics());
        }
        updateAxisGuideTransforms();
        if (m_travelPathRenderer && m_travelPathRenderer->isVisible())
            m_travelPathRenderer->updateTransforms(gd, kinematics());
        if (gd->hasView())
            gd->view()->Redraw();
    }
}

void CamModule::setCamContoursVisible(bool visible, bool updateView)
{
    GuiDocument* gd = activeGuiDocument();
    if (!gd)
        return;

    // Phase C：直接迭代 toolpathRef().contours() 按 ContourId 取 AIS。
    for (int index = 0; index < toolpathRef().contourCount(); ++index) {
        const LaserContour& contour = toolpathRef().contour(index);
        Handle(AIS_Shape) ais = gd->aisShapeForContour(contour.contourId);
        if (ais.IsNull())
            continue;
        const bool showContour = visible && contour.enabled;
        if (showContour)
            gd->scene()->displayObject(ais, false);
        else
            gd->scene()->eraseObject(ais, false);
    }
    gd->restoreCamContourSelectionModes();

    if (!updateView)
        return;

    if (!gd->context().IsNull())
        gd->context()->UpdateCurrentViewer();
    if (gd->hasView())
        gd->view()->Redraw();
}

void CamModule::setCamContourVisible(int contourIndex, bool visible, bool updateView)
{
    GuiDocument* gd = activeGuiDocument();
    if (!gd || contourIndex < 0 || contourIndex >= toolpathRef().contourCount())
        return;

    const std::uint64_t contourId = toolpathRef().contour(contourIndex).contourId;
    Handle(AIS_Shape) ais = gd->aisShapeForContour(contourId);
    if (ais.IsNull())
        return;

    if (visible)
        gd->scene()->displayObject(ais, false);
    else
        gd->scene()->eraseObject(ais, false);
    if (visible)
        gd->restoreCamContourSelectionModes();

    if (!updateView)
        return;

    if (!gd->context().IsNull())
        gd->context()->UpdateCurrentViewer();
    if (gd->hasView())
        gd->view()->Redraw();
}

void CamModule::applyCamContourVisibility()
{
    setCamContoursVisible(m_toolpathRenderer->isVisible());
}

void CamModule::applyCamContourTransforms()
{
    GuiDocument* gd = activeGuiDocument();
    MachineKinematics* kin = kinematics();
    if (!gd || !kin)
        return;

    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull())
        return;

    for (int index = 0; index < toolpathRef().contourCount(); ++index) {
        const LaserContour& contour = toolpathRef().contour(index);
        Handle(AIS_Shape) ais = gd->aisShapeForContour(contour.contourId);
        if (ais.IsNull())
            continue;

        gp_Trsf transform;
        if (!contour.workpieceEntry.isEmpty())
            transform = kin->computeWpcTransform(contour.workpieceEntry);

        ais->SetLocalTransformation(transform);
        ctx->RecomputePrsOnly(ais, Standard_False);
    }
}

QList<int> CamModule::selectedCamContourIndexes() const
{
    QList<int> result;
    GuiDocument* gd = activeGuiDocument();
    if (!gd || !m_camData)
        return result;

    // Phase C：AIS 直接以 ContourId 寻址，GuiDocument 反查得到 contourId 列表后
    // 通过 CamDataManager::contourIndexById 映射回 toolpath 中的位置。
    const QVector<std::uint64_t> selectedIds = gd->selectedContourIds();
    for (std::uint64_t cid : selectedIds) {
        const int idx = m_camData->contourIndexById(cid);
        if (idx >= 0)
            result.append(idx);
    }
    return result;
}

void CamModule::refreshMachineDisplay()
{
    if (!m_machineModelVisible) {
        if (auto* gd = activeGuiDocument()) {
            gd->eraseDomain(lcnc::ProjectDomain::Machine);
            if (gd->hasView())
                gd->view()->Redraw();
        }
        emit machineVisibilityChanged();
        return;
    }

    if (auto* gd = activeGuiDocument()) {
        gd->rebuildDomain(lcnc::ProjectDomain::Machine, machineDocument());
        gd->updateMachineWorkspaceTransforms(machineDocument(), workpieceDocument());
        if (machineDocument()) {
            const TDF_LabelSequence labels = machineDocument()->entityLabels(LcncDocument::EntityKind::Machine);
            if (!m_machineVisibilityInitialized) {
                m_visibleMachineEntries.clear();
                for (int i = 1; i <= labels.Length(); ++i)
                    m_visibleMachineEntries.insert(XcafUtils::entry(labels.Value(i)));
                m_machineVisibilityInitialized = true;
            }
            for (int i = 1; i <= labels.Length(); ++i) {
                const QString entry = XcafUtils::entry(labels.Value(i));
                Handle(AIS_Shape) ais = gd->aisShape(machineDocumentId(), entry);
                if (ais.IsNull())
                    continue;
                if (m_machineModelVisible && m_visibleMachineEntries.contains(entry))
                    gd->scene()->displayObject(ais, false);
                else
                    gd->scene()->eraseObject(ais, false);
            }
        }
        displayAxisGuides();
        if (gd->hasView())
            gd->view()->Redraw();
    }
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Machine);
    emit machineVisibilityChanged();
}

void CamModule::syncCamDocumentContours(bool forceRebuild)
{
    // Phase C：AIS 现在按 ContourId 寻址，不再走 CAM 域的 XCAF 镜像。
    // 函数名沿用旧名以减少调用方扩散修改；内部仅做 contour body 的 Display 同步。
    GuiDocument* gd = activeGuiDocument();
    if (!gd)
        return;

    // 把"当前 toolpath 中存在的 contourId 集"与"GuiDocument 已注册的 contourId 集"对齐。
    // 文档切换/读档恢复必须走差分路径，避免把已有 AIS 全部擦掉后重建造成闪烁。
    QSet<std::uint64_t> alive;
    for (int index = 0; index < toolpathRef().contourCount(); ++index) {
        const LaserContour& contour = toolpathRef().contour(index);
        if (contour.wire.IsNull() || contour.contourId == 0)
            continue;
        alive.insert(contour.contourId);
    }

    if (forceRebuild) {
        gd->eraseAllContours();
    } else {
        const QVector<std::uint64_t> displayedIds = gd->displayedContourIds();
        for (std::uint64_t contourId : displayedIds) {
            if (!alive.contains(contourId))
                gd->eraseContour(contourId);
        }
    }

    for (int index = 0; index < toolpathRef().contourCount(); ++index) {
        const LaserContour& contour = toolpathRef().contour(index);
        if (contour.wire.IsNull() || contour.contourId == 0)
            continue;
        if (!forceRebuild && !gd->aisShapeForContour(contour.contourId).IsNull())
            continue;
        const QString name = contour.name.trimmed().isEmpty()
            // 中文翻译：轮廓 %1
            ? tr("Outline %1").arg(index + 1)
            : contour.name;
        // 刀路生成后可能一次新增数百/数千条轮廓。不能在这里逐条提交
        // UpdateCurrentViewer/Redraw，否则主线程会被每条 AIS 的重绘占满。
        // 选择结构统一由后续的 setCamContoursVisible() 建立一次。
        gd->displayContourBody(contour.contourId,
                               contour.wire,
                               name,
                               /*updateViewer=*/false,
                               /*configureSelection=*/false);
    }

    applyCamContourTransforms();
    applyToolpathLayerColors(false);
    applyCamContourVisibility(); // 此处统一建立选择结构并只提交一次视图更新。

}

void CamModule::applyToolpathLayerColors(bool updateView)
{
    GuiDocument* gd = activeGuiDocument();
    if (!gd || !gd->scene())
        return;

    for (int index = 0; index < toolpathRef().contourCount(); ++index) {
        const LaserContour& contour = toolpathRef().contour(index);
        const ToolpathLayer* layer = m_camData ? m_camData->toolpathLayer(contour.layerId) : nullptr;
        if (!layer || !layer->color.isValid())
            continue;

        Handle(AIS_Shape) ais = gd->aisShapeForContour(contour.contourId);
        if (ais.IsNull())
            continue;

        const QColor color = layer->color;
        gd->scene()->setShapeColor(
            ais,
            Quantity_Color(color.redF(), color.greenF(), color.blueF(), Quantity_TOC_RGB),
            false);
    }

    if (!updateView)
        return;
    if (!gd->context().IsNull())
        gd->context()->UpdateCurrentViewer();
    if (gd->hasView())
        gd->view()->Redraw();
}

// ============================================================================
// 工程核心 CAM 数据加载后的视图刷新（数据 IO 已下沉到 core/project/cam）
// ============================================================================

void CamModule::writeContourGeometryToDocument()
{
    // 统一工程文档：把每条轮廓的 wire 作为 EntityKind::Cam 实体写入工程 doc，
    // 并把返回的 label entry 记录到 LaserContour.xcafEntry，供持久化 + 读回时重连几何。
    LcncDocument* doc = workpieceDocument();
    if (!doc)
        return;
    doc->clearEntityKind(LcncDocument::EntityKind::Cam);
    for (int i = 0; i < toolpathRef().contourCount(); ++i) {
        LaserContour& c = toolpathRef().contour(i);
        if (c.wire.IsNull() || c.contourId == 0) {
            c.xcafEntry.clear();
            continue;
        }
        // 实体名编码 contourId（"cam:<id>"）——XCAF 名称会随 .xbf 导出/导入保留，
        // 而 label entry 字符串在导出/导入后会变；故 relink 以名称里的 contourId 为准。
        const QString name = QStringLiteral("cam:%1").arg(c.contourId);
        const TDF_Label lbl = doc->addShapeEntity(c.wire, name, LcncDocument::EntityKind::Cam);
        c.xcafEntry = XcafUtils::entry(lbl);
    }
}

void CamModule::relinkContourGeometryFromDocument()
{
    // 读档后：core 已恢复轮廓元数据(含 xcafEntry)+采样点；此处按 xcafEntry 从工程 doc
    // 的 Cam 实体取回 wire 几何（v3 起几何随 XCAF 持久化，无需运行时重算）。
    LcncDocument* doc = workpieceDocument();
    if (!doc)
        return;
    // 以实体名里编码的 contourId 关联（"cam:<id>"）——稳定且跨 .xbf 导出/导入有效。
    QHash<std::uint64_t, TopoDS_Shape> byContourId;
    const TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Cam);
    for (int i = 1; i <= labels.Length(); ++i) {
        const TDF_Label lbl = labels.Value(i);
        const QString name = XcafUtils::name(lbl);
        if (!name.startsWith(QStringLiteral("cam:")))
            continue;
        bool ok = false;
        const std::uint64_t id = name.mid(4).toULongLong(&ok);
        if (ok && id != 0)
            byContourId.insert(id, XcafUtils::shape(lbl));
    }
    for (int i = 0; i < toolpathRef().contourCount(); ++i) {
        LaserContour& c = toolpathRef().contour(i);
        auto it = byContourId.constFind(c.contourId);
        if (it == byContourId.constEnd() || it.value().IsNull())
            continue;
        if (it.value().ShapeType() == TopAbs_WIRE)
            c.wire = TopoDS::Wire(it.value());
    }
}

void CamModule::pushGenerationParamsToCamData()
{
    // 把当前运行时生成参数固化到工程核心数据，供随工程持久化。
    if (!m_camData)
        return;
    auto& gp = m_camData->generationParams();
    gp.leadInLength         = toolpathRef().globalLeadInLength();
    gp.deflection           = m_deflection;
    gp.smoothAngle          = m_smoothAngle;
    gp.useFaceClassification = m_useFaceClassification;
    gp.extractionStrategy = m_extractionStrategy;
    if (m_toolpathRenderer)
        gp.normalSampleStep = m_toolpathRenderer->normalSampleStep();
}

void CamModule::applyGenerationParamsFromCamData()
{
    // 全局生成/显示参数归程序配置所有，不能被工程缓存反向覆盖。
    // 工程只保存每个轮廓自己的 applied/pending 参数与已生成结果。
    toolpathRef().setGlobalLeadInLength(m_config.leadInLength());
    m_deflection            = m_config.deflection();
    m_smoothAngle           = m_config.smoothAngle();
    m_useFaceClassification = m_config.useFaceClassification();
    m_extractionStrategy = m_config.extractionStrategy();
    if (m_toolpathRenderer) {
        m_toolpathRenderer->setShowNormals(m_config.showNormals());
        m_toolpathRenderer->setNormalSampleStep(m_config.normalSampleStep());
    }
}

void CamModule::onCamDataLoaded()
{
    // core 已把刀路灌入 CamDataManager；此处恢复程序级全局参数与工程轮廓 wire，
    // 再把数据映射到渲染层。
    applyGenerationParamsFromCamData();
    if (!m_camData || !m_camData->hasToolpath()) {
        clearToolpathViewState(/*emitSignals=*/true);
        LCNC_INFO(lcnc::LogCode::Generic,
                  "cam.toolpath: project has no cached CAM data; cleared view");
        return;
    }

    relinkContourGeometryFromDocument();
    m_workpieceShape = collectWorkpieceShape();
    // Rebind any persisted manual machining faces after project reload.
    rebindMachiningFacesFromRecords();
    const QList<WorkpieceShapeSource> sources = collectWorkpieceShapes();
    for (LaserContour& contour : toolpathRef().contours()) {
        for (const WorkpieceShapeSource& source : sources) {
            if (source.workpieceEntry == contour.workpieceEntry) {
                contour.sourceShape = source.shape;
                break;
            }
        }
        contour.leadIn.length = contour.appliedParams.leadInLength;
        if (contour.leadIn.valid
            && !contour.points.empty()
            && (!contour.points.front().crossSectionNormalValid
                || !contour.leadInSolution.valid)) {
            contour.needsRecalculation = true;
        }
    }
    syncCamDocumentContours();
    refreshToolpathDisplay();
    if (toolpathRef().contourCount() > 0)
        setActiveContourId(static_cast<lcnc::cam::ContourId>(toolpathRef().contour(0).contourId));
    emit toolpathGenerated();
    emit toolpathLayersChanged();
    refreshTravelPath();

    LCNC_INFO(lcnc::LogCode::Generic,
              "cam.toolpath: refreshed view for {} layers, {} contours",
              toolpathRef().layers().size(),
              toolpathRef().contourCount());
}

// ── Travel path 虚线显示 ─────────────────────────────────────────────────────

void CamModule::setTravelPathVisible(bool on)
{
    if (!m_travelPathRenderer) return;
    if (m_travelPathRenderer->isVisible() == on) return;
    m_travelPathRenderer->setVisible(on);
    if (on) {
        refreshTravelPath();
    } else {
        m_travelPathRenderer->erase(activeGuiDocument());
        if (auto* gd = activeGuiDocument()) {
            if (gd->hasView()) gd->view()->Redraw();
        }
    }
}

bool CamModule::isTravelPathVisible() const
{
    return m_travelPathRenderer && m_travelPathRenderer->isVisible();
}

void CamModule::refreshTravelPath()
{
    if (!m_travelPathRenderer || !m_travelPathRenderer->isVisible()) return;
    GuiDocument* gd = activeGuiDocument();
    if (!gd) return;

    // 从 Process 端只读视图取顺序，再到 snapshot 取端点。
    auto provider = lcnc::Kernel::current()
                        .services()
                        .getService<lcnc::process::IProcessCuttingPlanProvider>();
    if (!provider) {
        m_travelPathRenderer->refresh(gd, {});
        return;
    }
    const auto orderedIds = provider->orderedContourIds();
    if (orderedIds.size() < 2) {
        m_travelPathRenderer->refresh(gd, {});
        if (gd->hasView()) gd->view()->Redraw();
        return;
    }

    QHash<std::uint64_t, const LaserContour*> byId;
    byId.reserve(toolpathRef().contourCount());
    for (const LaserContour& contour : toolpathRef().contours()) {
        if (contour.contourId != 0)
            byId.insert(contour.contourId, &contour);
    }

    QVector<lcnc::view::TravelPathRenderer::Segment> segments;
    segments.reserve(orderedIds.size());
    for (auto id : orderedIds) {
        auto it = byId.find(id);
        if (it == byId.end()) continue;
        const LaserContour* c = it.value();
        if (!c || c->points.empty())
            continue;

        const gp_Pnt cutStartLocal = c->points.front().position;
        const gp_Pnt endLocal = c->points.back().position;
        gp_Pnt startLocal = cutStartLocal;
        if (c->leadInSolution.valid)
            startLocal = c->leadInSolution.point.position;

        lcnc::view::TravelPathRenderer::Segment s;
        s.contourId = id;
        s.workpieceEntry = c->workpieceEntry;
        s.sx = startLocal.X(); s.sy = startLocal.Y(); s.sz = startLocal.Z();
        s.ex = endLocal.X();   s.ey = endLocal.Y();   s.ez = endLocal.Z();
        segments.append(s);
    }
    m_travelPathRenderer->refresh(gd, segments);
    m_travelPathRenderer->updateTransforms(gd, kinematics());
    if (gd->hasView()) gd->view()->Redraw();
}
