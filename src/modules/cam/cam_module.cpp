
#include "modules/cam/cam_module.h"
#include "view/toolpath_renderer.h"
#include "view/travel_path_renderer.h"
#include "view/machine_guide_renderer.h"
#include "core/project/cam/cam_data_manager.h"
#include "core/project/cam/layer_container.h"
#include "core/project/cam/layer_manager.h"
#include "modules/cam/services/machine_axis_detector.h"
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

#include <QFileInfo>
#include <QPoint>
#include <QSignalBlocker>
#include <QByteArray>
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

class CamToolpathProviderAdapter final : public lcnc::cam::ICamToolpathProvider
{
public:
    explicit CamToolpathProviderAdapter(CamModule* module)
        : m_module(module)
    {
    }

    bool hasToolpath() const override
    {
        return m_module && m_module->hasToolpath();
    }

    std::uint64_t toolpathRevision() const override
    {
        return m_module ? m_module->toolpathRevision() : 0;
    }

    lcnc::cam::ToolpathExportSnapshot exportToolpathSnapshot() const override
    {
        return m_module ? m_module->exportToolpathSnapshot() : lcnc::cam::ToolpathExportSnapshot{};
    }

    lcnc::cam::ToolpathExportSnapshot exportToolpathSnapshotForOrder(
        const QVector<std::uint64_t>& orderedContourIds) const override
    {
        return m_module
            ? m_module->exportToolpathSnapshotForOrder(orderedContourIds)
            : lcnc::cam::ToolpathExportSnapshot{};
    }

private:
    CamModule* m_module{nullptr};
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
        if (auto* mgr = layerMgr()) {
            auto bump = [this] { ++m_revision; };
            QObject::connect(mgr, &lcnc::cam::LayerManager::layersReset,             this, bump);
            QObject::connect(mgr, &lcnc::cam::LayerManager::layerAdded,              this, [this](std::uint64_t) { ++m_revision; });
            QObject::connect(mgr, &lcnc::cam::LayerManager::layerRemoved,            this, [this](std::uint64_t) { ++m_revision; });
            QObject::connect(mgr, &lcnc::cam::LayerManager::layersReordered,         this, bump);
            QObject::connect(mgr, &lcnc::cam::LayerManager::layerPropertyChanged,    this, [this](std::uint64_t, lcnc::cam::LayerProperty) { ++m_revision; });
            QObject::connect(mgr, &lcnc::cam::LayerManager::contourMembershipChanged,this, bump);
            QObject::connect(mgr, &lcnc::cam::LayerManager::manualContourOrderChanged,this, bump);
            QObject::connect(mgr, &lcnc::cam::LayerManager::sortStrategyChanged,     this, [this](lcnc::cam::CuttingPlanSortStrategy) { ++m_revision; });
            QObject::connect(mgr, &lcnc::cam::LayerManager::lastAutoSortAxisChanged, this, [this](lcnc::cam::AutoSortAxis) { ++m_revision; });
        }
    }

    // ── 只读 ─────────────────────────────────────────────────────────────
    QVector<lcnc::cam::LayerSnapshot> layers() const override
    {
        QVector<lcnc::cam::LayerSnapshot> out;
        const auto* container = layerContainer();
        const auto* tp = container ? container->toolpath() : nullptr;
        if (!tp)
            return out;
        out.reserve(static_cast<int>(tp->layers().size()));
        for (const ToolpathLayer& layer : tp->layers()) {
            lcnc::cam::LayerSnapshot s;
            s.layerId           = layer.layerId;
            s.name              = layer.name;
            s.toolName          = layer.toolName;
            s.compensationIndex = layer.compensationIndex;
            s.enabled           = layer.enabled;
            s.includedContours  = layer.includedContours;
            s.contourIds.reserve(static_cast<int>(layer.contourIds.size()));
            for (std::uint64_t cid : layer.contourIds)
                s.contourIds.push_back(static_cast<lcnc::cam::ContourId>(cid));
            out.append(s);
        }
        return out;
    }

    QVector<lcnc::cam::ContourId> manualContourOrder() const override
    {
        const auto* c = layerContainer();
        return c ? c->manualContourOrder() : QVector<lcnc::cam::ContourId>{};
    }

    lcnc::cam::CuttingPlanSortStrategy sortStrategy() const override
    {
        const auto* c = layerContainer();
        return c ? c->sortStrategy() : lcnc::cam::CuttingPlanSortStrategy::LayerThenContour;
    }

    lcnc::cam::AutoSortAxis lastAutoSortAxis() const override
    {
        const auto* c = layerContainer();
        return c ? c->lastAutoSortAxis() : lcnc::cam::AutoSortAxis::XPos;
    }

    std::uint64_t revision() const override { return m_revision; }

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
        QStringLiteral("CAM模块"),
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
                    clearToolpathViewState(/*emitSignals=*/true);
                });
    }

    m_initialized = true;
    LCNC_INFO(lcnc::LogCode::Generic, "CamModule init done");

    // 订阅 Process 模块的"切割路径显示"开关与切割链表变化，驱动 TravelPathRenderer。
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
    m_initialized = false;
    LCNC_INFO(lcnc::LogCode::Generic, "CamModule stop done");
}

CamModule::CamModule(QObject* parent)
    : QObject(parent)
    , m_toolpathRenderer(std::make_unique<lcnc::view::ToolpathRenderer>())
    , m_guideRenderer(std::make_unique<lcnc::view::MachineGuideRenderer>())
    , m_travelPathRenderer(std::make_unique<lcnc::view::TravelPathRenderer>())
    , m_camData(lcnc::Kernel::current().projectManager()->camData())
    , m_toolpath(m_camData->toolpath())
{
    // 加载持久化配置（首次启动会自动迁移旧版 CamConfig.json -> cam.toml）。
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
    m_toolpath.setGlobalLeadInLength(config.leadInLength());
    m_toolpath.setGlobalNormalAngle(config.normalAngle());
    m_deflection = config.deflection();
    m_smoothAngle = config.smoothAngle();
    m_useFaceClassification = config.useFaceClassification();
    m_toolpathRenderer->setShowNormals(config.showNormals());
    m_toolpathRenderer->setNormalSampleStep(config.normalSampleStep());
    // 用全局默认值播种初始（空）工程的工程级生成参数。
    pushGenerationParamsToCamData();

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

GuiDocument* CamModule::workspaceGuiDocument() const
{
    return lcnc::Kernel::current().guiApp()->workspaceGuiDocument();
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

    TaskId taskId = lcnc::Kernel::current().taskManager()->run(tr("加载机台: %1").arg(fi.fileName()),
        [filePath, doc](TaskProgress* prog) {
            lcnc::cam::machine_io::loadMachineFromFile(doc, filePath, prog);
        });

    watchTask(this, taskId, [this, normalizedPath](bool ok) {
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
    if (auto* gd = workspaceGuiDocument())
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

    if (auto* gd = workspaceGuiDocument())
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
        result.append({QString(), tr("— 解除已有挂载 —")});

    for (const MachineAxisDef& axis : kin->axes()) {
        QString displayName;
        if (axis.name == QStringLiteral("BASE")) {
            displayName = tr("BASE（固定基座）");
        } else if (axis.motionType == MachineAxisDef::Rotary) {
            displayName = tr("%1 轴（旋转）").arg(axis.name);
        } else {
            displayName = tr("%1 轴（线性）").arg(axis.name);
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
    if (auto* gd = workspaceGuiDocument()) {
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
        emit operationFailed(tr("轴心快速填充"), errorMessage);
        return false;
    }

    const QString normalizedAxis = axisName.trimmed().toUpper();
    if (normalizedAxis != QStringLiteral("A") && normalizedAxis != QStringLiteral("C")) {
        emit operationFailed(tr("轴心快速填充"),
                             tr("当前快速填充仅支持 A 轴和 C 轴。"));
        return false;
    }

    gp_Pnt faceCenter;
    if (!resolveReferencePlaneCenter(occView, screenPos, faceCenter, &errorMessage)) {
        emit operationFailed(tr("轴心快速填充"), errorMessage);
        return false;
    }

    MachineKinematics* kin = kinematics();
    if (!kin) {
        emit operationFailed(tr("轴心快速填充"), tr("找不到机台轴系配置。"));
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
        recalcToolpath();

    return true;
}

bool CamModule::setCutterHeadModelPositionFromReferenceFace(WidgetOccView* occView,
                                                            const QPoint& screenPos)
{
    gp_Pnt faceCenter;
    QString errorMessage;
    if (!resolveReferencePlaneCenter(occView, screenPos, faceCenter, &errorMessage)) {
        emit operationFailed(tr("切割头对齐"), errorMessage);
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
        emit operationFailed(tr("机台标定位"), msg);
        return false;
    };

    QString reason;
    if (!ensureAcCenterCalibrationAvailable(&reason))
        return fail(reason);

    MachineKinematics* kin = kinematics();
    if (!kin)
        return fail(tr("找不到机台轴系配置。"));

    try {
        gp_Pnt configuredCenter;
        if (!currentAcRotationCenter(configuredCenter))
            return fail(tr("请先在应用程序选项的机台构型页填写 A/C 旋转中心。"));

        // 切割头模型点（BASE 局部坐标），直接采用拾取面中心。
        m_cutterHeadModelPosition = inputs.cutterHeadFaceCenter;

        // 进入"机台标定位"：A=0, C=0；XY 调整为切割头世界 XY 与配置旋转中心 XY 对齐。
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
        return fail(tr("OCC 异常：%1").arg(QString::fromUtf8(f.GetMessageString())));
    } catch (const std::exception& e) {
        return fail(tr("异常：%1").arg(QString::fromUtf8(e.what())));
    } catch (...) {
        return fail(tr("发生未知异常。"));
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
        emit operationFailed(tr("机台坐标系标定"), msg);
        return false;
    };

    // ── 1) 进入标定位（写入轴心、切割头模型点、A=C=0、XY 对齐） ────────
    QString reason;
    if (!enterStandardCalibrationPose(inputs, &reason))
        return fail(reason);

    try {
        gp_Pnt configuredCenter;
        if (!currentAcRotationCenter(configuredCenter))
            return fail(tr("请先在应用程序选项的机台构型页填写 A/C 旋转中心。"));

        // A/C 拾取只用于推导“模型当前的 AC 交点”，不写入物理旋转中心。
        const gp_Pnt pickedModelCenter(inputs.cFaceCenter.X(),
                                       inputs.aFaceCenter.Y(),
                                       inputs.aFaceCenter.Z());
        const gp_Vec translation(pickedModelCenter, configuredCenter);
        if (translation.SquareMagnitude() >= 1e-12) {
            if (!translateMachineGeometryOnly(translation, tr("机台模型对齐")))
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
        return fail(tr("OCC 异常：%1").arg(QString::fromUtf8(f.GetMessageString())));
    } catch (const std::exception& e) {
        return fail(tr("异常：%1").arg(QString::fromUtf8(e.what())));
    } catch (...) {
        return fail(tr("发生未知异常。"));
    }

    displayAxisGuides();
    refreshMachineDisplay();
    if (hasToolpath())
        recalcToolpath();

    LCNC_INFO(lcnc::LogCode::Generic,
              "CamModule::applyAxisCalibration done (model geometry aligned; rotation center kept from machine config)");
    emit machineWorkspaceChanged();
    return true;
}

bool CamModule::alignMachineToPhysicalCenter(const gp_Pnt& physicalCenter)
{
    QString errorMessage;
    if (!ensureAcCenterCalibrationAvailable(&errorMessage)) {
        emit operationFailed(tr("机台坐标系转换"), errorMessage);
        return false;
    }

    gp_Pnt currentCenter;
    if (!currentAcRotationCenter(currentCenter)) {
        emit operationFailed(tr("机台坐标系转换"), tr("无法计算当前模型 AC 中心。"));
        return false;
    }

    const gp_Vec translation(currentCenter, physicalCenter);
    if (translation.SquareMagnitude() < 1e-12)
        return true;

    return translateMachineWorkspace(translation, tr("机台坐标系转换"));
}

bool CamModule::alignMachineToPhysicalCutterHead()
{
    const gp_Vec translation(m_cutterHeadModelPosition, m_cutterHeadPhysicalPosition);
    if (translation.SquareMagnitude() < 1e-12)
        return true;

    return translateMachineWorkspace(translation, tr("切割头物理对齐"));
}

bool CamModule::translateMachineWorkspace(const gp_Vec& translation, const QString& operationTitle)
{
    if (translation.SquareMagnitude() < 1e-12)
        return true;

    LcncDocument* doc = machineDocument();
    MachineKinematics* kin = kinematics();
    if (!doc || !kin) {
        emit operationFailed(operationTitle, tr("找不到项目文档或轴系配置。"));
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
        emit operationFailed(operationTitle, tr("整机平移失败，当前模型已恢复原始位置。"));
        return false;
    }

    for (const MachineAxisDef& axis : axisSnapshot) {
        const gp_Pnt shiftedOrigin = axis.origin.Translated(translation);
        if (!kin->setAxisOrigin(axis.name, shiftedOrigin)) {
            rollback();
            emit operationFailed(operationTitle, tr("轴心平移失败，当前模型已恢复原始位置。"));
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
        emit operationFailed(operationTitle, tr("找不到机台项目文档。"));
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
        emit operationFailed(operationTitle, tr("机台几何平移失败，当前模型已恢复原始位置。"));
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
    const QString displayName = stateName.isEmpty() ? tr("当前工件") : stateName;

    result.append({doc->id(), tr("%1  (%2 形体)").arg(displayName).arg(workpieceCount), workpieceCount});

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
    GuiDocument* gd = workspaceGuiDocument();
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
    GuiDocument* gd = workspaceGuiDocument();
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
    GuiDocument* gd = workspaceGuiDocument();
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
        emit operationFailed(tr("工件安装位置"), tr("更新工件安装位置失败，当前安装位置未修改。"));
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
        if (hasToolpath())
            syncCamDocumentContours();
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

    auto* gd = workspaceGuiDocument();
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
        emit operationFailed(tr("工件安装位置"), tr("当前构型没有可用于对齐的工件旋转中心。"));
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
            emit operationFailed(tr("工件安装"), tr("移动工件到安装位置失败。"));
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
    if (GuiDocument* gd = workspaceGuiDocument()) {
        gd->rebuildDomain(lcnc::ProjectDomain::Workpiece, workpieceDocument());
        gd->updateMachineWorkspaceTransforms(machineDocument(), workpieceDocument());
        if (gd->hasView())
            gd->view()->Redraw();
    }
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Workpiece);
}

void CamModule::resetWorkpieceDisplayLocation()
{
    GuiDocument* gd = workspaceGuiDocument();
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

    if (auto* gd = workspaceGuiDocument())
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

bool CamModule::generateToolpath(double smoothAngle, bool useFaceClassification, double deflection)
{
    const QList<WorkpieceShapeSource> workpieceSources = collectWorkpieceShapes();
    if (workpieceSources.isEmpty())
        return false;

    // 用 resetIds=false 清理：保留 signature → id 映射，使重新生成的 id 与原项目对齐。
    eraseToolpathDisplay();
    m_camData->clearToolpath(/*resetIds=*/false);
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
    params.useFaceClassification = effectiveUseFaceClassification;
    params.deflection = deflection;

    std::vector<LaserContour> allContours;

    for (const WorkpieceShapeSource& source : workpieceSources) {
        if (source.shape.IsNull())
            continue;

        auto contours = LaserToolpathBuilder::extractContours(source.shape, params);
        if (contours.empty())
            continue;

        for (auto& contour : contours) {
            contour.workpieceEntry = source.workpieceEntry;
            contour.sourceShape = source.shape;
            if (workpieceSources.size() > 1) {
                contour.sourceInfo = contour.sourceInfo.isEmpty()
                    ? tr("工件源 #%1").arg(source.componentIndex + 1)
                    : tr("%1 · 工件源 #%2").arg(contour.sourceInfo).arg(source.componentIndex + 1);
            }

            if (contour.points.empty())
                LaserToolpathBuilder::discretizeContour(contour, source.shape, deflection);

            LaserToolpathBuilder::computeMachineCoordinates(
                contour, kinematics(), gp_Trsf(), nullptr);
            allContours.push_back(std::move(contour));
        }
    }

    if (allContours.empty())
        return false;

    m_toolpath.contours() = std::move(allContours);
    m_camData->ensureContourIds();
    m_camData->ensureToolpathLayers();
    applyDefaultCuttingOrder();
    m_camData->commitToolpathStates();    // 把 signature → id 映射固化下来，跨次稳定
    pushGenerationParamsToCamData();       // 固化本次生成所用参数，随工程持久化
    writeContourGeometryToDocument();      // 轮廓 wire 写入统一工程文档(EntityKind::Cam)
    relinkContourGeometryFromDocument();   // 与工程包加载路径一致：显示/拾取使用 XCAF 文档版 wire
    syncCamDocumentContours();
    m_toolpathRenderer->setVisible(workspaceGuiDocument(), true);

    refreshToolpathDisplay();
    emit toolpathGenerated();
    emit toolpathLayersChanged();
    refreshTravelPath();
    return true;
}

void CamModule::clearToolpath()
{
    clearToolpathViewState(/*emitSignals=*/false);
    m_camData->clearToolpath();
    // 统一工程文档：清掉轮廓几何(EntityKind::Cam)实体。
    if (LcncDocument* doc = workpieceDocument())
        doc->clearEntityKind(LcncDocument::EntityKind::Cam);
    m_clearingToolpath = true;
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    m_clearingToolpath = false;
    emit toolpathCleared();
    emit toolpathLayersChanged();
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
            *errorMessage = tr("找不到机台轴系配置。请先选择 AC 转台构型。");
        return false;
    }

    if (kin->configType() != QStringLiteral("VERTICAL_AC_TABLE")) {
        if (errorMessage)
            *errorMessage = tr("当前仅 VERTICAL_AC_TABLE 构型支持 A/C 模型对齐。");
        return false;
    }

    if (!kin->findAxis(QStringLiteral("A")) || !kin->findAxis(QStringLiteral("C"))) {
        if (errorMessage)
            *errorMessage = tr("当前 AC 转台轴定义不完整，缺少 A 轴或 C 轴。\n请先在应用程序选项的机台构型页完成配置。");
        return false;
    }

    return true;
}

void CamModule::translateToolpathWorldData(const gp_Vec& translation)
{
    if (translation.SquareMagnitude() < 1e-12)
        return;

    for (LaserContour& contour : m_toolpath.contours()) {
        const TopoDS_Shape movedWire = translatedShapeCopy(contour.wire, translation);
        if (!movedWire.IsNull() && movedWire.ShapeType() == TopAbs_WIRE)
            contour.wire = TopoDS::Wire(movedWire);
        contour.sourceShape = translatedShapeCopy(contour.sourceShape, translation);

        for (ToolpathPoint& point : contour.points)
            point.position.Translate(translation);

        if (contour.leadIn.valid)
            contour.leadIn.entryPoint.Translate(translation);
    }

    m_workpieceShape = translatedShapeCopy(m_workpieceShape, translation);

    if (m_previewLeadInValid)
        m_previewLeadInPoint.Translate(translation);
}

void CamModule::updateToolpathMachineCoordinates()
{
    for (LaserContour& contour : m_toolpath.contours()) {
        LaserToolpathBuilder::computeMachineCoordinates(
            contour, kinematics(), gp_Trsf(), nullptr);
    }
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
    items.reserve(m_toolpath.contourCount());

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
        order.reserve(m_toolpath.contourCount());
        for (const LaserContour& contour : m_toolpath.contours()) {
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
    for (const LaserContour& contour : m_toolpath.contours()) {
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

    if (items.size() != m_toolpath.contourCount())
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
    if (!m_camData || m_toolpath.contourCount() <= 0)
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
    return m_toolpath;
}

LaserToolpath& CamModule::toolpathRef()
{
    return m_toolpath;
}

bool CamModule::hasToolpath() const
{
    return m_toolpath.contourCount() > 0;
}

int CamModule::toolpathContourCount() const
{
    return m_toolpath.contourCount();
}

int CamModule::toolpathContourPointCount(int contourIndex) const
{
    if (contourIndex < 0 || contourIndex >= m_toolpath.contourCount())
        return 0;
    return static_cast<int>(m_toolpath.contour(contourIndex).points.size());
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

    mix(static_cast<std::uint64_t>(m_toolpath.contourCount()));
    for (const LaserContour& contour : m_toolpath.contours()) {
        mix(contour.contourId);
        mix(contour.layerId);
        mix(contour.enabled ? 1ull : 0ull);
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
    for (const ToolpathLayer& layer : m_toolpath.layers()) {
        mix(layer.layerId);
        mix(layer.enabled ? 1ull : 0ull);
        mix(static_cast<std::uint64_t>(layer.contourIds.size()));
    }
    return revision;
}

lcnc::cam::ToolpathExportSnapshot CamModule::exportToolpathSnapshot() const
{
    return buildToolpathExportSnapshot(
        m_toolpath.contours(),
        toolpathRevision(),
        hasToolpath() ? tr("CAM 刀路快照已导出") : tr("CAM 当前无刀路"));
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
        if (contourIdx < 0 || contourIdx >= m_toolpath.contourCount())
            continue;
        orderedContours.push_back(m_toolpath.contour(contourIdx));
    }

    MachineCoord continuityState;
    MachineKinematics* kin = kinematics();
    for (LaserContour& contour : orderedContours) {
        LaserToolpathBuilder::computeMachineCoordinates(
            contour, kin, gp_Trsf(), &continuityState);
    }

    LCNC_INFO(lcnc::LogCode::Generic,
              "cam.toolpath: exported ordered snapshot with continuous rotary planning, contours={}",
              orderedContours.size());

    return buildToolpathExportSnapshot(
        orderedContours,
        toolpathRevision(),
        orderedContours.empty() ? tr("CAM 当前无刀路") : tr("CAM 有序规划刀路快照已导出"));
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
        for (const ToolpathLayer& layer : m_toolpath.layers()) {
            if (layer.layerId == layerId)
                return &layer;
        }
        return nullptr;
    };

    MachineKinematics* kin = kinematics();
    const double leadInLen = m_toolpath.globalLeadInLength();
    const double leadInNormalAng = m_toolpath.globalNormalAngle();

    for (const LaserContour& contour : m_toolpath.contours()) {
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

            if (contour.leadIn.valid) {
                bool ok = false;
                gp_Pnt leadStartLocal = LaserToolpathBuilder::computeLeadInStartPoint(
                    contour, leadInLen, leadInNormalAng, &ok);
                if (ok) {
                    leadStartLocal.Transform(wpc);
                    startWorld = leadStartLocal;
                    hasLeadIn = true;
                } else {
                    startWorld = cutStartWorld;
                }
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

void CamModule::setLeadInEntry(int contourIdx, const gp_Pnt& entryPoint, double entryParam)
{
    if (contourIdx < 0 || contourIdx >= m_toolpath.contourCount()) return;

    LaserContour& c = m_toolpath.contour(contourIdx);
    c.leadIn.entryPoint = entryPoint;
    c.leadIn.entryParam = entryParam;
    c.leadIn.valid = true;

    lcnc::view::ToolpathRenderer::LeadInPreview preview{
        m_previewLeadInContour, m_previewLeadInPoint, m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refreshLeadIns(workspaceGuiDocument(), m_toolpath, kinematics(), preview);
}

void CamModule::setLeadInLength(double mm)
{
    m_toolpath.setGlobalLeadInLength(mm);
    lcnc::view::ToolpathRenderer::LeadInPreview preview{
        m_previewLeadInContour, m_previewLeadInPoint, m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refreshLeadIns(workspaceGuiDocument(), m_toolpath, kinematics(), preview);
}

double CamModule::leadInLength() const
{
    return m_toolpath.globalLeadInLength();
}

void CamModule::setNormalAngle(double deg)
{
    m_toolpath.setGlobalNormalAngle(deg);
    lcnc::view::ToolpathRenderer::LeadInPreview preview{
        m_previewLeadInContour, m_previewLeadInPoint, m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refreshLeadIns(workspaceGuiDocument(), m_toolpath, kinematics(), preview);
}

double CamModule::normalAngle() const
{
    return m_toolpath.globalNormalAngle();
}

void CamModule::setDeflection(double mm)
{
    if (mm <= 0.0)
        return;

    m_deflection = mm;
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
    m_toolpathRenderer->refreshNormals(workspaceGuiDocument(), m_toolpath, kinematics());
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
    if (m_toolpathRenderer->showNormals())
        m_toolpathRenderer->refreshNormals(workspaceGuiDocument(), m_toolpath, kinematics());
}

bool CamModule::resolveLeadInHit(WidgetOccView* occView,
                                 const QPoint& screenPos,
                                 int& contourIdx,
                                 gp_Pnt& entryPoint,
                                 double& entryParam) const
{
    return lcnc::cam::reference_pick::resolveLeadInHit(occView, screenPos, m_toolpath,
                                                       contourIdx, entryPoint, entryParam);
}

bool CamModule::updateLeadInPreview(WidgetOccView* occView, const QPoint& screenPos)
{
    int contourIdx = -1;
    gp_Pnt entryPoint;
    double entryParam = 0.0;

    if (!resolveLeadInHit(occView, screenPos, contourIdx, entryPoint, entryParam)) {
        if (m_previewLeadInValid)
            cancelLeadInPreview();
        return false;
    }

    if (m_previewLeadInValid
        && m_previewLeadInContour == contourIdx
        && m_previewLeadInPoint.Distance(entryPoint) < 1e-6
        && std::abs(m_previewLeadInParam - entryParam) < 1e-6) {
        return true;
    }

    m_previewLeadInContour = contourIdx;
    m_previewLeadInPoint = entryPoint;
    m_previewLeadInParam = entryParam;
    m_previewLeadInValid = true;
    lcnc::view::ToolpathRenderer::LeadInPreview preview{
        m_previewLeadInContour, m_previewLeadInPoint, m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refreshLeadIns(workspaceGuiDocument(), m_toolpath, kinematics(), preview);
    return true;
}

bool CamModule::commitLeadInPreview(WidgetOccView* occView, const QPoint& screenPos)
{
    if (!updateLeadInPreview(occView, screenPos) || !m_previewLeadInValid)
        return false;

    if (m_previewLeadInContour < 0 || m_previewLeadInContour >= m_toolpath.contourCount())
        return false;

    LaserContour& contour = m_toolpath.contour(m_previewLeadInContour);
    contour.leadIn.entryPoint = m_previewLeadInPoint;
    contour.leadIn.entryParam = m_previewLeadInParam;
    contour.leadIn.valid = true;

    m_previewLeadInContour = -1;
    m_previewLeadInParam = 0.0;
    m_previewLeadInValid = false;
    m_toolpathRenderer->refreshLeadIns(workspaceGuiDocument(), m_toolpath, kinematics());
    return true;
}

void CamModule::cancelLeadInPreview()
{
    if (!m_previewLeadInValid)
        return;

    m_previewLeadInContour = -1;
    m_previewLeadInParam = 0.0;
    m_previewLeadInValid = false;
    m_toolpathRenderer->refreshLeadIns(workspaceGuiDocument(), m_toolpath, kinematics());
}

void CamModule::setContourEnabled(int contourIdx, bool enabled)
{
    if (contourIdx < 0 || contourIdx >= m_toolpath.contourCount())
        return;

    LaserContour& contour = m_toolpath.contour(contourIdx);
    if (contour.enabled == enabled)
        return;

    contour.enabled = enabled;
    setCamContourVisible(contourIdx, enabled && m_toolpathRenderer->isVisible(), false);
    lcnc::view::ToolpathRenderer::LeadInPreview preview{
        m_previewLeadInContour, m_previewLeadInPoint, m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refreshContour(workspaceGuiDocument(), m_toolpath, kinematics(), contourIdx, preview);
    emit toolpathLayersChanged();
}

void CamModule::setAllContoursEnabled(bool enabled)
{
    bool changed = false;
    for (int index = 0; index < m_toolpath.contourCount(); ++index) {
        LaserContour& contour = m_toolpath.contour(index);
        if (contour.enabled == enabled)
            continue;
        contour.enabled = enabled;
        changed = true;
    }

    if (!changed)
        return;

    setCamContoursVisible(m_toolpathRenderer->isVisible(), false);
    lcnc::view::ToolpathRenderer::LeadInPreview preview{
        m_previewLeadInContour, m_previewLeadInPoint, m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refresh(workspaceGuiDocument(), m_toolpath, kinematics(), preview);
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
    if (m_toolpathRenderer->isVisible())
        refreshToolpathDisplay();
    emit toolpathLayersChanged();
}

void CamModule::reorderContours(const QList<int>& order)
{
    if (!m_camData || !m_camData->reorderContours(order))
        return;

    syncCamDocumentContours();
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
        m_previewLeadInContour, m_previewLeadInPoint, m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refresh(workspaceGuiDocument(), m_toolpath, kinematics(), preview);
    emit toolpathLayersChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    return true;
}

void CamModule::recalcToolpath()
{
    LcncDocument* machDoc = machineDocument();
    MachineKinematics* kin = machDoc ? machDoc->machineKinematics() : nullptr;
    for (int i = 0; i < m_toolpath.contourCount(); ++i) {
        LaserContour& contour = m_toolpath.contour(i);
        const TopoDS_Shape sourceShape = contour.sourceShape.IsNull()
            ? m_workpieceShape
            : contour.sourceShape;
        if (sourceShape.IsNull())
            continue;

        if (m_useFaceClassification) {
            FaceClassification classification =
                FaceClassifier::classifyFaces(sourceShape, m_smoothAngle);

            std::vector<TopoDS_Face> outerFaces;
            std::vector<TopoDS_Face> crossFaces;
            if (classification.outerGroup())
                outerFaces = classification.outerGroup()->faces;
            for (const auto* group : classification.crossSectionGroups()) {
                if (!group)
                    continue;
                for (const auto& face : group->faces)
                    crossFaces.push_back(face);
            }

            if (!outerFaces.empty() && !crossFaces.empty())
                LaserToolpathBuilder::discretizeContourWithClassification(
                    contour, outerFaces, crossFaces, m_deflection);
            else
                LaserToolpathBuilder::discretizeContour(contour, sourceShape, m_deflection);
        } else {
            LaserToolpathBuilder::discretizeContour(contour, sourceShape, m_deflection);
        }

        LaserToolpathBuilder::computeMachineCoordinates(
            contour, kin, gp_Trsf(), nullptr);
    }

    refreshToolpathDisplay();
    m_camData->ensureContourIds();
    syncCamDocumentContours();
}

void CamModule::setToolpathVisible(bool visible)
{
    if (m_toolpathRenderer->isVisible() == visible) {
        setCamContoursVisible(visible);
        return;
    }
    m_toolpathRenderer->setVisible(workspaceGuiDocument(), visible);
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
    m_smoothAngle = deg;
}

bool CamModule::useFaceClassification() const
{
    return m_useFaceClassification;
}

void CamModule::setUseFaceClassification(bool on)
{
    m_useFaceClassification = on;
}

void CamModule::eraseAxisGuideDisplay()
{
    m_guideRenderer->erase(workspaceGuiDocument());
}

void CamModule::displayAxisGuides()
{
    m_guideRenderer->refresh(workspaceGuiDocument(), kinematics(), cutterHeadWorldPosition());
}

void CamModule::updateAxisGuideTransforms()
{
    m_guideRenderer->updateTransforms(workspaceGuiDocument(), kinematics(), cutterHeadWorldPosition());
}

void CamModule::refreshToolpathDisplay()
{
    lcnc::view::ToolpathRenderer::LeadInPreview preview{
        m_previewLeadInContour, m_previewLeadInPoint, m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refresh(workspaceGuiDocument(), m_toolpath, kinematics(), preview);
}

void CamModule::eraseToolpathDisplay()
{
    m_toolpathRenderer->erase(workspaceGuiDocument());
}

void CamModule::resetProjectViewState()
{
    m_machineModelVisible = false;
    m_machineVisibilityInitialized = false;
    m_visibleMachineEntries.clear();
    m_lastCamSelectionContourIds.clear();
    if (m_guideRenderer)
        m_guideRenderer->erase(nullptr);
    emit machineVisibilityChanged();
}

void CamModule::clearToolpathViewState(bool emitSignals)
{
    eraseToolpathDisplay();
    // CAM AIS 按 ContourId 管理，eraseAllContours 删 GuiDocument 注册项。
    if (GuiDocument* gd = workspaceGuiDocument())
        gd->eraseAllContours();

    m_workpieceShape.Nullify();
    m_previewLeadInContour = -1;
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
    GuiDocument* gd = workspaceGuiDocument();
    if (!gd)
        return m_camContourAisCache;

    // Phase C：AIS 直接按 ContourId 查找，不再读 CAM XCAF 标签。
    for (int i = 0; i < m_toolpath.contourCount(); ++i) {
        const LaserContour& contour = m_toolpath.contour(i);
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

    if (auto* gd = workspaceGuiDocument()) {
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

void CamModule::setMachineModelVisible(bool visible)
{
    if (m_machineModelVisible == visible)
        return;

    m_machineModelVisible = visible;
    if (visible) {
        refreshMachineDisplay();
        return;
    }

    if (auto* gd = workspaceGuiDocument()) {
        gd->eraseDomain(lcnc::ProjectDomain::Machine);
        if (gd->hasView())
            gd->view()->Redraw();
    }
    emit machineVisibilityChanged();
}

void CamModule::setRotaryAxisGuidesVisible(bool visible)
{
    displayAxisGuides();
    m_guideRenderer->setRotaryAxisVisible(workspaceGuiDocument(), visible);
}

bool CamModule::rotaryAxisGuidesVisible() const
{
    return m_guideRenderer->rotaryAxisVisible();
}

void CamModule::setCutterHeadGuideVisible(bool visible)
{
    displayAxisGuides();
    m_guideRenderer->setCutterHeadVisible(workspaceGuiDocument(), visible);
}

bool CamModule::cutterHeadGuideVisible() const
{
    return m_guideRenderer->cutterHeadVisible();
}

void CamModule::setSelectedEntries(const QStringList& entries)
{
    GuiDocument* gd = workspaceGuiDocument();
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
    if (auto* gd = workspaceGuiDocument())
        return gd->selectedEntries(machineDocumentId());

    return {};
}

void CamModule::syncSelectionFromView()
{
    GuiDocument* gd = workspaceGuiDocument();
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
            if (idx < 0 || idx >= m_toolpath.contourCount()) continue;
            const std::uint64_t cid = m_toolpath.contour(idx).contourId;
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
    if (auto* gd = workspaceGuiDocument()) {
        gd->updateMachineWorkspaceTransforms(machineDocument(), workpieceDocument());
        if (m_toolpathRenderer && m_toolpathRenderer->isVisible()) {
            applyCamContourTransforms();
            m_toolpathRenderer->updateTransforms(gd, m_toolpath, kinematics());
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
    if (auto* gd = workspaceGuiDocument()) {
        gd->updateMachineWorkspaceTransforms(machineDocument(), workpieceDocument());
        if (m_toolpathRenderer && m_toolpathRenderer->isVisible()) {
            applyCamContourTransforms();
            m_toolpathRenderer->updateTransforms(gd, m_toolpath, kinematics());
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
    GuiDocument* gd = workspaceGuiDocument();
    if (!gd)
        return;

    // Phase C：直接迭代 m_toolpath.contours() 按 ContourId 取 AIS。
    for (int index = 0; index < m_toolpath.contourCount(); ++index) {
        const LaserContour& contour = m_toolpath.contour(index);
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
    GuiDocument* gd = workspaceGuiDocument();
    if (!gd || contourIndex < 0 || contourIndex >= m_toolpath.contourCount())
        return;

    const std::uint64_t contourId = m_toolpath.contour(contourIndex).contourId;
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
    GuiDocument* gd = workspaceGuiDocument();
    MachineKinematics* kin = kinematics();
    if (!gd || !kin)
        return;

    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull())
        return;

    for (int index = 0; index < m_toolpath.contourCount(); ++index) {
        const LaserContour& contour = m_toolpath.contour(index);
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
    GuiDocument* gd = workspaceGuiDocument();
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
    if (auto* gd = workspaceGuiDocument()) {
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

void CamModule::syncCamDocumentContours()
{
    // Phase C：AIS 现在按 ContourId 寻址，不再走 CAM 域的 XCAF 镜像。
    // 函数名沿用旧名以减少调用方扩散修改；内部仅做 contour body 的 Display 同步。
    GuiDocument* gd = workspaceGuiDocument();
    if (!gd)
        return;

    // 把"当前 toolpath 中存在的 contourId 集"与"GuiDocument 已注册的 contourId 集"对齐：
    //   1. 不存在的逐条 eraseContour
    //   2. 存在但 AIS 缺失的逐条 displayContourBody
    //   3. AIS 已存在的保留（避免反复创建）
    QSet<std::uint64_t> alive;
    for (int index = 0; index < m_toolpath.contourCount(); ++index) {
        const LaserContour& contour = m_toolpath.contour(index);
        if (contour.wire.IsNull() || contour.contourId == 0)
            continue;
        alive.insert(contour.contourId);
    }

    // 删除已经不属于当前 toolpath 的 AIS（按"现有显示 - alive"做差集需要 GuiDocument 帮助）。
    // 简化策略：先全擦再重画 —— 复杂度 O(N)，且 Phase C 阶段 toolpath 规模通常 <1k；
    // Phase F 可优化为差分增量。
    gd->eraseAllContours();

    for (int index = 0; index < m_toolpath.contourCount(); ++index) {
        const LaserContour& contour = m_toolpath.contour(index);
        if (contour.wire.IsNull() || contour.contourId == 0)
            continue;
        const QString name = contour.name.trimmed().isEmpty()
            ? tr("轮廓 %1").arg(index + 1)
            : contour.name;
        gd->displayContourBody(contour.contourId, contour.wire, name);
    }

    applyCamContourTransforms();
    applyToolpathLayerColors(false);
    applyCamContourVisibility();

    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
}

void CamModule::applyToolpathLayerColors(bool updateView)
{
    GuiDocument* gd = workspaceGuiDocument();
    if (!gd || !gd->scene())
        return;

    for (int index = 0; index < m_toolpath.contourCount(); ++index) {
        const LaserContour& contour = m_toolpath.contour(index);
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
    for (int i = 0; i < m_toolpath.contourCount(); ++i) {
        LaserContour& c = m_toolpath.contour(i);
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
    for (int i = 0; i < m_toolpath.contourCount(); ++i) {
        LaserContour& c = m_toolpath.contour(i);
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
    gp.leadInLength         = m_toolpath.globalLeadInLength();
    gp.normalAngle          = m_toolpath.globalNormalAngle();
    gp.deflection           = m_deflection;
    gp.smoothAngle          = m_smoothAngle;
    gp.useFaceClassification = m_useFaceClassification;
    if (m_toolpathRenderer)
        gp.normalSampleStep = m_toolpathRenderer->normalSampleStep();
}

void CamModule::applyGenerationParamsFromCamData()
{
    // 读档后把工程级生成参数应用到运行时（直接赋值，不触发重算以免覆盖已加载的
    // 每轮廓引刀线/采样点）。
    if (!m_camData)
        return;
    const auto& gp = m_camData->generationParams();
    m_toolpath.setGlobalLeadInLength(gp.leadInLength);
    m_toolpath.setGlobalNormalAngle(gp.normalAngle);
    m_deflection            = gp.deflection;
    m_smoothAngle           = gp.smoothAngle;
    m_useFaceClassification = gp.useFaceClassification;
    if (m_toolpathRenderer)
        m_toolpathRenderer->setNormalSampleStep(gp.normalSampleStep);
}

void CamModule::onCamDataLoaded()
{
    // core 已把刀路灌入 CamDataManager；此处先恢复工程级生成参数与轮廓 wire，
    // 再把数据映射到渲染层。
    applyGenerationParamsFromCamData();
    if (!m_camData || !m_camData->hasToolpath()) {
        clearToolpathViewState(/*emitSignals=*/true);
        LCNC_INFO(lcnc::LogCode::Generic,
                  "cam.toolpath: project has no cached CAM data; cleared view");
        return;
    }

    relinkContourGeometryFromDocument();
    syncCamDocumentContours();
    refreshToolpathDisplay();
    emit toolpathGenerated();
    emit toolpathLayersChanged();
    refreshTravelPath();

    LCNC_INFO(lcnc::LogCode::Generic,
              "cam.toolpath: refreshed view for {} layers, {} contours",
              m_toolpath.layers().size(),
              m_toolpath.contourCount());
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
        m_travelPathRenderer->erase(workspaceGuiDocument());
        if (auto* gd = workspaceGuiDocument()) {
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
    GuiDocument* gd = workspaceGuiDocument();
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
    byId.reserve(m_toolpath.contourCount());
    for (const LaserContour& contour : m_toolpath.contours()) {
        if (contour.contourId != 0)
            byId.insert(contour.contourId, &contour);
    }

    QVector<lcnc::view::TravelPathRenderer::Segment> segments;
    segments.reserve(orderedIds.size());
    const double leadInLen = m_toolpath.globalLeadInLength();
    const double leadInNormalAng = m_toolpath.globalNormalAngle();
    for (auto id : orderedIds) {
        auto it = byId.find(id);
        if (it == byId.end()) continue;
        const LaserContour* c = it.value();
        if (!c || c->points.empty())
            continue;

        const gp_Pnt cutStartLocal = c->points.front().position;
        const gp_Pnt endLocal = c->points.back().position;
        gp_Pnt startLocal = cutStartLocal;
        if (c->leadIn.valid) {
            bool ok = false;
            const gp_Pnt leadStart = LaserToolpathBuilder::computeLeadInStartPoint(
                *c, leadInLen, leadInNormalAng, &ok);
            if (ok)
                startLocal = leadStart;
        }

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

