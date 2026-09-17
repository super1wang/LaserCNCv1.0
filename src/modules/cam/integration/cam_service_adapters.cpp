#include "modules/cam/integration/cam_service_adapters.h"

#include "core/document/lcnc_document.h"
#include "core/document/xcaf_utils.h"
#include "core/kernel/i_kernel.h"
#include "core/kernel/kernel.h"
#include "core/kernel/service_registry.h"
#include "core/task/task_manager.h"
#include "modules/cam/toolpath/toolpath_solve_service.h"
#include "modules/cam/toolpath/toolpath_generation_service.h"
#include "core/logging/logger.h"
#include "core/project/cam/layer_container.h"
#include "core/project/cam/layer_manager.h"
#include "core/project/lcnc_project_manager.h"
#include "modules/cam/cam_module.h"
#include "modules/cam/contracts/i_cam_collision_configuration_provider.h"
#include "modules/cam/contracts/i_cam_collision_safety_domain.h"
#include "modules/cam/contracts/i_cam_contour_sequence_provider.h"
#include "modules/cam/contracts/i_cam_initial_approach_planner.h"
#include "modules/cam/contracts/i_cam_layer_provider.h"
#include "modules/cam/contracts/i_cam_offline_simulation_provider.h"
#include "modules/cam/contracts/i_cam_project_explorer_projection.h"
#include "modules/cam/contracts/i_cam_toolpath_provider.h"
#include "view/gui_document.h"

#include <Graphic3d_Camera.hxx>
#include <NCollection_Sequence.hxx>
#include <QCoreApplication>
#include <QMutex>
#include <QMutexLocker>
#include <QPointer>
#include <QThread>
#include <TDF_Label.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <atomic>
#include <algorithm>
#include <memory>

namespace lcnc::cam {
namespace {

class CamToolpathProviderAdapter final : public QObject, public ICamToolpathProvider {
  public:
    explicit CamToolpathProviderAdapter(CamModule& module)
        : m_module(module), m_configClock(module.config().changeClock()),
          m_tasks(lcnc::Kernel::current().taskManager()) {
        m_machineLoading = module.isMachineLoadPending();
        QObject::connect(&module, &CamModule::machineLoadPendingChanged, this, [this](bool pending) {
            QMutexLocker lock(&m_cacheMutex);
            m_machineLoading = pending;
            if (pending) m_currentKey.clear();
        });
        const auto refresh = [this] { refreshCache(); };
        if (auto* machine = lcnc::Kernel::current().service<lcnc::MachineConfigurationService>())
            QObject::connect(machine, &lcnc::MachineConfigurationService::machineConfigurationChanged, this, refresh);
        QObject::connect(lcnc::Kernel::current().projectManager(),
            &lcnc::LcncProjectManager::activeWorkspaceChanged, this, refresh);
        QObject::connect(lcnc::Kernel::current().projectManager(),
            &lcnc::LcncProjectManager::domainDataChanged, this, refresh);
        QObject::connect(&module, &CamModule::toolpathGenerated, this, refresh);
        QObject::connect(&module, &CamModule::toolpathCleared, this, refresh);
        QObject::connect(&module, &CamModule::toolpathLayersChanged, this, refresh);
        QObject::connect(&module, &CamModule::activeContourParametersChanged, this, refresh);
        QObject::connect(&module, &CamModule::machiningModeChanged, this, refresh);
        QObject::connect(&module, &CamModule::workpieceSetupTransformChanged, this, refresh);
        QObject::connect(&module, &CamModule::collisionConfigurationChanged, this, refresh);
        QObject::connect(&module, &CamModule::machineSafetyPackageChanged, this, refresh);
        QObject::connect(&module, &CamModule::machineWorkspaceChanged, this, refresh);
        QObject::connect(&module, &CamModule::axisAssignmentsChanged, this, refresh);
        QObject::connect(&module, &CamModule::workpieceMounted, this, refresh);
        QObject::connect(&module, &CamModule::workpieceUnmounted, this, refresh);
        QObject::connect(&module, &CamModule::pipelineStageChanged, this, refresh);
        QObject::connect(&module, &CamModule::contourOrderTravelPlanRebuilt, this,
            [this](const QVector<std::uint64_t>& order) { refreshPlannedCacheForOrder(order); });
        if (m_tasks) QObject::connect(m_tasks, &TaskManager::taskFinished, this,
            [this](TaskId id, bool success) {
                if (id != m_taskId) return;
                m_taskId = kInvalidTaskId;
                auto result = std::move(m_pending);
                const auto key = m_pendingKey;
                m_pendingKey.clear();
                if (!success || !result || !result->ok || result->cancelled->load()
                    || !m_module.adoptMotionPublication(result->snapshot, key)) return;
                QVector<std::uint64_t> order;
                for (const auto& contour : result->snapshot.contours) order.append(contour.contourId);
                auto published = m_module.exportToolpathSnapshotForOrder(order, true);
                QMutexLocker lock(&m_cacheMutex);
                m_plannedSnapshot = std::move(published);
                m_plannedKey = key;
                m_plannedConfigRevision = result->configurationRevision;
            });
        QObject::connect(&module, &QObject::destroyed, this, [this] {
            m_ownerAlive = false;
            if (m_pending) m_pending->cancelled->store(true);
            if (m_tasks) QObject::disconnect(m_tasks, nullptr, this, nullptr);
            QMutexLocker lock(&m_cacheMutex);
            m_currentKey.clear();
        });
        refreshCache();
    }

    ~CamToolpathProviderAdapter() override {
        // Workers only own frozen input/result, never this adapter or the module.
        if (m_pending) m_pending->cancelled->store(true);
        if (m_tasks && m_taskId != kInvalidTaskId) m_tasks->requestAbort(m_taskId);
    }

    bool hasToolpath() const override {
        QMutexLocker lock(&m_cacheMutex);
        return m_baseSnapshot.hasEnabledContours();
    }
    std::uint64_t toolpathRevision() const override {
        QMutexLocker lock(&m_cacheMutex);
        return m_baseSnapshot.revision;
    }
    ToolpathExportSnapshot exportToolpathCatalogSnapshot() const override {
        QMutexLocker lock(&m_cacheMutex);
        return m_baseSnapshot;
    }
    ToolpathExportSnapshot exportCommittedExecutionSnapshot() const override {
        QMutexLocker lock(&m_cacheMutex);
        if (!m_machineLoading && !m_plannedKey.isEmpty()
            && m_plannedKey == m_currentKey
            && m_plannedConfigRevision == m_configClock->load()
            && m_plannedSnapshot.revision == m_baseSnapshot.revision) return m_plannedSnapshot;
        ToolpathExportSnapshot blocked;
        blocked.revision = m_baseSnapshot.revision;
        blocked.description = QStringLiteral("Motion publication is stale or not ready");
        blocked.motionPlan.failureReason = blocked.description;
        blocked.travelPlan.failureReason = blocked.description;
        blocked.travelPlan.stale = true;
        return blocked; // Catalog points are never an execution fallback.
    }

  private:
    struct Pending {
        ToolpathExportSnapshot snapshot;
        std::shared_ptr<std::atomic_bool> cancelled{std::make_shared<std::atomic_bool>(false)};
        std::uint64_t configurationRevision{0};
        bool ok{false};
    };
    void refreshPlannedCacheForOrder(const QVector<std::uint64_t>& order) {
        if (!m_ownerAlive) return;
        if (QThread::currentThread() != m_module.thread()) return;
        auto key = m_module.currentMotionPublicationKey();
        auto base = m_module.exportToolpathBaseSnapshot();
        {
            QMutexLocker lock(&m_cacheMutex);
            m_currentKey = key;
            m_baseSnapshot = std::move(base);
        }
        if (key.isEmpty()) {
            if (m_pending) m_pending->cancelled->store(true);
            return;
        }
        if (m_pending && m_pendingKey == key && !m_pending->cancelled->load()) return;
        auto snapshot = m_module.exportToolpathSnapshotForOrder(order, true);
        if (!finalMotionPlanIdentityIsCurrent(snapshot.motionPlan)) return;
        key = m_module.currentMotionPublicationKey();
        if (key.isEmpty()) return;
        { QMutexLocker lock(&m_cacheMutex); m_currentKey = key; }
        const bool compiled = std::all_of(snapshot.motionPlan.blocks.cbegin(), snapshot.motionPlan.blocks.cend(),
            [](const auto& block) { return block.optimizationState != MotionOptimizationState::Raw; });
        if (compiled) {
            QMutexLocker lock(&m_cacheMutex);
            m_plannedSnapshot = std::move(snapshot);
            m_plannedKey = key;
            m_plannedConfigRevision = m_configClock->load();
            return;
        }
        const auto input = m_module.motionPublicationInput();
        if (!input || !m_tasks) return;
        if (m_pending) m_pending->cancelled->store(true);
        if (m_taskId != kInvalidTaskId) m_tasks->requestAbort(m_taskId);
        auto result = std::make_shared<Pending>();
        result->snapshot = std::move(snapshot);
        result->configurationRevision = input->configurationRevision;
        m_pending = result;
        m_pendingKey = key;
        const auto configClock = m_configClock;
        TaskSpec spec;
        spec.label = QStringLiteral("Compile CAM motion");
        spec.scope = QStringLiteral("cam.motion.compile");
        m_taskId = m_tasks->run(spec, [input, result, configClock](TaskProgress* progress) {
            const auto cancelled = [&] {
                return progress->isAbortRequested() || result->cancelled->load()
                    || configClock->load() != result->configurationRevision;
            };
            QString error;
            result->ok = !cancelled() && ToolpathSolveService::optimizeMotionSnapshot(
                &result->snapshot, *input, &error, cancelled) && !cancelled();
        });
    }

    void refreshCache() {
        if (!m_ownerAlive) return;
        if (QThread::currentThread() != m_module.thread()) return;
        const auto order = m_module.contourSequenceSnapshot().orderedContourIds;
        refreshPlannedCacheForOrder(QVector<std::uint64_t>(order.cbegin(), order.cend()));
    }

    CamModule& m_module;
    std::shared_ptr<const std::atomic_uint64_t> m_configClock;
    QPointer<TaskManager> m_tasks;
    TaskId m_taskId{kInvalidTaskId};
    std::shared_ptr<Pending> m_pending;
    QByteArray m_pendingKey;
    mutable QMutex m_cacheMutex;
    ToolpathExportSnapshot m_baseSnapshot, m_plannedSnapshot;
    QByteArray m_currentKey, m_plannedKey;
    std::uint64_t m_plannedConfigRevision{0};
    bool m_machineLoading{false};
    bool m_ownerAlive{true};
};

class CamOfflineSimulationProviderAdapter final : public QObject,
                                                  public ICamOfflineSimulationProvider {
  public:
    CamOfflineSimulationProviderAdapter(CamModule& module,
                                        std::shared_ptr<ICamToolpathProvider> toolpathProvider)
        : m_module(module), m_toolpathProvider(std::move(toolpathProvider)) {
        const auto changed = [this] { m_revision.fetch_add(1, std::memory_order_relaxed); };
        QObject::connect(&m_module, &CamModule::machineWorkspaceChanged, this, changed);
        QObject::connect(&m_module, &CamModule::axisAssignmentsChanged, this, changed);
        QObject::connect(&m_module, &CamModule::toolpathGenerated, this, changed);
        QObject::connect(&m_module, &CamModule::toolpathCleared, this, changed);
        QObject::connect(&m_module, &CamModule::toolpathLayersChanged, this, changed);
        QObject::connect(&m_module, &CamModule::cutterCollisionConfigurationChanged, this, changed);
        QObject::connect(&m_module, &CamModule::collisionConfigurationChanged, this, changed);
        QObject::connect(&m_module, &CamModule::contourOrderTravelPlanRebuilt, this,
                         [changed](const QVector<std::uint64_t>&) { changed(); });
    }

    std::uint64_t revision() const override {
        return m_revision.load(std::memory_order_relaxed);
    }

    OfflineSimulationSnapshot captureOfflineSimulationSnapshot() const override {
        OfflineSimulationSnapshot snapshot;
        snapshot.revision = revision();
        const auto* kinematics = m_module.kinematics();
        if (!m_toolpathProvider || !m_toolpathProvider->hasToolpath() || !kinematics) {
            snapshot.error = QCoreApplication::translate(
                "CamModule", "A solved CAM toolpath and machine kinematics are required");
            return snapshot;
        }

        snapshot.execution = m_toolpathProvider->exportCommittedExecutionSnapshot();
        snapshot.toolpath = m_module.toolpath();
        snapshot.axes = kinematics->axes();
        snapshot.kinematicsType = kinematics->configType();
        snapshot.workpieceSetup = kinematics->workpieceSetupTransform();
        snapshot.shapeAssignments = kinematics->shapeAssignments();
        snapshot.workpieceMounts = kinematics->wpcMounts();
        snapshot.collision = m_module.collisionConfiguration();
        snapshot.cutterHeadModelPosition = m_module.cutterHeadModelPosition();
        snapshot.showToolpath = m_module.isToolpathVisible();
        snapshot.showTravel = m_module.isTravelPathVisible();
        snapshot.showNormals = m_module.showNormals();
        snapshot.showMachine = m_module.isMachineModelVisible();
        const QStringList visible = m_module.visibleMachineEntries();
        snapshot.visibleMachineEntries = QSet<QString>(visible.cbegin(), visible.cend());
        snapshot.showRotaryGuides = m_module.rotaryAxisGuidesVisible();
        snapshot.showCutterHeadGuide = m_module.cutterHeadGuideVisible();
        snapshot.normalSampleStep = m_module.normalSampleStep();
        snapshot.collisionClearanceMm = m_module.config().cutterCollisionClearanceMm();
        if (GuiDocument* sourceView = m_module.activeGuiDocument();
            sourceView && !sourceView->view().IsNull()) {
            snapshot.camera = new Graphic3d_Camera();
            snapshot.camera->Copy(sourceView->view()->Camera());
        }
        QString proxyError;
        snapshot.cutterDisplayProxy = m_module.cutterDisplayProxyShape(&proxyError);
        if (snapshot.cutterDisplayProxy.IsNull() && !proxyError.isEmpty()) {
            LCNC_WARN(lcnc::LogCode::Generic, "Offline simulation snapshot has no cutter display proxy: {}",
                      proxyError.toStdString());
        }

        const auto appendBodies = [&snapshot](LcncDocument* document, LcncDocument::EntityKind kind,
                                              bool workpiece) {
            if (!document || !document->shapeTool())
                return;
            const NCollection_Sequence<TDF_Label> labels = document->entityLabels(kind);
            for (int index = 1; index <= labels.Length(); ++index) {
                const TDF_Label& label = labels.Value(index);
                const TopoDS_Shape shape = document->shapeTool()->GetShape(label);
                if (!shape.IsNull()) {
                    snapshot.bodies.append({XcafUtils::entry(label), shape, workpiece});
                }
            }
        };
        appendBodies(m_module.machineDocument(), LcncDocument::EntityKind::Machine, false);
        if (auto* project = lcnc::Kernel::current().projectManager()) {
            appendBodies(project->workpieceDocument(), LcncDocument::EntityKind::Workpiece, true);
        }
        return snapshot;
    }

  private:
    CamModule& m_module;
    std::shared_ptr<ICamToolpathProvider> m_toolpathProvider;
    std::atomic_uint64_t m_revision{1};
};

class CamLayerProviderAdapter final : public QObject, public ICamLayerProvider {
  public:
    explicit CamLayerProviderAdapter(CamModule& module) : m_module(module) {
        QObject::connect(&m_module, &CamModule::toolpathGenerated, this,
                         [this] { refreshCache(); });
        QObject::connect(&m_module, &CamModule::toolpathCleared, this, [this] { refreshCache(); });
        QObject::connect(&m_module, &CamModule::toolpathLayersChanged, this,
                         [this] { refreshCache(); });
        QObject::connect(&m_module, &CamModule::autoSortAxisChanged, this,
                         [this](AutoSortAxis) { refreshCache(); });
        refreshCache();
    }

    QVector<LayerSnapshot> layers() const override {
        QMutexLocker lock(&m_cacheMutex);
        return m_layers;
    }

    QVector<ContourId> manualContourOrder() const override {
        QMutexLocker lock(&m_cacheMutex);
        return m_manualContourOrder;
    }

    CuttingPlanSortStrategy sortStrategy() const override {
        QMutexLocker lock(&m_cacheMutex);
        return m_sortStrategy;
    }

    AutoSortAxis lastAutoSortAxis() const override {
        QMutexLocker lock(&m_cacheMutex);
        return m_lastAutoSortAxis;
    }

    std::uint64_t revision() const override {
        QMutexLocker lock(&m_cacheMutex);
        return m_revision;
    }

    QObject* notifier() const override {
        return layerManager();
    }

  private:
    void bindLayerManager() {
        auto* const manager = layerManager();
        if (m_layerManager == manager)
            return;
        if (m_layerManager)
            QObject::disconnect(m_layerManager, nullptr, this, nullptr);
        m_layerManager = manager;
        if (!m_layerManager)
            return;

        auto refresh = [this] { refreshCache(); };
        QObject::connect(m_layerManager, &LayerManager::layersReset, this, refresh);
        QObject::connect(m_layerManager, &LayerManager::layerAdded, this,
                         [this](std::uint64_t) { refreshCache(); });
        QObject::connect(m_layerManager, &LayerManager::layerRemoved, this,
                         [this](std::uint64_t) { refreshCache(); });
        QObject::connect(m_layerManager, &LayerManager::layersReordered, this, refresh);
        QObject::connect(m_layerManager, &LayerManager::layerPropertyChanged, this,
                         [this](std::uint64_t, LayerProperty) { refreshCache(); });
        QObject::connect(m_layerManager, &LayerManager::contourMembershipChanged, this, refresh);
        QObject::connect(m_layerManager, &LayerManager::manualContourOrderChanged, this, refresh);
        QObject::connect(m_layerManager, &LayerManager::sortStrategyChanged, this,
                         [this](CuttingPlanSortStrategy) { refreshCache(); });
    }

    void refreshCache() {
        if (QThread::currentThread() != m_module.thread())
            return;
        bindLayerManager();
        QVector<LayerSnapshot> layers;
        const auto* container = layerContainer();
        const auto* toolpath = container ? container->toolpath() : nullptr;
        if (toolpath) {
            layers.reserve(static_cast<int>(toolpath->layers().size()));
            for (const ToolpathLayer& layer : toolpath->layers()) {
                LayerSnapshot snapshot;
                snapshot.layerId = layer.layerId;
                snapshot.name = layer.name;
                snapshot.toolName = layer.toolName;
                snapshot.compensationIndex = layer.compensationIndex;
                snapshot.enabled = layer.enabled;
                snapshot.includedContours = layer.includedContours;
                snapshot.contourIds.reserve(static_cast<int>(layer.contourIds.size()));
                for (const std::uint64_t contourId : layer.contourIds)
                    snapshot.contourIds.push_back(static_cast<ContourId>(contourId));
                layers.append(std::move(snapshot));
            }
        }
        QMutexLocker lock(&m_cacheMutex);
        m_layers = std::move(layers);
        m_manualContourOrder = container ? container->manualContourOrder() : QVector<ContourId>{};
        m_sortStrategy =
            container ? container->sortStrategy() : CuttingPlanSortStrategy::LayerThenContour;
        m_lastAutoSortAxis = m_module.lastAutoContourSortAxis();
        ++m_revision;
    }

    LayerContainer* layerContainer() const {
        auto* data = m_module.camData();
        return data ? &data->layerContainer() : nullptr;
    }

    LayerManager* layerManager() const {
        auto* data = m_module.camData();
        return data ? data->layerManager() : nullptr;
    }

    CamModule& m_module;
    QPointer<LayerManager> m_layerManager;
    mutable QMutex m_cacheMutex;
    QVector<LayerSnapshot> m_layers;
    QVector<ContourId> m_manualContourOrder;
    CuttingPlanSortStrategy m_sortStrategy{CuttingPlanSortStrategy::LayerThenContour};
    AutoSortAxis m_lastAutoSortAxis{AutoSortAxis::XPos};
    std::uint64_t m_revision{1};
};

class CamContourSequenceProviderAdapter final : public ICamContourSequenceProvider {
  public:
    explicit CamContourSequenceProviderAdapter(CamModule& module) : m_module(module) {}
    ContourSequenceSnapshot contourSequence() const override {
        return m_module.contourSequenceSnapshot();
    }

  private:
    CamModule& m_module;
};

class CamCollisionConfigurationProviderAdapter final : public ICamCollisionConfigurationProvider {
  public:
    explicit CamCollisionConfigurationProviderAdapter(CamModule& module) : m_module(module) {}
    CollisionConfigurationSnapshot collisionConfiguration() const override {
        return m_module.collisionConfiguration();
    }
    bool setCollisionDetectionEnabled(bool enabled,
                                      QString* errorMessage = nullptr) override {
        return m_module.setCollisionDetectionEnabled(enabled, errorMessage);
    }
    void setCollisionSources(const QSet<QString>& active, const QSet<QString>& passive) override {
        m_module.setCollisionSources(active, passive);
    }

  private:
    CamModule& m_module;
};

class CamInitialApproachPlannerAdapter final : public ICamInitialApproachPlanner {
  public:
    explicit CamInitialApproachPlannerAdapter(CamModule& module) : m_module(module) {}
    InitialApproachSnapshot planInitialApproach(const InitialApproachRequest& request,
                                                std::atomic_bool* cancelRequested) const override {
        return m_module.planInitialApproach(request, cancelRequested);
    }

  private:
    CamModule& m_module;
};

class CamCollisionSafetyDomainAdapter final : public ICamCollisionSafetyDomain {
  public:
    explicit CamCollisionSafetyDomainAdapter(CamModule& module) : m_module(module) {}
    CollisionSafetyDomainSnapshot collisionSafetyDomain() const override {
        return m_module.collisionSafetyDomain();
    }
    CollisionValidationSnapshot
    validateCollisionPath(const CollisionSafetyPathRequest& request,
                          std::atomic_bool* cancelRequested) const override {
        return m_module.validateCollisionPath(request, cancelRequested);
    }
    CamMotionPermit requestMotionPermit(
        const CamMotionPermitRequest& request) const override {
        return m_module.requestMotionPermit(request);
    }

  private:
    CamModule& m_module;
};

class CamProjectExplorerProjectionAdapter final : public ICamProjectExplorerProjection {
  public:
    explicit CamProjectExplorerProjectionAdapter(CamModule& module) : m_module(module) {}
    ProjectExplorerSnapshot projectExplorerSnapshot() const override {
        return m_module.projectExplorerSnapshot();
    }

  private:
    CamModule& m_module;
};

} // namespace

void registerCamServiceAdapters(lcnc::IKernel& kernel, CamModule& module) {
    kernel.services().registerService<ICamCollisionConfigurationProvider>(
        std::make_shared<CamCollisionConfigurationProviderAdapter>(module));
    kernel.services().registerService<ICamCollisionSafetyDomain>(
        std::make_shared<CamCollisionSafetyDomainAdapter>(module));
    kernel.services().registerService<ICamInitialApproachPlanner>(
        std::make_shared<CamInitialApproachPlannerAdapter>(module));
    kernel.services().registerService<ICamProjectExplorerProjection>(
        std::make_shared<CamProjectExplorerProjectionAdapter>(module));
    auto toolpathProvider = std::make_shared<CamToolpathProviderAdapter>(module);
    kernel.services().registerService<ICamToolpathProvider>(toolpathProvider);
    kernel.services().registerService<ICamOfflineSimulationProvider>(
        std::make_shared<CamOfflineSimulationProviderAdapter>(module, toolpathProvider));
    kernel.services().registerService<ICamLayerProvider>(
        std::make_shared<CamLayerProviderAdapter>(module));
    kernel.services().registerService<ICamContourSequenceProvider>(
        std::make_shared<CamContourSequenceProviderAdapter>(module));
}

} // namespace lcnc::cam
