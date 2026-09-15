#include "modules/cam/integration/cam_service_adapters.h"

#include "core/document/lcnc_document.h"
#include "core/document/xcaf_utils.h"
#include "core/kernel/i_kernel.h"
#include "core/kernel/kernel.h"
#include "core/kernel/service_registry.h"
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
#include <memory>

namespace lcnc::cam {
namespace {

class CamToolpathProviderAdapter final : public QObject, public ICamToolpathProvider {
  public:
    explicit CamToolpathProviderAdapter(CamModule& module) : m_module(module) {
        QObject::connect(&m_module, &CamModule::toolpathGenerated, this,
                         [this] { refreshCache(); });
        QObject::connect(&m_module, &CamModule::toolpathCleared, this, [this] { refreshCache(); });
        QObject::connect(&m_module, &CamModule::toolpathLayersChanged, this,
                         [this] { refreshCache(); });
        QObject::connect(&m_module, &CamModule::activeContourParametersChanged, this,
                         [this] { refreshCache(); });
        QObject::connect(&m_module, &CamModule::machiningModeChanged, this,
                         [this](lcnc::MachiningMode) { refreshCache(true); });
        QObject::connect(&m_module, &CamModule::workpieceSetupTransformChanged, this,
                         [this] { refreshCache(true); });
        QObject::connect(&m_module, &CamModule::collisionConfigurationChanged, this,
                         [this] { refreshCache(true); });
        QObject::connect(&m_module, &CamModule::machineSafetyPackageChanged, this,
                         [this] { refreshCache(true); });
        QObject::connect(&m_module, &CamModule::pipelineStageChanged, this,
                         [this](CamPipelineStage) { refreshCache(); });
        QObject::connect(
            &m_module, &CamModule::contourOrderTravelPlanRebuilt, this,
            [this](const QVector<std::uint64_t>& order) { refreshPlannedCacheForOrder(order); });
        refreshCache();
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
        if (!m_module.isMachineLoadPending())
            return m_baseSnapshot;
        return machineLoadBlockedSnapshot(m_baseSnapshot);
    }

    ToolpathExportSnapshot exportCommittedExecutionSnapshot() const override {
        QMutexLocker lock(&m_cacheMutex);
        if (m_module.isMachineLoadPending())
            return machineLoadBlockedSnapshot(m_baseSnapshot);
        return m_plannedSnapshot.revision == m_baseSnapshot.revision ? m_plannedSnapshot
                                                                     : m_baseSnapshot;
    }

  private:
    static ToolpathExportSnapshot machineLoadBlockedSnapshot(const ToolpathExportSnapshot& base) {
        auto blocked = base;
        blocked.travelPlan = {};
        blocked.travelPlan.mode = TravelPlanningMode::FullEnvironment;
        blocked.travelPlan.failureReason = QCoreApplication::translate(
            "CamModule",
            "The machine model is loading; a verified rapid travel plan is not available yet");
        blocked.travelPlan.stale = false;
        blocked.motionPlan = {};
        blocked.motionPlan.revision = blocked.revision;
        blocked.motionPlan.collision.state = CollisionValidationState::Pending;
        blocked.motionPlan.collision.complete = false;
        blocked.motionPlan.collision.failureReason = blocked.travelPlan.failureReason;
        return blocked;
    }

    void refreshPlannedCacheForOrder(const QVector<std::uint64_t>& orderedContourIds) {
        if (QThread::currentThread() != m_module.thread())
            return;
        auto snapshot = orderedContourIds.isEmpty()
                            ? m_module.exportToolpathBaseSnapshot()
                            : m_module.exportToolpathSnapshotForOrder(orderedContourIds);
        auto baseSnapshot = m_module.exportToolpathBaseSnapshot();
        QMutexLocker lock(&m_cacheMutex);
        const auto samePlannedOrder = [this, &snapshot] {
            if (m_plannedSnapshot.revision != snapshot.revision ||
                m_plannedSnapshot.contours.size() != snapshot.contours.size() ||
                !(m_plannedSnapshot.travelPlan.key == snapshot.travelPlan.key)) {
                return false;
            }
            for (int index = 0; index < snapshot.contours.size(); ++index) {
                if (m_plannedSnapshot.contours.at(index).contourId !=
                    snapshot.contours.at(index).contourId) {
                    return false;
                }
            }
            return true;
        };
        if (samePlannedOrder()) {
            // Verification completes asynchronously after the planned
            // snapshot was first cached. Preserve coordinates solved with the
            // committed rapid path, but refresh every collision proof field;
            // otherwise Process keeps seeing Job Overlay=Building and missing
            // edge certificates after CAM has already published both.
            // 中文翻译：异步校验完成时保留已提交的运动坐标，但必须同步工件缓存就绪状态、
            // 碰撞结果和连续边证书，避免 Process 永久读取构建中的旧快照。
            if (!m_plannedSnapshot.mergeCollisionProofFrom(snapshot))
                m_plannedSnapshot = std::move(snapshot);
        } else {
            m_plannedSnapshot = std::move(snapshot);
        }
        m_baseSnapshot = std::move(baseSnapshot);
    }

    void refreshCache(bool invalidatePlannedPlan = false) {
        if (QThread::currentThread() != m_module.thread())
            return;
        auto snapshot = m_module.exportToolpathBaseSnapshot();
        QMutexLocker lock(&m_cacheMutex);
        if (invalidatePlannedPlan || m_plannedSnapshot.revision != snapshot.revision)
            m_plannedSnapshot = {};
        m_baseSnapshot = std::move(snapshot);
    }

    CamModule& m_module;
    mutable QMutex m_cacheMutex;
    ToolpathExportSnapshot m_baseSnapshot;
    ToolpathExportSnapshot m_plannedSnapshot;
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
