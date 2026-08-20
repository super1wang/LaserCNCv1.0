
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
using lcnc::cam::detail::faceBelongsToSource;
using lcnc::cam::detail::watchTask;
} // namespace

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

std::vector<lcnc::cam::MachiningFacePipelineService::Candidate>
CamModule::selectAutomaticMachiningFaces(const QList<WorkpieceShapeSource>& sources,
                                         ExtractionStrategy strategy,
                                         double smoothAngle)
{
    std::vector<lcnc::cam::MachiningFacePipelineService::Candidate> result;
    if (strategy == ExtractionStrategy::ManualFaceSelection)
        return result;

    if (strategy == ExtractionStrategy::LargestSmoothConnectedSurface) {
        for (const WorkpieceShapeSource& source : sources) {
            if (source.shape.IsNull())
                continue;
            const FaceClassification classification = FaceClassifier::classifyFaces(
                source.shape, smoothAngle);
            if (const auto* outer = classification.outerGroup()) {
                for (const TopoDS_Face& face : outer->faces) {
                    result.push_back({face, source.workpieceEntry,
                                      lcnc::cam::MachiningFaceRole::MachiningSurface});
                }
            }
            for (const FaceGroup* group : classification.crossSectionGroups()) {
                if (!group)
                    continue;
                for (const TopoDS_Face& face : group->faces) {
                    result.push_back({face, source.workpieceEntry,
                                      lcnc::cam::MachiningFaceRole::CrossSection});
                }
            }
        }
        return result;
    }

    struct SourceBounds {
        Bnd_Box bounds;
        double xMin{0.0}; double yMin{0.0}; double zMin{0.0};
        double xMax{0.0}; double yMax{0.0}; double zMax{0.0};
        bool valid{false};
    };
    std::vector<SourceBounds> bounds(static_cast<std::size_t>(sources.size()));
    for (int index = 0; index < sources.size(); ++index) {
        if (sources.at(index).shape.IsNull())
            continue;
        BRepBndLib::Add(sources.at(index).shape, bounds[static_cast<std::size_t>(index)].bounds);
        SourceBounds& value = bounds[static_cast<std::size_t>(index)];
        if (value.bounds.IsVoid())
            continue;
        value.bounds.Get(value.xMin, value.yMin, value.zMin,
                         value.xMax, value.yMax, value.zMax);
        value.valid = true;
    }

    // Components that almost completely overlap in XY are a vertical stack,
    // not several independent top surfaces.  Cull an entirely covered lower
    // component before its local face analysis.  This is a cheap projected
    // visibility pass and prevents every repeated tube layer from contributing
    // its own cap, hole walls and side fragments.
    std::vector<int> retainedSources;
    for (int index = 0; index < sources.size(); ++index) {
        const SourceBounds& current = bounds[static_cast<std::size_t>(index)];
        if (!current.valid)
            continue;
        const double currentWidth = std::max(0.0, current.xMax - current.xMin);
        const double currentHeight = std::max(0.0, current.yMax - current.yMin);
        const double currentArea = currentWidth * currentHeight;
        const double extent = std::max({currentWidth, currentHeight,
                                        current.zMax - current.zMin, 1.0});
        const double zTolerance = std::max(1e-4, extent * 1e-5);
        bool hiddenByHigherComponent = false;
        for (int otherIndex = 0; otherIndex < sources.size(); ++otherIndex) {
            if (otherIndex == index)
                continue;
            const SourceBounds& other = bounds[static_cast<std::size_t>(otherIndex)];
            if (!other.valid || other.zMax <= current.zMax + zTolerance)
                continue;
            const double overlapWidth = std::max(
                0.0, std::min(current.xMax, other.xMax) - std::max(current.xMin, other.xMin));
            const double overlapHeight = std::max(
                0.0, std::min(current.yMax, other.yMax) - std::max(current.yMin, other.yMin));
            const double overlapArea = overlapWidth * overlapHeight;
            if (currentArea > 1e-8 && overlapArea / currentArea >= 0.98) {
                hiddenByHigherComponent = true;
                break;
            }
        }
        if (!hiddenByHigherComponent)
            retainedSources.push_back(index);
    }

    for (int index : retainedSources) {
        const WorkpieceShapeSource& source = sources.at(index);
        for (const TopoDS_Face& face :
             LaserToolpathBuilder::selectTopVisibleFacesFromPositiveZ(source.shape)) {
            result.push_back({face, source.workpieceEntry,
                              lcnc::cam::MachiningFaceRole::MachiningSurface});
        }
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
    return m_machiningFacePipeline ? m_machiningFacePipeline->revision() : 0;
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
    mix(m_machineGeometryRevision);
    for (const QChar ch : activeMachineProfilePath())
        mix(ch.unicode());
    const lcnc::WorkpieceSetupTransform setup = workpieceSetupTransform();
    mixDouble(setup.x); mixDouble(setup.y); mixDouble(setup.z);
    mixDouble(setup.rotationXDeg); mixDouble(setup.rotationYDeg); mixDouble(setup.rotationZDeg);
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
    for (auto it = machine->shapeAssignments().cbegin(); it != machine->shapeAssignments().cend(); ++it) {
        for (const QChar ch : it.key()) mix(ch.unicode());
        for (const QChar ch : it.value()) mix(ch.unicode());
    }
    return hash;
}

void CamModule::invalidateMachineEnvironment()
{
    ++m_machineGeometryRevision;
    m_travelPlanCache.stale = true;
    m_travelCollisionGeometryCache.reset();
    if (m_collisionDomainPreparationTask != kInvalidTaskId) {
        if (auto* tasks = lcnc::Kernel::current().taskManager())
            tasks->requestAbort(m_collisionDomainPreparationTask);
        m_collisionDomainPreparationTask = kInvalidTaskId;
    }
}

QString CamModule::activeMachineProfilePath() const
{
    return m_loadedMachineModelPath.isEmpty()
        ? m_machineModelPath : m_loadedMachineModelPath;
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

    struct Result {
        std::vector<lcnc::cam::MachiningFacePipelineService::Candidate> faces;
        QString error;
        bool ok{false};
    };
    const auto result = std::make_shared<Result>();
    const double smoothAngle = m_smoothAngle;
    // 中文翻译：分离加工面
    TaskSpec spec{tr("Separate processing surface"), QStringLiteral("cam.pipeline"), TaskPriority::Normal, true};
    setProperty("camFaceSeparationRunning", true);
    const TaskId taskId = taskManager->run(spec,
        [sources, strategy, smoothAngle, result](TaskProgress* progress) {
            progress->setRange(0, 1);
            if (progress->isAbortRequested())
                // 中文翻译：加工面分离已取消
                throw std::runtime_error("Machining surface separation canceled");
            result->faces = CamModule::selectAutomaticMachiningFaces(
                sources, strategy, smoothAngle);
            progress->setValue(1);
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

        // The service preserves manual picks made after the worker started and
        // atomically replaces only the automatic portion after source checks.
        if (!m_machiningFacePipeline->replaceAutomaticFaces(result->faces)) {
            // 中文翻译：分离加工面；加工面识别结果为空。
            emit operationFailed(tr("Separate processing surface"), tr("The processing surface identification result is empty."));
            return;
        }
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
    const double leadInLength = toolpathRef().globalLeadInLength();
    const double cuttingOffsetMm = toolpathRef().globalCuttingOffsetMm();
    const double rapidOffsetMm = toolpathRef().globalRapidOffsetMm();
    const bool useLargestSmoothBoundary =
        m_extractionStrategy == static_cast<int>(ExtractionStrategy::LargestSmoothConnectedSurface);
    struct Result { std::vector<LaserContour> contours; QString error; bool ok{false}; };
    const auto result = std::make_shared<Result>();
    // 中文翻译：提取加工轮廓
    TaskSpec spec{tr("Extract machining contours"), QStringLiteral("cam.pipeline"), TaskPriority::Normal, true};
    const TaskId taskId = taskManager->run(spec,
        [sources, faces, smoothAngle, deflection, leadInLength,
         cuttingOffsetMm, rapidOffsetMm,
         useLargestSmoothBoundary, result](TaskProgress* progress) {
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
                    if (entry.workpieceEntry != source.workpieceEntry
                        || !faceBelongsToSource(entry.face, source, sources))
                        continue;
                    switch (entry.role) {
                    case lcnc::cam::MachiningFaceRole::MachiningSurface: machiningFaces.push_back(entry.face); break;
                    case lcnc::cam::MachiningFaceRole::CrossSection: crossSectionFaces.push_back(entry.face); break;
                    }
                }
                if (!machiningFaces.empty()) {
                    auto contours = useLargestSmoothBoundary
                        || crossSectionFaces.empty()
                        ? LaserToolpathBuilder::extractContoursFromFaces(
                            source.shape, machiningFaces, gp_Dir(0.0, 0.0, -1.0), params)
                        : LaserToolpathBuilder::extractTubeContoursFromFaceGroups(
                            source.shape, machiningFaces, crossSectionFaces, params);
                    if (useLargestSmoothBoundary) {
                        for (LaserContour& contour : contours) {
                            LaserToolpathBuilder::bindLeadInSurfaceContext(
                                contour, machiningFaces, crossSectionFaces);
                            LaserToolpathBuilder::discretizeContourWithClassification(
                                contour, machiningFaces, crossSectionFaces, params.deflection);
                        }
                    }
                    for (LaserContour& contour : contours) {
                        contour.workpieceEntry = source.workpieceEntry;
                        contour.sourceShape = source.shape;
                        contour.appliedParams = {leadInLength, deflection,
                                                 cuttingOffsetMm, rapidOffsetMm};
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
    const double leadInLength = toolpathRef().globalLeadInLength();
    const double cuttingOffsetMm = toolpathRef().globalCuttingOffsetMm();
    const double rapidOffsetMm = toolpathRef().globalRapidOffsetMm();
    struct Result { std::vector<LaserContour> contours; QString error; bool ok{false}; };
    const auto result = std::make_shared<Result>();
    // 中文翻译：离散加工轮廓
    TaskSpec spec{tr("Discrete machining contours"), QStringLiteral("cam.pipeline"), TaskPriority::Normal, true};
    const TaskId taskId = taskManager->run(spec,
        [input, deflection, leadInLength, cuttingOffsetMm, rapidOffsetMm,
         result](TaskProgress* progress) {
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
                contour.appliedParams = {leadInLength, deflection,
                                         cuttingOffsetMm, rapidOffsetMm};
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
    struct Result { std::vector<LaserContour> contours; QString error; QStringList leadInWarnings; bool ok{false}; };
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
                    if (error.isEmpty())
                        error = contour.leadInSolution.error;
                    // Tolerate a per-contour lead-in failure: keep the contour
                    // without a lead-in and continue with the remaining ones.
                    contour.leadInSolution.valid = false;
                    contour.leadInSolution.error = error;
                    // 中文翻译：轮廓 "%1"：%2
                    result->leadInWarnings.append(
                        QObject::tr("Contour \"%1\": %2").arg(contour.name, error));
                    progress->setValue(static_cast<int>(index + 1));
                    continue;
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
        if (!result->leadInWarnings.isEmpty()) {
            // 中文翻译：构造刀路
            emit operationWarning(tr("Construct toolpath"),
                // 中文翻译：以下轮廓未能生成下刀点，已保留轮廓但不添加下刀点，其余刀路已正常生成：
                tr("The following contours could not resolve a lead-in and were kept "
                   "without one; the rest of the toolpath was generated:\n%1")
                    .arg(result->leadInWarnings.join(QStringLiteral("\n"))));
        }
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
    QString autoSortError;
    if (!preparePersistedAutoSort(&autoSortError)) {
        emit operationFailed(tr("Solve for machine coordinates"), autoSortError);
        return kInvalidTaskId;
    }
    const std::vector<LaserContour> input = toolpathRef().contours();
    const QVector<lcnc::cam::ContourId> order = contourSequenceSnapshot().orderedContourIds;
    const QString configType = machine->configType();
    const lcnc::MachiningMode machiningMode = m_camData->machiningMode();
    const lcnc::MachineModeDefinition modeDefinition = m_machineConfig
        ? m_machineConfig->modeDefinition(machiningMode) : lcnc::MachineModeDefinition{};
    const QList<MachineAxisDef> axes =
        lcnc::cam_algo::offlinePlanningAxisBaseline(machine->axes(), modeDefinition);
    const lcnc::WorkpieceSetupTransform workpieceSetup = m_machineConfig->workpieceSetupTransform();
    const lcnc::HeadToolGeometry headToolGeometry = m_machineConfig
        ? m_machineConfig->headToolGeometry() : lcnc::HeadToolGeometry{};
    struct Result { std::vector<LaserContour> contours; QString error; bool ok{false}; };
    const auto result = std::make_shared<Result>();
    TaskSpec spec;
    // 中文翻译：求解机床坐标
    spec.label = tr("Solve for machine coordinates");
    spec.scope = QStringLiteral("cam.pipeline");
    spec.priority = TaskPriority::Normal;
    spec.cancellable = true;
    const TaskId taskId = taskManager->run(spec,
        [input, order, axes, configType, modeDefinition, workpieceSetup,
         headToolGeometry, result](TaskProgress* progress) {
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
            if (!LaserToolpathBuilder::solveToolpathForOrder(
                    ordered, &workerKinematics, gp_Trsf(), modeDefinition,
                    workpieceSetup, headToolGeometry, &result->error)) {
                return;
            }
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
    watchTask(this, taskId, [this, taskId, result, modeDefinition,
                            pathRevision = pathState.revision](bool success) {
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
        m_camData->setMachineAxisLayout(modeDefinition.interpolatedAxes);
        m_camData->setSolverId(modeDefinition.solverId);
        m_camData->setSolverVersion(modeDefinition.solverVersion);
        m_camData->setSolvedMachineConfigurationFingerprint(
            m_machineConfig ? m_machineConfig->configurationFingerprint() : QString());
        for (LaserContour& contour : toolpathRef().contours())
            contour.needsRecalculation = false;
        m_camData->commitPipelineStage(lcnc::cam::CamPipelineStage::MachineSolve,
                                       currentPath.revision);
        m_camData->setGenerationParamsDirty(false);
        m_camData->markDirty(true);
        m_camData->commitToolpathStates();
        refreshToolpathDisplay();
        refreshCuttingOrderOverlays();
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

TaskId CamModule::generateToolpathAsync(double smoothAngle, bool useFaceClassification, double deflection,
                                        AutoPipelineFaceMode mode)
{
    const bool reuseCurrentFaces = (mode == AutoPipelineFaceMode::ReuseCurrent);
    const QList<WorkpieceShapeSource> workpieceSources = collectWorkpieceShapes();
    auto* projectManager = lcnc::Kernel::current().projectManager();
    auto* taskManager = lcnc::Kernel::current().taskManager();
    if (workpieceSources.isEmpty() || !projectManager || !taskManager || !m_camData)
        return kInvalidTaskId;

    // The worker result belongs to the workspace that supplied the OCC shapes
    // and CamDataManager.  An active-workspace change must never redirect that
    // result into the replacement workspace.
    const ProjectWorkspaceId targetWorkspaceId = projectManager->activeWorkspaceId();
    lcnc::cam::CamDataManager* const targetCamData = m_camData;

    bool effectiveUseFaceClassification = useFaceClassification;
    if (m_machineConfig && m_camData)
        effectiveUseFaceClassification =
            m_camData->machiningMode() != lcnc::MachiningMode::Planar3Axis;
    const double leadInLength = toolpathRef().globalLeadInLength();
    const double cuttingOffsetMm = toolpathRef().globalCuttingOffsetMm();
    const double rapidOffsetMm = toolpathRef().globalRapidOffsetMm();
    QHash<std::uint64_t, LaserContour> previousBySignature;
    for (const LaserContour& contour : toolpathRef().contours())
        previousBySignature.insert(contour.signature, contour);

    struct GenerationResult {
        std::vector<LaserContour> contours;
        std::vector<lcnc::cam::MachiningFacePipelineService::Candidate> automaticFaces;
        QString error;
        QStringList leadInWarnings;
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
    const QString configType = machine->configType();
    const lcnc::MachiningMode machiningMode = m_camData->machiningMode();
    const lcnc::MachineModeDefinition modeDefinition = m_machineConfig
        ? m_machineConfig->modeDefinition(machiningMode) : lcnc::MachineModeDefinition{};
    LCNC_INFO(lcnc::LogCode::Generic,
              "cam.toolpath: global generation captures preset='{}' mode='{}' solver='{}' axes={}",
              configType.toStdString(),
              lcnc::machiningModeName(machiningMode).toStdString(),
              modeDefinition.solverId.toStdString(),
              modeDefinition.interpolatedAxes.count);
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
    generationStamp.cuttingOffsetMm = cuttingOffsetMm;
    generationStamp.rapidOffsetMm = rapidOffsetMm;
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
    // this (main) thread where the kinematics WPC mounts live; the worker only
    // consumes the frozen per-source beam directions.
    QVector<gp_Dir> beamDirs;
    beamDirs.reserve(workpieceSources.size());
    for (const WorkpieceShapeSource& s : workpieceSources)
        beamDirs.append(beamDirectionWpc(s.workpieceEntry));

    TaskSpec spec;
    // 中文翻译：全局生成刀路
    spec.label = tr("Generate toolpath globally");
    spec.scope = QStringLiteral("cam.toolpath");
    spec.priority = TaskPriority::Normal;
    const auto stageTimer = std::make_shared<QElapsedTimer>();
    stageTimer->start();
    LCNC_INFO(lcnc::LogCode::Generic,
              "stage=cam.toolpath.generate event=begin sources={} revision={} mode='{}'",
              workpieceSources.size(), sourceRevision,
              lcnc::machiningModeName(machiningMode).toStdString());
    const TaskId taskId = taskManager->run(spec,
        [workpieceSources, previousBySignature, leadInLength,
         cuttingOffsetMm, rapidOffsetMm, params, selectedFaceEntries,
         extractionStrategy, beamDirs, effectiveUseFaceClassification, result](TaskProgress* progress) {
            progress->setRange(0, 100);
            // 中文翻译：1/5 正在分离加工面与横截面
            progress->setStepName(QObject::tr("1/5 Separating the machined surface and cross section"));
            progress->setValue(5);
            if (params.strategy != ExtractionStrategy::ManualFaceSelection) {
                result->automaticFaces = CamModule::selectAutomaticMachiningFaces(
                    workpieceSources, params.strategy,
                    params.smoothAngleThresholdDeg);
                if (result->automaticFaces.empty()) {
                    // 中文翻译：未识别到可加工面，请改用手动选面。
                    result->error = QObject::tr("No machinable surface is identified, please select manual surface instead.");
                    return;
                }
            }
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
                        if (entry.workpieceEntry != source.workpieceEntry
                            || !faceBelongsToSource(entry.face, source, workpieceSources))
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
                    const bool useLargestSmoothBoundary =
                        extractionStrategy == static_cast<int>(
                            ExtractionStrategy::LargestSmoothConnectedSurface);
                    contours = useLargestSmoothBoundary || crossFaces.empty()
                        ? LaserToolpathBuilder::extractContoursFromFaces(
                            source.shape, outerFaces, perSourceParams.machiningBeamDirection, perSourceParams)
                        : LaserToolpathBuilder::extractTubeContoursFromFaceGroups(
                            source.shape, outerFaces, crossFaces, perSourceParams);
                    if (useLargestSmoothBoundary) {
                        for (LaserContour& contour : contours) {
                            LaserToolpathBuilder::bindLeadInSurfaceContext(
                                contour, outerFaces, crossFaces);
                            LaserToolpathBuilder::discretizeContourWithClassification(
                                contour, outerFaces, crossFaces, perSourceParams.deflection);
                        }
                    }
                } else if (perSourceParams.strategy == ExtractionStrategy::PlanarFaceWires) {
                    for (const auto& candidate : result->automaticFaces) {
                        if (candidate.workpieceEntry == source.workpieceEntry
                            && faceBelongsToSource(candidate.face, source, workpieceSources)) {
                            outerFaces.push_back(candidate.face);
                        }
                    }
                    // A lower component removed by the projected top-layer
                    // prefilter intentionally contributes no contours.
                    if (outerFaces.empty())
                        continue;
                    contours = LaserToolpathBuilder::extractContoursFromFaces(
                        source.shape, outerFaces,
                        perSourceParams.machiningBeamDirection, perSourceParams);
                } else {
                    const FaceClassification autoClassification = FaceClassifier::classifyFaces(
                        source.shape, perSourceParams.smoothAngleThresholdDeg);
                    if (perSourceParams.strategy == ExtractionStrategy::LargestSmoothConnectedSurface
                        && !autoClassification.hasOuter()) {
                        // 中文翻译：工件源 #%1 无法可靠识别最大顺滑连通加工面，请手动调整面组。
                        result->error = QObject::tr("Workpiece source #%1 cannot reliably identify the largest smooth-connected machining surface. Please adjust the surface group manually.")
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
                    contour.appliedParams = {leadInLength, params.deflection,
                                             cuttingOffsetMm, rapidOffsetMm};
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
                            if (leadInError.isEmpty())
                                leadInError = contour.leadInSolution.error;
                            // Tolerate a per-contour lead-in failure: keep the
                            // contour without a lead-in so the rest of the
                            // toolpath can still generate. It is exported with
                            // hasLeadIn=false and Process will reject it with a
                            // clear reason; the user is warned after generation.
                            contour.leadInSolution.valid = false;
                            contour.leadInSolution.error = leadInError;
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
            // Machine coordinates deliberately remain unsolved here.  The GUI-thread
            // adoption path first applies the persisted automatic direction, then
            // performs the only authoritative ordered solve and collision planning.
            // 中文翻译：此处仅生成几何刀路；主线程采纳结果后先按已保存方向自动排序，
            // 再执行唯一一次权威的有序机床坐标求解及碰撞规划。
            for (const LaserContour& contour : allContours) {
                if (!contour.leadInSolution.valid) {
                    const QString reason = contour.leadInSolution.error.trimmed().isEmpty()
                        // 中文翻译：下刀点不可用
                        ? QObject::tr("lead-in unavailable")
                        : contour.leadInSolution.error;
                    // 中文翻译：轮廓 "%1"：%2
                    result->leadInWarnings.append(
                        QObject::tr("Contour \"%1\": %2").arg(contour.name, reason));
                }
            }
            result->contours = std::move(allContours);
            result->ok = true;
            progress->setValue(100);
        });

    m_taskScope.track(taskId);
    watchTask(this, taskId,
        [this, taskId, result, leadInLength,
         effectiveUseFaceClassification, smoothAngle, deflection,
         generationStamp, reuseCurrentFaces, modeDefinition,
         targetWorkspaceId, targetCamData, stageTimer](bool success) {
            bool adopted = false;
            const auto stageLog = qScopeGuard([&] {
                LCNC_INFO(lcnc::LogCode::Generic,
                          "stage=cam.toolpath.generate event=end result={} contours={} elapsed_ms={} error='{}'",
                          adopted ? "success" : "failed",
                          adopted ? toolpathRef().contours().size()
                                  : result->contours.size(),
                          stageTimer->elapsed(), result->error.toStdString());
            });
            m_taskScope.release(taskId);
            if (!success || !result->ok) {
                // 中文翻译：全局生成刀路
                emit operationFailed(tr("Generate toolpath globally"),
                    // 中文翻译：刀路生成失败或已取消
                    result->error.isEmpty() ? tr("Tool path generation failed or canceled") : result->error);
                return;
            }
            auto* projectManager = lcnc::Kernel::current().projectManager();
            if (!projectManager
                || projectManager->activeWorkspaceId() != targetWorkspaceId
                || projectManager->camData() != targetCamData
                || m_camData != targetCamData) {
                // 中文翻译：全局生成刀路；计算期间活动工程已切换，后台结果已丢弃
                emit operationFailed(tr("Generate toolpath globally"),
                    tr("The active project changed during calculation and the background results were discarded"));
                return;
            }
            const QList<WorkpieceShapeSource> currentSources = collectWorkpieceShapes();
            bool currentEffectiveFaceClassification = m_useFaceClassification;
            if (m_machineConfig && m_camData)
                currentEffectiveFaceClassification =
                    m_camData->machiningMode() != lcnc::MachiningMode::Planar3Axis;
            lcnc::cam::ToolpathGenerationStamp currentStamp;
            currentStamp.toolpathRevision = toolpathRevision();
            currentStamp.machiningFaceRevision = machiningFaceSetRevision();
            currentStamp.machineSetupRevision = machineSetupRevision();
            currentStamp.leadInLength = toolpathRef().globalLeadInLength();
            currentStamp.smoothAngle = m_smoothAngle;
            currentStamp.deflection = m_deflection;
            currentStamp.cuttingOffsetMm = toolpathRef().globalCuttingOffsetMm();
            currentStamp.rapidOffsetMm = toolpathRef().globalRapidOffsetMm();
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
            toolpathRef().setGlobalCuttingOffsetMm(generationStamp.cuttingOffsetMm);
            toolpathRef().setGlobalRapidOffsetMm(generationStamp.rapidOffsetMm);
            m_smoothAngle = smoothAngle;
            m_useFaceClassification = effectiveUseFaceClassification;
            m_deflection = deflection;
            m_workpieceShape = collectWorkpieceShape();
            m_travelCollisionGeometryCache.reset();
            if (!reuseCurrentFaces) {
                // Capture the exact automatic face group.  It is project data,
                // not a renderer-only side effect: later manual stages continue
                // from these faces without reclassifying.
                const ExtractionStrategy currentStrategy =
                    static_cast<ExtractionStrategy>(m_extractionStrategy);
                if ((currentStrategy == ExtractionStrategy::LargestSmoothConnectedSurface
                     || currentStrategy == ExtractionStrategy::PlanarFaceWires)
                    && m_machiningFacePipeline->replaceAutomaticFaces(result->automaticFaces)) {
                    pushMachiningFaceRecordsToCamData();
                    refreshMachiningFaceDisplay();
                    emit machiningFacesChanged();
                }
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
            // ensureToolpathLayers() also assigns/reuses contour IDs.  Keep the
            // result adoption to one container pass; a second immediate pass is
            // redundant and was the crash site after the asynchronous move.
            m_camData->ensureToolpathLayers();
            QString autoSortError;
            if (!preparePersistedAutoSort(&autoSortError)) {
                m_camData->failPipelineStage(
                    lcnc::cam::CamPipelineStage::MachineSolve, autoSortError);
                emit operationFailed(tr("Generate toolpath globally"), autoSortError);
                return;
            }
            const auto committedOrder = contourSequenceSnapshot().orderedContourIds;
            if (!solveToolpathForOrder(
                    QVector<std::uint64_t>(committedOrder.cbegin(), committedOrder.cend()))) {
                const QString error = tr("CAM cannot solve the generated contour order");
                m_camData->failPipelineStage(lcnc::cam::CamPipelineStage::MachineSolve, error);
                emit operationFailed(tr("Generate toolpath globally"), error);
                return;
            }
            m_camData->commitPipelineStage(lcnc::cam::CamPipelineStage::MachineSolve,
                                           pathStage.revision);
            m_camData->setMachineAxisLayout(modeDefinition.interpolatedAxes);
            m_camData->setSolverId(modeDefinition.solverId);
            m_camData->setSolverVersion(modeDefinition.solverVersion);
            m_camData->setSolvedMachineConfigurationFingerprint(
                m_machineConfig ? m_machineConfig->configurationFingerprint() : QString());
            pushGenerationParamsToCamData();
            auto applied = m_camData->generationParams();
            applied.useFaceClassification = effectiveUseFaceClassification;
            m_camData->appliedGenerationParams() = applied;
            m_camData->setGenerationParamsDirty(false);
            m_camData->markDirty(true);
            m_camData->commitToolpathStates();
            QString rapidPlanWarning;
            rebuildTravelPlanForCurrentOrder(&rapidPlanWarning);
            writeContourGeometryToDocument();
            relinkContourGeometryFromDocument();
            syncCamDocumentContours(/*forceRebuild=*/true);
            lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
            m_toolpathRenderer->setVisible(activeGuiDocument(), true);
            refreshToolpathDisplay();
            setActiveContourId(0);
            m_lastCamSelectionContourIds.clear();
            if (auto selection = lcnc::Kernel::current().services()
                    .getService<lcnc::core::SelectionService>()) {
                selection->clear();
            }
            emit toolpathGenerated();
            emit toolpathLayersChanged();
            emit pipelineStageChanged(lcnc::cam::CamPipelineStage::MachineSolve);
            refreshCuttingOrderOverlays();
            if (!rapidPlanWarning.isEmpty()) {
                emit operationWarning(tr("Generate toolpath globally"),
                    tr("The toolpath was generated, but its rapid travel plan is unavailable: %1")
                        .arg(rapidPlanWarning));
            }
            if (!result->leadInWarnings.isEmpty()) {
                // 中文翻译：全局生成刀路
                emit operationWarning(tr("Generate toolpath globally"),
                    // 中文翻译：以下轮廓未能生成下刀点，已保留轮廓但不添加下刀点，其余刀路已正常生成：
                    tr("The following contours could not resolve a lead-in and were kept "
                       "without one; the rest of the toolpath was generated:\n%1")
                        .arg(result->leadInWarnings.join(QStringLiteral("\n"))));
            }
            adopted = true;
        });
    return taskId;
}
