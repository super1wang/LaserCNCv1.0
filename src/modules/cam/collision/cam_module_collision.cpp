
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
using CamTravelCollisionBody = lcnc::cam::TravelCollisionBody;
using CamTravelCollisionLeaf = lcnc::cam::TravelCollisionLeaf;
using CamTravelCollisionGeometryCache = lcnc::cam::TravelCollisionGeometryCache;
using lcnc::cam::buildCollisionGeometry;
using lcnc::cam::collisionAxisSourceId;
using lcnc::cam::collisionGeometryKey;
using lcnc::cam::transformCollisionAabb;
using lcnc::cam::transformCollisionObb;

bool isActiveCollisionAxis(lcnc::MachineAxisRole role)
{
    return role == lcnc::MachineAxisRole::LinearZ
        || role == lcnc::MachineAxisRole::HeadTiltPrimary
        || role == lcnc::MachineAxisRole::HeadTiltSecondary;
}
} // namespace

lcnc::cam::CollisionConfigurationSnapshot CamModule::collisionConfiguration() const
{
    lcnc::cam::CollisionConfigurationSnapshot snapshot;
    snapshot.machineProfilePath = activeMachineProfilePath();
    snapshot.enabled = m_config.collisionDetectionEnabledForMachine(snapshot.machineProfilePath);
    snapshot.activeSources = m_config.activeCollisionSourcesForMachine(snapshot.machineProfilePath);
    snapshot.passiveSources = m_config.passiveCollisionSourcesForMachine(snapshot.machineProfilePath);
    snapshot.revision = m_collisionConfigurationRevision;

    QString cutterError;
    const bool cutterAvailable = !const_cast<CamModule*>(this)->cutterCollisionProxyShape(&cutterError).IsNull();
    snapshot.sources.append({QStringLiteral("cutter"), tr("Cutting head"),
                             lcnc::cam::CollisionSemanticRole::CuttingHead,
                             true, false, cutterAvailable, cutterAvailable ? 1 : 0});

    int workpieceCount = 0;
    if (LcncDocument* document = workpieceDocument())
        workpieceCount = document->entityLabels(LcncDocument::EntityKind::Workpiece).Length();
    snapshot.sources.append({QStringLiteral("workpiece"), tr("Workpiece"),
                             lcnc::cam::CollisionSemanticRole::Workpiece,
                             false, true, workpieceCount > 0, workpieceCount});

    const MachineKinematics* machine = kinematics();
    if (!machine) {
        snapshot.activeSources.intersect(QSet<QString>{QStringLiteral("cutter")});
        snapshot.passiveSources.intersect(QSet<QString>{QStringLiteral("workpiece")});
    } else {
        QSet<QString> assigned;
        for (auto it = machine->shapeAssignments().cbegin(); it != machine->shapeAssignments().cend(); ++it)
            assigned.insert(it.key());
        int machineBodies = 0;
        if (LcncDocument* document = machineDocument())
            machineBodies = document->entityLabels(LcncDocument::EntityKind::Machine).Length();
        snapshot.unassignedMachineBodyCount = qMax(0, machineBodies - assigned.size());

        for (const MachineAxisDef& axis : machine->axes()) {
            const int bodyCount = machine->shapesForAxis(axis.name).size();
            const bool active = isActiveCollisionAxis(axis.role);
            snapshot.sources.append({collisionAxisSourceId(axis.name),
                                     axis.name == QStringLiteral("BASE")
                                         ? tr("BASE (static frame)")
                                         : tr("Axis %1").arg(axis.name),
                                     axis.name == QStringLiteral("BASE")
                                         ? lcnc::cam::CollisionSemanticRole::StaticFrame
                                         : lcnc::cam::CollisionSemanticRole::MovingPart,
                                     active, !active, bodyCount > 0, bodyCount});
        }
    }

    const auto selectedAvailable = [&snapshot](const QSet<QString>& sources, bool active) {
        for (const auto& source : snapshot.sources) {
            if (source.available && ((active && source.activeCandidate)
                || (!active && source.passiveCandidate)) && sources.contains(source.id))
                return true;
        }
        return false;
    };
    snapshot.valid = selectedAvailable(snapshot.activeSources, true)
        && selectedAvailable(snapshot.passiveSources, false);
    return snapshot;
}

lcnc::cam::CollisionSafetyDomainSnapshot CamModule::collisionSafetyDomain() const
{
    std::shared_ptr<CamTravelCollisionGeometryCache> geometry;
    const auto capture = [this, &geometry] {
        geometry = m_travelCollisionGeometryCache;
    };
    if (QThread::currentThread() == thread())
        capture();
    else
        QMetaObject::invokeMethod(const_cast<CamModule*>(this), capture,
                                  Qt::BlockingQueuedConnection);

    lcnc::cam::CollisionSafetyDomainSnapshot snapshot;
    if (!geometry)
        return snapshot;
    snapshot.environmentRevision = geometry->key.environmentRevision;
    snapshot.ready = !geometry->bodies.isEmpty() && geometry->safetyDomain;
    snapshot.geometryBodies = static_cast<std::size_t>(geometry->bodies.size());
    if (geometry->safetyDomain) {
        const auto statistics = geometry->safetyDomain->statistics();
        snapshot.certifiedSamples = statistics.samples;
        snapshot.cacheHits = statistics.hits;
        snapshot.cacheMisses = statistics.misses;
    }
    return snapshot;
}

lcnc::cam::CollisionValidationSnapshot CamModule::validateCollisionPath(
    const lcnc::cam::CollisionSafetyPathRequest& request,
    std::atomic_bool* cancelRequested) const
{
    lcnc::cam::CollisionValidationSnapshot result;
    result.key = request.environmentRevision;
    result.blockWarning = request.blockWarning;
    result.complete = true;
    std::shared_ptr<CamTravelCollisionGeometryCache> geometry;
    QElapsedTimer validationElapsed;
    validationElapsed.start();
    LCNC_INFO(lcnc::LogCode::Generic,
              "stage=cam.safety_domain.query event=begin poses={} scope={} environment_revision={}",
              request.poses.size(), static_cast<int>(request.scope),
              request.environmentRevision);
    const auto validationLog = qScopeGuard([&] {
        lcnc::cam_algo::CollisionSafetyDomainStatistics statistics;
        if (geometry && geometry->safetyDomain)
            statistics = geometry->safetyDomain->statistics();
        LCNC_INFO(lcnc::LogCode::Generic,
                  "stage=cam.safety_domain.query event=end state={} poses={} samples={} hits={} misses={} stores={} elapsed_ms={} reason='{}'",
                  static_cast<int>(result.state), request.poses.size(),
                  statistics.samples, statistics.hits, statistics.misses,
                  statistics.stores, validationElapsed.elapsed(),
                  result.failureReason.toStdString());
    });
    const auto cancelled = [cancelRequested] {
        return cancelRequested && cancelRequested->load();
    };
    if (request.poses.isEmpty() || !std::isfinite(request.clearanceMm)
        || request.clearanceMm < 0.0) {
        result.state = lcnc::cam::CollisionValidationState::Indeterminate;
        // 中文翻译：碰撞安全域路径请求无效
        result.failureReason = tr("Collision safety-domain path request is invalid");
        return result;
    }

    const auto capture = [this, &geometry] {
        geometry = m_travelCollisionGeometryCache;
    };
    if (QThread::currentThread() == thread())
        capture();
    else
        QMetaObject::invokeMethod(const_cast<CamModule*>(this), capture,
                                  Qt::BlockingQueuedConnection);
    if (!geometry || geometry->key.environmentRevision != request.environmentRevision
        || geometry->bodies.isEmpty() || !geometry->safetyDomain) {
        result.state = lcnc::cam::CollisionValidationState::Indeterminate;
        // 中文翻译：CAM 碰撞安全域尚未准备完成，请等待 CAM 准备任务或重新生成刀路。
        result.failureReason = tr("CAM collision safety domain is not ready for this environment");
        return result;
    }

    MachineKinematics kinematics;
    kinematics.setAxes(geometry->axes, geometry->configType);
    kinematics.setWorkpieceSetupTransform(geometry->workpieceSetup);
    for (auto it = geometry->assignments.cbegin(); it != geometry->assignments.cend(); ++it)
        kinematics.assignShape(it.key(), it.value());
    for (auto it = geometry->mounts.cbegin(); it != geometry->mounts.cend(); ++it)
        kinematics.mountWorkpiece(it.key(), it.value());

    result.nodeStates.fill(lcnc::cam::CollisionValidationState::Safe,
                           request.poses.size());
    bool warning = false;
    try {
        for (int poseIndex = 0; poseIndex < request.poses.size(); ++poseIndex) {
            if (cancelled()) {
                result.state = lcnc::cam::CollisionValidationState::Indeterminate;
                // 中文翻译：碰撞安全域路径校验已取消
                result.failureReason = tr("Collision safety-domain path validation was cancelled");
                return result;
            }
            const auto& item = request.poses.at(poseIndex);
            lcnc::cam_algo::applyOfflineMotionPose(
                &kinematics, geometry->axes,
                geometry->definition.interpolatedAxes,
                item.pose.kinematicAxes, item.pose.kinematicAxisMask);

            struct PoseBody {
                const CamTravelCollisionBody* body{nullptr};
                gp_Trsf transform;
                Bnd_Box aabb;
                Bnd_OBB obb;
            };
            QVector<PoseBody> activeBodies;
            QVector<PoseBody> passiveBodies;
            for (const CamTravelCollisionBody& body : geometry->bodies) {
                if (request.scope == lcnc::cam::CollisionSafetyScope::MachineOnly
                    && body.workpiece) {
                    continue;
                }
                gp_Trsf transform;
                if (body.cutterProxy) {
                    gp_Vec normal(item.pose.surfaceNormalX,
                                  item.pose.surfaceNormalY,
                                  item.pose.surfaceNormalZ);
                    if (normal.SquareMagnitude() <= Precision::SquareConfusion())
                        normal = gp_Vec(0.0, 0.0, 1.0);
                    normal.Normalize();
                    transform.SetDisplacement(
                        gp_Ax3(), gp_Ax3(gp_Pnt(item.pose.tcpX,
                                               item.pose.tcpY,
                                               item.pose.tcpZ),
                                        gp_Dir(normal)));
                } else if (body.workpiece) {
                    transform = kinematics.computeWpcTransform(item.workpieceEntry);
                } else {
                    transform = kinematics.computeShapeTransform(body.entry);
                }
                PoseBody poseBody{
                    &body, transform,
                    transformCollisionAabb(body.localAabb, transform,
                                           request.clearanceMm),
                    transformCollisionObb(body.localObb, transform,
                                          request.clearanceMm)};
                if (body.active)
                    activeBodies.append(poseBody);
                if (body.passive)
                    passiveBodies.append(poseBody);
            }
            if (activeBodies.isEmpty() || passiveBodies.isEmpty()) {
                result.state = lcnc::cam::CollisionValidationState::Indeterminate;
                // 中文翻译：碰撞安全域没有可用的碰撞源组合
                result.failureReason = tr("Collision safety domain has no usable source pair");
                return result;
            }

            bool poseBlocked = false;
            for (const PoseBody& active : std::as_const(activeBodies)) {
                if (poseBlocked)
                    break;
                for (const PoseBody& passive : std::as_const(passiveBodies)) {
                    if (active.body->source == passive.body->source)
                        continue;
                    lcnc::cam_algo::CollisionSafetyDomainQuery query;
                    query.pose = item.pose;
                    query.workpieceEntry = item.workpieceEntry;
                    query.phase = item.phase;
                    query.activeSource = active.body->source;
                    query.passiveSource = passive.body->source;
                    query.clearanceMm = request.clearanceMm;
                    lcnc::cam_algo::CollisionSafetyDomainSample sample;
                    if (!geometry->safetyDomain->lookup(query, &sample)) {
                        sample.state = lcnc::cam::CollisionValidationState::Safe;
                        sample.activeEntity = active.body->entry;
                        sample.passiveEntity = passive.body->entry;
                        sample.minimumDistanceMm =
                            std::numeric_limits<double>::infinity();

                        if (!active.aabb.IsOut(passive.aabb)
                            && !active.obb.IsOut(passive.obb)) {
                            bool separatedByMesh = false;
                            if (active.body->surfaceModel.isValid()
                                && passive.body->surfaceModel.isValid()) {
                                const double meshGuardMm = request.clearanceMm
                                    + active.body->surfaceModel.linearDeflectionMm()
                                    + passive.body->surfaceModel.linearDeflectionMm();
                                const auto prefilter =
                                    lcnc::cam_algo::prefilterSurfaceCollision(
                                        active.body->surfaceModel, active.transform,
                                        passive.body->surfaceModel, passive.transform,
                                        meshGuardMm);
                                separatedByMesh = prefilter.valid
                                    && prefilter.definitelySeparated;
                                if (prefilter.valid)
                                    sample.minimumDistanceMm = prefilter.meshDistanceMm;
                            }
                            if (!separatedByMesh) {
                                for (const auto& activeLeaf : active.body->leaves) {
                                    if (poseBlocked)
                                        break;
                                    const Bnd_Box activeBox = transformCollisionAabb(
                                        activeLeaf.localAabb, active.transform,
                                        request.clearanceMm);
                                    const Bnd_OBB activeObb = transformCollisionObb(
                                        activeLeaf.localObb, active.transform,
                                        request.clearanceMm);
                                    for (const auto& passiveLeaf : passive.body->leaves) {
                                        if (activeBox.IsOut(transformCollisionAabb(
                                                passiveLeaf.localAabb, passive.transform,
                                                request.clearanceMm))
                                            || activeObb.IsOut(transformCollisionObb(
                                                passiveLeaf.localObb, passive.transform,
                                                request.clearanceMm))) {
                                            continue;
                                        }
                                        const TopoDS_Shape first = activeLeaf.shape.Moved(
                                            TopLoc_Location(active.transform));
                                        const TopoDS_Shape second = passiveLeaf.shape.Moved(
                                            TopLoc_Location(passive.transform));
                                        lcnc::OcctExactOperationLock exactOperationLock;
                                        BRepExtrema_DistShapeShape distance(first, second);
                                        distance.SetDeflection(0.025);
                                        distance.SetMultiThread(Standard_False);
                                        distance.Perform();
                                        if (!distance.IsDone()) {
                                            result.state = lcnc::cam::CollisionValidationState::Indeterminate;
                                            // 中文翻译：碰撞安全域精确距离计算失败
                                            result.failureReason = tr("Collision safety-domain exact distance calculation failed");
                                            return result;
                                        }
                                        double minimumDistance = distance.Value();
                                        if (minimumDistance <= Precision::Confusion()
                                            && active.body->cutterProxy
                                            && passive.body->workpiece
                                            && item.phase == lcnc::cam::CamMotionPhase::Cutting) {
                                            gp_Vec outward(item.pose.surfaceNormalX,
                                                           item.pose.surfaceNormalY,
                                                           item.pose.surfaceNormalZ);
                                            if (outward.SquareMagnitude()
                                                <= Precision::SquareConfusion()) {
                                                outward = gp_Vec(0.0, 0.0, 1.0);
                                            }
                                            outward.Normalize();
                                            outward.Multiply(qMax(
                                                0.05, Precision::Confusion() * 10.0));
                                            gp_Trsf probeTransform;
                                            probeTransform.SetTranslation(outward);
                                            const TopoDS_Shape probe = first.Moved(
                                                TopLoc_Location(probeTransform));
                                            BRepExtrema_DistShapeShape probeDistance(
                                                probe, second);
                                            probeDistance.SetDeflection(0.025);
                                            probeDistance.SetMultiThread(Standard_False);
                                            probeDistance.Perform();
                                            if (probeDistance.IsDone()
                                                && probeDistance.Value()
                                                    > Precision::Confusion()) {
                                                minimumDistance = request.clearanceMm
                                                    + Precision::Confusion();
                                            }
                                        }
                                        sample.minimumDistanceMm = std::min(
                                            sample.minimumDistanceMm,
                                            minimumDistance);
                                        if (minimumDistance <= request.clearanceMm) {
                                            sample.state =
                                                lcnc::cam_algo::classifyCollisionDistance(
                                                    minimumDistance,
                                                    request.clearanceMm,
                                                    Precision::Confusion());
                                            sample.reason = sample.state
                                                    == lcnc::cam::CollisionValidationState::Collision
                                                // 中文翻译：碰撞安全域路径与 %1 和 %2 相交
                                                ? tr("Collision safety-domain path intersects %1 and %2")
                                                      .arg(active.body->source,
                                                           passive.body->source)
                                                // 中文翻译：碰撞安全域路径在 %1 和 %2 之间的间隙不足
                                                : tr("Collision safety-domain path clearance is insufficient between %1 and %2")
                                                      .arg(active.body->source,
                                                           passive.body->source);
                                            poseBlocked = true;
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                        geometry->safetyDomain->store(query, sample);
                    }

                    if (sample.state == lcnc::cam::CollisionValidationState::Safe)
                        continue;
                    const auto state = sample.state;
                    result.nodeStates[poseIndex] = state;
                    result.intervals.append({
                        poseIndex, poseIndex, state,
                        active.body->source, passive.body->source,
                        sample.activeEntity, sample.passiveEntity,
                        sample.minimumDistanceMm, sample.reason});
                    if (result.failureReason.isEmpty())
                        result.failureReason = sample.reason;
                    if (state == lcnc::cam::CollisionValidationState::Collision) {
                        result.state = state;
                        return result;
                    }
                    if (state == lcnc::cam::CollisionValidationState::Warning)
                        warning = true;
                    poseBlocked = true;
                    break;
                }
            }
        }
    } catch (const Standard_Failure& failure) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "CAM collision safety-domain OCCT failure: {}",
                 failure.GetMessageString());
        result.state = lcnc::cam::CollisionValidationState::Indeterminate;
        // 中文翻译：碰撞安全域几何运算失败
        result.failureReason = tr("Collision safety-domain geometry operation failed");
        return result;
    } catch (const std::exception& failure) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "CAM collision safety-domain failure: {}", failure.what());
        result.state = lcnc::cam::CollisionValidationState::Indeterminate;
        // 中文翻译：碰撞安全域校验失败
        result.failureReason = tr("Collision safety-domain validation failed");
        return result;
    } catch (...) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "CAM collision safety-domain unknown failure");
        result.state = lcnc::cam::CollisionValidationState::Indeterminate;
        // 中文翻译：碰撞安全域校验失败
        result.failureReason = tr("Collision safety-domain validation failed");
        return result;
    }

    result.state = warning ? lcnc::cam::CollisionValidationState::Warning
                           : lcnc::cam::CollisionValidationState::Safe;
    return result;
}

void CamModule::setCollisionDetectionEnabled(bool enabled)
{
    const QString profilePath = activeMachineProfilePath();
    if (m_config.collisionDetectionEnabledForMachine(profilePath) == enabled)
        return;
    m_config.setCollisionDetectionEnabledForMachine(profilePath, enabled);
    ++m_collisionConfigurationRevision;
    m_travelPlanCache.stale = true;
    m_travelCollisionGeometryCache.reset();
    if (m_travelVerificationTask != kInvalidTaskId) {
        if (auto* tasks = lcnc::Kernel::current().taskManager())
            tasks->requestAbort(m_travelVerificationTask);
    }
    if (m_collisionDomainPreparationTask != kInvalidTaskId) {
        if (auto* tasks = lcnc::Kernel::current().taskManager())
            tasks->requestAbort(m_collisionDomainPreparationTask);
        m_collisionDomainPreparationTask = kInvalidTaskId;
    }
    emit collisionConfigurationChanged();
}

void CamModule::setCollisionSources(const QSet<QString>& active,
                                    const QSet<QString>& passive)
{
    const auto current = collisionConfiguration();
    QSet<QString> nextActive;
    QSet<QString> nextPassive;
    for (const auto& source : current.sources) {
        if (source.activeCandidate && source.available && active.contains(source.id))
            nextActive.insert(source.id);
        if (source.passiveCandidate && source.available && passive.contains(source.id))
            nextPassive.insert(source.id);
    }
    if (current.activeSources == nextActive && current.passiveSources == nextPassive)
        return;
    m_config.setCollisionSourcesForMachine(activeMachineProfilePath(), nextActive, nextPassive);
    ++m_collisionConfigurationRevision;
    m_travelPlanCache.stale = true;
    m_travelCollisionGeometryCache.reset();
    if (m_travelVerificationTask != kInvalidTaskId) {
        if (auto* tasks = lcnc::Kernel::current().taskManager())
            tasks->requestAbort(m_travelVerificationTask);
    }
    if (m_collisionDomainPreparationTask != kInvalidTaskId) {
        if (auto* tasks = lcnc::Kernel::current().taskManager())
            tasks->requestAbort(m_collisionDomainPreparationTask);
        m_collisionDomainPreparationTask = kInvalidTaskId;
    }
    emit collisionConfigurationChanged();
}

bool CamModule::validateCurrentToolpathCollisions(QString* errorMessage)
{
    if (!m_camData || !m_camData->hasToolpath()) {
        // 中文翻译：没有可用于碰撞校验的刀路
        if (errorMessage) *errorMessage = tr("There is no toolpath available for collision validation");
        return false;
    }
    const auto collision = collisionConfiguration();
    if (!collision.valid) {
        // 中文翻译：碰撞源配置不完整
        if (errorMessage) *errorMessage = tr("Collision source configuration is incomplete");
        return false;
    }
    if (!lcnc::Kernel::current().taskManager() || !kinematics()) {
        // 中文翻译：碰撞校验后台服务不可用
        if (errorMessage) *errorMessage = tr("Collision validation background service is unavailable");
        return false;
    }

    const auto order = contourSequenceSnapshot().orderedContourIds;
    auto snapshot = exportToolpathSnapshotForOrder(
        QVector<std::uint64_t>(order.cbegin(), order.cend()));
    if (snapshot.motionPlan.nodes.isEmpty()
        || (snapshot.contours.size() > 1 && !snapshot.travelPlan.isExecutable())) {
        if (errorMessage) {
            *errorMessage = snapshot.travelPlan.failureReason.isEmpty()
                // 中文翻译：当前刀路没有可执行的运动计划
                ? tr("The current toolpath does not have an executable motion plan")
                : snapshot.travelPlan.failureReason;
        }
        return false;
    }

    lcnc::cam::TravelPlanSnapshot pending = snapshot.travelPlan;
    pending.stale = false;
    pending.failureReason.clear();
    pending.fullEnvironmentVerificationPending = true;
    pending.collision.state = lcnc::cam::CollisionValidationState::Pending;
    pending.collision.complete = false;
    pending.collision.failureReason.clear();
    pending.collision.blockWarning = m_config.blockMachiningOnCollisionWarning();
    if (pending.collision.key == 0) {
        pending.collision.key = pending.key.toolpathRevision
            ^ pending.key.orderHash ^ pending.key.environmentRevision;
    }
    pending.collision.nodeStates.fill(
        lcnc::cam::CollisionValidationState::Pending,
        snapshot.motionPlan.nodes.size());
    pending.collision.intervals.clear();
    m_travelPlanCache = pending;
    snapshot.travelPlan = pending;
    snapshot.motionPlan.collision = pending.collision;

    scheduleFullEnvironmentVerification(snapshot, true);
    emit contourOrderTravelPlanRebuilt(
        QVector<std::uint64_t>(order.cbegin(), order.cend()));
    return true;
}

void CamModule::scheduleCollisionSafetyDomainPreparation(
    const lcnc::cam::ToolpathExportSnapshot& snapshot)
{
    auto* tasks = lcnc::Kernel::current().taskManager();
    MachineKinematics* liveKinematics = kinematics();
    if (!tasks || !liveKinematics || !m_machineConfig)
        return;
    const auto collision = collisionConfiguration();
    if (!collision.valid)
        return;

    const lcnc::cam::TravelPlanKey geometryKey =
        collisionGeometryKey(snapshot.travelPlan.key);
    if (m_travelCollisionGeometryCache
        && m_travelCollisionGeometryCache->key == geometryKey) {
        return;
    }
    if (m_collisionDomainPreparationTask != kInvalidTaskId)
        return;

    QVector<CamTravelCollisionBody> bodies;
    if (LcncDocument* machine = machineDocument();
        machine && machine->shapeTool()) {
        const TDF_LabelSequence labels = machine->entityLabels(
            LcncDocument::EntityKind::Machine);
        for (int index = 1; index <= labels.Length(); ++index) {
            const TDF_Label& label = labels.Value(index);
            const QString entry = XcafUtils::entry(label);
            const QString source = collisionAxisSourceId(
                liveKinematics->axisForShape(entry));
            const bool active = collision.activeSources.contains(source);
            const bool passive = collision.passiveSources.contains(source);
            if (!active && !passive)
                continue;
            const TopoDS_Shape shape = machine->shapeTool()->GetShape(label);
            if (!shape.IsNull())
                bodies.append({entry, source, active, passive, false, false, shape});
        }
    }
    if (collision.activeSources.contains(QStringLiteral("cutter"))) {
        QString cutterError;
        const TopoDS_Shape cutter = cutterCollisionProxyShape(&cutterError);
        if (!cutter.IsNull()) {
            bodies.append({QStringLiteral("__cutter_proxy__"),
                           QStringLiteral("cutter"), true, false,
                           true, false, cutter});
        }
    }
    if (collision.passiveSources.contains(QStringLiteral("workpiece"))
        && !m_workpieceShape.IsNull()) {
        bodies.append({QStringLiteral("__workpiece__"),
                       QStringLiteral("workpiece"), false, true,
                       false, true, m_workpieceShape});
    }
    bool hasActive = false;
    bool hasPassive = false;
    for (const CamTravelCollisionBody& body : std::as_const(bodies)) {
        hasActive = hasActive || body.active;
        hasPassive = hasPassive || body.passive;
    }
    if (!hasActive || !hasPassive)
        return;

    const auto axes = lcnc::cam_algo::offlinePlanningAxisBaseline(
        liveKinematics->axes(),
        m_machineConfig->modeDefinition(snapshot.machiningMode));
    const QString configType = liveKinematics->configType();
    const gp_Trsf workpieceSetup = liveKinematics->workpieceSetupTransform();
    const auto assignments = liveKinematics->shapeAssignments();
    const auto mounts = liveKinematics->wpcMounts();
    const auto definition = m_machineConfig->modeDefinition(snapshot.machiningMode);
    const double clearanceMm = m_config.cutterCollisionClearanceMm();
    const double meshDeflectionMm = qBound(0.05, clearanceMm * 0.1, 0.25);
    const auto completed =
        std::make_shared<std::shared_ptr<CamTravelCollisionGeometryCache>>();

    TaskSpec spec;
    // This is intentionally a CAM preparation task. Process never builds OCC
    // geometry when machining starts; it only consumes the prepared domain.
    // 中文翻译：安全域在 CAM 阶段准备，加工启动时 Process 不构造 OCC 几何。
    // 中文翻译：准备 CAM 碰撞安全域
    spec.label = tr("Prepare CAM collision safety domain");
    spec.scope = QStringLiteral("cam.collision-domain");
    spec.priority = TaskPriority::Normal;
    spec.userVisible = false;
    spec.cancellable = true;
    const TaskId taskId = tasks->run(spec,
        [bodies, axes, configType, workpieceSetup, assignments, mounts,
         definition, geometryKey, meshDeflectionMm, completed](TaskProgress* progress) {
            progress->setRange(0, qMax(1, bodies.size()));
            std::unique_lock<std::timed_mutex> scanLock(
                lcnc::cam_algo::collisionScanExecutionMutex(), std::defer_lock);
            if (!lcnc::cam_algo::acquireCollisionScanExecution(scanLock, [progress]() {
                    return progress->isAbortRequested();
                })) {
                return;
            }
            auto geometry = std::make_shared<CamTravelCollisionGeometryCache>();
            geometry->key = geometryKey;
            geometry->bodies = bodies;
            geometry->axes = axes;
            geometry->configType = configType;
            geometry->workpieceSetup = workpieceSetup;
            geometry->assignments = assignments;
            geometry->mounts = mounts;
            geometry->definition = definition;
            int prepared = 0;
            for (CamTravelCollisionBody& body : geometry->bodies) {
                if (progress->isAbortRequested())
                    return;
                buildCollisionGeometry(&body, meshDeflectionMm);
                if (body.leaves.isEmpty())
                    return;
                progress->setValue(++prepared);
            }
            *completed = std::move(geometry);
        });
    m_collisionDomainPreparationTask = taskId;
    m_taskScope.track(taskId);
    QObject::connect(tasks, &TaskManager::taskFinishedDetailed, this,
        [this, taskId, geometryKey, completed](TaskId id,
                                               TaskExecutionStatus status,
                                               const QString&) {
            if (id != taskId)
                return;
            m_taskScope.release(taskId);
            if (m_collisionDomainPreparationTask != taskId)
                return;
            m_collisionDomainPreparationTask = kInvalidTaskId;
            if (status != TaskExecutionStatus::Succeeded || !*completed
                || !((*completed)->key == geometryKey)) {
                return;
            }
            if (collisionGeometryKey(m_travelPlanCache.key) == geometryKey)
                m_travelCollisionGeometryCache = *completed;
        });
}

void CamModule::scheduleFullEnvironmentVerification(
    const lcnc::cam::ToolpathExportSnapshot& snapshot, bool force)
{
    if ((!snapshot.travelPlan.fullEnvironmentVerificationPending && !force) || !m_machineConfig)
        return;
    auto* tasks = lcnc::Kernel::current().taskManager();
    LcncDocument* machine = machineDocument();
    MachineKinematics* liveKinematics = kinematics();
    // Full-path validation is CAM-owned even without a loaded machine model:
    // the cutter/workpiece proxy pair is still a valid collision environment.
    // A machine document only contributes additional selected axis bodies.
    // 中文翻译：即使未加载机台模型，切割头/工件代理仍须由 CAM 完整校验；机台文档只额外提供被选轴部件。
    if (!tasks || !liveKinematics)
        return;
    const auto collision = collisionConfiguration();
    const auto failBeforeScheduling = [this, &snapshot](const QString& reason) {
        if (!(m_travelPlanCache.key == snapshot.travelPlan.key))
            return;
        m_travelPlanCache.fullEnvironmentVerificationPending = false;
        m_travelPlanCache.collision.state = lcnc::cam::CollisionValidationState::Indeterminate;
        m_travelPlanCache.collision.complete = true;
        m_travelPlanCache.collision.failureReason = reason;
        m_travelPlanCache.failureReason = reason;
        QVector<std::uint64_t> order;
        order.reserve(snapshot.contours.size());
        for (const auto& contour : snapshot.contours)
            order.append(contour.contourId);
        emit contourOrderTravelPlanRebuilt(order);
    };
    if ((!collision.enabled && !force) || !collision.valid) {
        // 中文翻译：碰撞源配置不完整
        failBeforeScheduling(tr("Collision source configuration is incomplete"));
        return;
    }
    QVector<CamTravelCollisionBody> bodies;
    if (machine && machine->shapeTool()) {
        const TDF_LabelSequence labels = machine->entityLabels(LcncDocument::EntityKind::Machine);
        for (int index = 1; index <= labels.Length(); ++index) {
            const TDF_Label& label = labels.Value(index);
            const QString entry = XcafUtils::entry(label);
            const QString source = collisionAxisSourceId(liveKinematics->axisForShape(entry));
            const bool active = collision.activeSources.contains(source);
            const bool passive = collision.passiveSources.contains(source);
            if (!active && !passive)
                continue;
            const TopoDS_Shape shape = machine->shapeTool()->GetShape(label);
            if (!shape.IsNull())
                bodies.append({entry, source, active, passive, false, false, shape});
        }
    }
    if (collision.activeSources.contains(QStringLiteral("cutter"))) {
        QString cutterError;
        const TopoDS_Shape cutter = cutterCollisionProxyShape(&cutterError);
        if (cutter.IsNull()) {
            failBeforeScheduling(cutterError.isEmpty()
                ? tr("Unable to create the cutting nozzle collision proxy") : cutterError);
            return;
        }
        bodies.append({QStringLiteral("__cutter_proxy__"), QStringLiteral("cutter"),
                       true, false, true, false, cutter});
    }
    const bool verifyWorkpiece = collision.passiveSources.contains(QStringLiteral("workpiece"));
    if (verifyWorkpiece && !m_workpieceShape.IsNull()) {
        bodies.append({QStringLiteral("__workpiece__"), QStringLiteral("workpiece"),
                       false, true, false, true, m_workpieceShape});
    }
    if (bodies.isEmpty()) {
        failBeforeScheduling(tr("No selected collision source has usable geometry"));
        return;
    }
    QSet<QString> activeBodySources;
    for (const CamTravelCollisionBody& body : std::as_const(bodies)) {
        if (body.active)
            activeBodySources.insert(body.source);
    }
    const QStringList activeSources = lcnc::cam_algo::orderedActiveCollisionSources(activeBodySources);
    if (activeSources.isEmpty()) {
        failBeforeScheduling(tr("No active collision source has usable geometry"));
        return;
    }

    if (m_travelVerificationTask != kInvalidTaskId)
        tasks->requestAbort(m_travelVerificationTask);
    if (m_collisionDomainPreparationTask != kInvalidTaskId) {
        tasks->requestAbort(m_collisionDomainPreparationTask);
        m_collisionDomainPreparationTask = kInvalidTaskId;
    }

    const auto definition = m_machineConfig->modeDefinition(snapshot.machiningMode);
    const auto axes = lcnc::cam_algo::offlinePlanningAxisBaseline(
        liveKinematics->axes(), definition);
    const bool workpieceProxyMode =
        snapshot.travelPlan.mode == lcnc::cam::TravelPlanningMode::WorkpieceProxy;
    const QString configType = liveKinematics->configType();
    const gp_Trsf workpieceSetup = liveKinematics->workpieceSetupTransform();
    const auto assignments = liveKinematics->shapeAssignments();
    const auto mounts = liveKinematics->wpcMounts();
    const double clearanceMm = m_config.cutterCollisionClearanceMm();
    // The no-machine path is dominated by intentional cutter-tip contact at
    // every cutting node.  Its BVH therefore needs enough precision to prove
    // that the existing 0.05 mm outward contact probe has separated before
    // falling back to serial BRep distance.
    // 中文翻译：非机台模式的大量切割节点都存在正常刀尖接触；使用更精细的表面网格，
    // 使原有 0.05 mm 外移探针可在进入串行 BRep 精确距离前确认已分离。
    const double collisionMeshDeflectionMm = workpieceProxyMode
        ? qBound(0.005, clearanceMm * 0.02, 0.02)
        : qBound(0.05, clearanceMm * 0.1, 0.25);
    const lcnc::cam::TravelPlanKey planKey = snapshot.travelPlan.key;
    QVector<std::uint64_t> verifiedOrder;
    verifiedOrder.reserve(snapshot.contours.size());
    for (const auto& contour : snapshot.contours)
        verifiedOrder.append(contour.contourId);
    // Geometry preparation is independent of contour order, toolpath samples
    // and motion profile. Keeping those out of this key makes a regenerated
    // path reuse the expensive exact bounds and leaf decomposition.
    // 中文翻译：几何准备不依赖轮廓顺序、刀路采样和运动曲线；重生成刀路可复用精确包围盒和叶部件分解。
    const lcnc::cam::TravelPlanKey geometryKey = collisionGeometryKey(planKey);
    const auto result = std::make_shared<QString>();
    const auto cachedGeometry = (m_travelCollisionGeometryCache
                                 && m_travelCollisionGeometryCache->key == geometryKey)
        ? m_travelCollisionGeometryCache : std::shared_ptr<CamTravelCollisionGeometryCache>{};
    const auto completedGeometry = std::make_shared<std::shared_ptr<CamTravelCollisionGeometryCache>>(cachedGeometry);
    const auto completedNodeStates = std::make_shared<QVector<lcnc::cam::CollisionValidationState>>();
    const auto completedIntervals = std::make_shared<QVector<lcnc::cam::CollisionInterval>>();
    const auto completedIndeterminate = std::make_shared<std::atomic_bool>(false);
    const auto completedCollision = std::make_shared<std::atomic_bool>(false);
    const auto completedWarning = std::make_shared<std::atomic_bool>(false);
    TaskSpec spec;
    // This is the final phase of global toolpath generation.  Keep the CAM
    // scope so the task panel presents it as the generation continuation,
    // not as an offline-simulation activity.
    // 中文翻译：这是全局生成刀路的最终阶段；保留 CAM 生成范围，任务面板不会将其显示为仿真任务。
    spec.label = force
        // 中文翻译：校验当前刀路碰撞；全局生成刀路 — 碰撞校验
        ? tr("Validate current toolpath collisions")
        : tr("Generate toolpath globally — collision validation");
    spec.scope = QStringLiteral("cam.toolpath");
    spec.priority = TaskPriority::Normal;
    spec.userVisible = true;
    spec.cancellable = true;
    const auto stageTimer = std::make_shared<QElapsedTimer>();
    stageTimer->start();
    LCNC_INFO(lcnc::LogCode::Generic,
              "stage=cam.collision.validate event=begin force={} nodes={} bodies={} environment_revision={}",
              force, snapshot.motionPlan.nodes.size(), bodies.size(),
              planKey.environmentRevision);
    LCNC_INFO(lcnc::LogCode::Generic,
              "stage=cam.safety_domain.cache event=lookup result={} bodies={} environment_revision={}",
              cachedGeometry ? "hit" : "miss", bodies.size(),
              geometryKey.environmentRevision);
    const TaskId taskId = tasks->run(spec,
        [snapshot, bodies, activeSources, cachedGeometry, completedGeometry, axes, configType, workpieceSetup,
         assignments, mounts,
         definition, workpieceProxyMode, clearanceMm, collisionMeshDeflectionMm, geometryKey, result,
         completedNodeStates, completedIntervals,
         completedIndeterminate, completedCollision, completedWarning](TaskProgress* progress) {
            QElapsedTimer totalTimer;
            totalTimer.start();
            struct WorkItem {
                lcnc::cam::RapidPose pose;
                QString workpieceEntry;
                lcnc::cam::CamMotionPhase phase{lcnc::cam::CamMotionPhase::Rapid};
                int nodeIndex{-1};
            };
            QVector<WorkItem> workItems;
            // Scan every final CAM node beginning with node zero, which is the
            // first contour lead-in/cutting point.  A missing canonical plan
            // is indeterminate; falling back to inter-contour rapids would
            // skip the first contour and certify a different sequence.
            // 中文翻译：碰撞扫描从正式运动计划第 0 节点（首轮廓下刀/切割首点）开始；
            // 正式计划缺失时必须判为不确定，不能退回只扫描轮廓间空程。
            if (snapshot.motionPlan.nodes.isEmpty()) {
                *result = QObject::tr("The canonical CAM motion plan is unavailable for collision validation");
                return;
            }
            for (int nodeIndex = 0; nodeIndex < snapshot.motionPlan.nodes.size(); ++nodeIndex) {
                    const auto& node = snapshot.motionPlan.nodes.at(nodeIndex);
                    const auto contour = std::find_if(snapshot.contours.cbegin(), snapshot.contours.cend(),
                        [&node](const lcnc::cam::ToolpathExportContour& item) {
                            return item.contourId == node.contourId;
                        });
                    if (contour == snapshot.contours.cend()) {
                        *result = QObject::tr("Motion plan references an unknown contour");
                        return;
                    }
                    lcnc::cam::RapidPose pose;
                    pose.kinematicAxes = node.axes;
                    pose.kinematicAxisMask = node.axisMask;
                    pose.tcpX = node.tcpX; pose.tcpY = node.tcpY; pose.tcpZ = node.tcpZ;
                    pose.surfaceNormalX = node.normalX; pose.surfaceNormalY = node.normalY;
                    pose.surfaceNormalZ = node.normalZ;
                    workItems.append({pose, contour->workpieceEntry, node.phase, nodeIndex});
            }
            const int preparationSteps = cachedGeometry ? 0 : bodies.size();
            const int scanSteps = workItems.size() * activeSources.size();
            progress->setRange(0, qMax(1, preparationSteps + scanSteps));
            std::unique_lock<std::timed_mutex> scanLock(
                lcnc::cam_algo::collisionScanExecutionMutex(), std::defer_lock);
            if (!lcnc::cam_algo::acquireCollisionScanExecution(scanLock, [progress]() {
                    return progress->isAbortRequested();
                }))
                return;
            int prepared = 0;
            qint64 geometryPreparationMs = 0;
            std::shared_ptr<CamTravelCollisionGeometryCache> geometry = cachedGeometry;
            if (!geometry) {
                QElapsedTimer geometryTimer;
                geometryTimer.start();
                geometry = std::make_shared<CamTravelCollisionGeometryCache>();
                geometry->key = geometryKey;
                geometry->bodies = bodies;
                geometry->axes = axes;
                geometry->configType = configType;
                geometry->workpieceSetup = workpieceSetup;
                geometry->assignments = assignments;
                geometry->mounts = mounts;
                geometry->definition = definition;
                for (CamTravelCollisionBody& body : geometry->bodies) {
                    if (progress->isAbortRequested())
                        return;
                    buildCollisionGeometry(&body, collisionMeshDeflectionMm);
                    if (body.leaves.isEmpty()) {
                        *result = QObject::tr("Full-machine collision verification geometry is unavailable for %1")
                            .arg(body.source);
                        return;
                    }
                    progress->setValue(++prepared);
                }
                geometryPreparationMs = geometryTimer.elapsed();
            }
            *completedGeometry = geometry;
            if (workItems.isEmpty())
                return;
            completedNodeStates->fill(lcnc::cam::CollisionValidationState::Pending,
                                      workItems.size());
            completedIntervals->clear();

            const int threadCount = qMin(4, qMax(1, OSD_Parallel::NbLogicalProcessors() / 2));
            auto workerKinematics = std::make_shared<std::vector<std::unique_ptr<MachineKinematics>>>();
            workerKinematics->reserve(static_cast<std::size_t>(threadCount));
            for (int worker = 0; worker < threadCount; ++worker) {
                auto kinematics = std::make_unique<MachineKinematics>();
                kinematics->setAxes(axes, configType);
                kinematics->setWorkpieceSetupTransform(workpieceSetup);
                for (auto it = assignments.cbegin(); it != assignments.cend(); ++it)
                    kinematics->assignShape(it.key(), it.value());
                for (auto it = mounts.cbegin(); it != mounts.cend(); ++it)
                    kinematics->mountWorkpiece(it.key(), it.value());
                workerKinematics->push_back(std::move(kinematics));
            }
            const auto failed = std::make_shared<std::atomic_bool>(false);
            const auto resultMutex = std::make_shared<QMutex>();
            const auto progressMutex = std::make_shared<QMutex>();
            const auto finished = std::make_shared<std::atomic_int>(0);
            const auto meshQueries = std::make_shared<std::atomic_uint64_t>(0);
            const auto meshRejected = std::make_shared<std::atomic_uint64_t>(0);
            const auto exactQueries = std::make_shared<std::atomic_uint64_t>(0);
            const auto contactProbeRejected = std::make_shared<std::atomic_uint64_t>(0);
            const auto leafBoundsNs = std::make_shared<std::atomic_uint64_t>(0);
            const auto meshPrefilterNs = std::make_shared<std::atomic_uint64_t>(0);
            const auto exactLockWaitNs = std::make_shared<std::atomic_uint64_t>(0);
            const auto exactOperationNs = std::make_shared<std::atomic_uint64_t>(0);
            struct CollisionPairTiming {
                QString passiveSource;
                std::atomic_uint64_t meshQueries{0};
                std::atomic_uint64_t meshRejected{0};
                std::atomic_uint64_t exactQueries{0};
                std::atomic_uint64_t meshNs{0};
                std::atomic_uint64_t exactLockWaitNs{0};
                std::atomic_uint64_t exactOperationNs{0};
            };
            QElapsedTimer scanTimer;
            scanTimer.start();
            Handle(OSD_ThreadPool) pool = new OSD_ThreadPool(threadCount);
            OSD_ThreadPool::Launcher launcher(*pool, threadCount);
            for (const QString& activeSource : activeSources) {
                if (failed->load() || progress->isAbortRequested())
                    break;
                QHash<QString, std::shared_ptr<CollisionPairTiming>> pairTimings;
                for (const CamTravelCollisionBody& body : geometry->bodies) {
                    if (body.passive && body.source != activeSource
                        && !pairTimings.contains(body.source)) {
                        auto timing = std::make_shared<CollisionPairTiming>();
                        timing->passiveSource = body.source;
                        pairTimings.insert(body.source, std::move(timing));
                    }
                }
                QElapsedTimer phaseTimer;
                phaseTimer.start();
                launcher.Perform(0, workItems.size(), [geometry, workItems, workerKinematics, axes, definition,
                    workpieceProxyMode,
                    activeSource, clearanceMm, result, resultMutex, progressMutex, progress, finished,
                    preparationSteps, scanSteps, failed, completedNodeStates,
                    completedIndeterminate, completedCollision, completedWarning,
                    completedIntervals, meshQueries, meshRejected,
                    exactQueries, contactProbeRejected, leafBoundsNs, meshPrefilterNs,
                    exactLockWaitNs, exactOperationNs,
                    pairTimings](int threadIndex, int itemIndex) {
                if (failed->load() || progress->isAbortRequested())
                    return;
                MachineKinematics& kinematics = *workerKinematics->at(static_cast<std::size_t>(threadIndex));
                const WorkItem& item = workItems.at(itemIndex);
                lcnc::cam_algo::applyOfflineMotionPose(
                    &kinematics, axes, definition.interpolatedAxes,
                    item.pose.kinematicAxes, item.pose.kinematicAxisMask);
                struct PoseLeaf {
                    const CamTravelCollisionLeaf* leaf{nullptr};
                    Bnd_Box aabb;
                    Bnd_OBB obb;
                };
                struct PoseBody {
                    const CamTravelCollisionBody* body{nullptr};
                    gp_Trsf transform;
                    Bnd_Box aabb;
                    Bnd_OBB obb;
                    QVector<PoseLeaf> leaves;
                    bool leavesReady{false};
                    std::shared_ptr<CollisionPairTiming> timing;
                };
                QVector<PoseBody> activeBodies;
                QVector<PoseBody> passiveBodies;
                for (const CamTravelCollisionBody& body : geometry->bodies) {
                    gp_Trsf transform;
                    if (body.cutterProxy) {
                        gp_Vec normal(item.pose.surfaceNormalX, item.pose.surfaceNormalY, item.pose.surfaceNormalZ);
                        if (normal.SquareMagnitude() <= Precision::SquareConfusion()) normal = gp_Vec(0.0, 0.0, 1.0);
                        normal.Normalize();
                        const gp_Pnt tcp(item.pose.tcpX, item.pose.tcpY, item.pose.tcpZ);
                        transform.SetDisplacement(gp_Ax3(), gp_Ax3(tcp, gp_Dir(normal)));
                    } else if (body.workpiece) {
                        transform = kinematics.computeWpcTransform(item.workpieceEntry);
                    } else {
                        transform = kinematics.computeShapeTransform(body.entry);
                    }
                    PoseBody poseBody{&body, transform,
                        transformCollisionAabb(body.localAabb, transform, clearanceMm),
                        transformCollisionObb(body.localObb, transform, clearanceMm)};
                    if (body.active && body.source == activeSource) activeBodies.append(poseBody);
                    if (body.passive) {
                        poseBody.timing = pairTimings.value(body.source);
                        passiveBodies.append(poseBody);
                    }
                }
                const auto fail = [&result, &resultMutex, &failed,
                                   &completedIndeterminate](const QString& message) {
                    bool expected = false;
                    if (failed->compare_exchange_strong(expected, true)) {
                        completedIndeterminate->store(true);
                        QMutexLocker lock(resultMutex.get());
                        *result = message;
                    }
                };
                const auto recordFinding = [&result, &resultMutex, completedCollision,
                                               completedWarning, completedNodeStates,
                                               completedIntervals, &item](
                                                   lcnc::cam::CollisionValidationState state,
                                                   const QString& message,
                                                   const CamTravelCollisionBody& active,
                                                   const CamTravelCollisionBody& passive,
                                                   double distanceMm) {
                    if (state == lcnc::cam::CollisionValidationState::Collision)
                        completedCollision->store(true);
                    else if (state == lcnc::cam::CollisionValidationState::Warning)
                        completedWarning->store(true);
                    QMutexLocker lock(resultMutex.get());
                    if (item.nodeIndex >= 0 && item.nodeIndex < completedNodeStates->size()) {
                        auto& nodeState = completedNodeStates->operator[](item.nodeIndex);
                        if (state == lcnc::cam::CollisionValidationState::Collision
                            || nodeState != lcnc::cam::CollisionValidationState::Collision) {
                            nodeState = state;
                        }
                    }
                    completedIntervals->append({item.nodeIndex, item.nodeIndex,
                        state,
                        active.source, passive.source, active.entry, passive.entry,
                        distanceMm, message});
                    if (result->isEmpty())
                        *result = message;
                };
                const auto prepareLeafBounds = [clearanceMm, leafBoundsNs](PoseBody* poseBody) {
                    if (!poseBody || poseBody->leavesReady || !poseBody->body)
                        return;
                    const auto started = std::chrono::steady_clock::now();
                    poseBody->leaves.reserve(poseBody->body->leaves.size());
                    for (const CamTravelCollisionLeaf& leaf : poseBody->body->leaves) {
                        poseBody->leaves.append({&leaf,
                            transformCollisionAabb(leaf.localAabb, poseBody->transform, clearanceMm),
                            transformCollisionObb(leaf.localObb, poseBody->transform, clearanceMm)});
                    }
                    poseBody->leavesReady = true;
                    leafBoundsNs->fetch_add(static_cast<std::uint64_t>(
                        std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now() - started).count()),
                        std::memory_order_relaxed);
                };
                bool itemCollided = false;
                for (PoseBody& active : activeBodies) {
                    if (itemCollided) break;
                    for (PoseBody& passive : passiveBodies) {
                    if (itemCollided) break;
                    if (failed->load() || progress->isAbortRequested()) return;
                    // Source-internal pieces are one rigid collision source and are
                    // explicitly excluded even when an axis owns many labels.
                    if (active.body->source == passive.body->source)
                        continue;
                    lcnc::cam_algo::CollisionSafetyDomainQuery domainQuery;
                    domainQuery.pose = item.pose;
                    domainQuery.workpieceEntry = item.workpieceEntry;
                    domainQuery.phase = item.phase;
                    domainQuery.activeSource = active.body->source;
                    domainQuery.passiveSource = passive.body->source;
                    domainQuery.clearanceMm = clearanceMm;
                    lcnc::cam_algo::CollisionSafetyDomainSample domainSample;
                    if (geometry->safetyDomain
                        && geometry->safetyDomain->lookup(domainQuery, &domainSample)) {
                        if (domainSample.state
                            != lcnc::cam::CollisionValidationState::Safe) {
                            recordFinding(domainSample.state,
                                domainSample.reason, *active.body, *passive.body,
                                domainSample.minimumDistanceMm);
                            itemCollided = true;
                        }
                        continue;
                    }
                    domainSample.state = lcnc::cam::CollisionValidationState::Safe;
                    domainSample.activeEntity = active.body->entry;
                    domainSample.passiveEntity = passive.body->entry;
                    domainSample.minimumDistanceMm =
                        std::numeric_limits<double>::infinity();
                    if (active.aabb.IsOut(passive.aabb)
                        || active.obb.IsOut(passive.obb)) {
                        if (geometry->safetyDomain)
                            geometry->safetyDomain->store(domainQuery, domainSample);
                        continue;
                    }
                    // Query the immutable surface BVHs before enumerating BRep
                    // face pairs. Discrete poses remain fully parallel here.
                    // The guard includes both tessellation errors, so only a
                    // conservative "definitely separated" answer skips exact
                    // validation.
                    // 中文翻译：先并行查询只读表面 BVH；搜索距离包含两侧网格误差，
                    // 只有保守确认分离时才跳过精确校验。
                    if (active.body->surfaceModel.isValid()
                        && passive.body->surfaceModel.isValid()) {
                        const auto runPrefilter = [&](const gp_Trsf& activeTransform,
                                                      double searchDistanceMm) {
                            meshQueries->fetch_add(1, std::memory_order_relaxed);
                            if (passive.timing) {
                                passive.timing->meshQueries.fetch_add(
                                    1, std::memory_order_relaxed);
                            }
                            const auto started = std::chrono::steady_clock::now();
                            const auto prefilter = lcnc::cam_algo::prefilterSurfaceCollision(
                                active.body->surfaceModel, activeTransform,
                                passive.body->surfaceModel, passive.transform,
                                searchDistanceMm);
                            const auto elapsedNs = static_cast<std::uint64_t>(
                                std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::steady_clock::now() - started).count());
                            meshPrefilterNs->fetch_add(elapsedNs, std::memory_order_relaxed);
                            if (passive.timing)
                                passive.timing->meshNs.fetch_add(
                                    elapsedNs, std::memory_order_relaxed);
                            return prefilter;
                        };
                        const auto rejectByMesh = [&]() {
                            meshRejected->fetch_add(1, std::memory_order_relaxed);
                            if (passive.timing) {
                                passive.timing->meshRejected.fetch_add(
                                    1, std::memory_order_relaxed);
                            }
                        };
                        // In workpiece-proxy mode a cutting TCP is expected to
                        // touch the workpiece at the nozzle tip.  Mirror the
                        // existing exact-path contact probe in the BVH first:
                        // if a 0.05 mm outward move is conservatively separated
                        // by more than both mesh errors, this is normal tip
                        // contact and no serialized OCCT call is necessary.
                        // A body penetration remains near/intersecting after
                        // the probe and still falls through to exact BRep.
                        // 中文翻译：非机台模式先用表面 BVH 执行原有刀尖外移探针；
                        // 明确分离即为正常刀尖接触，刀体侵入仍进入 BRep 精确复检。
                        if (workpieceProxyMode
                            && active.body->cutterProxy && passive.body->workpiece
                            && item.phase == lcnc::cam::CamMotionPhase::Cutting) {
                            gp_Vec outward(item.pose.surfaceNormalX,
                                           item.pose.surfaceNormalY,
                                           item.pose.surfaceNormalZ);
                            if (outward.SquareMagnitude() <= Precision::SquareConfusion())
                                outward = gp_Vec(0.0, 0.0, 1.0);
                            outward.Normalize();
                            outward.Multiply(qMax(
                                0.05, Precision::Confusion() * 10.0));
                            gp_Trsf outwardTranslation;
                            outwardTranslation.SetTranslation(outward);
                            const gp_Trsf probeTransform =
                                outwardTranslation.Multiplied(active.transform);
                            const double probeGuardMm =
                                active.body->surfaceModel.linearDeflectionMm()
                                + passive.body->surfaceModel.linearDeflectionMm()
                                + Precision::Confusion();
                            const auto probe = runPrefilter(
                                probeTransform, probeGuardMm);
                            if (probe.valid && probe.definitelySeparated) {
                                contactProbeRejected->fetch_add(
                                    1, std::memory_order_relaxed);
                                rejectByMesh();
                                domainSample.minimumDistanceMm = probe.meshDistanceMm;
                                if (geometry->safetyDomain)
                                    geometry->safetyDomain->store(domainQuery, domainSample);
                                continue;
                            }
                        }
                        const double meshGuardMm = clearanceMm
                            + active.body->surfaceModel.linearDeflectionMm()
                            + passive.body->surfaceModel.linearDeflectionMm();
                        const auto prefilter = runPrefilter(
                            active.transform, meshGuardMm);
                        if (prefilter.valid && prefilter.definitelySeparated) {
                            rejectByMesh();
                            domainSample.minimumDistanceMm = prefilter.meshDistanceMm;
                            if (geometry->safetyDomain)
                                geometry->safetyDomain->store(domainQuery, domainSample);
                            continue;
                        }
                    }
                    prepareLeafBounds(&active);
                    prepareLeafBounds(&passive);
                    for (const PoseLeaf& activeLeaf : active.leaves) {
                        if (itemCollided) break;
                        for (const PoseLeaf& passiveLeaf : passive.leaves) {
                            if (itemCollided) break;
                            if (!activeLeaf.leaf || !passiveLeaf.leaf
                                || activeLeaf.aabb.IsOut(passiveLeaf.aabb)
                                || activeLeaf.obb.IsOut(passiveLeaf.obb))
                                continue;
                            // Rigid machine poses only need a TopLoc location;
                            // avoid running BRepBuilderAPI_Transform for every
                            // candidate face pair.
                            const TopoDS_Shape first = activeLeaf.leaf->shape.Moved(
                                TopLoc_Location(active.transform));
                            const TopoDS_Shape second = passiveLeaf.leaf->shape.Moved(
                                TopLoc_Location(passive.transform));
                            if (first.IsNull() || second.IsNull()) {
                                fail(QObject::tr("Full-machine collision verification cannot transform %1 and %2")
                                    .arg(active.body->source, passive.body->source));
                                break;
                            }
                            // ShapeProximity is deliberately absent.  It
                            // corrupts its NCollection buffers in this OCCT
                            // runtime even for one guarded call.  The stable
                            // path is face AABB/OBB filtering followed by an
                            // exact distance call under the process-wide gate.
                            // 中文翻译：禁用会损坏 NCollection 缓冲区的 ShapeProximity；
                            // 面级包围盒筛选后，仅将精确距离放入进程级安全区。
                            double minimumDistance = std::numeric_limits<double>::infinity();
                            const auto lockStarted = std::chrono::steady_clock::now();
                            {
                                lcnc::OcctExactOperationLock exactOperationLock;
                                const auto operationStarted = std::chrono::steady_clock::now();
                                const auto lockWaitElapsedNs = static_cast<std::uint64_t>(
                                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        operationStarted - lockStarted).count());
                                exactLockWaitNs->fetch_add(lockWaitElapsedNs,
                                    std::memory_order_relaxed);
                                if (passive.timing)
                                    passive.timing->exactLockWaitNs.fetch_add(
                                        lockWaitElapsedNs, std::memory_order_relaxed);
                                exactQueries->fetch_add(1, std::memory_order_relaxed);
                                if (passive.timing)
                                    passive.timing->exactQueries.fetch_add(1, std::memory_order_relaxed);
                                BRepExtrema_DistShapeShape distance(first, second);
                                distance.SetDeflection(0.025);
                                distance.SetMultiThread(Standard_False);
                                distance.Perform();
                                if (!distance.IsDone()) {
                                    fail(QObject::tr("Full-machine collision verification failed between %1 and %2")
                                        .arg(active.body->source, passive.body->source));
                                    return;
                                }
                                minimumDistance = distance.Value();
                                // A cutting TCP is expected to touch the workpiece
                                // at its tip.  Probe a tiny distance outwards only
                                // for that cutter/workpiece cutting-node pair: a
                                // tangent tip contact separates, while a body
                                // penetration remains a confirmed collision.
                                if (minimumDistance <= Precision::Confusion()
                                    && active.body->cutterProxy && passive.body->workpiece
                                    && item.phase == lcnc::cam::CamMotionPhase::Cutting) {
                                    gp_Vec outward(item.pose.surfaceNormalX, item.pose.surfaceNormalY,
                                                   item.pose.surfaceNormalZ);
                                    if (outward.SquareMagnitude() <= Precision::SquareConfusion())
                                        outward = gp_Vec(0.0, 0.0, 1.0);
                                    outward.Normalize();
                                    outward.Multiply(qMax(0.05, Precision::Confusion() * 10.0));
                                    gp_Trsf probeTransform;
                                    probeTransform.SetTranslation(outward);
                                    const TopoDS_Shape probe = first.Moved(
                                        TopLoc_Location(probeTransform));
                                    BRepExtrema_DistShapeShape probeDistance(probe, second);
                                    probeDistance.SetDeflection(0.025);
                                    probeDistance.SetMultiThread(Standard_False);
                                    probeDistance.Perform();
                                    if (probeDistance.IsDone()
                                        && probeDistance.Value() > Precision::Confusion())
                                        minimumDistance = clearanceMm + Precision::Confusion();
                                }
                                const auto exactElapsedNs = static_cast<std::uint64_t>(
                                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::steady_clock::now() - operationStarted).count());
                                exactOperationNs->fetch_add(exactElapsedNs,
                                    std::memory_order_relaxed);
                                if (passive.timing)
                                    passive.timing->exactOperationNs.fetch_add(
                                        exactElapsedNs, std::memory_order_relaxed);
                            }
                            if (minimumDistance <= clearanceMm) {
                                const auto state = lcnc::cam_algo::classifyCollisionDistance(
                                    minimumDistance, clearanceMm, Precision::Confusion());
                                const QString message = state == lcnc::cam::CollisionValidationState::Collision
                                    ? QObject::tr("Full-machine collision verification failed between %1 and %2")
                                        .arg(active.body->source, passive.body->source)
                                    : QObject::tr("Full-machine collision clearance warning between %1 and %2")
                                        .arg(active.body->source, passive.body->source);
                                domainSample.state = state;
                                domainSample.minimumDistanceMm = minimumDistance;
                                domainSample.reason = message;
                                if (geometry->safetyDomain)
                                    geometry->safetyDomain->store(domainQuery, domainSample);
                                recordFinding(state, message, *active.body,
                                              *passive.body, minimumDistance);
                                itemCollided = true;
                                break;
                            }
                            domainSample.minimumDistanceMm = std::min(
                                domainSample.minimumDistanceMm, minimumDistance);
                        }
                    }
                    if (!itemCollided && geometry->safetyDomain)
                        geometry->safetyDomain->store(domainQuery, domainSample);
                }
                }
                const int done = finished->fetch_add(1) + 1;
                if ((done & 0x0f) == 0 || done == scanSteps) {
                    QMutexLocker lock(progressMutex.get());
                    progress->setValue(preparationSteps + done);
                }
                });
                for (const auto& timing : std::as_const(pairTimings)) {
                    LCNC_INFO(lcnc::LogCode::Generic,
                              "CAM collision pair: active={}, passive={}, mesh_queries={}, mesh_rejected={}, exact_queries={}, mesh_ms={}, exact_lock_wait_ms={}, exact_ms={}, phase_ms={}",
                              activeSource.toStdString(), timing->passiveSource.toStdString(),
                              timing->meshQueries.load(), timing->meshRejected.load(),
                              timing->exactQueries.load(), timing->meshNs.load() / 1000000,
                              timing->exactLockWaitNs.load() / 1000000,
                              timing->exactOperationNs.load() / 1000000,
                              phaseTimer.elapsed());
                }
            }
            if (!progress->isAbortRequested() && !failed->load()) {
                for (auto& state : *completedNodeStates) {
                    if (state == lcnc::cam::CollisionValidationState::Pending)
                        state = lcnc::cam::CollisionValidationState::Safe;
                }
                progress->setValue(preparationSteps + scanSteps);
            }
            LCNC_INFO(lcnc::LogCode::Generic,
                      "CAM collision scan: mode={}, nodes={}, sources={}, mesh_queries={}, mesh_rejected={}, contact_probe_rejected={}, exact_queries={}, geometry_ms={}, leaf_bounds_ms={}, mesh_ms={}, exact_lock_wait_ms={}, exact_ms={}, scan_ms={}, total_ms={}",
                      workpieceProxyMode ? "workpiece_proxy" : "full_environment",
                      workItems.size(), activeSources.size(), meshQueries->load(),
                      meshRejected->load(), contactProbeRejected->load(),
                      exactQueries->load(), geometryPreparationMs,
                      leafBoundsNs->load() / 1000000, meshPrefilterNs->load() / 1000000,
                      exactLockWaitNs->load() / 1000000, exactOperationNs->load() / 1000000,
                      scanTimer.elapsed(), totalTimer.elapsed());
            if (geometry->safetyDomain) {
                const auto domainStats = geometry->safetyDomain->statistics();
                LCNC_INFO(lcnc::LogCode::Generic,
                          "CAM collision safety domain: samples={}, hits={}, misses={}, stores={}",
                          domainStats.samples, domainStats.hits,
                          domainStats.misses, domainStats.stores);
            }
        });
    m_travelVerificationTask = taskId;
    m_taskScope.track(taskId);
    QObject::connect(tasks, &TaskManager::taskFinishedDetailed, this,
        [this, tasks, taskId, planKey, geometryKey, verifiedOrder, result, completedGeometry,
         completedNodeStates, completedIntervals, completedIndeterminate,
         completedCollision, completedWarning, stageTimer](TaskId id,
                                                            TaskExecutionStatus status,
                                                            const QString&) {
            if (id != taskId)
                return;
            const auto stageLog = qScopeGuard([&] {
                LCNC_INFO(lcnc::LogCode::Generic,
                          "stage=cam.collision.validate event=end task_status={} collision={} warning={} indeterminate={} findings={} elapsed_ms={} reason='{}'",
                          static_cast<int>(status), completedCollision->load(),
                          completedWarning->load(), completedIndeterminate->load(),
                          completedIntervals->size(), stageTimer->elapsed(),
                          result->toStdString());
                lcnc::cam_algo::CollisionSafetyDomainStatistics statistics;
                const bool ready = *completedGeometry
                    && (*completedGeometry)->safetyDomain;
                if (ready)
                    statistics = (*completedGeometry)->safetyDomain->statistics();
                LCNC_INFO(lcnc::LogCode::Generic,
                          "stage=cam.safety_domain.cache event=end result={} samples={} hits={} misses={} stores={} elapsed_ms={}",
                          ready ? "ready" : "unavailable", statistics.samples,
                          statistics.hits, statistics.misses, statistics.stores,
                          stageTimer->elapsed());
            });
            m_taskScope.release(taskId);
            if (m_travelVerificationTask != taskId)
                return;
            m_travelVerificationTask = kInvalidTaskId;
            if (!(m_travelPlanCache.key == planKey) || !m_travelPlanCache.fullEnvironmentVerificationPending)
                return;
            if (status != TaskExecutionStatus::Succeeded) {
                // Keep the plan pending (and therefore Process-blocked) when
                // a task is cancelled or a geometry algorithm cannot finish.
                m_travelPlanCache.fullEnvironmentVerificationPending = false;
                m_travelPlanCache.collision.state = lcnc::cam::CollisionValidationState::Indeterminate;
                m_travelPlanCache.collision.complete = true;
                m_travelPlanCache.collision.failureReason = tr("Full-path collision validation was cancelled or failed");
                m_travelPlanCache.failureReason = m_travelPlanCache.collision.failureReason;
                emit contourOrderTravelPlanRebuilt(verifiedOrder);
                return;
            }
            m_travelPlanCache.fullEnvironmentVerificationPending = false;
            if (*completedGeometry && (*completedGeometry)->key == geometryKey)
                m_travelCollisionGeometryCache = *completedGeometry;
            const auto finalState = completedIndeterminate->load()
                ? lcnc::cam::CollisionValidationState::Indeterminate
                : (completedCollision->load()
                    ? lcnc::cam::CollisionValidationState::Collision
                    : (completedWarning->load()
                        ? lcnc::cam::CollisionValidationState::Warning
                        : lcnc::cam::CollisionValidationState::Safe));
            m_travelPlanCache.collision.state = finalState;
            m_travelPlanCache.collision.complete = true;
            m_travelPlanCache.collision.failureReason = *result;
            m_travelPlanCache.collision.nodeStates = *completedNodeStates;
            m_travelPlanCache.collision.intervals = *completedIntervals;
            if (finalState == lcnc::cam::CollisionValidationState::Collision
                || finalState == lcnc::cam::CollisionValidationState::Indeterminate
                || (finalState == lcnc::cam::CollisionValidationState::Warning
                    && m_travelPlanCache.collision.blockWarning)) {
                m_travelPlanCache.failureReason = *result;
            }
            emit contourOrderTravelPlanRebuilt(verifiedOrder);
            refreshTravelPath();
        });
}

bool CamModule::refreshCutterCollisionConfiguration()
{
    QString proxyError;
    TopoDS_Shape updated = lcnc::cam::buildCutterCollisionProxy(m_config, &proxyError);
    if (updated.IsNull()) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "cam.collisionProxy: failed to apply collision proxy: {}",
                 proxyError.toStdString());
        emit operationFailed(tr("Cutting nozzle collision proxy"), proxyError);
        return false;
    }
    m_cutterCollisionProxyShape = std::move(updated);
    m_travelPlanCache.stale = true;
    m_travelCollisionGeometryCache.reset();
    if (m_guideRenderer)
        m_guideRenderer->setCutterCollisionProxy(m_cutterCollisionProxyShape);
    displayAxisGuides();
    emit cutterCollisionConfigurationChanged();
    refreshTravelPath();
    return true;
}

TopoDS_Shape CamModule::cutterCollisionProxyShape(QString* errorMessage)
{
    if (!m_cutterCollisionProxyShape.IsNull())
        return m_cutterCollisionProxyShape;

    QString proxyError;
    TopoDS_Shape proxy = lcnc::cam::buildCutterCollisionProxy(m_config, &proxyError);
    if (proxy.IsNull()) {
        if (errorMessage)
            *errorMessage = proxyError;
        return {};
    }
    m_cutterCollisionProxyShape = proxy;
    return m_cutterCollisionProxyShape;
}
