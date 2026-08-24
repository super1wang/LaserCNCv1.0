
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
#include "modules/cam/collision/coal_collision_backend.h"
#include "modules/cam/collision/continuous_motion_certificate_builder.h"
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
#include "core/algorithms/cam/machine_motion_certificate.h"
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
#include <QCryptographicHash>
#include <QDateTime>
#include <QFile>
#include <QFileInfo>
#include <QMutex>
#include <QMutexLocker>
#include <QPoint>
#include <QPointer>
#include <QScopeGuard>
#include <QSaveFile>
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
using lcnc::cam::buildCoalCollisionPairs;
using lcnc::cam::collisionAxisSourceId;
using lcnc::cam::collisionGeometryKey;
using lcnc::cam::machineCollisionAxisPairKeys;
using lcnc::cam::machineCollisionAxisSources;
using lcnc::cam::transformCollisionAabb;
using lcnc::cam::transformCollisionObb;

void appendMachineSafetyHotPoseFeedback(
    const QString& packagePath,
    const lcnc::cam::ToolpathExportSnapshot& snapshot,
    const lcnc::cam_algo::MachineSafetyIndex& index,
    const QVector<lcnc::cam::CamMotionEdgeCertificate>& certificates)
{
    if (packagePath.isEmpty() || snapshot.machineAxisLayout.count <= 0)
        return;
    static QMutex feedbackMutex;
    QMutexLocker lock(&feedbackMutex);
    const QString feedbackPath = packagePath + QStringLiteral(".hot_apos.txt");
    QSet<QByteArray> lines;
    QFile existing(feedbackPath);
    if (existing.open(QIODevice::ReadOnly)) {
        for (QByteArray line : existing.readAll().split('\n')) {
            line = line.trimmed();
            if (!line.isEmpty())
                lines.insert(line);
        }
    }
    const auto appendNode = [&](int nodeIndex) {
        if (nodeIndex < 0 || nodeIndex >= snapshot.motionPlan.nodes.size())
            return;
        const auto& node = snapshot.motionPlan.nodes.at(nodeIndex);
        QByteArray line;
        for (int indexAxis = 0; indexAxis < index.axes().size(); ++indexAxis) {
            const QString& axisName = index.axes().at(indexAxis).name;
            int layoutIndex = -1;
            for (int candidate = 0;
                 candidate < snapshot.machineAxisLayout.count; ++candidate) {
                if (snapshot.machineAxisLayout.axes[candidate].name.compare(
                        axisName, Qt::CaseInsensitive) == 0) {
                    layoutIndex = candidate;
                    break;
                }
            }
            if (layoutIndex < 0 || !(node.axisMask & (1u << layoutIndex)))
                return;
            if (!line.isEmpty())
                line.append(',');
            line.append(axisName.toUtf8());
            line.append('=');
            line.append(QByteArray::number(node.axes[layoutIndex], 'g', 17));
        }
        if (!line.isEmpty())
            lines.insert(std::move(line));
    };
    for (const auto& certificate : certificates) {
        if (certificate.state
            != lcnc::cam::CamMotionCertificateState::BoundaryUnknown) {
            continue;
        }
        appendNode(certificate.firstNode);
        appendNode(certificate.lastNode);
    }
    if (lines.isEmpty())
        return;
    QList<QByteArray> ordered = lines.values();
    std::sort(ordered.begin(), ordered.end());
    constexpr qsizetype kMaximumFeedbackPoses = 4096;
    if (ordered.size() > kMaximumFeedbackPoses)
        ordered = ordered.sliced(ordered.size() - kMaximumFeedbackPoses);
    QSaveFile output(feedbackPath);
    if (!output.open(QIODevice::WriteOnly))
        return;
    for (const QByteArray& line : std::as_const(ordered)) {
        if (output.write(line) != line.size() || output.write("\n", 1) != 1) {
            output.cancelWriting();
            return;
        }
    }
    output.commit();
}

QSet<QString> workpieceRigidAxisChain(const MachineKinematics& machine)
{
    QSet<QString> result{QStringLiteral("BASE")};
    for (auto mount = machine.wpcMounts().cbegin();
         mount != machine.wpcMounts().cend(); ++mount) {
        QString axisName = mount.value().trimmed().toUpper();
        while (!axisName.isEmpty() && !result.contains(axisName)) {
            result.insert(axisName);
            const auto found = std::find_if(
                machine.axes().cbegin(), machine.axes().cend(),
                [&axisName](const MachineAxisDef& axis) {
                    return axis.name.compare(axisName, Qt::CaseInsensitive) == 0;
                });
            if (found == machine.axes().cend())
                break;
            axisName = found->parentAxis.trimmed().toUpper();
        }
    }
    return result;
}
} // namespace

lcnc::cam::CollisionConfigurationSnapshot CamModule::collisionConfiguration() const
{
    lcnc::cam::CollisionConfigurationSnapshot snapshot;
    snapshot.machineProfilePath = activeMachineProfilePath();
    const auto package = m_machineSafetyPackageManager.status();
    const bool machineLoaded = !m_loadedMachineModelPath.isEmpty();
    snapshot.activationAvailable = lcnc::cam_algo::canActivateCollisionDetection(
        machineLoaded, package.executionEligible(), package.buildInProgress);
    if (!machineLoaded) {
        // 中文翻译：必须先加载机台安全包，才能启用碰撞检测。
        snapshot.activationFailureReason = tr(
            "Load a machine safety package before enabling collision detection.");
    } else if (package.buildInProgress) {
        // 中文翻译：机台安全包正在构建；构建完成并重新验证前无法启用碰撞检测。
        snapshot.activationFailureReason = tr(
            "The machine safety package is being built; collision detection cannot be enabled until it has been validated.");
    } else if (!package.executionEligible()) {
        // 中文翻译：当前机台没有有效安全包；请先生成或加载有效的 .lmsp 包。
        snapshot.activationFailureReason = package.reason.isEmpty()
            ? tr("The loaded machine has no valid safety package; generate or load a valid .lmsp package before enabling collision detection.")
            : package.reason;
    }
    // Keep the operator's global intent independent from runtime readiness.
    // If a previously enabled package/overlay starts rebuilding, Process must
    // see enabled=true plus ready=false and fail closed.  Collapsing those two
    // facts into enabled=false would silently enter the non-collision workflow.
    // 中文翻译：全局碰撞意图与运行时就绪状态分离；构建中必须失败关闭。
    snapshot.enabled = m_config.collisionDetectionEnabledForMachine(
        snapshot.machineProfilePath);
    // Collision roles are no longer operator-selected. The machine package
    // owns every fixed Machine/Machine pair, while the Job Overlay checks the
    // current workpiece only against machine bodies outside its rigid mount
    // chain. The display cone/nozzle never enters this source list.
    // 中文翻译：碰撞角色由固定机台装配自动派生；示意锥头/喷嘴不进入安全源列表。
    snapshot.activeSources.clear();
    snapshot.passiveSources = {QStringLiteral("workpiece")};
    snapshot.revision = m_collisionConfigurationRevision;

    int workpieceCount = 0;
    if (LcncDocument* document = workpieceDocument())
        workpieceCount = document->entityLabels(LcncDocument::EntityKind::Workpiece).Length();
    snapshot.sources.append({QStringLiteral("workpiece"), tr("Workpiece"),
                             lcnc::cam::CollisionSemanticRole::Workpiece,
                             false, true, workpieceCount > 0, workpieceCount});

    const MachineKinematics* machine = kinematics();
    if (machine) {
        const QSet<QString> rigidAxes = workpieceRigidAxisChain(*machine);
        QSet<QString> assigned;
        for (auto it = machine->shapeAssignments().cbegin(); it != machine->shapeAssignments().cend(); ++it)
            assigned.insert(it.key());
        int machineBodies = 0;
        if (LcncDocument* document = machineDocument())
            machineBodies = document->entityLabels(LcncDocument::EntityKind::Machine).Length();
        snapshot.unassignedMachineBodyCount = qMax(0, machineBodies - assigned.size());

        for (const MachineAxisDef& axis : machine->axes()) {
            const int bodyCount = machine->shapesForAxis(axis.name).size();
            const bool active = bodyCount > 0
                && !rigidAxes.contains(axis.name.trimmed().toUpper());
            if (active)
                snapshot.activeSources.insert(collisionAxisSourceId(axis.name));
            snapshot.sources.append({collisionAxisSourceId(axis.name),
                                     axis.name == QStringLiteral("BASE")
                                         ? tr("BASE (static frame)")
                                         : tr("Axis %1").arg(axis.name),
                                     axis.name == QStringLiteral("BASE")
                                         ? lcnc::cam::CollisionSemanticRole::StaticFrame
                                         : lcnc::cam::CollisionSemanticRole::MovingPart,
                                     active, false, bodyCount > 0, bodyCount});
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
    snapshot.valid = snapshot.unassignedMachineBodyCount == 0
        && selectedAvailable(snapshot.activeSources, true)
        && selectedAvailable(snapshot.passiveSources, false);
    return snapshot;
}

lcnc::cam::CollisionSafetyDomainSnapshot CamModule::collisionSafetyDomain() const
{
    std::shared_ptr<CamTravelCollisionGeometryCache> geometry;
    const auto capture = [this, &geometry] {
        geometry = m_jobSafetyOverlayManager.geometry();
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
    std::shared_ptr<lcnc::cam_algo::MachineSafetyIndex> machineSafetyIndex;
    QByteArray machineSafetyPackageKey;
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
    const auto policy = lcnc::cam_algo::CollisionPolicyMatrix::policy(
        request.purpose,
        request.scope == lcnc::cam::CollisionSafetyScope::MachineOnly);
    if (request.poses.isEmpty() || !std::isfinite(request.clearanceMm)
        || request.clearanceMm < 0.0) {
        result.state = lcnc::cam::CollisionValidationState::Indeterminate;
        // 中文翻译：碰撞安全域路径请求无效
        result.failureReason = tr("Collision safety-domain path request is invalid");
        return result;
    }

    const auto capture = [this, &geometry, &machineSafetyIndex,
                          &machineSafetyPackageKey] {
        geometry = m_jobSafetyOverlayManager.geometry();
        const auto package = m_machineSafetyPackageManager.runtimeSnapshot();
        machineSafetyIndex = package.index;
        machineSafetyPackageKey = package.status.packageKeySha256;
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

    if (request.scope == lcnc::cam::CollisionSafetyScope::MachineOnly
        && machineSafetyIndex && machineSafetyIndex->isValid()
        && request.clearanceMm <= machineSafetyIndex->clearanceMm()) {
        QVector<lcnc::cam_algo::MachineSafetyPose> indexedPoses;
        indexedPoses.reserve(request.poses.size());
        bool allAxesAvailable = true;
        for (const auto& item : request.poses) {
            lcnc::cam_algo::MachineSafetyPose pose;
            pose.count = static_cast<std::uint8_t>(machineSafetyIndex->axes().size());
            for (int axisIndex = 0; axisIndex < machineSafetyIndex->axes().size();
                 ++axisIndex) {
                const int physicalIndex = geometry->definition.interpolatedAxes.indexOfName(
                    machineSafetyIndex->axes().at(axisIndex).name);
                if (physicalIndex < 0
                    || (item.pose.kinematicAxisMask & (1u << physicalIndex)) == 0) {
                    allAxesAvailable = false;
                    break;
                }
                pose.values[axisIndex] = item.pose.kinematicAxes[physicalIndex];
            }
            if (!allAxesAvailable)
                break;
            indexedPoses.append(pose);
        }
        if (allAxesAvailable) {
            result.nodeStates.fill(lcnc::cam::CollisionValidationState::Safe,
                                   request.poses.size());
            bool allCertified = true;
            if (indexedPoses.size() == 1) {
                const auto query = machineSafetyIndex->query(indexedPoses.constFirst());
                if (query.state == lcnc::cam_algo::MachineSafetyIndexState::CollisionSample) {
                    result.state = lcnc::cam::CollisionValidationState::Collision;
                    result.nodeStates[0] = result.state;
                    result.failureReason = tr("The machine safety package blocks this pose");
                    return result;
                }
                allCertified = query.state
                    == lcnc::cam_algo::MachineSafetyIndexState::CertifiedSafe;
            } else {
                lcnc::cam_algo::MachineMotionCertificateKey key;
                key.machineSourceSha256 = machineSafetyIndex->sourceSha256();
                key.safetyIndexSha256 = machineSafetyIndex->contentSha256();
                key.safetyPolicySha256 = machineSafetyPackageKey;
                key.pathRevision = request.environmentRevision;
                for (int edge = 1; edge < indexedPoses.size(); ++edge) {
                    key.edgeId = static_cast<std::uint64_t>(edge - 1);
                    const auto certificate =
                        lcnc::cam_algo::certifyLinearMotionEdgeWithIndex(
                            *machineSafetyIndex, key,
                            indexedPoses.at(edge - 1), indexedPoses.at(edge));
                    if (certificate.state
                        == lcnc::cam_algo::MachineMotionCertificateState::Blocked) {
                        result.state = lcnc::cam::CollisionValidationState::Collision;
                        result.nodeStates[edge - 1] = result.state;
                        result.nodeStates[edge] = result.state;
                        result.intervals.append({
                            edge - 1, edge, result.state,
                            QStringLiteral("machine_safety_package"),
                            QStringLiteral("machine"), {}, {},
                            -1.0, tr("The machine safety package blocks this motion edge")});
                        result.failureReason = result.intervals.constLast().reason;
                        return result;
                    }
                    if (certificate.state
                        != lcnc::cam_algo::MachineMotionCertificateState::CertifiedSafe) {
                        allCertified = false;
                        break;
                    }
                }
            }
            if (allCertified) {
                result.state = lcnc::cam::CollisionValidationState::Safe;
                return result;
            }
        }
    }

    MachineKinematics kinematics;
    kinematics.setAxes(geometry->axes, geometry->configType);
    kinematics.setWorkpieceSetupTransform(geometry->workpieceSetup);
    for (auto it = geometry->assignments.cbegin(); it != geometry->assignments.cend(); ++it)
        kinematics.assignShape(it.key(), it.value());
    for (auto it = geometry->mounts.cbegin(); it != geometry->mounts.cend(); ++it)
        kinematics.mountWorkpiece(it.key(), it.value());

    const QVector<QPair<int, int>> proofPairs =
        lcnc::cam::collisionProofPairs(
            *geometry,
            request.scope == lcnc::cam::CollisionSafetyScope::MachineOnly);
    if (proofPairs.isEmpty()) {
        result.state = lcnc::cam::CollisionValidationState::Indeterminate;
        // 中文翻译：碰撞安全域没有可用的碰撞源组合
        result.failureReason = tr("Collision safety domain has no usable source pair");
        return result;
    }

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
                int bodyIndex{-1};
                gp_Trsf transform;
                Bnd_Box aabb;
                Bnd_OBB obb;
            };
            QVector<PoseBody> poseBodies;
            poseBodies.reserve(geometry->bodies.size());
            for (int bodyIndex = 0; bodyIndex < geometry->bodies.size(); ++bodyIndex) {
                const CamTravelCollisionBody& body = geometry->bodies.at(bodyIndex);
                gp_Trsf transform;
                if (body.workpiece) {
                    transform = kinematics.computeWpcTransform(item.workpieceEntry);
                } else {
                    transform = kinematics.computeShapeTransform(body.entry);
                }
                poseBodies.append({
                    &body, bodyIndex, transform,
                    transformCollisionAabb(body.localAabb, transform,
                                           request.clearanceMm),
                    transformCollisionObb(body.localObb, transform,
                                          request.clearanceMm)});
            }

            bool poseBlocked = false;
            for (const auto& proofPair : std::as_const(proofPairs)) {
                if (poseBlocked)
                    break;
                const PoseBody& active = poseBodies.at(proofPair.first);
                const PoseBody& passive = poseBodies.at(proofPair.second);
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
                            bool separatedByBackend = false;
                            if (policy.allowCoalFallback) {
                                const auto coal = lcnc::cam::queryCoalCollisionPair(
                                    lcnc::cam::coalCollisionPair(
                                        *geometry, active.bodyIndex, passive.bodyIndex),
                                    active.transform, passive.transform,
                                    request.clearanceMm
                                        + active.body->surfaceModel.linearDeflectionMm()
                                        + passive.body->surfaceModel.linearDeflectionMm());
                                separatedByBackend = coal.available
                                    && coal.definitelySeparated;
                                if (coal.available)
                                    sample.minimumDistanceMm = coal.distanceLowerBoundMm;
                            }
                            if (!separatedByBackend
                                && active.body->surfaceModel.isValid()
                                && passive.body->surfaceModel.isValid()) {
                                const double meshGuardMm = request.clearanceMm
                                    + active.body->surfaceModel.linearDeflectionMm()
                                    + passive.body->surfaceModel.linearDeflectionMm();
                                const auto prefilter =
                                    lcnc::cam_algo::prefilterSurfaceCollision(
                                        active.body->surfaceModel, active.transform,
                                        passive.body->surfaceModel, passive.transform,
                                        meshGuardMm);
                                separatedByBackend = prefilter.valid
                                    && prefilter.definitelySeparated;
                                if (prefilter.valid)
                                    sample.minimumDistanceMm = prefilter.meshDistanceMm;
                            }
                            if (!separatedByBackend) {
                                if (!policy.allowOcctExactFallback) {
                                    result.state = lcnc::cam::CollisionValidationState::Indeterminate;
                                    // 中文翻译：在线查询不允许回退到耗时的 OCCT 精确计算
                                    result.failureReason = tr("The online collision query cannot fall back to an expensive OCCT exact calculation");
                                    return result;
                                }
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
                                        const double minimumDistance = distance.Value();
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

lcnc::cam::CamMotionPermit CamModule::requestMotionPermit(
    const lcnc::cam::CamMotionPermitRequest& request) const
{
    lcnc::cam::CamMotionPermit permit;
    permit.kind = request.kind;
    permit.commandedAxis = request.commandedAxis.trimmed().toUpper();
    permit.direction = request.direction;
    permit.maximumDistance = request.maximumDistance;
    permit.firstApos = request.firstApos;
    permit.lastApos = request.lastApos;
    permit.issuedUtcMs = QDateTime::currentMSecsSinceEpoch();
    permit.expiresUtcMs = permit.issuedUtcMs
        + qBound<qint64>(qint64{50}, request.validityMs, qint64{5000});

    const MachineKinematics* live = kinematics();
    if (!live || request.firstApos.isEmpty() || request.lastApos.isEmpty()) {
        permit.state = lcnc::cam::CamMotionCertificateState::BoundaryUnknown;
        // 中文翻译：运动许可证缺少机台运动学或 APOS 端点
        permit.reason = tr("The motion permit is missing machine kinematics or APOS endpoints");
        return permit;
    }

    const auto collision = collisionConfiguration();
    const auto package = m_machineSafetyPackageManager.runtimeSnapshot();
    const auto overlay = m_jobSafetyOverlayManager.runtimeSnapshot();
    const bool packageRequired = collision.enabled;
    const bool overlayRequired = collision.enabled
        && collision.passiveSources.contains(QStringLiteral("workpiece"));
    if (collision.enabled && !collision.valid) {
        permit.state = lcnc::cam::CamMotionCertificateState::BoundaryUnknown;
        // 中文翻译：碰撞检测配置不完整，运动许可证失败关闭
        permit.reason = tr("Collision detection configuration is incomplete; the motion permit is fail-closed");
        return permit;
    }
    if (packageRequired && (!package.status.executionEligible()
                            || package.status.buildInProgress || !package.index)) {
        permit.state = lcnc::cam::CamMotionCertificateState::BoundaryUnknown;
        // 中文翻译：机台安全包未就绪或正在构建，运动许可证失败关闭
        permit.reason = package.status.reason.isEmpty()
            ? tr("The machine safety package is unavailable or being built; the motion permit is fail-closed")
            : package.status.reason;
        return permit;
    }
    if (overlayRequired && (!overlay.status.executionEligible()
                            || overlay.status.buildInProgress || !overlay.geometry)) {
        permit.state = lcnc::cam::CamMotionCertificateState::BoundaryUnknown;
        // 中文翻译：工件碰撞叠加缓存未就绪或正在构建，运动许可证失败关闭
        permit.reason = overlay.status.reason.isEmpty()
            ? tr("The workpiece collision overlay is unavailable or being built; the motion permit is fail-closed")
            : overlay.status.reason;
        return permit;
    }

    lcnc::cam::ToolpathExportSnapshot snapshot;
    snapshot.revision = static_cast<std::uint64_t>(permit.issuedUtcMs);
    snapshot.motionPlan.revision = snapshot.revision;
    snapshot.travelPlan.key.environmentRevision = overlay.status.environmentRevision;
    snapshot.travelPlan.key.motionProfileHash = static_cast<std::uint64_t>(
        qHash(permit.commandedAxis)) ^ static_cast<std::uint64_t>(request.kind);
    snapshot.collisionSafety.enabled = collision.enabled;
    snapshot.collisionSafety.machinePackageRequired = packageRequired;
    snapshot.collisionSafety.machinePackageReady = package.status.executionEligible();
    snapshot.collisionSafety.packageBuildInProgress = package.status.buildInProgress;
    snapshot.collisionSafety.jobOverlayRequired = overlayRequired;
    snapshot.collisionSafety.jobOverlayReady = overlay.status.executionEligible();
    snapshot.collisionSafety.jobOverlayBuildInProgress = overlay.status.buildInProgress;

    for (const MachineAxisDef& axis : live->axes()) {
        if (axis.name.compare(QStringLiteral("BASE"), Qt::CaseInsensitive) == 0)
            continue;
        if (!snapshot.machineAxisLayout.append(axis.name, axis.role))
            break;
    }
    if (snapshot.machineAxisLayout.count == 0) {
        permit.state = lcnc::cam::CamMotionCertificateState::BoundaryUnknown;
        // 中文翻译：运动许可证无法建立物理轴布局
        permit.reason = tr("The motion permit could not establish a physical axis layout");
        return permit;
    }

    QString workpieceEntry;
    if (!live->wpcMounts().isEmpty())
        workpieceEntry = live->wpcMounts().cbegin().key();
    if (overlayRequired && workpieceEntry.isEmpty()) {
        permit.state = lcnc::cam::CamMotionCertificateState::BoundaryUnknown;
        // 中文翻译：运动许可证无法确定工件安装坐标系
        permit.reason = tr("The motion permit could not determine the mounted workpiece coordinate system");
        return permit;
    }
    lcnc::cam::ToolpathExportContour contour;
    contour.contourId = 1;
    contour.workpieceEntry = workpieceEntry.isEmpty()
        ? QStringLiteral("__no_workpiece__") : workpieceEntry;
    snapshot.contours.append(contour);

    QString cutterCarrierAxis;
    for (const MachineAxisDef& axis : live->axes()) {
        if (axis.role == lcnc::MachineAxisRole::LinearZ
            && cutterCarrierAxis.isEmpty()) {
            cutterCarrierAxis = axis.name;
        }
        if (axis.role == lcnc::MachineAxisRole::HeadTiltPrimary)
            cutterCarrierAxis = axis.name;
        if (axis.role == lcnc::MachineAxisRole::HeadTiltSecondary)
            cutterCarrierAxis = axis.name;
    }
    const auto makeNode = [&](const QMap<QString, double>& apos) {
        lcnc::cam::CamMotionNode node;
        node.phase = lcnc::cam::CamMotionPhase::Rapid;
        node.contourId = 1;
        MachineKinematics posed;
        posed.setAxes(live->axes(), live->configType());
        posed.setWorkpieceSetupTransform(live->workpieceSetupTransform());
        for (auto it = live->shapeAssignments().cbegin();
             it != live->shapeAssignments().cend(); ++it) {
            posed.assignShape(it.key(), it.value());
        }
        for (auto it = live->wpcMounts().cbegin();
             it != live->wpcMounts().cend(); ++it) {
            posed.mountWorkpiece(it.key(), it.value());
        }
        for (int index = 0; index < snapshot.machineAxisLayout.count; ++index) {
            const QString name = snapshot.machineAxisLayout.axes[index].name;
            const double value = apos.value(name,
                apos.value(name.toUpper(), 0.0));
            node.axes[index] = value;
            node.axisMask |= static_cast<std::uint8_t>(1u << index);
            posed.setAxisPosition(name, value);
        }
        gp_Pnt tcp = m_cutterHeadModelPosition;
        gp_Vec normal(0.0, 0.0, 1.0);
        const gp_Trsf cutterTransform =
            posed.computeAxisTransform(cutterCarrierAxis);
        tcp.Transform(cutterTransform);
        normal.Transform(cutterTransform);
        if (normal.SquareMagnitude() <= Precision::SquareConfusion())
            normal = gp_Vec(0.0, 0.0, 1.0);
        normal.Normalize();
        node.tcpX = tcp.X(); node.tcpY = tcp.Y(); node.tcpZ = tcp.Z();
        node.normalX = normal.X(); node.normalY = normal.Y();
        node.normalZ = normal.Z();
        return node;
    };
    snapshot.motionPlan.nodes.append(makeNode(request.firstApos));
    snapshot.motionPlan.nodes.append(makeNode(request.lastApos));

    lcnc::cam::ContinuousMotionCertificateBuildContext context;
    context.machineIndex = package.index;
    context.geometry = overlay.geometry;
    context.packageKeySha256 = package.status.packageKeySha256;
    context.runtimeConfigurationSha256 =
        package.status.runtimeConfigurationSha256;
    context.clearanceMm = m_config.cutterCollisionClearanceMm();
    if (request.kind == lcnc::cam::CamMotionPermitKind::ContinuousJog)
        context.maximumSubdivisionDepth = 4;
    const auto certificates = lcnc::cam::buildContinuousMotionCertificates(
        snapshot, context);
    if (certificates.size() != 1) {
        permit.state = lcnc::cam::CamMotionCertificateState::BoundaryUnknown;
        // 中文翻译：运动许可证未能生成唯一的连续运动证书
        permit.reason = tr("The motion permit did not produce exactly one continuous-motion certificate");
        return permit;
    }
    const auto& certificate = certificates.constFirst();
    permit.state = certificate.state;
    permit.packageKeySha256 = certificate.packageKeySha256;
    permit.environmentRevision = certificate.environmentRevision;
    permit.reason = certificate.reason;
    QByteArray canonical;
    canonical += QByteArray::number(static_cast<int>(permit.kind));
    canonical += QByteArray::number(permit.issuedUtcMs);
    canonical += QByteArray::number(permit.expiresUtcMs);
    canonical += permit.commandedAxis.toUtf8();
    canonical += QByteArray::number(permit.direction);
    canonical += QByteArray::number(permit.maximumDistance, 'g', 17);
    canonical += permit.packageKeySha256;
    canonical += QByteArray::number(permit.environmentRevision);
    for (auto it = request.firstApos.cbegin(); it != request.firstApos.cend(); ++it) {
        canonical += it.key().toUtf8();
        canonical += QByteArray::number(it.value(), 'g', 17);
    }
    for (auto it = request.lastApos.cbegin(); it != request.lastApos.cend(); ++it) {
        canonical += it.key().toUtf8();
        canonical += QByteArray::number(it.value(), 'g', 17);
    }
    permit.tokenSha256 = QCryptographicHash::hash(canonical,
                                                  QCryptographicHash::Sha256);
    return permit;
}

bool CamModule::setCollisionDetectionEnabled(bool enabled,
                                             QString* errorMessage)
{
    const auto current = collisionConfiguration();
    if (enabled && !current.activationAvailable) {
        if (errorMessage)
            *errorMessage = current.activationFailureReason;
        return false;
    }
    if (enabled && !current.valid) {
        if (errorMessage) {
            *errorMessage = tr(
                "The immutable machine package has incomplete axis assignments or the current workpiece is unavailable.");
        }
        return false;
    }
    const QString profilePath = activeMachineProfilePath();
    if (m_config.collisionDetectionEnabledForMachine(profilePath) == enabled)
        return true;
    m_config.setCollisionDetectionEnabledForMachine(profilePath, enabled);
    ++m_collisionConfigurationRevision;
    m_travelPlanCache.stale = true;
    if (m_travelVerificationTask != kInvalidTaskId) {
        if (auto* tasks = lcnc::Kernel::current().taskManager())
            tasks->requestAbort(m_travelVerificationTask);
    }
    if (enabled)
        scheduleWorkpieceSafetyOverlayPreparation();
    emit collisionConfigurationChanged();
    return true;
}

void CamModule::setCollisionSources(const QSet<QString>& active,
                                    const QSet<QString>& passive)
{
    Q_UNUSED(active);
    Q_UNUSED(passive);
    // Machine/workpiece roles are derived from the immutable assembly. Keep
    // the compatibility service method side-effect free for older UI callers.
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
    for (auto& transition : pending.transitions)
        transition.collisionStates.clear();
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
    const auto machineSafetyPackage =
        m_machineSafetyPackageManager.runtimeSnapshot();
    if (!collision.valid || !machineSafetyPackage.status.executionEligible()
        || !machineSafetyPackage.index)
        return;

    const lcnc::cam::TravelPlanKey geometryKey =
        collisionGeometryKey(snapshot.travelPlan.key);
    const auto existingOverlay = m_jobSafetyOverlayManager.geometry();
    if (existingOverlay && existingOverlay->key == geometryKey) {
        return;
    }
    if (m_collisionDomainPreparationTask != kInvalidTaskId)
        return;

    const QSet<QString> machineAxisPairKeys =
        machineCollisionAxisPairKeys(*machineSafetyPackage.index);
    const QSet<QString> machinePairSources =
        machineCollisionAxisSources(*machineSafetyPackage.index);
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
            if (!active && !passive
                && !machinePairSources.contains(source)) {
                continue;
            }
            const TopoDS_Shape shape = machine->shapeTool()->GetShape(label);
            if (!shape.IsNull())
                bodies.append({entry, source, active, passive, false, shape});
        }
    }
    if (collision.passiveSources.contains(QStringLiteral("workpiece"))
        && !m_workpieceShape.IsNull()) {
        bodies.append({QStringLiteral("__workpiece__"),
                       QStringLiteral("workpiece"), false, true,
                       true, m_workpieceShape});
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
    m_jobSafetyOverlayManager.beginBuild(geometryKey.environmentRevision);

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
         definition, geometryKey, meshDeflectionMm, machineSafetyPackage,
         machineAxisPairKeys, completed](TaskProgress* progress) {
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
            geometry->machineAxisPairKeys = machineAxisPairKeys;
            QHash<QString, int> persistedBodyOrdinal;
            int prepared = 0;
            for (CamTravelCollisionBody& body : geometry->bodies) {
                if (progress->isAbortRequested())
                    return;
                if (!body.workpiece
                    && body.source.startsWith(QStringLiteral("axis:"))) {
                    const QString axisName = body.source.mid(5);
                    const int requestedOrdinal =
                        persistedBodyOrdinal.value(axisName);
                    int matchingOrdinal = 0;
                    for (int persistedIndex = 0;
                         persistedIndex
                             < machineSafetyPackage.index->bodies().size();
                         ++persistedIndex) {
                        if (machineSafetyPackage.index->bodies()
                                .at(persistedIndex).axisName.compare(
                                    axisName, Qt::CaseInsensitive) != 0) {
                            continue;
                        }
                        if (matchingOrdinal++ != requestedOrdinal)
                            continue;
                        if (const auto* persisted = machineSafetyPackage.index
                                ->persistedSurfaceModel(persistedIndex)) {
                            body.surfaceModel = *persisted;
                        }
                        break;
                    }
                    persistedBodyOrdinal[axisName] = requestedOrdinal + 1;
                }
                const double bodyMeshDeflectionMm =
                    body.workpiece
                    ? qBound(0.005, meshDeflectionMm * 0.1, 0.01)
                    : meshDeflectionMm;
                buildCollisionGeometry(&body, bodyMeshDeflectionMm);
                if (body.leaves.isEmpty())
                    return;
                progress->setValue(++prepared);
            }
            buildCoalCollisionPairs(geometry.get());
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
                m_jobSafetyOverlayManager.finishBuildFailure(
                    geometryKey.environmentRevision,
                    tr("Job collision overlay preparation failed or was cancelled"));
                return;
            }
            if (!m_workpieceShape.IsNull()
                && collisionEnvironmentRevision()
                    == geometryKey.environmentRevision) {
                m_jobSafetyOverlayManager.publish(
                    geometryKey.environmentRevision, *completed);
            }
        });
}

void CamModule::scheduleWorkpieceSafetyOverlayPreparation()
{
    if (m_workpieceShape.IsNull()
        || !m_machineSafetyPackageManager.status().executionEligible()
        || !m_machineConfig) {
        return;
    }
    lcnc::cam::ToolpathExportSnapshot snapshot;
    snapshot.machiningMode = machiningMode();
    snapshot.travelPlan.key.environmentRevision =
        collisionEnvironmentRevision();
    scheduleCollisionSafetyDomainPreparation(snapshot);
}

void CamModule::scheduleFullEnvironmentVerification(
    const lcnc::cam::ToolpathExportSnapshot& snapshot, bool force)
{
    if ((!snapshot.travelPlan.fullEnvironmentVerificationPending && !force) || !m_machineConfig)
        return;
    auto* tasks = lcnc::Kernel::current().taskManager();
    LcncDocument* machine = machineDocument();
    MachineKinematics* liveKinematics = kinematics();
    // Full-path validation requires the immutable machine package. The
    // presentation cone is never a substitute for missing Z-axis geometry.
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
    const auto machineSafetyPackage =
        m_machineSafetyPackageManager.runtimeSnapshot();
    if (!machineSafetyPackage.status.executionEligible()
        || !machineSafetyPackage.index) {
        // 中文翻译：碰撞校验所需的机台安全包不可用
        failBeforeScheduling(tr(
            "The machine safety package is unavailable for collision validation"));
        return;
    }
    const QSet<QString> machineAxisPairKeys =
        machineCollisionAxisPairKeys(*machineSafetyPackage.index);
    const QSet<QString> machinePairSources =
        machineCollisionAxisSources(*machineSafetyPackage.index);
    QVector<CamTravelCollisionBody> bodies;
    if (machine && machine->shapeTool()) {
        const TDF_LabelSequence labels = machine->entityLabels(LcncDocument::EntityKind::Machine);
        for (int index = 1; index <= labels.Length(); ++index) {
            const TDF_Label& label = labels.Value(index);
            const QString entry = XcafUtils::entry(label);
            const QString source = collisionAxisSourceId(liveKinematics->axisForShape(entry));
            const bool active = collision.activeSources.contains(source);
            const bool passive = collision.passiveSources.contains(source);
            if (!active && !passive
                && !machinePairSources.contains(source)) {
                continue;
            }
            const TopoDS_Shape shape = machine->shapeTool()->GetShape(label);
            if (!shape.IsNull())
                bodies.append({entry, source, active, passive, false, shape});
        }
    }
    const bool verifyWorkpiece = collision.passiveSources.contains(QStringLiteral("workpiece"));
    if (verifyWorkpiece && !m_workpieceShape.IsNull()) {
        bodies.append({QStringLiteral("__workpiece__"), QStringLiteral("workpiece"),
                       false, true, true, m_workpieceShape});
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
    const QString configType = liveKinematics->configType();
    const gp_Trsf workpieceSetup = liveKinematics->workpieceSetupTransform();
    const auto assignments = liveKinematics->shapeAssignments();
    const auto mounts = liveKinematics->wpcMounts();
    const double clearanceMm = m_config.cutterCollisionClearanceMm();
    const double collisionMeshDeflectionMm =
        qBound(0.05, clearanceMm * 0.1, 0.25);
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
    const auto currentOverlay = m_jobSafetyOverlayManager.geometry();
    const auto cachedGeometry = (currentOverlay && currentOverlay->key == geometryKey)
        ? currentOverlay : std::shared_ptr<CamTravelCollisionGeometryCache>{};
    if (!cachedGeometry)
        m_jobSafetyOverlayManager.beginBuild(geometryKey.environmentRevision);
    const auto completedGeometry = std::make_shared<std::shared_ptr<CamTravelCollisionGeometryCache>>(cachedGeometry);
    const auto completedNodeStates = std::make_shared<QVector<lcnc::cam::CollisionValidationState>>();
    const auto completedIntervals = std::make_shared<QVector<lcnc::cam::CollisionInterval>>();
    const auto completedMotionCertificates =
        std::make_shared<QVector<lcnc::cam::CamMotionEdgeCertificate>>();
    const auto completedRapidStates = std::make_shared<
        QHash<std::uint64_t, QVector<lcnc::cam::CamMotionCertificateState>>>();
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
        [snapshot, force, bodies, activeSources, cachedGeometry, completedGeometry, axes, configType, workpieceSetup,
         assignments, mounts,
         definition, clearanceMm, collisionMeshDeflectionMm, geometryKey, result,
         machineSafetyPackage, machineAxisPairKeys,
         completedNodeStates, completedIntervals,
         completedMotionCertificates,
         completedRapidStates,
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
            const int certificateSteps = qMax(0, workItems.size() - 1);
            const int scanSteps = workItems.size() * activeSources.size();
            progress->setRange(
                0, qMax(1, preparationSteps + certificateSteps + scanSteps));
            // 中文翻译：准备碰撞几何与工件安全缓存
            progress->setStepName(QObject::tr(
                "Preparing collision geometry and Job Overlay"));
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
                geometry->machineAxisPairKeys = machineAxisPairKeys;
                QHash<QString, int> persistedBodyOrdinal;
                for (CamTravelCollisionBody& body : geometry->bodies) {
                    if (progress->isAbortRequested())
                        return;
                    if (!body.workpiece
                        && machineSafetyPackage.index
                        && body.source.startsWith(QStringLiteral("axis:"))) {
                        const QString axisName = body.source.mid(5);
                        int requestedOrdinal = persistedBodyOrdinal.value(axisName);
                        int matchingOrdinal = 0;
                        for (int persistedIndex = 0;
                             persistedIndex < machineSafetyPackage.index->bodies().size();
                             ++persistedIndex) {
                            if (machineSafetyPackage.index->bodies().at(persistedIndex)
                                    .axisName.compare(axisName, Qt::CaseInsensitive) != 0) {
                                continue;
                            }
                            if (matchingOrdinal++ != requestedOrdinal)
                                continue;
                            if (const auto* persisted =
                                    machineSafetyPackage.index->persistedSurfaceModel(
                                        persistedIndex)) {
                                body.surfaceModel = *persisted;
                            }
                            break;
                        }
                        persistedBodyOrdinal[axisName] = requestedOrdinal + 1;
                    }
                    const double bodyMeshDeflectionMm =
                        body.workpiece
                        ? qBound(0.005, clearanceMm * 0.01, 0.01)
                        : collisionMeshDeflectionMm;
                    buildCollisionGeometry(&body, bodyMeshDeflectionMm);
                    if (body.leaves.isEmpty()) {
                        *result = QObject::tr("Full-machine collision verification geometry is unavailable for %1")
                            .arg(body.source);
                        return;
                    }
                    progress->setValue(++prepared);
                }
                buildCoalCollisionPairs(geometry.get());
                std::uint64_t surfaceTriangles = 0;
                int coalModels = 0;
                for (const auto& body : std::as_const(geometry->bodies)) {
                    surfaceTriangles += body.surfaceModel.triangleCount();
                    coalModels += body.coalModel ? 1 : 0;
                }
                LCNC_INFO(lcnc::LogCode::Generic,
                          "CAM collision geometry: bodies={}, surface_triangles={}, coal_models={}, coal_pairs={}",
                          geometry->bodies.size(), surfaceTriangles,
                          coalModels, geometry->coalPairs.size());
                geometryPreparationMs = geometryTimer.elapsed();
            }
            *completedGeometry = geometry;
            if (workItems.isEmpty())
                return;
            lcnc::cam::ContinuousMotionCertificateBuildContext certificateContext;
            certificateContext.machineIndex = machineSafetyPackage.index;
            certificateContext.geometry = geometry;
            certificateContext.packageKeySha256 =
                machineSafetyPackage.status.packageKeySha256;
            certificateContext.runtimeConfigurationSha256 =
                machineSafetyPackage.status.runtimeConfigurationSha256;
            certificateContext.clearanceMm = clearanceMm;
            QElapsedTimer certificateTimer;
            certificateTimer.start();
            // 中文翻译：构建连续运动碰撞证书
            progress->setStepName(QObject::tr(
                "Building continuous-motion collision certificates"));
            *completedMotionCertificates =
                lcnc::cam::buildContinuousMotionCertificates(
                    snapshot, certificateContext,
                    [progress]() { return progress->isAbortRequested(); },
                    [progress, preparationSteps](int completed, int total) {
                        if ((completed & 0x0f) == 0 || completed == total)
                         progress->setValue(preparationSteps + completed);
                     });
            completedRapidStates->clear();
            for (const auto& certificate :
                 std::as_const(*completedMotionCertificates)) {
                if (certificate.lastNode < 0
                    || certificate.lastNode >= snapshot.motionPlan.nodes.size()) {
                    continue;
                }
                const auto& node = snapshot.motionPlan.nodes.at(
                    certificate.lastNode);
                if (node.phase == lcnc::cam::CamMotionPhase::Rapid)
                    (*completedRapidStates)[node.contourId].append(certificate.state);
            }
            std::uint64_t certificateIntervals = 0;
            std::uint64_t certificateBroadPhaseRejected = 0;
            std::uint64_t certificateLocalFieldQueries = 0;
            std::uint64_t certificateLocalFieldRejected = 0;
            std::uint64_t certificateSurfaceBvhQueries = 0;
            std::uint64_t certificateSurfaceBvhRejected = 0;
            std::uint64_t certificateCoalQueries = 0;
            std::uint64_t certificateOcctExactQueries = 0;
            std::uint64_t certificateSurfaceBvhQueryNs = 0;
            std::uint64_t certificateLocalFieldQueryNs = 0;
            std::uint64_t certificateCoalQueryNs = 0;
            std::uint64_t certificateOcctExactQueryNs = 0;
            std::uint64_t certificateOcctExactLockWaitNs = 0;
            std::uint64_t certificateOcctExactMaximumQueryNs = 0;
            QString certificateOcctExactSlowestPair;
            int certifiedSafeEdges = 0;
            int blockedEdges = 0;
            int unknownEdges = 0;
            int budgetExhaustedEdges = 0;
            int maximumCertificateDepth = 0;
            QHash<QString, int> certificateUnknownReasons;
            QHash<QString, int> certificateFallbackPairs;
            for (const auto& certificate :
                 std::as_const(*completedMotionCertificates)) {
                certificateIntervals += certificate.intervalQueries;
                certificateBroadPhaseRejected += certificate.broadPhaseRejected;
                certificateLocalFieldQueries += certificate.localFieldQueries;
                certificateLocalFieldRejected += certificate.localFieldRejected;
                certificateSurfaceBvhQueries += certificate.surfaceBvhQueries;
                certificateSurfaceBvhRejected += certificate.surfaceBvhRejected;
                certificateCoalQueries += certificate.coalPairQueries;
                certificateOcctExactQueries += certificate.occtExactQueries;
                certificateSurfaceBvhQueryNs += certificate.surfaceBvhQueryNs;
                certificateLocalFieldQueryNs += certificate.localFieldQueryNs;
                certificateCoalQueryNs += certificate.coalQueryNs;
                certificateOcctExactQueryNs += certificate.occtExactQueryNs;
                certificateOcctExactLockWaitNs +=
                    certificate.occtExactLockWaitNs;
                if (certificate.occtExactMaximumQueryNs
                    > certificateOcctExactMaximumQueryNs) {
                    certificateOcctExactMaximumQueryNs =
                        certificate.occtExactMaximumQueryNs;
                    certificateOcctExactSlowestPair =
                        certificate.occtExactSlowestPair;
                }
                maximumCertificateDepth = std::max(
                    maximumCertificateDepth,
                    certificate.maximumSubdivisionDepth);
                budgetExhaustedEdges += certificate.budgetExhausted ? 1 : 0;
                if (certificate.state
                    == lcnc::cam::CamMotionCertificateState::CertifiedSafe) {
                    ++certifiedSafeEdges;
                } else if (certificate.state
                           == lcnc::cam::CamMotionCertificateState::Blocked) {
                    ++blockedEdges;
                } else if (!certificate.executionEligible()) {
                    ++unknownEdges;
                    ++certificateUnknownReasons[certificate.reason];
                    if (!certificate.fallbackSourcePair.isEmpty())
                        ++certificateFallbackPairs[certificate.fallbackSourcePair];
                }
            }
            LCNC_INFO(lcnc::LogCode::Generic,
                      "CAM continuous certificates: edges={}, certified_safe={}, blocked={}, unknown={}, budget_exhausted={}, intervals={}, broad_phase_rejected={}, local_field_queries={}, local_field_rejected={}, local_field_cpu_ms={}, surface_bvh_queries={}, surface_bvh_rejected={}, surface_bvh_cpu_ms={}, coal_queries={}, coal_cpu_ms={}, occt_exact_queries={}, occt_exact_cpu_ms={}, occt_exact_lock_wait_ms={}, occt_exact_max_ms={}, occt_exact_slowest_pair='{}', maximum_depth={}, elapsed_ms={}",
                      completedMotionCertificates->size(), certifiedSafeEdges,
                      blockedEdges, unknownEdges, budgetExhaustedEdges,
                      certificateIntervals, certificateBroadPhaseRejected,
                      certificateLocalFieldQueries,
                      certificateLocalFieldRejected,
                      certificateLocalFieldQueryNs / 1000000,
                      certificateSurfaceBvhQueries,
                      certificateSurfaceBvhRejected,
                      certificateSurfaceBvhQueryNs / 1000000,
                      certificateCoalQueries,
                      certificateCoalQueryNs / 1000000,
                      certificateOcctExactQueries,
                      certificateOcctExactQueryNs / 1000000,
                      certificateOcctExactLockWaitNs / 1000000,
                      certificateOcctExactMaximumQueryNs / 1000000,
                      certificateOcctExactSlowestPair.toStdString(),
                      maximumCertificateDepth, certificateTimer.elapsed());
            for (auto it = certificateUnknownReasons.cbegin();
                 it != certificateUnknownReasons.cend(); ++it) {
                LCNC_INFO(lcnc::LogCode::Generic,
                          "CAM continuous certificate unknown: count={}, reason='{}'",
                          it.value(), it.key().toStdString());
            }
            for (auto it = certificateFallbackPairs.cbegin();
                 it != certificateFallbackPairs.cend(); ++it) {
                LCNC_INFO(lcnc::LogCode::Generic,
                          "CAM continuous certificate fallback pair: count={}, pair='{}'",
                          it.value(), it.key().toStdString());
            }
            if (completedMotionCertificates->size()
                != qMax(0, snapshot.motionPlan.nodes.size() - 1)) {
                // 中文翻译：连续运动证书未覆盖全部运动边
                *result = QObject::tr("Continuous-motion certificate generation did not cover every motion edge");
                completedIndeterminate->store(true);
                return;
            }
            for (const auto& certificate : std::as_const(*completedMotionCertificates)) {
                if (certificate.state
                    == lcnc::cam::CamMotionCertificateState::Blocked) {
                    completedCollision->store(true);
                    if (result->isEmpty())
                        *result = certificate.reason;
                } else if (!certificate.executionEligible()) {
                    completedIndeterminate->store(true);
                    if (result->isEmpty())
                        *result = certificate.reason;
                }
            }
            if (machineSafetyPackage.index && !progress->isAbortRequested()) {
                appendMachineSafetyHotPoseFeedback(
                    machineSafetyPackage.status.packagePath, snapshot,
                    *machineSafetyPackage.index,
                    *completedMotionCertificates);
            }
            // The continuous certificates cover every endpoint and every
            // interpolation interval. Automatic generation consumes them as
            // the production decision and must not repeat the legacy discrete
            // OCCT audit. An explicit user-requested validation (force=true)
            // keeps that diagnostic audit backend available.
            // 中文翻译：连续证书已覆盖全部端点和插值区间；自动生成不再重复旧的逐点 OCCT 审计，
            // 用户显式执行“校验当前刀路”时仍保留该诊断后端。
            if (!force && workItems.size() > 1) {
                completedNodeStates->fill(
                    lcnc::cam::CollisionValidationState::Safe,
                    workItems.size());
                completedIntervals->clear();
                const auto assignNodeState =
                    [completedNodeStates](int node,
                                          lcnc::cam::CollisionValidationState state) {
                        if (node < 0 || node >= completedNodeStates->size())
                            return;
                        auto& current = (*completedNodeStates)[node];
                        if (current != lcnc::cam::CollisionValidationState::Collision
                            || state == lcnc::cam::CollisionValidationState::Collision) {
                            current = state;
                        }
                    };
                for (const auto& certificate :
                     std::as_const(*completedMotionCertificates)) {
                    lcnc::cam::CollisionValidationState state =
                        lcnc::cam::CollisionValidationState::Safe;
                    if (certificate.state
                        == lcnc::cam::CamMotionCertificateState::Blocked) {
                        state = lcnc::cam::CollisionValidationState::Collision;
                    } else if (!certificate.executionEligible()) {
                        state = lcnc::cam::CollisionValidationState::Indeterminate;
                    } else {
                        continue;
                    }
                    assignNodeState(certificate.firstNode, state);
                    assignNodeState(certificate.lastNode, state);
                    completedIntervals->append({
                        certificate.firstNode, certificate.lastNode, state,
                        QStringLiteral("continuous_certificate"),
                        QStringLiteral("machine_and_job"), {}, {}, -1.0,
                        certificate.reason});
                }
                progress->setValue(
                    preparationSteps + certificateSteps + scanSteps);
                LCNC_INFO(lcnc::LogCode::Generic,
                          "CAM collision scan: mode=continuous_certificates, nodes={}, edges={}, geometry_ms={}, certificate_ms={}, total_ms={}",
                          workItems.size(), completedMotionCertificates->size(),
                          geometryPreparationMs, certificateTimer.elapsed(),
                          totalTimer.elapsed());
                return;
            }
            completedNodeStates->fill(lcnc::cam::CollisionValidationState::Pending,
                                      workItems.size());
            completedIntervals->clear();
            // 中文翻译：校验运动节点的碰撞状态
            progress->setStepName(QObject::tr(
                "Validating collision states at motion nodes"));

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
                    activeSource, clearanceMm, result, resultMutex, progressMutex, progress, finished,
                    preparationSteps, certificateSteps, scanSteps, failed,
                    completedNodeStates,
                    completedIndeterminate, completedCollision, completedWarning,
                    completedIntervals, meshQueries, meshRejected,
                    exactQueries, leafBoundsNs, meshPrefilterNs,
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
                    int bodyIndex{-1};
                    gp_Trsf transform;
                    Bnd_Box aabb;
                    Bnd_OBB obb;
                    QVector<PoseLeaf> leaves;
                    bool leavesReady{false};
                    std::shared_ptr<CollisionPairTiming> timing;
                };
                QVector<PoseBody> activeBodies;
                QVector<PoseBody> passiveBodies;
                for (int bodyIndex = 0; bodyIndex < geometry->bodies.size(); ++bodyIndex) {
                    const CamTravelCollisionBody& body = geometry->bodies.at(bodyIndex);
                    gp_Trsf transform;
                    if (body.workpiece) {
                        transform = kinematics.computeWpcTransform(item.workpieceEntry);
                    } else {
                        transform = kinematics.computeShapeTransform(body.entry);
                    }
                    PoseBody poseBody{&body, bodyIndex, transform,
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
                    const double coalGuardMm = clearanceMm
                        + active.body->surfaceModel.linearDeflectionMm()
                        + passive.body->surfaceModel.linearDeflectionMm();
                    const auto coal = lcnc::cam::queryCoalCollisionPair(
                        lcnc::cam::coalCollisionPair(
                            *geometry, active.bodyIndex, passive.bodyIndex),
                        active.transform, passive.transform, coalGuardMm);
                    if (coal.available && coal.definitelySeparated) {
                        domainSample.minimumDistanceMm = coal.distanceLowerBoundMm;
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
                    progress->setValue(
                        preparationSteps + certificateSteps + done);
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
                for (auto& certificate : *completedMotionCertificates) {
                    const auto firstState = completedNodeStates->value(
                        certificate.firstNode,
                        lcnc::cam::CollisionValidationState::Indeterminate);
                    const auto lastState = completedNodeStates->value(
                        certificate.lastNode,
                        lcnc::cam::CollisionValidationState::Indeterminate);
                    if (firstState == lcnc::cam::CollisionValidationState::Collision
                        || lastState == lcnc::cam::CollisionValidationState::Collision) {
                        certificate.state =
                            lcnc::cam::CamMotionCertificateState::Blocked;
                        // 中文翻译：连续运动边端点已确认发生碰撞
                        certificate.reason = QObject::tr(
                            "A continuous-motion edge endpoint has a confirmed collision");
                    } else if (firstState == lcnc::cam::CollisionValidationState::Warning
                               || lastState == lcnc::cam::CollisionValidationState::Warning
                               || firstState == lcnc::cam::CollisionValidationState::Indeterminate
                               || lastState == lcnc::cam::CollisionValidationState::Indeterminate) {
                        certificate.state =
                            lcnc::cam::CamMotionCertificateState::BoundaryUnknown;
                        // 中文翻译：连续运动边端点未被确认安全
                        certificate.reason = QObject::tr(
                            "A continuous-motion edge endpoint is not certified safe");
                    }
                }
                progress->setValue(
                    preparationSteps + certificateSteps + scanSteps);
            }
            LCNC_INFO(lcnc::LogCode::Generic,
                      "CAM collision scan: mode=full_environment, nodes={}, sources={}, mesh_queries={}, mesh_rejected={}, exact_queries={}, geometry_ms={}, leaf_bounds_ms={}, mesh_ms={}, exact_lock_wait_ms={}, exact_ms={}, scan_ms={}, total_ms={}",
                      workItems.size(), activeSources.size(), meshQueries->load(),
                      meshRejected->load(), exactQueries->load(), geometryPreparationMs,
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
         completedMotionCertificates, completedRapidStates,
         completedCollision, completedWarning,
         stageTimer](TaskId id,
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
                // Geometry construction is an independent, reusable result.
                // Retain it even when certificate/node validation is aborted;
                // only the motion plan remains fail-closed.
                // 中文翻译：即使路径校验取消，也保留已完整构建的工件安全缓存；运动计划仍按失败关闭。
                if (*completedGeometry
                    && (*completedGeometry)->key == geometryKey) {
                    m_jobSafetyOverlayManager.publish(
                        geometryKey.environmentRevision, *completedGeometry);
                } else {
                    m_jobSafetyOverlayManager.finishBuildFailure(
                        geometryKey.environmentRevision,
                        tr("Full-path collision validation was cancelled or failed"));
                }
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
                m_jobSafetyOverlayManager.publish(
                    geometryKey.environmentRevision, *completedGeometry);
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
            m_travelPlanCache.motionCertificates =
                *completedMotionCertificates;
            for (auto& transition : m_travelPlanCache.transitions) {
                const auto states = completedRapidStates->value(
                    transition.toContourId);
                transition.collisionStates =
                    states.size() == transition.segments.size()
                    ? states
                    : QVector<lcnc::cam::CamMotionCertificateState>{};
            }
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
    TopoDS_Shape updated = lcnc::cam::buildCutterDisplayProxy(m_config, &proxyError);
    if (updated.IsNull()) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "cam.cutterDisplayProxy: failed to apply display proxy: {}",
                 proxyError.toStdString());
        emit operationFailed(tr("Cutting nozzle display proxy"), proxyError);
        return false;
    }
    // This setting controls presentation only. It must not invalidate motion
    // plans, machine LMSI data, or the workpiece safety overlay.
    m_cutterDisplayProxyShape = std::move(updated);
    if (m_guideRenderer)
        m_guideRenderer->setCutterDisplayProxy(m_cutterDisplayProxyShape);
    displayAxisGuides();
    emit cutterCollisionConfigurationChanged();
    return true;
}

void CamModule::refreshCollisionSafetyPolicy()
{
    m_travelPlanCache.stale = true;
    const std::uint64_t currentRevision = collisionEnvironmentRevision();
    const auto overlay = m_jobSafetyOverlayManager.status();
    if (overlay.environmentRevision != currentRevision) {
        if (m_collisionDomainPreparationTask != kInvalidTaskId) {
            if (auto* tasks = lcnc::Kernel::current().taskManager())
                tasks->requestAbort(m_collisionDomainPreparationTask);
            m_collisionDomainPreparationTask = kInvalidTaskId;
        }
        m_jobSafetyOverlayManager.invalidate(
            tr("Workpiece collision-field policy changed"));
        scheduleWorkpieceSafetyOverlayPreparation();
    }
    emit collisionConfigurationChanged();
}

TopoDS_Shape CamModule::cutterDisplayProxyShape(QString* errorMessage)
{
    if (!m_cutterDisplayProxyShape.IsNull())
        return m_cutterDisplayProxyShape;

    QString proxyError;
    TopoDS_Shape proxy = lcnc::cam::buildCutterDisplayProxy(m_config, &proxyError);
    if (proxy.IsNull()) {
        if (errorMessage)
            *errorMessage = proxyError;
        return {};
    }
    m_cutterDisplayProxyShape = proxy;
    return m_cutterDisplayProxyShape;
}
