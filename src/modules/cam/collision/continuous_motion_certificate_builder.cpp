#include "modules/cam/collision/continuous_motion_certificate_builder.h"

#include "core/algorithms/cam/collision_scan_policy.h"
#include "core/algorithms/cam/machine_motion_certificate.h"
#include "core/algorithms/cam/machine_safety_index.h"
#include "core/algorithms/occt_exact_operation_lock.h"
#include "core/kinematics/machine_kinematics.h"
#include "modules/cam/collision/coal_collision_backend.h"
#include "modules/cam/collision/collision_geometry_cache.h"
#include "modules/cam/contracts/toolpath_export_dto.h"

#include <QCryptographicHash>
#include <QHash>
#include <QMutex>
#include <QMutexLocker>
#include <QSemaphore>
#include <QSet>
#include <QScopeGuard>

#include <BRepExtrema_DistShapeShape.hxx>
#include <OSD_Parallel.hxx>
#include <OSD_ThreadPool.hxx>
#include <Precision.hxx>
#include <TopLoc_Location.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

namespace lcnc::cam {
namespace {

constexpr double kPi = 3.14159265358979323846;

struct EdgePose
{
    std::array<double, MachineAxisLayout::kMaxAxes> axes{};
    std::uint8_t axisMask{0};
    gp_Pnt tcp;
    gp_Vec normal{0.0, 0.0, 1.0};
};

struct AdaptiveMetrics
{
    std::uint64_t intervals{0};
    std::uint64_t broadPhaseRejected{0};
    std::uint64_t localFieldQueries{0};
    std::uint64_t localFieldRejected{0};
    std::uint64_t surfaceBvhQueries{0};
    std::uint64_t surfaceBvhRejected{0};
    std::uint64_t pairQueries{0};
    std::uint64_t occtExactQueries{0};
    std::uint64_t surfaceBvhQueryNs{0};
    std::uint64_t localFieldQueryNs{0};
    std::uint64_t coalQueryNs{0};
    std::uint64_t occtExactQueryNs{0};
    std::uint64_t occtExactLockWaitNs{0};
    std::uint64_t occtExactMaximumQueryNs{0};
    QString occtExactSlowestPair;
    int maximumDepth{0};
    bool budgetExhausted{false};
    QString fallbackSourcePair;
};

struct CertificateWorkerContext
{
    MachineKinematics kinematics;
    QHash<quint64, std::shared_ptr<CoalCollisionPair>> coalPairs;
};

struct CertificateSharedContext
{
    explicit CertificateSharedContext(int coalConcurrency)
        : coalSlots(std::max(1, coalConcurrency))
    {
    }

    bool coalEnabled(quint64 pairKey)
    {
        QMutexLocker locker(&slowPairMutex);
        return !slowCoalPairs.contains(pairKey);
    }

    void disableCoal(quint64 pairKey)
    {
        QMutexLocker locker(&slowPairMutex);
        slowCoalPairs.insert(pairKey);
    }

    QSemaphore coalSlots;
    QMutex slowPairMutex;
    QSet<quint64> slowCoalPairs;
};

quint64 workerPairKey(int firstBody, int secondBody)
{
    const quint32 first = static_cast<quint32>(
        std::min(firstBody, secondBody));
    const quint32 second = static_cast<quint32>(
        std::max(firstBody, secondBody));
    return (static_cast<quint64>(first) << 32) | second;
}

double clampUnit(double value)
{
    return std::max(-1.0, std::min(1.0, value));
}

EdgePose interpolateNode(const CamMotionNode& first,
                         const CamMotionNode& last,
                         double parameter)
{
    EdgePose result;
    result.axisMask = first.axisMask | last.axisMask;
    for (int index = 0; index < MachineAxisLayout::kMaxAxes; ++index) {
        result.axes[index] = first.axes[index]
            + (last.axes[index] - first.axes[index]) * parameter;
    }
    result.tcp = gp_Pnt(first.tcpX + (last.tcpX - first.tcpX) * parameter,
                       first.tcpY + (last.tcpY - first.tcpY) * parameter,
                       first.tcpZ + (last.tcpZ - first.tcpZ) * parameter);
    gp_Vec normal(first.normalX + (last.normalX - first.normalX) * parameter,
                  first.normalY + (last.normalY - first.normalY) * parameter,
                  first.normalZ + (last.normalZ - first.normalZ) * parameter);
    if (normal.SquareMagnitude() <= Precision::SquareConfusion())
        normal = gp_Vec(0.0, 0.0, 1.0);
    normal.Normalize();
    result.normal = normal;
    return result;
}

double localBoundingRadius(const Bnd_Box& box)
{
    if (box.IsVoid())
        return 0.0;
    double minimumX = 0.0, minimumY = 0.0, minimumZ = 0.0;
    double maximumX = 0.0, maximumY = 0.0, maximumZ = 0.0;
    box.Get(minimumX, minimumY, minimumZ,
            maximumX, maximumY, maximumZ);
    double radius = 0.0;
    for (double x : {minimumX, maximumX}) {
        for (double y : {minimumY, maximumY}) {
            for (double z : {minimumZ, maximumZ})
                radius = std::max(radius, gp_Vec(x, y, z).Magnitude());
        }
    }
    return radius;
}

const MachineAxisDef* findAxis(const QList<MachineAxisDef>& axes,
                               const QString& name)
{
    for (const MachineAxisDef& axis : axes) {
        if (axis.name.compare(name, Qt::CaseInsensitive) == 0)
            return &axis;
    }
    return nullptr;
}

int layoutAxisIndex(const MachineAxisLayout& layout, const QString& name)
{
    for (int index = 0; index < layout.count; ++index) {
        if (layout.axes[index].name.compare(name, Qt::CaseInsensitive) == 0)
            return index;
    }
    return -1;
}

QSet<QString> axisChain(const QList<MachineAxisDef>& axes, QString axisName)
{
    QSet<QString> result;
    while (!axisName.isEmpty()
           && axisName.compare(QStringLiteral("BASE"), Qt::CaseInsensitive) != 0
           && !result.contains(axisName.toUpper())) {
        result.insert(axisName.toUpper());
        const MachineAxisDef* axis = findAxis(axes, axisName);
        if (!axis)
            break;
        axisName = axis->parentAxis;
    }
    return result;
}

double bodyRadiusAboutAxis(const TravelCollisionBody& body,
                           const TravelCollisionGeometryCache& geometry,
                           const MachineAxisDef& axis)
{
    if (body.localAabb.IsVoid())
        return 0.0;
    double minimumX = 0.0, minimumY = 0.0, minimumZ = 0.0;
    double maximumX = 0.0, maximumY = 0.0, maximumZ = 0.0;
    body.localAabb.Get(minimumX, minimumY, minimumZ,
                       maximumX, maximumY, maximumZ);
    const gp_Vec axisDirection(axis.direction);
    double radius = 0.0;
    for (double x : {minimumX, maximumX}) {
        for (double y : {minimumY, maximumY}) {
            for (double z : {minimumZ, maximumZ}) {
                gp_Pnt point(x, y, z);
                if (body.workpiece)
                    point.Transform(geometry.workpieceSetup);
                const gp_Vec offset(axis.origin, point);
                radius = std::max(radius,
                                  offset.Crossed(axisDirection).Magnitude());
            }
        }
    }
    return radius;
}

double bodyMotionBound(const TravelCollisionBody& body,
                       const TravelCollisionGeometryCache& geometry,
                       const ToolpathExportSnapshot& snapshot,
                       const EdgePose& first,
                       const EdgePose& last,
                       const EdgePose& midpoint,
                       const QString& workpieceEntry)
{
    QString mountedAxis;
    if (body.workpiece)
        mountedAxis = geometry.mounts.value(workpieceEntry);
    else
        mountedAxis = geometry.assignments.value(body.entry);
    const QSet<QString> chain = axisChain(geometry.axes, mountedAxis);
    if (chain.isEmpty())
        return 0.0;

    double bound = 0.0;
    for (const MachineAxisDef& axis : geometry.axes) {
        if (!chain.contains(axis.name.toUpper()))
            continue;
        const int layoutIndex = layoutAxisIndex(snapshot.machineAxisLayout,
                                                axis.name);
        if (layoutIndex < 0)
            continue;
        const double halfDelta = std::abs(
            last.axes[layoutIndex] - first.axes[layoutIndex]) * 0.5;
        if (axis.motionType == MachineAxisDef::Rotary) {
            const double radius = bodyRadiusAboutAxis(body, geometry, axis);
            const double angle = std::min(180.0, halfDelta);
            bound += 2.0 * radius * std::sin(angle * kPi / 360.0);
        } else {
            bound += halfDelta;
        }
    }
    Q_UNUSED(midpoint);
    return bound;
}

gp_Trsf bodyTransform(const TravelCollisionBody& body,
                      MachineKinematics* kinematics,
                      const QString& workpieceEntry,
                      const EdgePose& pose)
{
    if (body.workpiece)
        return kinematics->computeWpcTransform(workpieceEntry);
    Q_UNUSED(pose);
    return kinematics->computeShapeTransform(body.entry);
}

bool localClearanceFieldCertifiesPair(
    const TravelCollisionBody& first,
    const gp_Trsf& firstTransform,
    const TravelCollisionBody& second,
    const gp_Trsf& secondTransform,
    double requiredSeparationMm)
{
    const TravelCollisionBody* active = nullptr;
    const TravelCollisionBody* passive = nullptr;
    gp_Trsf activeTransform;
    gp_Trsf passiveTransform;
    if (first.active && second.passive && !first.sphereCover.isEmpty()
        && second.localClearanceField) {
        active = &first;
        passive = &second;
        activeTransform = firstTransform;
        passiveTransform = secondTransform;
    } else if (second.active && first.passive && !second.sphereCover.isEmpty()
               && first.localClearanceField) {
        active = &second;
        passive = &first;
        activeTransform = secondTransform;
        passiveTransform = firstTransform;
    }
    if (!active || !passive || !passive->localClearanceField->isValid())
        return false;
    gp_Trsf relative = passiveTransform.Inverted();
    relative.Multiply(activeTransform);
    const double activeMeshError = active->surfaceModel.linearDeflectionMm();
    const auto certifiesSphere = [&](const ConservativeCollisionSphere& sphere) {
        gp_Pnt localCenter = sphere.center;
        localCenter.Transform(relative);
        return passive->localClearanceField->certifiesSphereSeparated(
            localCenter, sphere.radiusMm + activeMeshError,
            requiredSeparationMm);
    };
    if (!active->sphereHierarchy.isEmpty()) {
        QVector<int> pending{
            static_cast<int>(active->sphereHierarchy.size() - 1)};
        while (!pending.isEmpty()) {
            const int index = pending.takeLast();
            const auto& node = active->sphereHierarchy.at(index);
            if (certifiesSphere(node.sphere))
                continue;
            if (node.isLeaf())
                return false;
            pending.append(node.firstChild);
            pending.append(node.secondChild);
        }
        return true;
    }
    for (const ConservativeCollisionSphere& sphere : active->sphereCover) {
        if (!certifiesSphere(sphere))
            return false;
    }
    return !active->sphereCover.isEmpty();
}

void initializeKinematics(
    MachineKinematics* result,
    const TravelCollisionGeometryCache& geometry)
{
    if (!result)
        return;
    result->setAxes(geometry.axes, geometry.configType);
    result->setWorkpieceSetupTransform(geometry.workpieceSetup);
    for (auto it = geometry.assignments.cbegin(); it != geometry.assignments.cend(); ++it)
        result->assignShape(it.key(), it.value());
    for (auto it = geometry.mounts.cbegin(); it != geometry.mounts.cend(); ++it)
        result->mountWorkpiece(it.key(), it.value());
}

void configureKinematicsAt(
    MachineKinematics* result,
    const TravelCollisionGeometryCache& geometry,
    const ToolpathExportSnapshot& snapshot,
    const EdgePose& pose)
{
    if (!result)
        return;
    cam_algo::applyOfflineMotionPose(
        result, geometry.axes, snapshot.machineAxisLayout,
        pose.axes, pose.axisMask);
}

enum class ExactPairProof
{
    Separated,
    NeedsSplit,
    Cancelled,
    Failed
};

ExactPairProof exactLeafPairProof(
    const TravelCollisionBody& firstModel,
    const gp_Trsf& firstTransform,
    const TravelCollisionBody& secondModel,
    const gp_Trsf& secondTransform,
    double requiredSeparation,
    std::uint64_t maximumQueries,
    AdaptiveMetrics* metrics,
    const std::function<bool()>& cancelled)
{
    if (!metrics || !std::isfinite(requiredSeparation)
        || requiredSeparation < 0.0
        || firstModel.leaves.isEmpty() || secondModel.leaves.isEmpty()) {
        return ExactPairProof::Failed;
    }
    struct LeafPose
    {
        TopoDS_Shape shape;
        Bnd_Box aabb;
        Bnd_OBB obb;
    };
    std::vector<LeafPose> firstLeaves;
    std::vector<LeafPose> secondLeaves;
    firstLeaves.reserve(static_cast<std::size_t>(firstModel.leaves.size()));
    secondLeaves.reserve(static_cast<std::size_t>(secondModel.leaves.size()));
    for (const TravelCollisionLeaf& leaf : firstModel.leaves) {
        if (cancelled && cancelled())
            return ExactPairProof::Cancelled;
        firstLeaves.push_back({
            leaf.shape.Moved(TopLoc_Location(firstTransform)),
            transformCollisionAabb(
                leaf.localAabb, firstTransform, requiredSeparation),
            transformCollisionObb(
                leaf.localObb, firstTransform, requiredSeparation)});
    }
    for (const TravelCollisionLeaf& leaf : secondModel.leaves) {
        if (cancelled && cancelled())
            return ExactPairProof::Cancelled;
        secondLeaves.push_back({
            leaf.shape.Moved(TopLoc_Location(secondTransform)),
            transformCollisionAabb(leaf.localAabb, secondTransform, 0.0),
            transformCollisionObb(leaf.localObb, secondTransform, 0.0)});
    }
    for (const LeafPose& firstLeaf : firstLeaves) {
        if (firstLeaf.shape.IsNull())
            return ExactPairProof::Failed;
        for (const LeafPose& secondLeaf : secondLeaves) {
            if (cancelled && cancelled())
                return ExactPairProof::Cancelled;
            if (!firstLeaf.aabb.IsVoid() && !secondLeaf.aabb.IsVoid()
                && firstLeaf.aabb.IsOut(secondLeaf.aabb)) {
                continue;
            }
            if (!firstLeaf.obb.IsVoid() && !secondLeaf.obb.IsVoid()
                && firstLeaf.obb.IsOut(secondLeaf.obb)) {
                continue;
            }
            if (metrics->occtExactQueries >= maximumQueries) {
                metrics->budgetExhausted = true;
                return ExactPairProof::Failed;
            }
            if (secondLeaf.shape.IsNull())
                return ExactPairProof::Failed;

            const auto lockStarted = std::chrono::steady_clock::now();
            double minimumDistance = std::numeric_limits<double>::infinity();
            bool completed = false;
            {
                lcnc::OcctExactOperationLock exactOperationLock;
                const auto operationStarted = std::chrono::steady_clock::now();
                metrics->occtExactLockWaitNs += static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        operationStarted - lockStarted).count());
                ++metrics->occtExactQueries;
                BRepExtrema_DistShapeShape distance(
                    firstLeaf.shape, secondLeaf.shape);
                distance.SetDeflection(0.025);
                distance.SetMultiThread(false);
                distance.Perform();
                completed = distance.IsDone();
                if (completed)
                    minimumDistance = distance.Value();
                const auto operationElapsedNs = static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now()
                        - operationStarted).count());
                metrics->occtExactQueryNs += operationElapsedNs;
                if (operationElapsedNs > metrics->occtExactMaximumQueryNs) {
                    metrics->occtExactMaximumQueryNs = operationElapsedNs;
                    metrics->occtExactSlowestPair = firstModel.source
                        + QStringLiteral(" -> ") + secondModel.source;
                }
            }
            if (!completed)
                return ExactPairProof::Failed;
            if (minimumDistance <= requiredSeparation)
                return ExactPairProof::NeedsSplit;
        }
    }
    if (cam_algo::surfaceCollisionContainmentDetected(
            firstModel.surfaceModel, firstTransform,
            secondModel.surfaceModel, secondTransform)) {
        return ExactPairProof::NeedsSplit;
    }
    return ExactPairProof::Separated;
}

bool certifiesInterval(
    const ToolpathExportSnapshot& snapshot,
    const ContinuousMotionCertificateBuildContext& context,
    const CamMotionNode& edgeFirst,
    const CamMotionNode& edgeLast,
    const QString& workpieceEntry,
    bool machineAlreadyCertified,
    double parameterFirst,
    double parameterLast,
    int depth,
    CertificateWorkerContext* worker,
    CertificateSharedContext* shared,
    AdaptiveMetrics* metrics,
    const std::function<bool()>& cancelled)
{
    if (!context.geometry || !worker || !shared
        || (cancelled && cancelled()))
        return false;
    const int maximumSubdivisionDepth =
        edgeLast.phase == CamMotionPhase::Rapid
        ? std::min(context.maximumSubdivisionDepth,
                   context.maximumRapidSubdivisionDepth)
        : context.maximumSubdivisionDepth;
    if (metrics->intervals >= context.maximumIntervalsPerEdge) {
        metrics->budgetExhausted = true;
        return false;
    }
    metrics->maximumDepth = std::max(metrics->maximumDepth, depth);
    ++metrics->intervals;
    const double parameterMidpoint = (parameterFirst + parameterLast) * 0.5;
    const EdgePose first = interpolateNode(edgeFirst, edgeLast, parameterFirst);
    const EdgePose last = interpolateNode(edgeFirst, edgeLast, parameterLast);
    const EdgePose midpoint = interpolateNode(edgeFirst, edgeLast,
                                              parameterMidpoint);
    configureKinematicsAt(
        &worker->kinematics, *context.geometry, snapshot, midpoint);
    bool evaluatedPair = false;
    bool evaluatedMachinePair = false;
    bool needsGeometricSplit = false;
    for (int firstBody = 0; firstBody < context.geometry->bodies.size(); ++firstBody) {
        const auto& firstModel = context.geometry->bodies.at(firstBody);
        for (int secondBody = firstBody + 1;
             secondBody < context.geometry->bodies.size(); ++secondBody) {
            const auto& secondModel = context.geometry->bodies.at(secondBody);
            const CollisionPairProofOwner proofOwner =
                collisionPairProofOwner(
                    *context.geometry, firstBody, secondBody);
            if (proofOwner == CollisionPairProofOwner::None) {
                continue;
            }
            // LMSI owns every immutable machine/machine pair. Every pair with
            // the current workpiece remains a Job Overlay responsibility even
            // when the machine package certified this APOS interval.
            if (machineAlreadyCertified
                && proofOwner == CollisionPairProofOwner::MachinePackage) {
                continue;
            }
            evaluatedPair = true;
            if (proofOwner == CollisionPairProofOwner::MachinePackage)
                evaluatedMachinePair = true;
            const double firstBound = bodyMotionBound(
                firstModel, *context.geometry, snapshot, first, last, midpoint,
                workpieceEntry);
            const double secondBound = bodyMotionBound(
                secondModel, *context.geometry, snapshot, first, last, midpoint,
                workpieceEntry);
            const double motionBound = firstBound + secondBound;
            gp_Trsf firstTransform = bodyTransform(
                firstModel, &worker->kinematics, workpieceEntry, midpoint);
            gp_Trsf secondTransform = bodyTransform(
                secondModel, &worker->kinematics, workpieceEntry, midpoint);
            const double exactRequiredSeparation = motionBound
                + context.clearanceMm;
            const double requiredSeparation = exactRequiredSeparation
                + firstModel.surfaceModel.linearDeflectionMm()
                + secondModel.surfaceModel.linearDeflectionMm();
            const Bnd_Box firstAabb = transformCollisionAabb(
                firstModel.localAabb, firstTransform, requiredSeparation);
            const Bnd_Box secondAabb = transformCollisionAabb(
                secondModel.localAabb, secondTransform, 0.0);
            if (!firstAabb.IsVoid() && !secondAabb.IsVoid()
                && firstAabb.IsOut(secondAabb)) {
                ++metrics->broadPhaseRejected;
                continue;
            }
            const Bnd_OBB firstObb = transformCollisionObb(
                firstModel.localObb, firstTransform, requiredSeparation);
            const Bnd_OBB secondObb = transformCollisionObb(
                secondModel.localObb, secondTransform, 0.0);
            if (!firstObb.IsVoid() && !secondObb.IsVoid()
                && firstObb.IsOut(secondObb)) {
                ++metrics->broadPhaseRejected;
                continue;
            }
            ++metrics->localFieldQueries;
            const auto localFieldStarted = std::chrono::steady_clock::now();
            const bool localFieldSeparated = localClearanceFieldCertifiesPair(
                firstModel, firstTransform, secondModel, secondTransform,
                exactRequiredSeparation);
            metrics->localFieldQueryNs += static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now() - localFieldStarted).count());
            if (localFieldSeparated) {
                ++metrics->localFieldRejected;
                continue;
            }
            // A large interval margin makes a whole-mesh distance traversal
            // touch most of the A/C-table BVH. Split first, then run the narrow
            // phase only on intervals whose swept radius is reasonably local.
            if (motionBound > context.maximumNarrowPhaseMotionBoundMm
                && depth < maximumSubdivisionDepth) {
                metrics->fallbackSourcePair = firstModel.source
                    + QStringLiteral(" -> ") + secondModel.source;
                needsGeometricSplit = true;
                break;
            }
            if (firstModel.surfaceModel.isValid()
                && secondModel.surfaceModel.isValid()) {
                ++metrics->surfaceBvhQueries;
                const auto prefilterStarted =
                    std::chrono::steady_clock::now();
                // The prefilter transforms and allocates scratch state only
                // for its first model. Distance is symmetric, so keep the
                // compact cutter/proxy first instead of repeatedly copying a
                // persisted A/C-table BVH for every motion interval.
                const bool secondIsSmaller =
                    secondModel.surfaceModel.triangleCount()
                    < firstModel.surfaceModel.triangleCount();
                const auto prefilter = secondIsSmaller
                    ? cam_algo::prefilterSurfaceCollision(
                          secondModel.surfaceModel, secondTransform,
                          firstModel.surfaceModel, firstTransform,
                          requiredSeparation)
                    : cam_algo::prefilterSurfaceCollision(
                          firstModel.surfaceModel, firstTransform,
                          secondModel.surfaceModel, secondTransform,
                          requiredSeparation);
                metrics->surfaceBvhQueryNs +=
                    static_cast<std::uint64_t>(
                        std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now()
                            - prefilterStarted).count());
                if (prefilter.valid && prefilter.definitelySeparated) {
                    ++metrics->surfaceBvhRejected;
                    continue;
                }
                if (prefilter.valid && prefilter.meshIntersection) {
                    // A mesh intersection cannot certify an exact collision,
                    // but it already proves that this interval cannot receive
                    // a conservative safe certificate. Keep it fail-closed and
                    // avoid repeating serialized OCCT audits down the tree.
                    metrics->fallbackSourcePair = firstModel.source
                        + QStringLiteral(" -> ") + secondModel.source;
                    return false;
                }
            }
            bool separated = false;
            bool coalUnresolved = false;
            if (collisionPairUsesCoalFallback(firstModel, secondModel)) {
                if (metrics->pairQueries
                    >= context.maximumCoalPairQueriesPerEdge) {
                    metrics->budgetExhausted = true;
                    return false;
                }
                const quint64 pairKey = workerPairKey(firstBody, secondBody);
                auto queryPair = worker->coalPairs.value(pairKey);
                if (!queryPair) {
                    queryPair = cloneCoalCollisionPair(
                        coalCollisionPair(*context.geometry,
                                          firstBody, secondBody));
                    if (queryPair)
                        worker->coalPairs.insert(pairKey, queryPair);
                }
                if (queryPair && shared->coalEnabled(pairKey)) {
                    while (!shared->coalSlots.tryAcquire(1, 10)) {
                        if (cancelled && cancelled())
                            return false;
                    }
                    const auto releaseCoalSlot = qScopeGuard(
                        [shared] { shared->coalSlots.release(); });
                    if (shared->coalEnabled(pairKey)) {
                        ++metrics->pairQueries;
                        const auto coalStarted = std::chrono::steady_clock::now();
                        const auto query = queryCoalCollisionPair(
                            queryPair, firstTransform, secondTransform,
                            requiredSeparation);
                        const auto coalElapsedNs = static_cast<std::uint64_t>(
                            std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::steady_clock::now()
                                - coalStarted).count());
                        metrics->coalQueryNs += coalElapsedNs;
                        if (coalElapsedNs
                            > context.maximumCoalQueryWallTimeMs * 1'000'000) {
                            shared->disableCoal(pairKey);
                        }
                        separated = query.available
                            && query.definitelySeparated;
                        coalUnresolved = query.available && !separated;
                    }
                }
            }
            if (separated)
                continue;
            if (coalUnresolved) {
                // Coal is the online narrow phase for compact persisted mesh
                // pairs. An unresolved security-margin query is subdivided and
                // ultimately remains BoundaryUnknown; do not serialize the
                // batch behind OCCT merely to reach the same fail-closed state.
                metrics->fallbackSourcePair = firstModel.source
                    + QStringLiteral(" -> ") + secondModel.source;
                if (depth >= maximumSubdivisionDepth)
                    return false;
                needsGeometricSplit = true;
                break;
            }

            if (context.maximumOcctExactQueriesPerEdge == 0) {
                metrics->fallbackSourcePair = firstModel.source
                    + QStringLiteral(" -> ") + secondModel.source;
                return false;
            }
            const ExactPairProof exact = exactLeafPairProof(
                firstModel, firstTransform, secondModel, secondTransform,
                exactRequiredSeparation,
                context.maximumOcctExactQueriesPerEdge,
                metrics, cancelled);
            if (exact == ExactPairProof::Separated)
                continue;
            if (exact == ExactPairProof::Cancelled)
                return false;
            metrics->fallbackSourcePair = firstModel.source
                + QStringLiteral(" -> ") + secondModel.source;
            if (depth >= maximumSubdivisionDepth)
                return false;
            needsGeometricSplit = true;
            break;
        }
        if (needsGeometricSplit)
            break;
    }
    if (!evaluatedPair)
        return machineAlreadyCertified;
    if (!needsGeometricSplit) {
        // When LMSI is Unknown, this traversal has evaluated every applicable
        // machine pair as well as every Job Overlay pair. A complete geometric
        // proof may therefore certify the edge. Job-only geometry must not
        // promote LMSI uncertainty, so require at least one machine pair when
        // the package did not already certify the interval.
        // 中文翻译：LMSI 未知时，只有几何后端同时完整证明了机台碰撞对，才能认证该边；
        // 仅工件叠加层证明安全不得提升机台未知状态。
        return machineAlreadyCertified || evaluatedMachinePair;
    }
    if (depth >= maximumSubdivisionDepth)
        return false;
    return certifiesInterval(
               snapshot, context, edgeFirst, edgeLast, workpieceEntry,
               machineAlreadyCertified, parameterFirst, parameterMidpoint,
               depth + 1, worker, shared, metrics, cancelled)
        && certifiesInterval(
               snapshot, context, edgeFirst, edgeLast, workpieceEntry,
               machineAlreadyCertified, parameterMidpoint, parameterLast,
               depth + 1, worker, shared, metrics, cancelled);
}

cam_algo::MachineSafetyPose machineSafetyPoseForNode(
    const ToolpathExportSnapshot& snapshot,
    const TravelCollisionGeometryCache* geometry,
    const cam_algo::MachineSafetyIndex& index,
    const CamMotionNode& node)
{
    cam_algo::MachineSafetyPose result;
    result.count = static_cast<std::uint8_t>(
        std::min<int>(static_cast<int>(index.axes().size()),
                      cam_algo::kMachineSafetyMaximumAxes));
    for (int axisIndex = 0; axisIndex < result.count; ++axisIndex) {
        const QString& name = index.axes().at(axisIndex).name;
        const int layoutIndex = layoutAxisIndex(snapshot.machineAxisLayout, name);
        if (layoutIndex >= 0 && (node.axisMask & (1u << layoutIndex))) {
            result.values[axisIndex] = node.axes[layoutIndex];
            continue;
        }
        if (geometry) {
            if (const MachineAxisDef* axis = findAxis(geometry->axes, name))
                result.values[axisIndex] = axis->currentPos;
        }
    }
    return result;
}

cam_algo::MachineMotionEdgeCertificate certifyPeriodicMachineEdge(
    const cam_algo::MachineSafetyIndex& index,
    const cam_algo::MachineMotionCertificateKey& key,
    const cam_algo::MachineSafetyPose& physicalFirst,
    const cam_algo::MachineSafetyPose& physicalLast,
    const cam_algo::MachineMotionCertificationOptions& baseOptions)
{
    cam_algo::MachineMotionEdgeCertificate combined;
    combined.key = key;
    combined.first = physicalFirst;
    combined.last = physicalLast;
    combined.deviationTolerance = baseOptions.deviationTolerance;
    combined.state = cam_algo::MachineMotionCertificateState::CertifiedSafe;

    const auto interpolatedPose = [](
                                      const cam_algo::MachineSafetyPose& first,
                                      const cam_algo::MachineSafetyPose& last,
                                      double parameter) {
        cam_algo::MachineSafetyPose result;
        result.count = first.count;
        for (int axis = 0; axis < result.count; ++axis) {
            result.values[axis] = first.values[axis]
                + (last.values[axis] - first.values[axis]) * parameter;
        }
        return result;
    };
    std::function<bool(const cam_algo::MachineSafetyPose&,
                       const cam_algo::MachineSafetyPose&, int)> certify;
    certify = [&](const cam_algo::MachineSafetyPose& first,
                  const cam_algo::MachineSafetyPose& last,
                  int depth) {
        double firstSeamParameter = 1.0;
        for (int axis = 0; axis < first.count; ++axis) {
            const auto& grid = index.axes().at(axis);
            if (!grid.rotary || grid.maximum - grid.minimum < 359.0)
                continue;
            const double delta = last.values[axis] - first.values[axis];
            if (std::abs(delta) <= 1e-12)
                continue;
            const double low = std::min(first.values[axis], last.values[axis]);
            const double high = std::max(first.values[axis], last.values[axis]);
            const double revolution = 360.0;
            const double seam = grid.minimum + revolution
                * (std::floor((low - grid.minimum) / revolution) + 1.0);
            if (seam <= low + 1e-10 || seam >= high - 1e-10)
                continue;
            const double parameter =
                (seam - first.values[axis]) / delta;
            if (parameter > 1e-10
                && parameter < firstSeamParameter - 1e-10) {
                firstSeamParameter = parameter;
            }
        }
        if (firstSeamParameter < 1.0 && depth < 8) {
            const auto seamPose = interpolatedPose(
                first, last, firstSeamParameter);
            return certify(first, seamPose, depth + 1)
                && certify(seamPose, last, depth + 1);
        }

        auto options = baseOptions;
        for (int axis = 0; axis < first.count; ++axis) {
            const auto& grid = index.axes().at(axis);
            if (!grid.rotary || grid.maximum - grid.minimum < 359.0)
                continue;
            const double physicalMidpoint =
                0.5 * (first.values[axis] + last.values[axis]);
            const double canonicalCenter =
                0.5 * (grid.minimum + grid.maximum);
            options.canonicalAxisOffset[axis] = 360.0
                * std::round((physicalMidpoint - canonicalCenter) / 360.0);
        }
        const auto certificate = cam_algo::certifyLinearMotionEdgeWithIndex(
            index, key, first, last, options);
        combined.parameterIntervals += certificate.parameterIntervals;
        combined.visitedFineCells += certificate.visitedFineCells;
        if (certificate.state
            == cam_algo::MachineMotionCertificateState::Blocked) {
            combined.state = certificate.state;
            combined.limitingPair = certificate.limitingPair;
            return false;
        }
        if (certificate.state
            != cam_algo::MachineMotionCertificateState::CertifiedSafe) {
            combined.state =
                cam_algo::MachineMotionCertificateState::BoundaryUnknown;
            if (combined.limitingPair < 0)
                combined.limitingPair = certificate.limitingPair;
        }
        return true;
    };
    (void)certify(physicalFirst, physicalLast, 0);
    return combined;
}

} // namespace

QVector<CamMotionEdgeCertificate> buildContinuousMotionCertificates(
    const ToolpathExportSnapshot& snapshot,
    const ContinuousMotionCertificateBuildContext& context,
    const std::function<bool()>& cancelled,
    const std::function<void(int, int)>& progress)
{
    const int totalEdges = static_cast<int>(std::max<qsizetype>(
        0, snapshot.motionPlan.nodes.size() - 1));
    QVector<CamMotionEdgeCertificate> result(totalEdges);
    if (totalEdges == 0)
        return result;
    if (snapshot.collisionSafety.effectiveVerificationMode()
            == CollisionVerificationMode::Disabled) {
        for (int edgeIndex = 0; edgeIndex < totalEdges; ++edgeIndex) {
            auto& certificate = result[edgeIndex];
            certificate.edgeId = (snapshot.motionPlan.revision << 20)
                ^ static_cast<std::uint64_t>(edgeIndex);
            certificate.firstNode = edgeIndex;
            certificate.lastNode = edgeIndex + 1;
            certificate.phase = snapshot.motionPlan.nodes.at(edgeIndex + 1).phase;
            certificate.environmentRevision =
                snapshot.travelPlan.key.environmentRevision;
            certificate.state = CamMotionCertificateState::Disabled;
            certificate.reason = QObject::tr(
                "Collision verification is disabled; motion is not collision-certified");
        }
        if (progress)
            progress(totalEdges, totalEdges);
        return result;
    }
    CamMotionEdgeCertificate* const resultData = result.data();
    QHash<std::uint64_t, QString> workpieceByContour;
    workpieceByContour.reserve(snapshot.contours.size());
    for (const ToolpathExportContour& contour : snapshot.contours)
        workpieceByContour.insert(contour.contourId, contour.workpieceEntry);

    const int availableWorkers = std::max(
        1, OSD_Parallel::NbLogicalProcessors() / 2);
    const int workerCount = std::max(1, std::min({
        totalEdges, availableWorkers,
        std::max(1, context.maximumParallelWorkers)}));
    std::vector<std::unique_ptr<CertificateWorkerContext>> workers;
    workers.reserve(static_cast<std::size_t>(workerCount));
    for (int workerIndex = 0; workerIndex < workerCount; ++workerIndex) {
        auto worker = std::make_unique<CertificateWorkerContext>();
        if (context.geometry)
            initializeKinematics(&worker->kinematics, *context.geometry);
        workers.push_back(std::move(worker));
    }
    CertificateSharedContext shared(context.maximumParallelCoalQueries);
    std::atomic_int completedEdges{0};
    QMutex progressMutex;
    const auto buildEdge = [&](int workerIndex, int edgeIndex) {
        const int lastNode = edgeIndex + 1;
        const CamMotionNode& first = snapshot.motionPlan.nodes.at(lastNode - 1);
        const CamMotionNode& last = snapshot.motionPlan.nodes.at(lastNode);
        CamMotionEdgeCertificate certificate;
        certificate.edgeId = (snapshot.motionPlan.revision << 20)
            ^ static_cast<std::uint64_t>(lastNode - 1);
        certificate.firstNode = lastNode - 1;
        certificate.lastNode = lastNode;
        certificate.phase = last.phase;
        certificate.packageKeySha256 = context.packageKeySha256;
        certificate.environmentRevision = snapshot.travelPlan.key.environmentRevision;
        const auto finish = [&] {
            resultData[edgeIndex] = std::move(certificate);
            const int completed = completedEdges.fetch_add(
                1, std::memory_order_relaxed) + 1;
            if (progress
                && ((completed & 0x0f) == 0 || completed == totalEdges)) {
                QMutexLocker locker(&progressMutex);
                progress(completed, totalEdges);
            }
        };
        if (cancelled && cancelled()) {
            certificate.state = CamMotionCertificateState::BoundaryUnknown;
            // 中文翻译：连续运动证书构建已取消
            certificate.reason = QObject::tr("Continuous-motion certificate generation was cancelled");
            finish();
            return;
        }
        const QString firstWorkpiece = workpieceByContour.value(first.contourId);
        const QString lastWorkpiece = workpieceByContour.value(last.contourId);
        if (firstWorkpiece.isEmpty() || lastWorkpiece.isEmpty()
            || firstWorkpiece != lastWorkpiece) {
            certificate.state = CamMotionCertificateState::BoundaryUnknown;
            // 中文翻译：连续运动边跨越不同或未知工件坐标系
            certificate.reason = QObject::tr("A continuous-motion edge crosses different or unknown workpiece coordinate systems");
            finish();
            return;
        }

        bool machineCertified = !snapshot.collisionSafety.machinePackageRequired;
        if (snapshot.collisionSafety.machinePackageRequired && context.machineIndex) {
            cam_algo::MachineMotionCertificateKey key;
            key.machineSourceSha256 = context.machineIndex->sourceSha256();
            key.safetyIndexSha256 = context.machineIndex->contentSha256();
            key.safetyPolicySha256 = context.runtimeConfigurationSha256;
            key.motionProfileSha256 = QCryptographicHash::hash(
                QByteArray::number(snapshot.travelPlan.key.motionProfileHash),
                QCryptographicHash::Sha256);
            key.pathRevision = snapshot.motionPlan.revision;
            key.edgeId = certificate.edgeId;
            cam_algo::MachineMotionCertificationOptions options;
            for (int axis = 0; axis < context.machineIndex->axes().size()
                 && axis < cam_algo::kMachineSafetyMaximumAxes; ++axis) {
                const auto& grid = context.machineIndex->axes().at(axis);
                // A full-revolution axis is certified in canonical periodic
                // space. Its commanded centerline may cross the +/-180 seam;
                // tracking-error bounds remain a Process concern and are not
                // encoded in CamMotionEdgeCertificate, so do not expand this
                // offline centerline across both representations of the seam.
                options.deviationTolerance[axis] = grid.rotary
                    ? (grid.maximum - grid.minimum >= 359.0 ? 0.0 : 0.01)
                    : 0.02;
            }
            const auto machine = certifyPeriodicMachineEdge(
                *context.machineIndex, key,
                machineSafetyPoseForNode(snapshot, context.geometry.get(),
                                         *context.machineIndex, first),
                machineSafetyPoseForNode(snapshot, context.geometry.get(),
                                         *context.machineIndex, last),
                options);
            if (machine.state == cam_algo::MachineMotionCertificateState::Blocked) {
                certificate.state = CamMotionCertificateState::Blocked;
                // 中文翻译：机台安全索引在连续运动边上发现碰撞样本
                certificate.reason = QObject::tr("The machine safety index found a collision sample on the continuous-motion edge");
                finish();
                return;
            }
            machineCertified = machine.state
                == cam_algo::MachineMotionCertificateState::CertifiedSafe;
            certificate.intervalQueries += machine.parameterIntervals;
        }

        const bool overlayRequired = snapshot.collisionSafety.jobOverlayRequired;
        bool certified = machineCertified && !overlayRequired;
        AdaptiveMetrics metrics;
        if (!certified && context.geometry) {
            CertificateWorkerContext* worker = workers.at(
                static_cast<std::size_t>(workerIndex)).get();
            certified = certifiesInterval(
                snapshot, context, first, last, firstWorkpiece,
                machineCertified, 0.0, 1.0, 0, worker,
                &shared, &metrics, cancelled);
        }
        certificate.intervalQueries += metrics.intervals;
        certificate.broadPhaseRejected = metrics.broadPhaseRejected;
        certificate.localFieldQueries = metrics.localFieldQueries;
        certificate.localFieldRejected = metrics.localFieldRejected;
        certificate.surfaceBvhQueries = metrics.surfaceBvhQueries;
        certificate.surfaceBvhRejected = metrics.surfaceBvhRejected;
        certificate.coalPairQueries = metrics.pairQueries;
        certificate.occtExactQueries = metrics.occtExactQueries;
        certificate.surfaceBvhQueryNs = metrics.surfaceBvhQueryNs;
        certificate.localFieldQueryNs = metrics.localFieldQueryNs;
        certificate.coalQueryNs = metrics.coalQueryNs;
        certificate.occtExactQueryNs = metrics.occtExactQueryNs;
        certificate.occtExactLockWaitNs = metrics.occtExactLockWaitNs;
        certificate.occtExactMaximumQueryNs =
            metrics.occtExactMaximumQueryNs;
        certificate.occtExactSlowestPair = metrics.occtExactSlowestPair;
        certificate.maximumSubdivisionDepth = metrics.maximumDepth;
        certificate.budgetExhausted = metrics.budgetExhausted;
        certificate.fallbackSourcePair = metrics.fallbackSourcePair;
        certificate.state = certified
            ? CamMotionCertificateState::CertifiedSafe
            : CamMotionCertificateState::BoundaryUnknown;
        if (!certified) {
            if (metrics.budgetExhausted) {
                // 中文翻译：连续运动证书超出保守查询预算
                certificate.reason = QObject::tr(
                    "Continuous-motion certification exceeded its conservative query budget");
            } else {
                // 中文翻译：LMSI 与几何回退后端未能保守证明整条连续运动边安全
                certificate.reason = QObject::tr("LMSI and the geometric fallback backends could not conservatively certify the complete continuous-motion edge");
            }
        }
        finish();
    };
    if (workerCount == 1) {
        for (int edgeIndex = 0; edgeIndex < totalEdges; ++edgeIndex)
            buildEdge(0, edgeIndex);
    } else {
        Handle(OSD_ThreadPool) pool = new OSD_ThreadPool(workerCount);
        OSD_ThreadPool::Launcher launcher(*pool, workerCount);
        launcher.Perform(0, totalEdges, buildEdge);
    }
    return result;
}

} // namespace lcnc::cam
