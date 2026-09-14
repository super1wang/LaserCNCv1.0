#include "modules/cam/collision/collision_geometry_cache.h"

#include "core/algorithms/cam/machine_safety_index.h"
#include "modules/cam/collision/coal_collision_backend.h"

#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <NCollection_Map.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <algorithm>

namespace lcnc::cam {

QString collisionAxisSourceId(const QString& axisName)
{
    return QStringLiteral("axis:") + axisName.trimmed().toUpper();
}

QString collisionAxisPairKey(const QString& firstSource,
                             const QString& secondSource)
{
    QString first = firstSource.trimmed().toUpper();
    QString second = secondSource.trimmed().toUpper();
    if (first > second)
        std::swap(first, second);
    return first + QLatin1Char('|') + second;
}

QSet<QString> machineCollisionAxisPairKeys(
    const cam_algo::MachineSafetyIndex& index)
{
    QSet<QString> result;
    for (const auto& pair : index.pairs()) {
        if (pair.firstBody >= index.bodies().size()
            || pair.secondBody >= index.bodies().size()) {
            continue;
        }
        result.insert(collisionAxisPairKey(
            collisionAxisSourceId(index.bodies().at(pair.firstBody).axisName),
            collisionAxisSourceId(index.bodies().at(pair.secondBody).axisName)));
    }
    return result;
}

QSet<QString> machineCollisionAxisSources(
    const cam_algo::MachineSafetyIndex& index)
{
    QSet<QString> result;
    for (const auto& pair : index.pairs()) {
        if (pair.firstBody >= index.bodies().size()
            || pair.secondBody >= index.bodies().size()) {
            continue;
        }
        result.insert(collisionAxisSourceId(
            index.bodies().at(pair.firstBody).axisName));
        result.insert(collisionAxisSourceId(
            index.bodies().at(pair.secondBody).axisName));
    }
    return result;
}

Bnd_Box transformCollisionAabb(const Bnd_Box& local, const gp_Trsf& trsf, double gapMm)
{
    Bnd_Box result;
    if (local.IsVoid())
        return result;
    double xmin = 0.0, ymin = 0.0, zmin = 0.0;
    double xmax = 0.0, ymax = 0.0, zmax = 0.0;
    local.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    for (const double x : {xmin, xmax}) {
        for (const double y : {ymin, ymax}) {
            for (const double z : {zmin, zmax}) {
                gp_Pnt point(x, y, z);
                point.Transform(trsf);
                result.Add(point);
            }
        }
    }
    result.SetGap(gapMm);
    return result;
}

Bnd_OBB transformCollisionObb(const Bnd_OBB& local, const gp_Trsf& trsf, double gapMm)
{
    if (local.IsVoid())
        return {};
    gp_Pnt center(local.Center());
    center.Transform(trsf);
    gp_Dir x(local.XDirection()); x.Transform(trsf);
    gp_Dir y(local.YDirection()); y.Transform(trsf);
    gp_Dir z(local.ZDirection()); z.Transform(trsf);
    Bnd_OBB result(center, x, y, z,
                   local.XHSize(), local.YHSize(), local.ZHSize());
    result.Enlarge(gapMm);
    return result;
}

void buildCollisionGeometry(TravelCollisionBody* body, double meshDeflectionMm)
{
    if (!body || body->sourceShape.IsNull())
        return;
    BRepBuilderAPI_Copy copy(body->sourceShape, true, true);
    const TopoDS_Shape shape = copy.IsDone() ? copy.Shape() : TopoDS_Shape{};
    if (shape.IsNull())
        return;

    BRepBndLib::AddOptimal(shape, body->localAabb, false, false);
    BRepBndLib::AddOBB(shape, body->localObb, false, false, false);
    NCollection_Map<TopoDS_Shape, TopTools_ShapeMapHasher> seen;
    const auto append = [&body, &seen](const TopoDS_Shape& leafShape) {
        if (leafShape.IsNull() || !seen.Add(leafShape))
            return;
        TravelCollisionLeaf leaf;
        leaf.shape = leafShape;
        BRepBndLib::AddOptimal(leafShape, leaf.localAabb, false, false);
        BRepBndLib::AddOBB(leafShape, leaf.localObb, false, false, false);
        if (!leaf.localAabb.IsVoid() && !leaf.localObb.IsVoid())
            body->leaves.append(std::move(leaf));
    };
    for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More(); explorer.Next())
        append(explorer.Current());
    if (body->leaves.isEmpty()) {
        for (TopExp_Explorer explorer(shape, TopAbs_SHELL); explorer.More(); explorer.Next())
            append(explorer.Current());
    }
    if (body->leaves.isEmpty())
        append(shape);

    if (!body->surfaceModel.isValid()) {
        body->surfaceModel = cam_algo::SurfaceCollisionModel::build(
            shape, meshDeflectionMm);
    }
    if (body->active && body->surfaceModel.isValid()
        && body->sphereCover.isEmpty()) {
        body->sphereCover = buildConservativeSphereCover(body->surfaceModel);
        body->sphereHierarchy =
            buildConservativeSphereHierarchy(body->sphereCover);
    }
    if (body->passive && body->surfaceModel.isValid()
        && body->surfaceModel.isClosedSolid()
        && !body->localClearanceField) {
        body->localClearanceField = std::make_shared<LocalClearanceField>(
            body->surfaceModel);
    }
}

namespace {

quint64 collisionPairKey(int firstBody, int secondBody)
{
    const quint32 first = static_cast<quint32>(std::min(firstBody, secondBody));
    const quint32 second = static_cast<quint32>(std::max(firstBody, secondBody));
    return (static_cast<quint64>(first) << 32) | second;
}

} // namespace

CollisionPairProofOwner collisionPairProofOwner(
    const TravelCollisionGeometryCache& geometry,
    int firstBody,
    int secondBody)
{
    if (firstBody < 0 || secondBody < 0
        || firstBody >= geometry.bodies.size()
        || secondBody >= geometry.bodies.size()
        || firstBody == secondBody) {
        return CollisionPairProofOwner::None;
    }
    const auto& first = geometry.bodies.at(firstBody);
    const auto& second = geometry.bodies.at(secondBody);
    if (first.source == second.source)
        return CollisionPairProofOwner::None;
    if (first.workpiece || second.workpiece) {
        if ((!first.active && !second.active)
            || (!first.passive && !second.passive)) {
            return CollisionPairProofOwner::None;
        }
        return CollisionPairProofOwner::JobOverlay;
    }
    return geometry.machineAxisPairKeys.contains(
               collisionAxisPairKey(first.source, second.source))
        ? CollisionPairProofOwner::MachinePackage
        : CollisionPairProofOwner::None;
}

QVector<QPair<int, int>> collisionProofPairs(
    const TravelCollisionGeometryCache& geometry,
    bool machineOnly)
{
    QVector<QPair<int, int>> result;
    for (int firstBody = 0; firstBody < geometry.bodies.size(); ++firstBody) {
        for (int secondBody = firstBody + 1;
             secondBody < geometry.bodies.size(); ++secondBody) {
            const auto owner = collisionPairProofOwner(
                geometry, firstBody, secondBody);
            if (owner == CollisionPairProofOwner::None)
                continue;
            if (machineOnly && owner != CollisionPairProofOwner::MachinePackage)
                continue;
            result.append({firstBody, secondBody});
        }
    }
    return result;
}

bool collisionPairUsesCoalFallback(const TravelCollisionBody& first,
                                   const TravelCollisionBody& second)
{
    // Whole-mesh Coal traversal is reserved for compact residual pairs. The
    // workpiece-local field owns the common machine/workpiece proof; large
    // meshes remain BoundaryUnknown instead of stalling online execution.
    // The leaf-level OCCT path has bounded cancellation points and was already
    // measured at roughly 20 seconds for 5,660 exact calls.
    constexpr std::size_t kMaximumCoalPairTriangles = 100'000;
    const std::size_t firstTriangles = first.surfaceModel.triangleCount();
    const std::size_t secondTriangles = second.surfaceModel.triangleCount();
    if (firstTriangles > kMaximumCoalPairTriangles
        || secondTriangles > kMaximumCoalPairTriangles
        || firstTriangles > kMaximumCoalPairTriangles - secondTriangles) {
        return false;
    }
    return firstTriangles + secondTriangles <= kMaximumCoalPairTriangles;
}

void buildCoalCollisionPairs(TravelCollisionGeometryCache* geometry)
{
    if (!geometry)
        return;
    geometry->coalPairs.clear();
    for (int first = 0; first < geometry->bodies.size(); ++first) {
        for (int second = first + 1; second < geometry->bodies.size(); ++second) {
            const auto& firstBody = geometry->bodies.at(first);
            const auto& secondBody = geometry->bodies.at(second);
            if (collisionPairProofOwner(*geometry, first, second)
                == CollisionPairProofOwner::None) {
                continue;
            }
            if (!collisionPairUsesCoalFallback(firstBody, secondBody))
                continue;
            auto& mutableFirst = geometry->bodies[first];
            auto& mutableSecond = geometry->bodies[second];
            if (!mutableFirst.coalModel) {
                mutableFirst.coalModel = buildCoalCollisionBodyModel(
                    mutableFirst.surfaceModel);
            }
            if (!mutableSecond.coalModel) {
                mutableSecond.coalModel = buildCoalCollisionBodyModel(
                    mutableSecond.surfaceModel);
            }
            auto pair = buildCoalCollisionPair(mutableFirst.coalModel,
                                               mutableSecond.coalModel);
            if (pair)
                geometry->coalPairs.insert(collisionPairKey(first, second),
                                           std::move(pair));
        }
    }
}

std::shared_ptr<CoalCollisionPair> coalCollisionPair(
    const TravelCollisionGeometryCache& geometry, int firstBody, int secondBody)
{
    return geometry.coalPairs.value(collisionPairKey(firstBody, secondBody));
}

TravelPlanKey collisionGeometryKey(TravelPlanKey key)
{
    key.toolpathRevision = 0;
    key.orderHash = 0;
    key.motionProfileHash = 0;
    key.compensationOffsetX = 0.0;
    key.compensationOffsetY = 0.0;
    return key;
}

} // namespace lcnc::cam
