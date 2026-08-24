#pragma once

#include "core/algorithms/cam/collision_safety_domain.h"
#include "core/algorithms/cam/surface_collision_prefilter.h"
#include "core/kinematics/machine_configuration_service.h"
#include "core/kinematics/machine_kinematics.h"
#include "modules/cam/contracts/toolpath_export_dto.h"
#include "modules/cam/collision/workpiece_clearance_field.h"

#include <Bnd_Box.hxx>
#include <Bnd_OBB.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Trsf.hxx>

#include <QMap>
#include <QHash>
#include <QPair>
#include <QSet>
#include <QString>
#include <QVector>

#include <memory>

namespace lcnc::cam_algo {
class MachineSafetyIndex;
}

namespace lcnc::cam {

class CoalCollisionBodyModel;
class CoalCollisionPair;

struct TravelCollisionLeaf
{
    TopoDS_Shape shape;
    Bnd_Box localAabb;
    Bnd_OBB localObb;
};

struct TravelCollisionBody
{
    QString entry;
    QString source;
    bool active{false};
    bool passive{false};
    bool workpiece{false};
    TopoDS_Shape sourceShape;
    QVector<TravelCollisionLeaf> leaves;
    cam_algo::SurfaceCollisionModel surfaceModel;
    std::shared_ptr<CoalCollisionBodyModel> coalModel;
    QVector<ConservativeCollisionSphere> sphereCover;
    QVector<ConservativeCollisionSphereNode> sphereHierarchy;
    std::shared_ptr<LocalClearanceField> localClearanceField;
    Bnd_Box localAabb;
    Bnd_OBB localObb;
};

enum class CollisionPairProofOwner
{
    None,
    MachinePackage,
    JobOverlay
};

/// Immutable geometry and sparse collision certificates shared by verification
/// workers. The cache key excludes ordering and motion-profile fields so a
/// machine/workpiece environment can be reused across plans.
struct TravelCollisionGeometryCache
{
    TravelPlanKey key;
    QVector<TravelCollisionBody> bodies;
    QVector<MachineAxisDef> axes;
    QString configType;
    gp_Trsf workpieceSetup;
    QMap<QString, QString> assignments;
    QMap<QString, QString> mounts;
    MachineModeDefinition definition;
    /// Axis-source pairs copied from the loaded LMSI. They are the only
    /// machine/machine pairs that the online geometric fallback may certify.
    /// Job Overlay eligibility remains controlled by active/passive roles.
    QSet<QString> machineAxisPairKeys;
    std::shared_ptr<cam_algo::CollisionSafetyDomainCache> safetyDomain{
        std::make_shared<cam_algo::CollisionSafetyDomainCache>()};
    QHash<quint64, std::shared_ptr<CoalCollisionPair>> coalPairs;
};

QString collisionAxisSourceId(const QString& axisName);
QString collisionAxisPairKey(const QString& firstSource,
                             const QString& secondSource);
QSet<QString> machineCollisionAxisPairKeys(
    const cam_algo::MachineSafetyIndex& index);
QSet<QString> machineCollisionAxisSources(
    const cam_algo::MachineSafetyIndex& index);
/// Resolves both pair eligibility and proof ownership. Rigidly related bodies
/// are excluded unless the loaded machine package explicitly owns their pair.
/// Workpiece geometry is always the sole Job Overlay responsibility.
CollisionPairProofOwner collisionPairProofOwner(
    const TravelCollisionGeometryCache& geometry,
    int firstBody,
    int secondBody);
QVector<QPair<int, int>> collisionProofPairs(
    const TravelCollisionGeometryCache& geometry,
    bool machineOnly);
Bnd_Box transformCollisionAabb(const Bnd_Box& local, const gp_Trsf& trsf, double gapMm);
Bnd_OBB transformCollisionObb(const Bnd_OBB& local, const gp_Trsf& trsf, double gapMm);
void buildCollisionGeometry(TravelCollisionBody* body, double meshDeflectionMm);
/// Returns true only for pairs small enough for predictable whole-mesh Coal
/// traversal. Workpiece-local fields own machine/workpiece queries; compact
/// residual pairs use Coal before the optional OCCT audit backend.
bool collisionPairUsesCoalFallback(const TravelCollisionBody& first,
                                   const TravelCollisionBody& second);
void buildCoalCollisionPairs(TravelCollisionGeometryCache* geometry);
std::shared_ptr<CoalCollisionPair> coalCollisionPair(
    const TravelCollisionGeometryCache& geometry, int firstBody, int secondBody);
TravelPlanKey collisionGeometryKey(TravelPlanKey key);

} // namespace lcnc::cam
