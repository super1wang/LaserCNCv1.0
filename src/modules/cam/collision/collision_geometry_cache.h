#pragma once

#include "core/algorithms/cam/collision_safety_domain.h"
#include "core/algorithms/cam/surface_collision_prefilter.h"
#include "core/kinematics/machine_configuration_service.h"
#include "core/kinematics/machine_kinematics.h"
#include "modules/cam/contracts/toolpath_export_dto.h"

#include <Bnd_Box.hxx>
#include <Bnd_OBB.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Trsf.hxx>

#include <QMap>
#include <QString>
#include <QVector>

#include <memory>

namespace lcnc::cam {

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
    bool cutterProxy{false};
    bool workpiece{false};
    TopoDS_Shape sourceShape;
    QVector<TravelCollisionLeaf> leaves;
    cam_algo::SurfaceCollisionModel surfaceModel;
    Bnd_Box localAabb;
    Bnd_OBB localObb;
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
    std::shared_ptr<cam_algo::CollisionSafetyDomainCache> safetyDomain{
        std::make_shared<cam_algo::CollisionSafetyDomainCache>()};
};

QString collisionAxisSourceId(const QString& axisName);
Bnd_Box transformCollisionAabb(const Bnd_Box& local, const gp_Trsf& trsf, double gapMm);
Bnd_OBB transformCollisionObb(const Bnd_OBB& local, const gp_Trsf& trsf, double gapMm);
void buildCollisionGeometry(TravelCollisionBody* body, double meshDeflectionMm);
TravelPlanKey collisionGeometryKey(TravelPlanKey key);

} // namespace lcnc::cam
