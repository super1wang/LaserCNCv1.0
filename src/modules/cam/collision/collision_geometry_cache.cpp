#include "modules/cam/collision/collision_geometry_cache.h"

#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_MapOfShape.hxx>

#include <TopAbs_ShapeEnum.hxx>

namespace lcnc::cam {

QString collisionAxisSourceId(const QString& axisName)
{
    return QStringLiteral("axis:") + axisName.trimmed().toUpper();
}

Bnd_Box transformCollisionAabb(const Bnd_Box& local, const gp_Trsf& trsf, double gapMm)
{
    Bnd_Box result;
    if (local.IsVoid())
        return result;
    Standard_Real xmin = 0.0, ymin = 0.0, zmin = 0.0;
    Standard_Real xmax = 0.0, ymax = 0.0, zmax = 0.0;
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
    BRepBuilderAPI_Copy copy(body->sourceShape, Standard_True, Standard_True);
    const TopoDS_Shape shape = copy.IsDone() ? copy.Shape() : TopoDS_Shape{};
    if (shape.IsNull())
        return;

    BRepBndLib::AddOptimal(shape, body->localAabb, Standard_False, Standard_False);
    BRepBndLib::AddOBB(shape, body->localObb,
                       Standard_False, Standard_False, Standard_False);
    TopTools_MapOfShape seen;
    const auto append = [&body, &seen](const TopoDS_Shape& leafShape) {
        if (leafShape.IsNull() || !seen.Add(leafShape))
            return;
        TravelCollisionLeaf leaf;
        leaf.shape = leafShape;
        BRepBndLib::AddOptimal(leafShape, leaf.localAabb,
                               Standard_False, Standard_False);
        BRepBndLib::AddOBB(leafShape, leaf.localObb,
                           Standard_False, Standard_False, Standard_False);
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

    body->surfaceModel = cam_algo::SurfaceCollisionModel::build(shape, meshDeflectionMm);
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
