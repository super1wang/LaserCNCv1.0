#include "core/algorithms/cad/measure.h"
#include "core/algorithms/occt_exact_operation_lock.h"

#include <BRepAdaptor_Surface.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Geom_Surface.hxx>
#include <GeomLProp_SLProps.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace lcnc::cad_algo {

double minDistance(const TopoDS_Shape& a, const TopoDS_Shape& b)
{
    if (a.IsNull() || b.IsNull())
        throw std::invalid_argument("Distance input shape is null");
    lcnc::OcctExactOperationLock exactOperationLock;
    BRepExtrema_DistShapeShape distance(a, b);
    distance.Perform();
    if (!distance.IsDone())
        throw Standard_Failure("Distance calculation failed");
    return distance.Value();
}

gp_Vec firstFaceNormal(const TopoDS_Shape& shape)
{
    if (shape.IsNull())
        throw std::invalid_argument("Normal input shape is null");
    TopExp_Explorer explorer(shape, TopAbs_FACE);
    if (!explorer.More())
        return {};
    const TopoDS_Face& face = TopoDS::Face(explorer.Current());
    BRepAdaptor_Surface surface(face);
    const double u = 0.5 * (surface.FirstUParameter() + surface.LastUParameter());
    const double v = 0.5 * (surface.FirstVParameter() + surface.LastVParameter());
    GeomLProp_SLProps properties(surface.Surface().Surface(), u, v, 1, 1e-6);
    if (!properties.IsNormalDefined())
        return {};
    return gp_Vec(properties.Normal());
}

double angleBetween(const gp_Vec& a, const gp_Vec& b)
{
    const double la = a.Magnitude();
    const double lb = b.Magnitude();
    if (la < 1e-9 || lb < 1e-9) return -1.0;
    double cosA = a.Dot(b) / (la * lb);
    cosA = std::max(-1.0, std::min(1.0, cosA));
    return std::acos(cosA) * 180.0 / M_PI;
}

double surfaceArea(const TopoDS_Shape& shape)
{
    if (shape.IsNull())
        throw std::invalid_argument("Surface-area input shape is null");
    GProp_GProps properties;
    BRepGProp::SurfaceProperties(shape, properties);
    return properties.Mass();
}

} // namespace lcnc::cad_algo
