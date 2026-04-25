#include "core/algorithms/cad/measure.h"

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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace lcnc::cad_algo {

double minDistance(const TopoDS_Shape& a, const TopoDS_Shape& b)
{
    if (a.IsNull() || b.IsNull()) return -1.0;
    try {
        BRepExtrema_DistShapeShape dss(a, b);
        dss.Perform();
        if (!dss.IsDone()) return -1.0;
        return dss.Value();
    } catch (const Standard_Failure&) {
        return -1.0;
    }
}

gp_Vec firstFaceNormal(const TopoDS_Shape& shape)
{
    if (shape.IsNull()) return {};
    try {
        TopExp_Explorer ex(shape, TopAbs_FACE);
        if (!ex.More()) return {};
        const TopoDS_Face& face = TopoDS::Face(ex.Current());
        BRepAdaptor_Surface surf(face);
        const double u = 0.5 * (surf.FirstUParameter() + surf.LastUParameter());
        const double v = 0.5 * (surf.FirstVParameter() + surf.LastVParameter());
        GeomLProp_SLProps props(surf.Surface().Surface(), u, v, 1, 1e-6);
        if (!props.IsNormalDefined()) return {};
        return gp_Vec(props.Normal());
    } catch (const Standard_Failure&) {
        return {};
    }
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
    if (shape.IsNull()) return 0.0;
    try {
        GProp_GProps props;
        BRepGProp::SurfaceProperties(shape, props);
        return props.Mass();
    } catch (const Standard_Failure&) {
        return 0.0;
    }
}

} // namespace lcnc::cad_algo
