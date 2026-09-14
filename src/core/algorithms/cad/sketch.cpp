#include "core/algorithms/cad/sketch.h"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <Standard_Failure.hxx>
#include <gp_Circ.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <cmath>
#include <stdexcept>

namespace lcnc::cad_algo {

namespace {

gp_Pnt toWorld(const SketchPlane& plane, SketchPoint2d point)
{
    gp_Pnt result = plane.axes.Location();
    result.Translate(gp_Vec(plane.axes.XDirection()).Multiplied(point.x));
    result.Translate(gp_Vec(plane.axes.YDirection()).Multiplied(point.y));
    return result;
}

TopoDS_Wire checkedWire(BRepBuilderAPI_MakeWire& builder, const char* failure)
{
    if (!builder.IsDone())
        throw Standard_Failure(failure);
    return builder.Wire();
}

} // namespace

SketchPlane SketchPlane::xy()
{
    return {gp_Ax3(gp::Origin(), gp::DZ(), gp::DX())};
}

SketchPlane SketchPlane::yz()
{
    return {gp_Ax3(gp::Origin(), gp::DX(), gp::DY())};
}

SketchPlane SketchPlane::zx()
{
    return {gp_Ax3(gp::Origin(), gp::DY(), gp::DZ())};
}

TopoDS_Wire makeRectangleWire(const SketchPlane& plane,
                              double width,
                              double height,
                              SketchPoint2d center)
{
    if (width <= 0.0 || height <= 0.0)
        throw std::invalid_argument("Rectangle width and height must be > 0");

    const double halfWidth = width * 0.5;
    const double halfHeight = height * 0.5;
    BRepBuilderAPI_MakePolygon polygon;
    polygon.Add(toWorld(plane, {center.x - halfWidth, center.y - halfHeight}));
    polygon.Add(toWorld(plane, {center.x + halfWidth, center.y - halfHeight}));
    polygon.Add(toWorld(plane, {center.x + halfWidth, center.y + halfHeight}));
    polygon.Add(toWorld(plane, {center.x - halfWidth, center.y + halfHeight}));
    polygon.Close();
    if (!polygon.IsDone())
        throw Standard_Failure("Rectangle wire build failed");
    return polygon.Wire();
}

TopoDS_Wire makeCircleWire(const SketchPlane& plane,
                           double radius,
                           SketchPoint2d center)
{
    if (radius <= 0.0)
        throw std::invalid_argument("Circle radius must be > 0");

    gp_Ax2 circleAxes(toWorld(plane, center), plane.axes.Direction(), plane.axes.XDirection());
    BRepBuilderAPI_MakeEdge edge(gp_Circ(circleAxes, radius));
    if (!edge.IsDone())
        throw Standard_Failure("Circle edge build failed");
    BRepBuilderAPI_MakeWire wire(edge.Edge());
    return checkedWire(wire, "Circle wire build failed");
}

TopoDS_Face makeFaceFromWire(const TopoDS_Wire& wire)
{
    if (wire.IsNull())
        throw std::invalid_argument("Sketch wire is null");
    BRepBuilderAPI_MakeFace face(wire, true);
    if (!face.IsDone())
        throw Standard_Failure("Sketch face build failed");
    return face.Face();
}

TopoDS_Wire makeLineWire(const SketchPlane& plane,
                         SketchPoint2d start,
                         SketchPoint2d end)
{
    const gp_Pnt first = toWorld(plane, start);
    const gp_Pnt second = toWorld(plane, end);
    if (first.Distance(second) <= 1.0e-9)
        throw std::invalid_argument("Line endpoints coincide");

    BRepBuilderAPI_MakeEdge edge(first, second);
    if (!edge.IsDone())
        throw Standard_Failure("Line edge build failed");
    BRepBuilderAPI_MakeWire wire(edge.Edge());
    return checkedWire(wire, "Line wire build failed");
}

TopoDS_Wire makeArcWire(const SketchPlane& plane,
                        SketchPoint2d start,
                        SketchPoint2d mid,
                        SketchPoint2d end)
{
    const gp_Pnt first = toWorld(plane, start);
    const gp_Pnt middle = toWorld(plane, mid);
    const gp_Pnt last = toWorld(plane, end);
    if (first.Distance(last) <= 1.0e-9
        || first.Distance(middle) <= 1.0e-9
        || middle.Distance(last) <= 1.0e-9) {
        throw std::invalid_argument("Arc points must be distinct");
    }

    GC_MakeArcOfCircle arc(first, middle, last);
    if (!arc.IsDone())
        throw Standard_Failure("Arc geometry build failed");
    BRepBuilderAPI_MakeEdge edge(arc.Value());
    if (!edge.IsDone())
        throw Standard_Failure("Arc edge build failed");
    BRepBuilderAPI_MakeWire wire(edge.Edge());
    return checkedWire(wire, "Arc wire build failed");
}

TopoDS_Wire makePolygonWire(const SketchPlane& plane,
                            int sides,
                            double radius,
                            SketchPoint2d center)
{
    if (sides < 3)
        throw std::invalid_argument("Polygon must have at least 3 sides");
    if (radius <= 0.0)
        throw std::invalid_argument("Polygon radius must be > 0");

    constexpr double kPi = 3.14159265358979323846;
    BRepBuilderAPI_MakePolygon polygon;
    for (int index = 0; index < sides; ++index) {
        const double angle =
            (2.0 * kPi * static_cast<double>(index)) / static_cast<double>(sides);
        polygon.Add(toWorld(plane,
                            {center.x + radius * std::cos(angle),
                             center.y + radius * std::sin(angle)}));
    }
    polygon.Close();
    if (!polygon.IsDone())
        throw Standard_Failure("Polygon wire build failed");
    return polygon.Wire();
}

} // namespace lcnc::cad_algo
