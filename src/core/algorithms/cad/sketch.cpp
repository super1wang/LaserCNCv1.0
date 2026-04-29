#include "core/algorithms/cad/sketch.h"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Standard_Failure.hxx>
#include <gp_Circ.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <QString>

#include <cmath>

namespace lcnc::cad_algo {

namespace {
void setErr(QString* errMsg, const QString& message)
{
    if (errMsg)
        *errMsg = message;
}

gp_Pnt toWorld(const SketchPlane& plane, SketchPoint2d point)
{
    gp_Pnt p = plane.axes.Location();
    p.Translate(gp_Vec(plane.axes.XDirection()).Multiplied(point.x));
    p.Translate(gp_Vec(plane.axes.YDirection()).Multiplied(point.y));
    return p;
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
                              SketchPoint2d center,
                              QString* errMsg)
{
    if (width <= 0.0 || height <= 0.0) {
        setErr(errMsg, QStringLiteral("Rectangle width and height must be > 0"));
        return {};
    }

    try {
        const double halfWidth = width * 0.5;
        const double halfHeight = height * 0.5;
        BRepBuilderAPI_MakePolygon polygon;
        polygon.Add(toWorld(plane, {center.x - halfWidth, center.y - halfHeight}));
        polygon.Add(toWorld(plane, {center.x + halfWidth, center.y - halfHeight}));
        polygon.Add(toWorld(plane, {center.x + halfWidth, center.y + halfHeight}));
        polygon.Add(toWorld(plane, {center.x - halfWidth, center.y + halfHeight}));
        polygon.Close();
        if (!polygon.IsDone()) {
            setErr(errMsg, QStringLiteral("Rectangle wire build failed"));
            return {};
        }
        return polygon.Wire();
    } catch (const Standard_Failure& f) {
        setErr(errMsg, QString::fromUtf8(f.GetMessageString()));
        return {};
    }
}

TopoDS_Wire makeCircleWire(const SketchPlane& plane,
                           double radius,
                           SketchPoint2d center,
                           QString* errMsg)
{
    if (radius <= 0.0) {
        setErr(errMsg, QStringLiteral("Circle radius must be > 0"));
        return {};
    }

    try {
        gp_Ax2 circleAxes(toWorld(plane, center), plane.axes.Direction(), plane.axes.XDirection());
        BRepBuilderAPI_MakeEdge edge(gp_Circ(circleAxes, radius));
        if (!edge.IsDone()) {
            setErr(errMsg, QStringLiteral("Circle edge build failed"));
            return {};
        }
        BRepBuilderAPI_MakeWire wire(edge.Edge());
        if (!wire.IsDone()) {
            setErr(errMsg, QStringLiteral("Circle wire build failed"));
            return {};
        }
        return wire.Wire();
    } catch (const Standard_Failure& f) {
        setErr(errMsg, QString::fromUtf8(f.GetMessageString()));
        return {};
    }
}

TopoDS_Face makeFaceFromWire(const TopoDS_Wire& wire, QString* errMsg)
{
    if (wire.IsNull()) {
        setErr(errMsg, QStringLiteral("Sketch wire is null"));
        return {};
    }

    try {
        BRepBuilderAPI_MakeFace face(wire, true);
        if (!face.IsDone()) {
            setErr(errMsg, QStringLiteral("Sketch face build failed"));
            return {};
        }
        return face.Face();
    } catch (const Standard_Failure& f) {
        setErr(errMsg, QString::fromUtf8(f.GetMessageString()));
        return {};
    }
}

TopoDS_Wire makeLineWire(const SketchPlane& plane,
                         SketchPoint2d start,
                         SketchPoint2d end,
                         QString* errMsg)
{
    const gp_Pnt p1 = toWorld(plane, start);
    const gp_Pnt p2 = toWorld(plane, end);
    if (p1.Distance(p2) <= 1e-9) {
        setErr(errMsg, QStringLiteral("Line endpoints coincide"));
        return {};
    }

    try {
        BRepBuilderAPI_MakeEdge edge(p1, p2);
        if (!edge.IsDone()) {
            setErr(errMsg, QStringLiteral("Line edge build failed"));
            return {};
        }
        BRepBuilderAPI_MakeWire wire(edge.Edge());
        if (!wire.IsDone()) {
            setErr(errMsg, QStringLiteral("Line wire build failed"));
            return {};
        }
        return wire.Wire();
    } catch (const Standard_Failure& f) {
        setErr(errMsg, QString::fromUtf8(f.GetMessageString()));
        return {};
    }
}

TopoDS_Wire makeArcWire(const SketchPlane& plane,
                        SketchPoint2d start,
                        SketchPoint2d mid,
                        SketchPoint2d end,
                        QString* errMsg)
{
    const gp_Pnt p1 = toWorld(plane, start);
    const gp_Pnt p2 = toWorld(plane, mid);
    const gp_Pnt p3 = toWorld(plane, end);
    if (p1.Distance(p3) <= 1e-9 || p1.Distance(p2) <= 1e-9 || p2.Distance(p3) <= 1e-9) {
        setErr(errMsg, QStringLiteral("Arc points must be distinct"));
        return {};
    }

    try {
        GC_MakeArcOfCircle arcMaker(p1, p2, p3);
        if (!arcMaker.IsDone()) {
            setErr(errMsg, QStringLiteral("Arc geometry build failed"));
            return {};
        }
        BRepBuilderAPI_MakeEdge edge(arcMaker.Value());
        if (!edge.IsDone()) {
            setErr(errMsg, QStringLiteral("Arc edge build failed"));
            return {};
        }
        BRepBuilderAPI_MakeWire wire(edge.Edge());
        if (!wire.IsDone()) {
            setErr(errMsg, QStringLiteral("Arc wire build failed"));
            return {};
        }
        return wire.Wire();
    } catch (const Standard_Failure& f) {
        setErr(errMsg, QString::fromUtf8(f.GetMessageString()));
        return {};
    }
}

TopoDS_Wire makePolygonWire(const SketchPlane& plane,
                            int sides,
                            double radius,
                            SketchPoint2d center,
                            QString* errMsg)
{
    if (sides < 3) {
        setErr(errMsg, QStringLiteral("Polygon must have at least 3 sides"));
        return {};
    }
    if (radius <= 0.0) {
        setErr(errMsg, QStringLiteral("Polygon radius must be > 0"));
        return {};
    }

    try {
        BRepBuilderAPI_MakePolygon polygon;
        for (int i = 0; i < sides; ++i) {
            const double theta = (2.0 * M_PI * static_cast<double>(i)) / static_cast<double>(sides);
            polygon.Add(toWorld(plane,
                                {center.x + radius * std::cos(theta),
                                 center.y + radius * std::sin(theta)}));
        }
        polygon.Close();
        if (!polygon.IsDone()) {
            setErr(errMsg, QStringLiteral("Polygon wire build failed"));
            return {};
        }
        return polygon.Wire();
    } catch (const Standard_Failure& f) {
        setErr(errMsg, QString::fromUtf8(f.GetMessageString()));
        return {};
    }
}

} // namespace lcnc::cad_algo