#include "core/algorithms/cam/laser_toolpath.h"

#include <QCoreApplication>
#include <QTextStream>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRep_Tool.hxx>
#include <Geom_BSplineCurve.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <array>

namespace {

int fail(const QString& message)
{
    QTextStream(stderr) << message << '\n';
    return 1;
}

ToolpathPoint sample(const gp_Pnt& position, int sourceEdgeIndex)
{
    ToolpathPoint point;
    point.position = position;
    point.normal = gp_Dir(0.0, 0.0, 1.0);
    point.tangent = gp_Dir(1.0, 0.0, 0.0);
    point.sourceEdgeIndex = sourceEdgeIndex;
    return point;
}

LaserContour makeRectangleContour()
{
    const std::array<gp_Pnt, 4> corners{
        gp_Pnt(0.0, 0.0, 0.0),
        gp_Pnt(20.0, 0.0, 0.0),
        gp_Pnt(20.0, 10.0, 0.0),
        gp_Pnt(0.0, 10.0, 0.0)};

    BRepBuilderAPI_MakeWire wire;
    for (std::size_t index = 0; index < corners.size(); ++index) {
        BRepBuilderAPI_MakeEdge edge(
            corners[index], corners[(index + 1) % corners.size()]);
        if (edge.IsDone())
            wire.Add(edge.Edge());
    }

    LaserContour contour;
    if (wire.IsDone())
        contour.wire = wire.Wire();
    for (std::size_t index = 0; index < corners.size(); ++index)
        contour.points.push_back(sample(corners[index], static_cast<int>(index)));
    contour.points.push_back(sample(corners.front(), 0));
    contour.appliedParams.cuttingOffsetMm = 2.0;
    return contour;
}

int verifySharpRectangleOffset()
{
    const LaserContour contour = makeRectangleContour();
    if (contour.wire.IsNull())
        return fail(QStringLiteral("rectangle source wire construction failed"));

    const TopoDS_Shape display =
        LaserToolpathBuilder::buildOffsetDisplayShape(contour);
    if (display.IsNull() || display.ShapeType() != TopAbs_EDGE)
        return fail(QStringLiteral("offset display is not one compact polyline edge"));

    double firstParameter = 0.0;
    double lastParameter = 0.0;
    const Handle(Geom_Curve) curve = BRep_Tool::Curve(
        TopoDS::Edge(display), firstParameter, lastParameter);
    const Handle(Geom_BSplineCurve) polyline =
        Handle(Geom_BSplineCurve)::DownCast(curve);
    if (polyline.IsNull() || polyline->Degree() != 1)
        return fail(QStringLiteral("offset display is not an exact degree-one polyline"));
    if (polyline->NbPoles() != 5 || polyline->NbKnots() != 5
        || polyline->Multiplicity(1) != 2
        || polyline->Multiplicity(polyline->NbKnots()) != 2) {
        return fail(QStringLiteral("offset display lost a rectangle corner or closure"));
    }

    const std::array<gp_Pnt, 5> expected{
        gp_Pnt(0.0, 0.0, 2.0),
        gp_Pnt(20.0, 0.0, 2.0),
        gp_Pnt(20.0, 10.0, 2.0),
        gp_Pnt(0.0, 10.0, 2.0),
        gp_Pnt(0.0, 0.0, 2.0)};
    for (int index = 1; index <= polyline->NbPoles(); ++index) {
        if (polyline->Pole(index).Distance(expected[static_cast<std::size_t>(index - 1)])
            > 1.0e-9) {
            return fail(QStringLiteral(
                "offset display moved an executable sample or rounded a sharp corner"));
        }
    }
    return 0;
}

int verifyZeroOffsetPreservesTopology()
{
    LaserContour contour = makeRectangleContour();
    contour.appliedParams.cuttingOffsetMm = 0.0;
    const TopoDS_Shape display =
        LaserToolpathBuilder::buildOffsetDisplayShape(contour);
    if (display.IsNull() || !display.IsSame(contour.wire)) {
        return fail(QStringLiteral(
            "zero cutting offset rebuilt or changed the original contour topology"));
    }
    return 0;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    if (const int rc = verifySharpRectangleOffset(); rc != 0)
        return rc;
    return verifyZeroOffsetPreservesTopology();
}
