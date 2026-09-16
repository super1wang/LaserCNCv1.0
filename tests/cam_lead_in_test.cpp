#include "core/algorithms/cam/face_classifier.h"
#include "core/algorithms/cam/laser_toolpath.h"
#include "core/document/lcnc_document.h"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <NCollection_Sequence.hxx>
#include <QCoreApplication>
#include <QFileInfo>
#include <QTextStream>
#include <STEPCAFControl_Reader.hxx>
#include <TDF_Label.hxx>
#include <TDocStd_Document.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <XCAFDoc_Documenttool.hxx>
#include <XCAFDoc_Shapetool.hxx>
#include <algorithm>
#include <cmath>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Elips.hxx>
#include <gp_Lin.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

namespace {

int fail(const QString& message)
{
    QTextStream(stderr) << message << '\n';
    return 1;
}

int verifySourceSeamAndAnchor()
{
    const gp_Pnt a(0.0, 0.0, 0.0);
    const gp_Pnt b(10.0, 0.0, 0.0);
    const gp_Pnt c(10.0, 10.0, 0.0);
    BRepBuilderAPI_MakeWire wire;
    wire.Add(BRepBuilderAPI_MakeEdge(a, b));
    wire.Add(BRepBuilderAPI_MakeEdge(b, c));
    if (!wire.IsDone())
        return fail(QStringLiteral("source-seam wire construction failed"));
    LaserContour contour;
    contour.wire = wire.Wire();
    contour.leadIn.valid = true;
    contour.leadIn.entryEdgeIndex = 0;
    contour.leadIn.entryParam = 0.0;
    LaserToolpathBuilder::discretizeContour(contour, contour.wire, 0.1);
    bool foundSeam = false;
    bool foundAnchor = false;
    for (std::size_t i = 0; i < contour.points.size(); ++i) {
        const ToolpathPoint& point = contour.points[i];
        if (point.sourceEdgeIndex == 0 && std::abs(point.param) <= 1e-12) {
            foundAnchor = true;
            if (!std::isfinite(point.normal.X()) || !std::isfinite(point.tangent.X()))
                return fail(QStringLiteral("source anchor lost its derived directions"));
        }
        if (i > 0 && contour.points[i - 1].sourceEdgeIndex == 0
            && point.sourceEdgeIndex == 1
            && contour.points[i - 1].position.SquareDistance(point.position) <= 1e-24)
            foundSeam = true;
    }
    if (!foundAnchor || !foundSeam)
        return fail(QStringLiteral("OCC edge seam or lead-in source anchor was discarded"));
    for (ToolpathPoint& point : contour.points) {
        point.machineCoord.valid = true;
        point.machineCoord.solvedPose.valid = true;
    }
    std::vector<ToolpathPoint> remapped = contour.points;
    remapped.front().sourceEdgeIndex = 7;
    LaserToolpathBuilder::replaceGeometrySamples(contour, std::move(remapped));
    for (const ToolpathPoint& point : contour.points) {
        if (point.machineCoord.valid || point.machineCoord.solvedPose.valid)
            return fail(QStringLiteral("geometry source remap reused an old machine solve"));
    }
    return 0;
}

int verifyBoundedGeometryRefinement()
{
    const gp_Circ circle(gp_Ax2(gp_Pnt(0, 0, 5), gp_Dir(0, 0, 1)), 10.0);
    BRepBuilderAPI_MakeWire wire;
    wire.Add(BRepBuilderAPI_MakeEdge(circle, 0.0, 1.5707963267948966));
    if (!wire.IsDone())
        return fail(QStringLiteral("geometry refinement arc construction failed"));
    const TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(10.0, 10.0).Shape();
    LaserContour baseline;
    baseline.wire = wire.Wire();
    GeometrySamplingPolicy loose;
    loose.maxTangentStepDeg = 180.0;
    loose.maxNormalStepDeg = 180.0;
    LaserToolpathBuilder::discretizeContour(baseline, cylinder, 10.0, loose);
    LaserContour tangent = baseline;
    GeometrySamplingPolicy tangentPolicy = loose;
    tangentPolicy.maxTangentStepDeg = 5.0;
    tangentPolicy.maxSampleMultiplier = 64;
    LaserToolpathBuilder::discretizeContour(tangent, cylinder, 10.0, tangentPolicy);
    if (!tangent.geometrySamplingComplete || tangent.points.size() <= baseline.points.size())
        return fail(QStringLiteral("tangent variation did not refine OCC source interval"));
    LaserContour normal = baseline;
    GeometrySamplingPolicy normalPolicy = loose;
    normalPolicy.maxNormalStepDeg = 5.0;
    normalPolicy.maxSampleMultiplier = 64;
    LaserToolpathBuilder::discretizeContour(normal, cylinder, 10.0, normalPolicy);
    if (!normal.geometrySamplingComplete || normal.points.size() <= baseline.points.size())
        return fail(QStringLiteral("surface-normal variation did not refine OCC source interval"));
    LaserContour fenced = baseline;
    GeometrySamplingPolicy fencedPolicy = loose;
    fencedPolicy.processBarriers.push_back({0, 0.7853981633974483});
    LaserToolpathBuilder::discretizeContour(fenced, cylinder, 10.0, fencedPolicy);
    const bool foundFence = std::any_of(fenced.points.begin(), fenced.points.end(),
        [](const ToolpathPoint& point) {
            return point.semanticHardBarrier
                && std::abs(point.param - 0.7853981633974483) < 1e-10;
        });
    if (!fenced.geometrySamplingComplete || !foundFence)
        return fail(QStringLiteral("process hard barrier was lost during source refinement"));
    LaserContour exhausted = baseline;
    GeometrySamplingPolicy limited = normalPolicy;
    limited.maxSubdivisionDepth = 0;
    LaserToolpathBuilder::discretizeContour(exhausted, cylinder, 10.0, limited);
    if (exhausted.geometrySamplingComplete
        || exhausted.points.size() < baseline.points.size())
        return fail(QStringLiteral("geometry budget exhaustion silently certified a sparse path"));
    return 0;
}

int verifySourceTopologyAndTrimmedGeometry()
{
    const gp_Pnt a(0, 0, 0), b(4, 0, 0), c(4, 3, 0), d(0, 3, 0);
    BRepBuilderAPI_MakeWire rectangle;
    rectangle.Add(BRepBuilderAPI_MakeEdge(a, b));
    rectangle.Add(BRepBuilderAPI_MakeEdge(b, c));
    rectangle.Add(BRepBuilderAPI_MakeEdge(c, d));
    rectangle.Add(BRepBuilderAPI_MakeEdge(d, a));
    if (!rectangle.IsDone())
        return fail(QStringLiteral("closed source wire construction failed"));
    LaserContour closed;
    closed.wire = rectangle.Wire();
    LaserToolpathBuilder::discretizeContour(closed, closed.wire, 0.1);
    if (!closed.geometrySamplingComplete || closed.points.size() < 8
        || closed.points.front().position.Distance(closed.points.back().position) > 1e-9)
        return fail(QStringLiteral("closed contour lost its closing source point"));
    int lastEdge = -1;
    for (std::size_t i = 0; i < closed.points.size(); ++i) {
        const auto& point = closed.points[i];
        if (point.sourceEdgeIndex < lastEdge)
            return fail(QStringLiteral("closed contour source wire order changed"));
        lastEdge = point.sourceEdgeIndex;
        if (i > 0 && point.sourceEdgeIndex == closed.points[i - 1].sourceEdgeIndex
            && std::abs(point.param - closed.points[i - 1].param) <= 1e-12)
            return fail(QStringLiteral("strict duplicate source sample survived"));
    }

    const gp_Lin line(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0));
    BRepBuilderAPI_MakeWire trimmedLine;
    trimmedLine.Add(BRepBuilderAPI_MakeEdge(line, 2.0, 5.0));
    LaserContour lineContour;
    lineContour.wire = trimmedLine.Wire();
    LaserToolpathBuilder::discretizeContour(lineContour, lineContour.wire, 0.1);
    if (lineContour.points.size() < 2
        || std::abs(lineContour.points.front().param - 2.0) > 1e-9
        || std::abs(lineContour.points.back().param - 5.0) > 1e-9)
        return fail(QStringLiteral("trimmed line lost its source parameter bounds"));
    BRepBuilderAPI_MakeWire reversedLine;
    reversedLine.Add(TopoDS::Edge(BRepBuilderAPI_MakeEdge(line, 2.0, 5.0).Edge().Reversed()));
    LaserContour reverseContour;
    reverseContour.wire = reversedLine.Wire();
    LaserToolpathBuilder::discretizeContour(reverseContour, reverseContour.wire, 0.1);
    if (reverseContour.points.size() < 2
        || reverseContour.points.front().param <= reverseContour.points.back().param)
        return fail(QStringLiteral("reversed OCC edge lost descending source parameters"));

    const gp_Circ circle(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 10.0);
    BRepBuilderAPI_MakeWire trimmedArc;
    trimmedArc.Add(BRepBuilderAPI_MakeEdge(circle, 0.25, 1.25));
    LaserContour arcContour;
    arcContour.wire = trimmedArc.Wire();
    GeometrySamplingPolicy loose;
    loose.maxTangentStepDeg = 180.0;
    loose.maxNormalStepDeg = 180.0;
    LaserToolpathBuilder::discretizeContour(arcContour, arcContour.wire, 0.1, loose);
    if (!arcContour.geometrySamplingComplete || arcContour.points.size() < 3
        || std::abs(arcContour.points.front().param - 0.25) > 1e-9
        || std::abs(arcContour.points.back().param - 1.25) > 1e-9)
        return fail(QStringLiteral("trimmed arc lost its source bounds"));
    for (std::size_t i = 1; i < arcContour.points.size(); ++i) {
        const double span = std::abs(arcContour.points[i].param - arcContour.points[i - 1].param);
        const double sagitta = 10.0 * (1.0 - std::cos(span * 0.5));
        if (sagitta > 0.100001)
            return fail(QStringLiteral("curved source was falsely accepted as a sparse line"));
    }
    return 0;
}

int verifyShape(const TopoDS_Shape& shape, const QString& label)
{
    ContourExtractionParams params;
    params.smoothAngleThresholdDeg = 5.0;
    params.deflection = 0.1;
    params.strategy = ExtractionStrategy::LargestSmoothConnectedSurface;

    FaceClassification classification;
    classification = FaceClassifier::classifyFaces(shape, params.smoothAngleThresholdDeg);
    std::vector<TopoDS_Face> outerFaces;
    std::vector<TopoDS_Face> crossSectionFaces;
    if (const auto* outer = classification.outerGroup())
        outerFaces = outer->faces;
    for (const auto* group : classification.crossSectionGroups())
        crossSectionFaces.insert(crossSectionFaces.end(), group->faces.begin(), group->faces.end());
    auto contours = LaserToolpathBuilder::extractTubeContoursFromFaceGroups(
        shape, outerFaces, crossSectionFaces, params);
    if (contours.empty())
        return fail(label + QStringLiteral(": no machining contours extracted"));
    if (!classification.hasOuter() || !classification.hasCrossSection())
        return fail(label + QStringLiteral(": face classification is incomplete"));

    for (std::size_t contourIndex = 0; contourIndex < contours.size(); ++contourIndex) {
        LaserContour& contour = contours[contourIndex];
        contour.sourceShape = shape;
        contour.leadIn.length = 0.1;

        if (contour.leadInSurfaceContext.empty())
            return fail(label + QStringLiteral(": missing edge surface context"));
        for (const LeadInEdgeSurfaceContext& context : contour.leadInSurfaceContext) {
            if (context.outerFaces.empty() || context.crossSectionFaces.empty()) {
                return fail(label
                    + QStringLiteral(": contour edge is not bound to both adjacent face kinds"));
            }
        }

        QString error;
        if (!LaserToolpathBuilder::setAutomaticContourStart(contour, &error)
            || !contour.leadInSolution.valid) {
            return fail(label + QStringLiteral(": contour %1 lead-in failed: %2")
                .arg(contourIndex).arg(error));
        }

        const ToolpathPoint& start = contour.points.front();
        if (!start.crossSectionNormalValid)
            return fail(label + QStringLiteral(": missing adjacent cross-section normal"));

        const gp_Vec outer(start.normal);
        const gp_Vec cross(start.crossSectionNormal);
        gp_Vec projected = cross - outer * cross.Dot(outer);
        if (projected.Magnitude() <= 1e-9)
            return fail(label + QStringLiteral(": degenerate projected cross-section normal"));
        projected.Normalize();

        const double alignment =
            projected.Dot(gp_Vec(contour.leadInSolution.direction));
        if (alignment < 0.5) {
            return fail(label
                + QStringLiteral(": lead-in does not point toward the suspended cross-section side"));
        }
    }

    return 0;
}

int verifyImportedShape(const TopoDS_Shape& shape, const QString& label)
{
    ContourExtractionParams params;
    params.smoothAngleThresholdDeg = 5.0;
    params.deflection = 0.1;
    params.strategy = ExtractionStrategy::LargestSmoothConnectedSurface;

    FaceClassification classification;
    auto contours = LaserToolpathBuilder::extractContours(
        shape, params, &classification);
    if (contours.empty())
        return 0;

    for (std::size_t contourIndex = 0; contourIndex < contours.size(); ++contourIndex) {
        LaserContour& contour = contours[contourIndex];
        contour.sourceShape = shape;
        contour.leadIn.length = 0.1;
        if (contour.points.empty())
            LaserToolpathBuilder::discretizeContour(contour, shape, params.deflection);
        if (contour.points.empty())
            continue;

        QString error;
        if (!LaserToolpathBuilder::setAutomaticContourStart(contour, &error)
            || !contour.leadInSolution.valid) {
            return fail(label + QStringLiteral(": contour %1 lead-in failed: %2")
                .arg(contourIndex).arg(error));
        }
    }
    return 0;
}

int verifyOuterFaceProjectionOverridesCrossNormal(const TopoDS_Shape& shape,
                                                   const QString& label)
{
    ContourExtractionParams params;
    params.smoothAngleThresholdDeg = 5.0;
    params.deflection = 0.1;
    params.strategy = ExtractionStrategy::LargestSmoothConnectedSurface;

    const FaceClassification classification = FaceClassifier::classifyFaces(
        shape, params.smoothAngleThresholdDeg);
    std::vector<TopoDS_Face> outerFaces;
    std::vector<TopoDS_Face> crossSectionFaces;
    if (const auto* outer = classification.outerGroup())
        outerFaces = outer->faces;
    for (const auto* group : classification.crossSectionGroups())
        crossSectionFaces.insert(crossSectionFaces.end(), group->faces.begin(), group->faces.end());
    auto contours = LaserToolpathBuilder::extractTubeContoursFromFaceGroups(
        shape, outerFaces, crossSectionFaces, params);
    if (contours.empty())
        return fail(label + QStringLiteral(": no contour for projection test"));

    for (LaserContour& contour : contours) {
        contour.sourceShape = shape;
        contour.leadIn.length = 0.5;
        QString error;
        if (!LaserToolpathBuilder::setAutomaticContourStart(contour, &error)
            || !contour.leadInSolution.valid
            || !contour.points.front().crossSectionNormalValid) {
            continue;
        }

        const gp_Dir expectedDirection = contour.leadInSolution.direction;
        LaserContour misleadingContour = contour;
        misleadingContour.points.front().crossSectionNormal.Reverse();
        const LeadInSolution corrected = LaserToolpathBuilder::computeLeadInSolution(
            misleadingContour, misleadingContour.leadIn.length);
        if (!corrected.valid)
            return fail(label + QStringLiteral(": reversed cross normal rejected safe lead-in"));
        if (gp_Vec(expectedDirection).Dot(gp_Vec(corrected.direction)) < 0.99) {
            return fail(label + QStringLiteral(
                ": cross-section normal overrode the outer-face projection safety gate"));
        }
        return 0;
    }

    return fail(label + QStringLiteral(": no contour with a valid adjacent cross-section normal"));
}

int verifyStepFile(const QString& path)
{
    Handle(TDocStd_Document) xdeDocument =
        new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
    XCAFDoc_DocumentTool::Set(xdeDocument->Main());

    STEPCAFControl_Reader reader;
    reader.SetNameMode(true);
    if (reader.ReadFile(path.toUtf8().constData()) != IFSelect_RetDone)
        return fail(QStringLiteral("cannot read STEP file: %1").arg(path));
    if (!reader.Transfer(xdeDocument))
        return fail(QStringLiteral("cannot transfer STEP file: %1").arg(path));

    auto document = LcncDocument::createStandalone(
        999, QStringLiteral("lead-in-regression"));
    document->importFromXcaf(
        xdeDocument, LcncDocument::EntityKind::Workpiece);
    const NCollection_Sequence<TDF_Label> labels =
        document->entityLabels(LcncDocument::EntityKind::Workpiece);
    const Handle(XCAFDoc_ShapeTool) shapeTool = document->shapeTool();

    int sourceIndex = 0;
    for (int labelIndex = 1; labelIndex <= labels.Length(); ++labelIndex) {
        const TopoDS_Shape shape = shapeTool->GetShape(labels.Value(labelIndex));
        if (shape.IsNull())
            continue;

        if (shape.ShapeType() == TopAbs_COMPOUND
            || shape.ShapeType() == TopAbs_COMPSOLID) {
            for (TopoDS_Iterator it(shape, true, true); it.More(); it.Next()) {
                const int rc = verifyImportedShape(
                    it.Value(),
                    QFileInfo(path).fileName()
                        + QStringLiteral("/source-%1").arg(++sourceIndex));
                if (rc != 0)
                    return rc;
            }
        } else {
            const int rc = verifyImportedShape(
                shape,
                QFileInfo(path).fileName()
                    + QStringLiteral("/source-%1").arg(++sourceIndex));
            if (rc != 0)
                return rc;
        }
    }
    return 0;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    if (const int rc = verifySourceSeamAndAnchor(); rc != 0)
        return rc;
    if (const int rc = verifyBoundedGeometryRefinement(); rc != 0)
        return rc;
    if (const int rc = verifySourceTopologyAndTrimmedGeometry(); rc != 0)
        return rc;
    if (app.arguments().size() > 1) {
        for (int i = 1; i < app.arguments().size(); ++i) {
            if (const int rc = verifyStepFile(app.arguments().at(i)); rc != 0)
                return rc;
        }
        return 0;
    }

    const TopoDS_Shape cylinder =
        BRepPrimAPI_MakeCylinder(10.0, 30.0).Shape();
    if (const int rc = verifyShape(cylinder, QStringLiteral("cylinder")); rc != 0)
        return rc;

    const TopoDS_Shape transverseHole =
        BRepPrimAPI_MakeCylinder(
            gp_Ax2(gp_Pnt(-15.0, 0.0, 15.0), gp_Dir(1.0, 0.0, 0.0)),
            2.0,
            30.0)
            .Shape();
    BRepAlgoAPI_Cut cut(cylinder, transverseHole);
    cut.Build();
    if (!cut.IsDone())
        return fail(QStringLiteral("side-hole boolean cut failed"));

    if (const int rc = verifyShape(cut.Shape(), QStringLiteral("cylinder-side-hole")); rc != 0)
        return rc;

    const gp_Elips ellipse(
        gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
        12.0,
        8.0);
    BRepBuilderAPI_MakeWire ellipseWire;
    ellipseWire.Add(BRepBuilderAPI_MakeEdge(ellipse));
    if (!ellipseWire.IsDone())
        return fail(QStringLiteral("elliptic profile wire failed"));
    const TopoDS_Face ellipseProfile =
        BRepBuilderAPI_MakeFace(ellipseWire.Wire()).Face();
    const TopoDS_Shape ellipticCylinder =
        BRepPrimAPI_MakePrism(ellipseProfile, gp_Vec(0.0, 0.0, 30.0)).Shape();
    if (const int rc = verifyShape(ellipticCylinder, QStringLiteral("elliptic-cylinder")); rc != 0)
        return rc;
    return verifyOuterFaceProjectionOverridesCrossNormal(
        ellipticCylinder, QStringLiteral("elliptic-cylinder"));
}
