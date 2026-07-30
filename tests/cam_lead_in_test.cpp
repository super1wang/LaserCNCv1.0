#include "core/algorithms/cam/laser_toolpath.h"
#include "core/algorithms/cam/face_classifier.h"
#include "core/document/lcnc_document.h"

#include <QCoreApplication>
#include <QFileInfo>
#include <QTextStream>

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDocStd_Document.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Iterator.hxx>
#include <XCAFDoc_Documenttool.hxx>
#include <XCAFDoc_Shapetool.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Elips.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <cmath>

namespace {

int fail(const QString& message)
{
    QTextStream(stderr) << message << '\n';
    return 1;
}

int verifyShape(const TopoDS_Shape& shape, const QString& label)
{
    ContourExtractionParams params;
    params.smoothAngleThresholdDeg = 5.0;
    params.deflection = 0.1;
    params.strategy = ExtractionStrategy::TubeClassification;

    FaceClassification classification;
    auto contours = LaserToolpathBuilder::extractContours(
        shape, params, &classification);
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
    params.strategy = ExtractionStrategy::TubeClassification;

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
    params.strategy = ExtractionStrategy::TubeClassification;

    auto contours = LaserToolpathBuilder::extractContours(shape, params);
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
    reader.SetNameMode(Standard_True);
    if (reader.ReadFile(path.toUtf8().constData()) != IFSelect_RetDone)
        return fail(QStringLiteral("cannot read STEP file: %1").arg(path));
    if (!reader.Transfer(xdeDocument))
        return fail(QStringLiteral("cannot transfer STEP file: %1").arg(path));

    auto document = LcncDocument::createStandalone(
        999, QStringLiteral("lead-in-regression"));
    document->importFromXcaf(
        xdeDocument, LcncDocument::EntityKind::Workpiece);
    const TDF_LabelSequence labels =
        document->entityLabels(LcncDocument::EntityKind::Workpiece);
    const Handle(XCAFDoc_ShapeTool) shapeTool = document->shapeTool();

    int sourceIndex = 0;
    for (int labelIndex = 1; labelIndex <= labels.Length(); ++labelIndex) {
        const TopoDS_Shape shape = shapeTool->GetShape(labels.Value(labelIndex));
        if (shape.IsNull())
            continue;

        if (shape.ShapeType() == TopAbs_COMPOUND
            || shape.ShapeType() == TopAbs_COMPSOLID) {
            for (TopoDS_Iterator it(shape, Standard_True, Standard_True);
                 it.More(); it.Next()) {
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
