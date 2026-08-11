// Diagnostic: inspect a STEP part and run every extraction strategy + lead-in
// pipeline, reporting per-contour success/failure, error reason and timing.
// Invoked manually with a STEP path - not a unit test.
#include "core/algorithms/cam/laser_toolpath.h"
#include "core/algorithms/cam/face_classifier.h"
#include "core/document/lcnc_document.h"

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QMap>
#include <QTextStream>

#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRep_Builder.hxx>
#include <Bnd_Box.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <GProp_GProps.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDocStd_Document.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Shape.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_Shapetool.hxx>
#include <gp_Dir.hxx>

QTextStream out(stdout);
bool g_planeOnly{false};

QString surfaceTypeName(const TopoDS_Face& face)
{
    BRepAdaptor_Surface surf(face, Standard_True);
    switch (surf.GetType()) {
    case GeomAbs_Plane: return QStringLiteral("plane");
    case GeomAbs_Cylinder: return QStringLiteral("cylinder");
    case GeomAbs_Cone: return QStringLiteral("cone");
    case GeomAbs_Sphere: return QStringLiteral("sphere");
    case GeomAbs_Torus: return QStringLiteral("torus");
    case GeomAbs_BezierSurface: return QStringLiteral("bezier");
    case GeomAbs_BSplineSurface: return QStringLiteral("bspline");
    case GeomAbs_SurfaceOfRevolution: return QStringLiteral("revol");
    case GeomAbs_SurfaceOfExtrusion: return QStringLiteral("extrusion");
    default: return QStringLiteral("other");
    }
}

void printShapeStats(const TopoDS_Shape& shape)
{
    QMap<QString, int> faceTypes;
    int faceCount = 0, edgeCount = 0, wireCount = 0;
    for (TopExp_Explorer f(shape, TopAbs_FACE); f.More(); f.Next()) {
        const TopoDS_Face face = TopoDS::Face(f.Current());
        faceTypes[surfaceTypeName(face)]++;
        ++faceCount;
    }
    for (TopExp_Explorer e(shape, TopAbs_EDGE); e.More(); e.Next()) ++edgeCount;
    for (TopExp_Explorer w(shape, TopAbs_WIRE); w.More(); w.Next()) ++wireCount;
    out << "  shape stats: faces=" << faceCount << " edges=" << edgeCount
        << " wires=" << wireCount << "\n";
    Bnd_Box bounds;
    BRepBndLib::Add(shape, bounds);
    if (!bounds.IsVoid()) {
        Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
        bounds.Get(xMin, yMin, zMin, xMax, yMax, zMax);
        out << "  bounds=(" << xMin << "," << yMin << "," << zMin << ")..("
            << xMax << "," << yMax << "," << zMax << ")\n";
    }
    for (auto it = faceTypes.constBegin(); it != faceTypes.constEnd(); ++it)
        out << "    " << it.key() << ": " << it.value() << "\n";
}

void runStrategy(const TopoDS_Shape& shape, ExtractionStrategy strategy,
                 const gp_Dir& beam, const QString& stratName, double leadInLength)
{
    ContourExtractionParams params;
    params.smoothAngleThresholdDeg = 5.0;
    params.deflection = 0.1;
    params.strategy = strategy;
    params.machiningBeamDirection = beam;

    QElapsedTimer extractTimer;
    extractTimer.start();
    FaceClassification classification;
    auto contours = LaserToolpathBuilder::extractContours(
        shape, params, &classification);
    const qint64 extractMs = extractTimer.elapsed();

    out << "\n  -- strategy=" << stratName
        << " beam=(" << beam.X() << "," << beam.Y() << "," << beam.Z() << ")"
        << " leadIn=" << leadInLength << "\n";
    out << "    extraction: " << contours.size() << " contours, "
        << extractMs << " ms\n";
    if (classification.outerGroup())
        out << "    outer group: " << classification.outerGroup()->faces.size() << " face(s)\n";
    out << "    cross-section groups: " << classification.crossSectionGroups().size() << "\n";

    int ok = 0, fail = 0;
    qint64 totalMs = 0;
    QMap<QString, int> errorCounts;
    for (std::size_t i = 0; i < contours.size(); ++i) {
        LaserContour& c = contours[i];
        c.sourceShape = shape;
        c.leadIn.length = leadInLength;
        if (c.points.empty())
            LaserToolpathBuilder::discretizeContour(c, shape, params.deflection);

        QElapsedTimer t;
        t.start();
        QString error;
        const bool set = LaserToolpathBuilder::setAutomaticContourStart(c, &error);
        const qint64 ms = t.elapsed();
        totalMs += ms;
        const bool valid = set && c.leadInSolution.valid;
        if (valid) {
            ++ok;
        } else {
            ++fail;
            const QString reason = error.isEmpty() ? c.leadInSolution.error : error;
            errorCounts[reason]++;
            if (fail <= 5)  // only print first few failures
                out << "    [" << i << "] " << c.name.toStdString().c_str()
                    << " pts=" << c.points.size() << " FAIL (" << ms << "ms): "
                    << reason.toStdString().c_str() << "\n";
        }
    }
    out << "    summary: " << ok << " ok, " << fail << " failed, lead-in total "
        << totalMs << " ms\n";
    if (!errorCounts.isEmpty()) {
        out << "    error histogram:\n";
        for (auto it = errorCounts.constBegin(); it != errorCounts.constEnd(); ++it)
            out << "      x" << it.value() << "  " << it.key().toStdString().c_str() << "\n";
    }
}

void diagnoseShape(const TopoDS_Shape& shape, const QString& label)
{
    out << "\n======== " << label << " ========\n";
    printShapeStats(shape);
    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
        const TopoDS_Face face = TopoDS::Face(exp.Current());
        if (!LaserToolpathBuilder::isPlanarFace(face))
            continue;
        GProp_GProps props;
        BRepGProp::SurfaceProperties(face, props);
        gp_Dir normal = BRepAdaptor_Surface(face, Standard_True).Plane().Axis().Direction();
        if (face.Orientation() == TopAbs_REVERSED)
            normal.Reverse();
        const gp_Pnt center = props.CentreOfMass();
        out << "  plane center=(" << center.X() << "," << center.Y() << "," << center.Z()
            << ") normal=(" << normal.X() << "," << normal.Y() << "," << normal.Z()
            << ") area=" << props.Mass() << "\n";
    }

    const gp_Dir beamDown(0, 0, -1);
    const gp_Dir beamSide(0, -1, 0);

    {
        QElapsedTimer timer;
        timer.start();
        QString selectionInfo;
        const std::vector<TopoDS_Face> visibleFaces =
            LaserToolpathBuilder::selectTopVisibleFacesFromPositiveZ(shape, &selectionInfo);
        QMap<QString, int> types;
        double totalArea = 0.0;
        for (const TopoDS_Face& face : visibleFaces) {
            GProp_GProps props;
            BRepGProp::SurfaceProperties(face, props);
            types[surfaceTypeName(face)]++;
            totalArea += props.Mass();
        }
        out << "\n  -- strategy=PlaneZLight --\n"
            << "    visible faces: " << visibleFaces.size()
            << ", area=" << totalArea << ", " << timer.elapsed() << " ms";
        if (!selectionInfo.isEmpty())
            out << ", info=" << selectionInfo.toStdString().c_str();
        out << "\n";
        for (auto it = types.constBegin(); it != types.constEnd(); ++it)
            out << "    " << it.key().toStdString().c_str() << ": "
                << it.value() << "\n";
        for (const TopoDS_Face& face : visibleFaces) {
            GProp_GProps props;
            BRepGProp::SurfaceProperties(face, props);
            const gp_Pnt center = props.CentreOfMass();
            BRepAdaptor_Surface surface(face, Standard_True);
            Standard_Real uMin, uMax, vMin, vMax;
            BRepTools::UVBounds(face, uMin, uMax, vMin, vMax);
            gp_Pnt point;
            gp_Vec dU;
            gp_Vec dV;
            surface.D1((uMin + uMax) * 0.5, (vMin + vMax) * 0.5, point, dU, dV);
            gp_Vec normal = dU.Crossed(dV);
            if (face.Orientation() == TopAbs_REVERSED)
                normal.Reverse();
            if (normal.SquareMagnitude() > 1e-18)
                normal.Normalize();
            out << "    selected " << surfaceTypeName(face).toStdString().c_str()
                << " center=(" << center.X() << "," << center.Y()
                << "," << center.Z() << ") normal=(" << normal.X() << ","
                << normal.Y() << "," << normal.Z() << ") area=" << props.Mass() << "\n";
        }
    }
    if (g_planeOnly)
        return;

    runStrategy(shape, ExtractionStrategy::LargestSmoothConnectedSurface, beamDown,
                QStringLiteral("LargestSmoothConnectedSurface"), 5.0);
    // Skip the slow 850-contour strategies; focus on the manual-selection
    // scenarios that reproduce the user's failures.

    // ── Manual-face-selection simulation ────────────────────────────────
    // Collect every face with its area + surface type, sorted by area desc.
    struct FaceInfo { TopoDS_Face face; double area; QString type; };
    std::vector<FaceInfo> allFaces;
    for (TopExp_Explorer f(shape, TopAbs_FACE); f.More(); f.Next()) {
        const TopoDS_Face face = TopoDS::Face(f.Current());
        GProp_GProps props;
        BRepGProp::SurfaceProperties(face, props);
        allFaces.push_back({face, props.Mass(), surfaceTypeName(face)});
    }
    std::sort(allFaces.begin(), allFaces.end(),
              [](const FaceInfo& a, const FaceInfo& b) { return a.area > b.area; });
    out << "\n  -- top 5 faces by area --\n";
    for (int i = 0; i < std::min<int>(5, int(allFaces.size())); ++i)
        out << "    #" << i << "  " << allFaces[i].type.toStdString().c_str()
            << "  area=" << allFaces[i].area << "\n";

    auto runContours = [&](std::vector<LaserContour> contours, const QString& label) {
        out << "\n  -- " << label << " --\n";
        out << "    extracted: " << contours.size() << " contours\n";
        int ok = 0, fail = 0, noPts = 0;
        qint64 totalMs = 0;
        QMap<QString, int> errorCounts;
        for (std::size_t i = 0; i < contours.size(); ++i) {
            LaserContour& c = contours[i];
            c.sourceShape = shape;
            c.leadIn.length = 5.0;
            if (c.points.empty())
                LaserToolpathBuilder::discretizeContour(c, shape, 0.1);
            if (c.points.empty()) { ++noPts; continue; }
            QElapsedTimer t; t.start();
            QString error;
            const bool set = LaserToolpathBuilder::setAutomaticContourStart(c, &error);
            totalMs += t.elapsed();
            if (set && c.leadInSolution.valid) ++ok;
            else { ++fail; errorCounts[error.isEmpty() ? c.leadInSolution.error : error]++; }
        }
        out << "    summary: " << ok << " ok, " << fail << " failed, " << noPts
            << " empty, lead-in total " << totalMs << " ms\n";
        for (auto it = errorCounts.constBegin(); it != errorCounts.constEnd(); ++it)
            out << "      x" << it.value() << "  " << it.key().toStdString().c_str() << "\n";
    };

    // Interpretation A: outer = non-bspline skin (sphere+plane), cross =
    // bspline hole walls. This is the likely "tube" workflow for this part.
    {
        std::vector<TopoDS_Face> outer, cross;
        for (const FaceInfo& fi : allFaces) {
            if (fi.type == QStringLiteral("sphere") || fi.type == QStringLiteral("plane"))
                outer.push_back(fi.face);
            else if (fi.type == QStringLiteral("bspline"))
                cross.push_back(fi.face);
        }
        out << "\n  [interp A] outer(sphere+plane)=" << outer.size()
            << " cross(bspline)=" << cross.size() << "\n";
        ContourExtractionParams params; params.deflection = 0.1;
        runContours(LaserToolpathBuilder::extractContoursFromFaces(
                        shape, outer, beamDown, params),
                    QStringLiteral("interp A: outer-only (sphere+plane)"));
        runContours(LaserToolpathBuilder::extractTubeContoursFromFaceGroups(
                        shape, outer, cross, params),
                    QStringLiteral("interp A: outer+cross (tube)"));
    }
    out.flush();
}

int diagnoseStepFile(const QString& path)
{
    Handle(TDocStd_Document) xdeDocument =
        new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
    XCAFDoc_DocumentTool::Set(xdeDocument->Main());

    STEPCAFControl_Reader reader;
    reader.SetNameMode(Standard_True);
    if (reader.ReadFile(path.toUtf8().constData()) != IFSelect_RetDone) {
        out << "cannot read STEP file: " << path << "\n";
        return 1;
    }
    if (!reader.Transfer(xdeDocument)) {
        out << "cannot transfer STEP file: " << path << "\n";
        return 1;
    }
    auto document = LcncDocument::createStandalone(
        999, QStringLiteral("lead-in-diag"));
    document->importFromXcaf(
        xdeDocument, LcncDocument::EntityKind::Workpiece);
    const TDF_LabelSequence labels =
        document->entityLabels(LcncDocument::EntityKind::Workpiece);
    const Handle(XCAFDoc_ShapeTool) shapeTool = document->shapeTool();

    out << "top-level workpiece labels: " << labels.Length() << "\n";
    TopoDS_Compound globalWorkpiece;
    BRep_Builder globalBuilder;
    globalBuilder.MakeCompound(globalWorkpiece);
    int sourceIndex = 0;
    for (int li = 1; li <= labels.Length(); ++li) {
        const TopoDS_Shape shape = shapeTool->GetShape(labels.Value(li));
        if (shape.IsNull())
            continue;
        globalBuilder.Add(globalWorkpiece, shape);
        out << "label " << li << " top-level type: "
            << shape.ShapeType() << "\n";
        if (shape.ShapeType() == TopAbs_COMPOUND
            || shape.ShapeType() == TopAbs_COMPSOLID) {
            for (TopoDS_Iterator it(shape, Standard_True, Standard_True);
                 it.More(); it.Next()) {
                diagnoseShape(it.Value(),
                    QFileInfo(path).fileName()
                        + QStringLiteral("/source-%1").arg(++sourceIndex));
            }
        } else {
            diagnoseShape(shape,
                QFileInfo(path).fileName()
                    + QStringLiteral("/source-%1").arg(++sourceIndex));
        }
    }
    diagnoseShape(globalWorkpiece, QFileInfo(path).fileName() + QStringLiteral("/global"));
    return 0;
}

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    if (app.arguments().size() < 2) {
        out << "usage: lcnc_cam_lead_in_diag <step-file>\n";
        return 2;
    }
    int rc = 0;
    for (int i = 1; i < app.arguments().size(); ++i) {
        const QString argument = app.arguments().at(i);
        if (argument == QStringLiteral("--plane-only")) {
            g_planeOnly = true;
            continue;
        }
        if (const int r = diagnoseStepFile(argument); r != 0)
            rc = r;
    }
    return rc;
}
