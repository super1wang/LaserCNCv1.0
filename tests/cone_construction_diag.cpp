// Regression test for the cutter-head cone construction used by
// MachineGuideRenderer.
//
// Root cause that motivated this test: in OCCT 7.9, BRepPrimAPI_MakeOneAxis
// constructors (MakeCone / MakeCylinder / MakeSphere) do NOT call Done(), so
// IsDone() stays false until Build() is called -- even though Shape() returns
// a valid solid. MachineGuideRenderer checked IsDone() before Shape(), so the
// solid branch was never taken and the cone always fell back to the wireframe
// (8 generators + base circle). This test asserts the construction the renderer
// now uses (Build() then IsDone()) yields a valid, meshed solid.

#include <QString>
#include <QTextStream>

#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>

namespace {
void logLine(const QString& s) { QTextStream(stderr) << s << '\n'; }
int fail(const QString& s) { logLine(QStringLiteral("FAIL: ") + s); return 1; }
} // namespace

int main()
{
    // Mirror of MachineGuideRenderer::refresh cone parameters.
    constexpr double kHeadConeHeight = 30.0;
    constexpr double kHeadConeRadius = 8.0;
    constexpr double kHeadConeTipRadius = 0.35;

    BRepPrimAPI_MakeCone coneMaker(kHeadConeTipRadius, kHeadConeRadius, kHeadConeHeight);
    coneMaker.Build();  // OCCT 7.9: required, else IsDone() is false.

    if (!coneMaker.IsDone())
        return fail(QStringLiteral("IsDone() false after Build() -- cone solid branch would be skipped"));
    TopoDS_Shape coneShape = coneMaker.Shape();
    if (coneShape.IsNull())
        return fail(QStringLiteral("Shape() null after Build()"));

    if (coneShape.ShapeType() != TopAbs_SOLID)
        return fail(QStringLiteral("expected SOLID, got type=%1")
            .arg(static_cast<int>(coneShape.ShapeType())));

    BRepMesh_IncrementalMesh(coneShape, 0.5);

    int faceCount = 0;
    int facesWithTriangulation = 0;
    int totalTriangles = 0;
    for (TopExp_Explorer ex(coneShape, TopAbs_FACE); ex.More(); ex.Next()) {
        const TopoDS_Face& face = TopoDS::Face(ex.Current());
        ++faceCount;
        TopLoc_Location loc;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
        if (!tri.IsNull()) {
            ++facesWithTriangulation;
            totalTriangles += tri->NbTriangles();
        }
    }

    // A frustum cone has 3 faces (lateral + 2 caps); all must be triangulated
    // so the AIS_Shaded presentation can fill them.
    if (faceCount < 2)
        return fail(QStringLiteral("expected >=2 faces, got %1").arg(faceCount));
    if (facesWithTriangulation != faceCount)
        return fail(QStringLiteral("only %1 of %2 faces triangulated")
            .arg(facesWithTriangulation).arg(faceCount));
    if (totalTriangles == 0)
        return fail(QStringLiteral("no triangles -- shaded fill would be empty"));

    logLine(QStringLiteral("OK: cone solid faces=%1 tri=%2 tris=%3")
        .arg(faceCount).arg(facesWithTriangulation).arg(totalTriangles));
    return 0;
}
