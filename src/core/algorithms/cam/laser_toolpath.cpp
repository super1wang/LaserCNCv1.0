#include "core/algorithms/cam/laser_toolpath.h"
#include "core/algorithms/cam/face_classifier.h"
#include "core/kinematics/ik_solver.h"

#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepTools.hxx>
#include <GCPnts_UniformDeflection.hxx>
#include <BRepLProp_CLProps.hxx>
#include <BRepGProp_Face.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRep_Builder.hxx>
#include <BRepBndLib.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <ShapeAnalysis_Surface.hxx>
#include <Geom_Surface.hxx>
#include <GeomLProp_SLProps.hxx>
#include <Bnd_Box.hxx>

#include <gp_Vec.hxx>
#include <gp_Ax1.hxx>
#include <gp_Trsf.hxx>

#include <cmath>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =============================================================================
// LaserToolpath
// =============================================================================

void LaserToolpath::clear()
{
    m_contours.clear();
    m_globalLeadInLength = 5.0;
    m_globalNormalAngle  = 0.0;
}

namespace {

gp_Pnt bboxCenter(const Bnd_Box& box)
{
    if (box.IsVoid())
        return gp_Pnt(0, 0, 0);

    Standard_Real xmin = 0.0;
    Standard_Real ymin = 0.0;
    Standard_Real zmin = 0.0;
    Standard_Real xmax = 0.0;
    Standard_Real ymax = 0.0;
    Standard_Real zmax = 0.0;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    return gp_Pnt(0.5 * (xmin + xmax),
                  0.5 * (ymin + ymax),
                  0.5 * (zmin + zmax));
}

gp_Pnt shapeCenter(const TopoDS_Shape& shape)
{
    if (shape.IsNull())
        return gp_Pnt(0, 0, 0);

    Bnd_Box box;
    BRepBndLib::Add(shape, box);
    return bboxCenter(box);
}

gp_Pnt faceGroupCenter(const std::vector<TopoDS_Face>& faces)
{
    Bnd_Box box;
    for (const auto& face : faces)
        BRepBndLib::Add(face, box);
    return bboxCenter(box);
}

gp_Dir enforceOutwardDirection(const gp_Pnt& point,
                               const gp_Dir& candidate,
                               const gp_Pnt& center)
{
    gp_Vec outward(center, point);
    if (outward.Magnitude() <= 1e-9)
        return candidate;

    gp_Dir result = candidate;
    if (gp_Vec(result).Dot(outward) < 0.0)
        result.Reverse();
    return result;
}

bool findClosestFaceNormal(const gp_Pnt& pt,
                           const std::vector<TopoDS_Face>& faces,
                           gp_Dir& normal,
                           double& bestDistance)
{
    bool found = false;
    bestDistance = std::numeric_limits<double>::max();

    for (const auto& face : faces) {
        Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
        if (surf.IsNull())
            continue;

        ShapeAnalysis_Surface sas(surf);
        const gp_Pnt2d uv = sas.ValueOfUV(pt, 1.0);

        gp_Pnt surfPt;
        surf->D0(uv.X(), uv.Y(), surfPt);
        const double dist = pt.Distance(surfPt);
        if (dist >= bestDistance)
            continue;

        GeomLProp_SLProps props(surf, uv.X(), uv.Y(), 1, 0.01);
        if (!props.IsNormalDefined())
            continue;

        normal = props.Normal();
        if (face.Orientation() == TopAbs_REVERSED)
            normal.Reverse();
        bestDistance = dist;
        found = true;
    }

    return found;
}

gp_Dir avoidCrossSectionDirection(const gp_Pnt& point,
                                  const gp_Dir& candidate,
                                  const gp_Pnt& center,
                                  const std::vector<TopoDS_Face>& crossFaces)
{
    gp_Dir result = enforceOutwardDirection(point, candidate, center);
    if (crossFaces.empty())
        return result;

    gp_Dir crossNormal;
    double bestCrossDistance = 0.0;
    if (!findClosestFaceNormal(point, crossFaces, crossNormal, bestCrossDistance))
        return result;

    const double alignment = std::abs(gp_Vec(result).Dot(gp_Vec(crossNormal)));
    if (alignment <= 0.85)
        return result;

    gp_Vec outward(center, point);
    if (outward.Magnitude() <= 1e-9)
        return result;

    gp_Vec projected = outward - gp_Vec(crossNormal) * outward.Dot(gp_Vec(crossNormal));
    if (projected.Magnitude() > 1e-9)
        result = gp_Dir(projected);
    else
        result = gp_Dir(outward);

    return enforceOutwardDirection(point, result, center);
}

} // namespace

// =============================================================================
// LaserToolpathBuilder — contour extraction
// =============================================================================

std::vector<LaserContour> LaserToolpathBuilder::extractContours(const TopoDS_Shape& workpiece)
{
    std::vector<LaserContour> result;

    if (workpiece.IsNull())
        return result;

    // ── Collect only OUTER wires ──────────────────────────────────────────
    // Laser cutting can only process outer contours (外轮廓).
    // Inner wires (holes, pockets) are skipped because the laser beam
    // cannot reach internal features without cutting through material first.
    //
    // Strategy: iterate all Faces, use BRepTools::OuterWire() to identify
    // each face's outer boundary wire, and deduplicate by TopoDS::IsSame().
    std::vector<TopoDS_Wire> outerWires;
    auto isAlreadyCollected = [&](const TopoDS_Wire& w) {
        for (const auto& ow : outerWires)
            if (ow.IsSame(w)) return true;
        return false;
    };

    for (TopExp_Explorer faceExp(workpiece, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
        const TopoDS_Face& face = TopoDS::Face(faceExp.Current());
        TopoDS_Wire outer = BRepTools::OuterWire(face);
        if (!outer.IsNull() && !isAlreadyCollected(outer))
            outerWires.push_back(outer);
    }

    int wireIdx = 0;
    for (auto& w : outerWires) {
        LaserContour c;
        c.wire = w;
        c.name = QString::fromUtf8("外轮廓 %1").arg(++wireIdx);
        result.push_back(std::move(c));
    }

    // Fallback: if no faces/wires found (e.g. pure wireframe geometry),
    // collect individual edges and wrap each into a wire.
    if (result.empty()) {
        int edgeIdx = 0;
        for (TopExp_Explorer exp(workpiece, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& edge = TopoDS::Edge(exp.Current());
            BRepBuilderAPI_MakeWire wireMaker(edge);
            if (wireMaker.IsDone()) {
                LaserContour c;
                c.wire = wireMaker.Wire();
                c.name = QString::fromUtf8("边缘 %1").arg(++edgeIdx);
                result.push_back(std::move(c));
            }
        }
    }

    return result;
}

// =============================================================================
// LaserToolpathBuilder — face-classification-based contour extraction
// =============================================================================

std::vector<LaserContour> LaserToolpathBuilder::extractContours(
    const TopoDS_Shape& workpiece,
    const ContourExtractionParams& params)
{
    // If face classification is disabled, use the legacy method directly.
    if (!params.useFaceClassification)
        return extractContours(workpiece);

    if (workpiece.IsNull())
        return {};

    // ── Step 1: classify faces by smooth connectivity ────────────────────
    FaceClassification classification =
        FaceClassifier::classifyFaces(workpiece, params.smoothAngleThresholdDeg);

    // Fallback: if no meaningful classification (no outer or no cross-section),
    // use the legacy OuterWire-based extraction but filter out inner surfaces.
    if (!classification.hasOuter() || !classification.hasCrossSection()) {
        // Collect inner face set for filtering
        TopTools_IndexedMapOfShape innerFaceMap;
        for (const auto* ig : classification.innerGroups())
            for (const auto& f : ig->faces)
                innerFaceMap.Add(f);

        // Use legacy extraction but skip wires from inner faces
        std::vector<LaserContour> result;
        std::vector<TopoDS_Wire> outerWires;
        auto isAlreadyCollected = [&](const TopoDS_Wire& w) {
            for (const auto& ow : outerWires)
                if (ow.IsSame(w)) return true;
            return false;
        };
        for (TopExp_Explorer faceExp(workpiece, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
            const TopoDS_Face& face = TopoDS::Face(faceExp.Current());
            if (innerFaceMap.Contains(face))
                continue;  // skip inner surfaces
            TopoDS_Wire outer = BRepTools::OuterWire(face);
            if (!outer.IsNull() && !isAlreadyCollected(outer))
                outerWires.push_back(outer);
        }
        int wireIdx = 0;
        for (auto& w : outerWires) {
            LaserContour c;
            c.wire = w;
            c.name = QString::fromUtf8("外轮廓 %1").arg(++wireIdx);
            result.push_back(std::move(c));
        }
        return result;
    }

    // ── Step 2: extract contour edges at outer/cross-section boundary ────
    std::vector<TopoDS_Edge> contourEdges =
        FaceClassifier::extractContourEdges(classification);

    if (contourEdges.empty()) {
        // Fallback: disable face classification to use legacy OuterWire method
        ContourExtractionParams fallback;
        fallback.smoothAngleThresholdDeg = params.smoothAngleThresholdDeg;
        fallback.deflection = params.deflection;
        fallback.useFaceClassification = false;
        return extractContours(workpiece, fallback);
    }

    // ── Step 3: chain edges into wires ───────────────────────────────────
    std::vector<TopoDS_Wire> wires =
        FaceClassifier::chainEdgesToWires(contourEdges);

    if (wires.empty()) {
        // Fallback: disable face classification to use legacy OuterWire method
        ContourExtractionParams fallback;
        fallback.smoothAngleThresholdDeg = params.smoothAngleThresholdDeg;
        fallback.deflection = params.deflection;
        fallback.useFaceClassification = false;
        return extractContours(workpiece, fallback);
    }

    // ── Step 4: build LaserContour objects ────────────────────────────────
    std::vector<LaserContour> result;
    result.reserve(wires.size());

    // Collect outer and cross-section faces for normal computation
    std::vector<TopoDS_Face> outerFaces;
    if (classification.outerGroup())
        outerFaces = classification.outerGroup()->faces;

    std::vector<TopoDS_Face> crossFaces;
    for (const auto* cg : classification.crossSectionGroups())
        for (const auto& f : cg->faces)
            crossFaces.push_back(f);

    int wireIdx = 0;
    for (auto& w : wires) {
        LaserContour c;
        c.wire = w;
        c.name = QString::fromUtf8("加工轮廓 %1").arg(++wireIdx);
        c.contourType = static_cast<int>(FaceGroupKind::CrossSection);
        c.sourceInfo  = QString::fromUtf8("外表面(%1面) ∩ 截面(%2面)")
                            .arg(outerFaces.size()).arg(crossFaces.size());

        // Discretise with face-classification-aware normals
        discretizeContourWithClassification(c, outerFaces, crossFaces,
                                            params.deflection);
        result.push_back(std::move(c));
    }

    return result;
}

// =============================================================================
// LaserToolpathBuilder — discretization
// =============================================================================

void LaserToolpathBuilder::discretizeContour(LaserContour& contour,
                                             const TopoDS_Shape& workpiece,
                                             double deflection)
{
    contour.points.clear();
    if (contour.wire.IsNull())
        return;

    const gp_Pnt workpieceCenter = shapeCenter(workpiece);

    for (TopExp_Explorer exp(contour.wire, TopAbs_EDGE); exp.More(); exp.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(exp.Current());
        if (BRep_Tool::Degenerated(edge))
            continue;

        BRepAdaptor_Curve curve(edge);
        GCPnts_UniformDeflection sampler(curve, deflection);

        if (!sampler.IsDone())
            continue;

        for (int i = 1; i <= sampler.NbPoints(); ++i) {
            ToolpathPoint tp;
            tp.param    = sampler.Parameter(i);
            tp.position = sampler.Value(i);

            // Compute surface normal at this point
            tp.normal = enforceOutwardDirection(
                tp.position,
                findSurfaceNormal(workpiece, tp.position),
                workpieceCenter);

            contour.points.push_back(tp);
        }
    }
}

// =============================================================================
// LaserToolpathBuilder — classification-aware discretization
// =============================================================================

void LaserToolpathBuilder::discretizeContourWithClassification(
    LaserContour& contour,
    const std::vector<TopoDS_Face>& outerFaces,
    const std::vector<TopoDS_Face>& crossFaces,
    double deflection)
{
    contour.points.clear();
    if (contour.wire.IsNull())
        return;

    const gp_Pnt outerCenter = faceGroupCenter(outerFaces);

    for (TopExp_Explorer exp(contour.wire, TopAbs_EDGE); exp.More(); exp.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(exp.Current());
        if (BRep_Tool::Degenerated(edge))
            continue;

        BRepAdaptor_Curve curve(edge);
        GCPnts_UniformDeflection sampler(curve, deflection);

        if (!sampler.IsDone())
            continue;

        for (int i = 1; i <= sampler.NbPoints(); ++i) {
            ToolpathPoint tp;
            tp.param    = sampler.Parameter(i);
            tp.position = sampler.Value(i);

            // Compute machining normal using face classification
            tp.normal = avoidCrossSectionDirection(
                tp.position,
                findMachiningNormal(tp.position, outerFaces, crossFaces),
                outerCenter,
                crossFaces);

            // Compute tangent along the curve
            gp_Pnt pDummy;
            gp_Vec tangentVec;
            curve.D1(tp.param, pDummy, tangentVec);
            if (tangentVec.Magnitude() > 1e-10)
                tp.tangent = gp_Dir(tangentVec);
            else
                tp.tangent = gp_Dir(0, 0, 1);

            contour.points.push_back(tp);
        }
    }
}

// =============================================================================
// LaserToolpathBuilder — machining normal (face-classification-aware)
// =============================================================================

gp_Dir LaserToolpathBuilder::findMachiningNormal(
    const gp_Pnt& pt,
    const std::vector<TopoDS_Face>& outerFaces,
    const std::vector<TopoDS_Face>& crossFaces)
{
    gp_Dir defaultNormal(0, 0, 1);

    // ── Find the closest outer-surface face and its normal ───────────────
    gp_Dir outerNormal = defaultNormal;
    double bestOuterDist = std::numeric_limits<double>::max();
    bool   foundOuter = false;

    for (const auto& face : outerFaces) {
        Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
        if (surf.IsNull()) continue;

        ShapeAnalysis_Surface sas(surf);
        gp_Pnt2d uv = sas.ValueOfUV(pt, 1.0);

        gp_Pnt surfPt;
        surf->D0(uv.X(), uv.Y(), surfPt);
        double dist = pt.Distance(surfPt);

        if (dist < bestOuterDist) {
            bestOuterDist = dist;
            GeomLProp_SLProps props(surf, uv.X(), uv.Y(), 1, 0.01);
            if (props.IsNormalDefined()) {
                outerNormal = props.Normal();
                if (face.Orientation() == TopAbs_REVERSED)
                    outerNormal.Reverse();
                foundOuter = true;
            }
        }
    }

    if (!foundOuter)
        return defaultNormal;

    return outerNormal;
}

// =============================================================================
// LaserToolpathBuilder — surface normal lookup
// =============================================================================

gp_Dir LaserToolpathBuilder::findSurfaceNormal(const TopoDS_Shape& workpiece,
                                               const gp_Pnt& pt)
{
    // Default: +Z (upward)
    gp_Dir bestNormal(0, 0, 1);
    double bestDist = std::numeric_limits<double>::max();

    for (TopExp_Explorer faceExp(workpiece, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
        const TopoDS_Face& face = TopoDS::Face(faceExp.Current());
        Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
        if (surf.IsNull())
            continue;

        ShapeAnalysis_Surface sas(surf);
        gp_Pnt2d uv = sas.ValueOfUV(pt, 1.0); // tolerance 1mm

        // Evaluate the surface at the UV parameters
        gp_Pnt surfPt;
        surf->D0(uv.X(), uv.Y(), surfPt);
        double dist = pt.Distance(surfPt);

        if (dist < bestDist) {
            bestDist = dist;

            // Get surface normal via GeomLProp_SLProps
            GeomLProp_SLProps props(surf, uv.X(), uv.Y(), 1, 0.01);
            if (props.IsNormalDefined()) {
                bestNormal = props.Normal();
                // Reverse if face orientation is reversed
                if (face.Orientation() == TopAbs_REVERSED)
                    bestNormal.Reverse();
            }
        }
    }

    return bestNormal;
}

// =============================================================================
// LaserToolpathBuilder — lead-in computation
// =============================================================================

gp_Dir LaserToolpathBuilder::ensureNotFromAbove(const gp_Dir& approachDir,
                                                double thresholdDeg)
{
    const gp_Dir zUp(0, 0, 1);
    double angle = approachDir.Angle(zUp);  // radians

    double thresholdRad = thresholdDeg * M_PI / 180.0;

    // If the approach direction is nearly aligned with +Z (coming from above)
    // or nearly aligned with -Z (coming from below, which means going upward
    // toward the workpiece top), rotate it away from vertical.
    if (angle < thresholdRad) {
        // Nearly parallel to +Z → project onto XY and use that direction
        gp_Vec v(approachDir);
        gp_Vec projected(v.X(), v.Y(), 0.0);

        if (projected.Magnitude() < 1e-6) {
            // Approach is exactly along Z — pick an arbitrary horizontal direction
            projected = gp_Vec(1, 0, 0);
        }
        projected.Normalize();

        // Tilt slightly downward from horizontal (at the threshold angle from Z)
        double zComp = std::cos(thresholdRad);
        double xyComp = std::sin(thresholdRad);
        return gp_Dir(projected.X() * xyComp,
                      projected.Y() * xyComp,
                      zComp);
    }

    return approachDir;
}

TopoDS_Edge LaserToolpathBuilder::computeLeadInEdge(const LaserContour& contour,
                                                    double length,
                                                    double normalAngleDeg)
{
    if (!contour.leadIn.valid || length <= 0.0)
        return TopoDS_Edge();

    const gp_Pnt& entryPt = contour.leadIn.entryPoint;

    // Find the machining normal near the entry point; the lead-in start should stay
    // outside the outer contour instead of landing on another surface.
    gp_Dir normal(0, 0, 1);
    gp_Dir tangent(1, 0, 0);
    double leadStartZ = entryPt.Z();
    if (!contour.points.empty()) {
        leadStartZ = contour.points.front().position.Z();
        double bestDist = std::numeric_limits<double>::max();
        for (const auto& tp : contour.points) {
            double d = entryPt.Distance(tp.position);
            if (d < bestDist) {
                bestDist = d;
                normal = tp.normal;
                tangent = tp.tangent;
            }
        }
    }

    gp_Vec approachVec(normal.X(), normal.Y(), 0.0);
    if (approachVec.Magnitude() <= 1e-6) {
        const gp_Vec tangentVec(tangent.X(), tangent.Y(), 0.0);
        if (tangentVec.Magnitude() > 1e-6)
            approachVec = gp_Vec(-tangentVec.Y(), tangentVec.X(), 0.0);
    }
    if (approachVec.Magnitude() <= 1e-6)
        approachVec = gp_Vec(1.0, 0.0, 0.0);
    approachVec.Normalize();

    // Apply normal angle offset in the machining plane, preserving the lead-in height.
    if (std::abs(normalAngleDeg) > 0.01) {
        gp_Trsf rot;
        rot.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)),
                        normalAngleDeg * M_PI / 180.0);
        approachVec.Transform(rot);
        if (approachVec.Magnitude() > 1e-6)
            approachVec.Normalize();
    }

    gp_Pnt startPt(
        entryPt.X() + approachVec.X() * length,
        entryPt.Y() + approachVec.Y() * length,
        leadStartZ
    );

    // Build the lead-in edge
    BRepBuilderAPI_MakeEdge edgeMaker(startPt, entryPt);
    if (!edgeMaker.IsDone())
        return TopoDS_Edge();

    return edgeMaker.Edge();
}

// =============================================================================
// LaserToolpathBuilder — machine coordinate computation (IK)
// =============================================================================

void LaserToolpathBuilder::computeMachineCoordinates(LaserContour& contour,
                                                     MachineKinematics* kinematics,
                                                     const gp_Trsf& wpcTransform)
{
    if (!kinematics) return;

    for (auto& pt : contour.points) {
        // Transform from workpiece-local to world frame
        gp_Pnt worldPos = pt.position.Transformed(wpcTransform);
        gp_Dir worldDir = pt.normal.Transformed(wpcTransform);

        pt.machineCoord = IKSolver::solve(kinematics, worldPos, worldDir);
    }
}
