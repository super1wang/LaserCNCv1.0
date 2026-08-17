#include "core/algorithms/cam/laser_toolpath.h"
#include "core/algorithms/cam/face_classifier.h"
#include "core/kinematics/ik_solver.h"
#include "core/kinematics/toolpath_kinematics_solver.h"
#include "core/logging/logger.h"

#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <BRep_tool.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <GCPnts_UniformDeflection.hxx>
#include <BRepLProp_CLProps.hxx>
#include <BRepGProp_Face.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRep_Builder.hxx>
#include <BRepBndLib.hxx>
#include <BRepOffset_Analyse.hxx>
#include <BRepOffset_ListOfInterval.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <ShapeAnalysis_Surface.hxx>
#include <Geom_Surface.hxx>
#include <GeomLProp_SLProps.hxx>
#include <Bnd_Box.hxx>
#include <IntCurvesFace_ShapeIntersector.hxx>
#include <Poly_Triangulation.hxx>
#include <Poly_Triangle.hxx>
#include <TopLoc_Location.hxx>

#include <gp_Vec.hxx>
#include <gp_Trsf.hxx>
#include <gp_Pnt2d.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <utility>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =============================================================================
// LaserToolpath
// =============================================================================

void LaserToolpath::clear()
{
    m_contours.clear();
    m_layers.clear();
    m_globalLeadInLength = 5.0;
}

namespace {

struct RangeStats
{
    double min{0.0};
    double max{0.0};
    bool has{false};

    void add(double value)
    {
        if (!has) {
            min = value;
            max = value;
            has = true;
            return;
        }
        if (value < min) min = value;
        if (value > max) max = value;
    }
};

double rangeMin(const RangeStats& range)
{
    return range.has ? range.min : 0.0;
}

double rangeMax(const RangeStats& range)
{
    return range.has ? range.max : 0.0;
}

bool camToolpathTraceEnabled()
{
    static const bool enabled = [] {
        const char* value = std::getenv("LCNC_CAM_TOOLPATH_TRACE");
        return value && value[0] != '\0' && value[0] != '0';
    }();
    return enabled;
}

const char* motionTypeText(MachineAxisDef::MotionType type)
{
    return type == MachineAxisDef::Rotary ? "Rotary" : "Linear";
}

double normalizeSigned180(double deg)
{
    while (deg > 180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return std::abs(deg) < 1e-10 ? 0.0 : deg;
}

double normalizeAxisReference(const QString& axisName, double deg)
{
    return axisName.trimmed().toUpper() == QStringLiteral("C")
        ? deg
        : normalizeSigned180(deg);
}

std::pair<QString, QString> orderedRotaryAxisNames(const MachineKinematics* kin)
{
    if (!kin)
        return {};

    QString rotaryAxes[2];
    int rotaryCount = 0;
    for (const auto& axis : kin->axes()) {
        if (axis.motionType == MachineAxisDef::Rotary && rotaryCount < 2)
            rotaryAxes[rotaryCount++] = axis.name;
    }
    if (rotaryCount == 0)
        return {};
    if (rotaryCount == 1)
        return {rotaryAxes[0], QString()};

    const MachineAxisDef* a0 = kin->findAxis(rotaryAxes[0]);
    const MachineAxisDef* a1 = kin->findAxis(rotaryAxes[1]);
    const MachineAxisDef* child = nullptr;
    const MachineAxisDef* parent = nullptr;
    if (a0 && a1) {
        QString cur = a1->parentAxis;
        while (!cur.isEmpty()) {
            if (cur == a0->name) { child = a1; parent = a0; break; }
            const MachineAxisDef* d = kin->findAxis(cur);
            if (!d) break;
            cur = d->parentAxis;
        }
        if (!child) {
            cur = a0->parentAxis;
            while (!cur.isEmpty()) {
                if (cur == a1->name) { child = a0; parent = a1; break; }
                const MachineAxisDef* d = kin->findAxis(cur);
                if (!d) break;
                cur = d->parentAxis;
            }
        }
    }

    if (child && parent)
        return {child->name, parent->name};
    return {rotaryAxes[0], rotaryAxes[1]};
}

MachineCoord currentRotaryReference(const MachineKinematics* kin)
{
    MachineCoord ref;
    const auto [r1Name, r2Name] = orderedRotaryAxisNames(kin);
    if (r1Name.isEmpty())
        return ref;

    const MachineAxisDef* r1 = kin->findAxis(r1Name);
    const MachineAxisDef* r2 = r2Name.isEmpty() ? nullptr : kin->findAxis(r2Name);
    if (!r1)
        return ref;

    ref.r1Name = r1Name;
    ref.r2Name = r2Name;
    ref.r1 = normalizeAxisReference(r1Name, r1->currentPos);
    ref.r2 = r2 ? normalizeAxisReference(r2Name, r2->currentPos) : 0.0;
    ref.valid = true;
    return ref;
}

QString primaryRotaryTraversalAxis(const MachineKinematics* kin)
{
    if (!kin)
        return {};

    if (const MachineAxisDef* cAxis = kin->findAxis(QStringLiteral("C"))) {
        if (cAxis->motionType == MachineAxisDef::Rotary)
            return cAxis->name;
    }

    const auto [r1Name, r2Name] = orderedRotaryAxisNames(kin);
    if (!r1Name.isEmpty())
        return r1Name;
    return r2Name;
}

bool rotaryAxisFrame(const MachineKinematics* kin,
                     const QString& axisName,
                     gp_Pnt& origin,
                     gp_Dir& axisDir,
                     gp_Dir& zeroDir)
{
    if (!kin || axisName.isEmpty())
        return false;

    const MachineAxisDef* axis = kin->findAxis(axisName);
    if (!axis || axis->motionType != MachineAxisDef::Rotary)
        return false;

    const gp_Trsf axisTrsf = kin->computeAxisTransform(axisName);
    origin = axis->origin.Transformed(axisTrsf);
    axisDir = axis->direction.Transformed(axisTrsf);

    const gp_Vec axisVec(axisDir);
    gp_Vec ref(gp_Dir(1, 0, 0).Transformed(axisTrsf));
    ref = ref - axisVec * ref.Dot(axisVec);
    if (ref.Magnitude() <= 1e-9) {
        ref = gp_Vec(gp_Dir(0, 1, 0).Transformed(axisTrsf));
        ref = ref - axisVec * ref.Dot(axisVec);
    }
    if (ref.Magnitude() <= 1e-9) {
        ref = gp_Vec(1, 0, 0) - axisVec * gp_Vec(1, 0, 0).Dot(axisVec);
    }
    if (ref.Magnitude() <= 1e-9) {
        ref = gp_Vec(0, 1, 0) - axisVec * gp_Vec(0, 1, 0).Dot(axisVec);
    }
    if (ref.Magnitude() <= 1e-9)
        return false;

    zeroDir = gp_Dir(ref);
    return true;
}

bool pointRotaryAngleDeg(const gp_Pnt& point,
                         const gp_Pnt& origin,
                         const gp_Dir& axisDir,
                         const gp_Dir& zeroDir,
                         double& angleDeg)
{
    const gp_Vec axisVec(axisDir);
    gp_Vec radial(origin, point);
    radial = radial - axisVec * radial.Dot(axisVec);
    if (radial.Magnitude() <= 1e-8)
        return false;

    radial.Normalize();
    const gp_Vec zeroVec(zeroDir);
    const double sinV = axisVec.Dot(zeroVec.Crossed(radial));
    const double cosV = zeroVec.Dot(radial);
    angleDeg = normalizeSigned180(std::atan2(sinV, cosV) * 180.0 / M_PI);
    return true;
}

void reverseToolpathPoints(std::vector<ToolpathPoint>& points)
{
    std::reverse(points.begin(), points.end());
    for (ToolpathPoint& point : points)
        point.tangent = gp_Dir(-point.tangent.X(), -point.tangent.Y(), -point.tangent.Z());
}

bool contourHasClosingPoint(const LaserContour& contour)
{
    return contour.points.size() > 2
        && contour.points.front().position.SquareDistance(contour.points.back().position) < 1e-10;
}

void rotateClosedContourStart(std::vector<ToolpathPoint>& points,
                              const std::vector<double>& angles)
{
    if (points.size() < 3 || points.size() != angles.size())
        return;

    auto bestIt = std::min_element(
        angles.begin(), angles.end(),
        [](double a, double b) {
            const double aa = std::abs(normalizeSigned180(a));
            const double bb = std::abs(normalizeSigned180(b));
            if (std::abs(aa - bb) > 1e-9)
                return aa < bb;
            return a < b;
        });
    if (bestIt == angles.end())
        return;

    const auto start = static_cast<std::size_t>(std::distance(angles.begin(), bestIt));
    std::rotate(points.begin(), points.begin() + static_cast<std::ptrdiff_t>(start), points.end());
}

void normalizeContourTraversal(LaserContour& contour,
                               const MachineKinematics* kin,
                               const gp_Trsf& wpcTransform)
{
    if (contour.points.size() < 2)
        return;

    const QString axisName = primaryRotaryTraversalAxis(kin);
    gp_Pnt origin;
    gp_Dir axisDir(0, 0, 1);
    gp_Dir zeroDir(1, 0, 0);
    if (!rotaryAxisFrame(kin, axisName, origin, axisDir, zeroDir))
        return;

    const bool closedByWire = !contour.wire.IsNull() && contour.wire.Closed();
    const bool hadClosingPoint = contourHasClosingPoint(contour);
    const bool isClosed = closedByWire || hadClosingPoint;
    ToolpathPoint originalClosingPoint;
    if (hadClosingPoint) {
        originalClosingPoint = contour.points.back();
        contour.points.pop_back();
    }
    auto restoreOriginalClosingPoint = [&] {
        if (hadClosingPoint)
            contour.points.push_back(originalClosingPoint);
    };
    if (contour.points.size() < 2) {
        restoreOriginalClosingPoint();
        return;
    }

    std::vector<double> angles;
    angles.reserve(contour.points.size());
    for (const ToolpathPoint& point : contour.points) {
        const gp_Pnt worldPoint = point.position.Transformed(wpcTransform);
        double angle = 0.0;
        if (!pointRotaryAngleDeg(worldPoint, origin, axisDir, zeroDir, angle)) {
            restoreOriginalClosingPoint();
            return;
        }
        angles.push_back(angle);
    }

    double angularTravel = 0.0;
    for (std::size_t i = 1; i < angles.size(); ++i)
        angularTravel += normalizeSigned180(angles[i] - angles[i - 1]);
    if (isClosed && angles.size() > 2)
        angularTravel += normalizeSigned180(angles.front() - angles.back());

    if (angularTravel > 1e-6) {
        reverseToolpathPoints(contour.points);
        std::reverse(angles.begin(), angles.end());
    }

    if (isClosed) {
        if (contour.leadIn.valid) {
            auto selected = std::min_element(
                contour.points.begin(), contour.points.end(),
                [&contour](const ToolpathPoint& a, const ToolpathPoint& b) {
                    return a.position.SquareDistance(contour.leadIn.entryPoint)
                        < b.position.SquareDistance(contour.leadIn.entryPoint);
                });
            if (selected != contour.points.end())
                std::rotate(contour.points.begin(), selected, contour.points.end());
        } else {
            rotateClosedContourStart(contour.points, angles);
        }
    } else if (contour.leadIn.valid && contour.points.size() > 1) {
        const double frontDistance = contour.points.front().position.SquareDistance(
            contour.leadIn.entryPoint);
        const double backDistance = contour.points.back().position.SquareDistance(
            contour.leadIn.entryPoint);
        if (backDistance < frontDistance)
            reverseToolpathPoints(contour.points);
    }

    if (isClosed)
        contour.points.push_back(contour.points.front());
    if (contour.leadIn.valid && !contour.points.empty()) {
        contour.leadIn.entryPoint = contour.points.front().position;
        contour.leadIn.entryParam = contour.points.front().param;
        contour.leadIn.entryEdgeIndex = contour.points.front().sourceEdgeIndex;
        contour.leadIn.entryPointIndex = 0;
    }
}

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

constexpr std::array<double, 4> kLeadInDirectionProbeDistances{
    0.001, 0.0025, 0.005, 0.01};

struct MachiningFaceProbe
{
    TopoDS_Face face;
    Handle(Geom_Surface) surface;
    double tolerance{1e-7};
};

bool classifyPointOnFace(const MachiningFaceProbe& probe, const gp_Pnt& point)
{
    ShapeAnalysis_Surface analysis(probe.surface);
    const gp_Pnt2d uv = analysis.ValueOfUV(point, probe.tolerance);
    BRepClass_FaceClassifier classifier(probe.face, uv, probe.tolerance);
    return classifier.State() == TopAbs_IN || classifier.State() == TopAbs_ON;
}

std::vector<MachiningFaceProbe> findMachiningFacesAtStart(
    const LaserContour& contour,
    const ToolpathPoint& start)
{
    std::vector<MachiningFaceProbe> result;
    if (contour.sourceShape.IsNull())
        return result;

    constexpr double kNormalAlignment = 0.9;
    constexpr double kStartDistanceTolerance = 1e-4;
    std::vector<TopoDS_Face> sourceFaces;
    const bool hasBoundFaces =
        start.sourceEdgeIndex >= 0
        && start.sourceEdgeIndex < static_cast<int>(contour.leadInSurfaceContext.size())
        && !contour.leadInSurfaceContext[static_cast<std::size_t>(
                start.sourceEdgeIndex)].outerFaces.empty();
    if (hasBoundFaces) {
        sourceFaces = contour.leadInSurfaceContext[static_cast<std::size_t>(
            start.sourceEdgeIndex)].outerFaces;
    } else {
        for (TopExp_Explorer exp(contour.sourceShape, TopAbs_FACE); exp.More(); exp.Next())
            sourceFaces.push_back(TopoDS::Face(exp.Current()));
    }

    for (const TopoDS_Face& face : sourceFaces) {
        Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
        if (surface.IsNull())
            continue;

        const double tolerance = std::max(kStartDistanceTolerance,
                                          static_cast<double>(BRep_Tool::Tolerance(face)));
        ShapeAnalysis_Surface analysis(surface);
        const gp_Pnt2d uv = analysis.ValueOfUV(start.position, tolerance);

        gp_Pnt surfacePoint;
        surface->D0(uv.X(), uv.Y(), surfacePoint);
        if (surfacePoint.Distance(start.position) > tolerance)
            continue;

        GeomLProp_SLProps props(surface, uv.X(), uv.Y(), 1, tolerance);
        if (!props.IsNormalDefined())
            continue;
        gp_Dir faceNormal = props.Normal();
        if (face.Orientation() == TopAbs_REVERSED)
            faceNormal.Reverse();
        const double alignment = gp_Vec(faceNormal).Dot(gp_Vec(start.normal));
        if ((hasBoundFaces && std::abs(alignment) < 0.5)
            || (!hasBoundFaces && alignment < kNormalAlignment))
            continue;

        BRepClass_FaceClassifier classifier(face, uv, tolerance);
        if (classifier.State() != TopAbs_IN && classifier.State() != TopAbs_ON)
            continue;

        result.push_back({face, surface, tolerance});
    }
    return result;
}

bool liesOnMachiningFace(const std::vector<MachiningFaceProbe>& faces,
                         const gp_Pnt& point)
{
    return std::any_of(faces.begin(), faces.end(), [&](const MachiningFaceProbe& face) {
        return classifyPointOnFace(face, point);
    });
}

bool leadInProjectsOntoMachiningFace(const std::vector<MachiningFaceProbe>& faces,
                                     const gp_Pnt& start,
                                     const gp_Vec& direction,
                                     double length)
{
    // A point travelling in the tangent plane of a convex surface can be
    // outside the solid while still sitting directly above material. Classify
    // its projection against the trimmed outer-face domain instead of using
    // only the 3D solid state. Check the complete lead-in, including points
    // immediately next to the contour boundary.
    for (const double distance : kLeadInDirectionProbeDistances) {
        if (distance <= length
            && liesOnMachiningFace(faces, start.Translated(direction * distance))) {
            return true;
        }
    }

    constexpr double kMaximumSampleStep = 0.05;
    constexpr int kMaximumSamples = 256;
    const int sampleCount = std::clamp(
        static_cast<int>(std::ceil(length / kMaximumSampleStep)),
        2,
        kMaximumSamples);
    for (int sampleIndex = 1; sampleIndex <= sampleCount; ++sampleIndex) {
        const double distance = length * static_cast<double>(sampleIndex)
            / static_cast<double>(sampleCount);
        if (liesOnMachiningFace(faces, start.Translated(direction * distance)))
            return true;
    }
    return false;
}

bool appendFaceIfContainsEdge(std::vector<TopoDS_Face>& result,
                              const std::vector<TopoDS_Face>& candidates,
                              const TopoDS_Edge& edge)
{
    bool found = false;
    for (const TopoDS_Face& face : candidates) {
        bool contains = false;
        for (TopExp_Explorer exp(face, TopAbs_EDGE); exp.More(); exp.Next()) {
            if (edge.IsSame(exp.Current())) {
                contains = true;
                break;
            }
        }
        if (!contains)
            continue;

        const bool duplicate = std::any_of(
            result.begin(), result.end(),
            [&face](const TopoDS_Face& existing) { return existing.IsSame(face); });
        if (!duplicate)
            result.push_back(face);
        found = true;
    }
    return found;
}

void bindOwnedWireSurfaceContext(
    LaserContour& contour,
    const TopoDS_Face& owner,
    const TopTools_IndexedDataMapOfShapeListOfShape& edgeToFaces)
{
    contour.leadInSurfaceContext.clear();
    for (BRepTools_WireExplorer exp(contour.wire); exp.More(); exp.Next()) {
        LeadInEdgeSurfaceContext context;
        context.outerFaces.push_back(owner);
        const TopoDS_Edge edge = exp.Current();
        if (edgeToFaces.Contains(edge)) {
            const TopTools_ListOfShape& adjacentFaces = edgeToFaces.FindFromKey(edge);
            for (TopTools_ListIteratorOfListOfShape it(adjacentFaces);
                 it.More(); it.Next()) {
                const TopoDS_Face face = TopoDS::Face(it.Value());
                if (face.IsSame(owner))
                    continue;
                const bool duplicate = std::any_of(
                    context.crossSectionFaces.begin(),
                    context.crossSectionFaces.end(),
                    [&face](const TopoDS_Face& existing) {
                        return existing.IsSame(face);
                    });
                if (!duplicate)
                    context.crossSectionFaces.push_back(face);
            }
        }
        contour.leadInSurfaceContext.push_back(std::move(context));
    }
}

bool isMaterialSideOfWorkpiece(const TopoDS_Shape& workpiece,
                               const gp_Pnt& surfacePoint,
                               const gp_Dir& outwardNormal,
                               const gp_Vec& lateralDirection,
                               double lateralDistance)
{
    if (workpiece.IsNull())
        return false;

    // The face-UV test above is precise for a regular outer face, but a seam,
    // narrow trimmed face, or split analytic surface may leave both lateral
    // probes outside that one face.  Move each probe a small distance into the
    // workpiece and ask the solid classifier which side contains material.
    const double inwardDistance = std::max(0.0005, std::min(0.01, lateralDistance));
    const gp_Pnt probe = surfacePoint.Translated(lateralDirection * lateralDistance)
        .Translated(gp_Vec(outwardNormal) * -inwardDistance);
    BRepClass3d_SolidClassifier classifier(workpiece, probe, inwardDistance * 0.1);
    return classifier.State() == TopAbs_IN;
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

// ── Deterministic signature ────────────────────────────────────────────────
// FNV-1a-like 64-bit hash mixer used to fingerprint a contour wire so that
// re-running extractContours() on identical geometry produces identical IDs.
inline void mixHash(std::uint64_t& h, std::uint64_t v) noexcept
{
    h ^= v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
}

inline void mixQString(std::uint64_t& h, const QString& s) noexcept
{
    for (QChar ch : s)
        mixHash(h, static_cast<std::uint64_t>(ch.unicode()));
}

void computeContourSignature(LaserContour& contour)
{
    std::uint64_t h = 1469598103934665603ull; // FNV-1a 64-bit offset basis
    for (TopExp_Explorer exp(contour.wire, TopAbs_VERTEX); exp.More(); exp.Next()) {
        const gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(exp.Current()));
        mixHash(h, static_cast<std::uint64_t>(std::llround(p.X() * 1000.0)));
        mixHash(h, static_cast<std::uint64_t>(std::llround(p.Y() * 1000.0)));
        mixHash(h, static_cast<std::uint64_t>(std::llround(p.Z() * 1000.0)));
    }
    mixQString(h, contour.workpieceEntry);
    mixQString(h, contour.sourceInfo);
    contour.signature = h;
}

// ── Holes-first ordering rank ───────────────────────────────────────────────
// Lower rank = cut earlier. InnerHole before TubeCrossSection before
// OuterBoundary, so the outer ring is cut last and the part stays clamped.
int contourCutOrderRank(int contourType)
{
    switch (static_cast<ContourKind>(contourType)) {
    case ContourKind::InnerHole:        return 0;
    case ContourKind::TubeCrossSection: return 1;
    case ContourKind::OuterBoundary:    return 2;
    case ContourKind::Unknown:          return 3;
    }
    return 3;
}

bool mayTraverseExteriorShell(const BRepOffset_Analyse& concavity,
                              const TopoDS_Edge& edge)
{
    try {
        const BRepOffset_ListOfInterval& intervals = concavity.Type(edge);
        bool hasExteriorTransition = false;
        for (BRepOffset_ListIteratorOfListOfInterval it(intervals); it.More(); it.Next()) {
            switch (it.Value().Type()) {
            case ChFiDS_Concave:
            case ChFiDS_Mixed:
                // Crossing a concave edge enters a pocket, bore or other
                // cavity.  It must never promote that cavity to the exterior
                // machining shell.
                return false;
            case ChFiDS_Convex:
            case ChFiDS_Tangential:
            case ChFiDS_FreeBound:
                hasExteriorTransition = true;
                break;
            case ChFiDS_Other:
                break;
            }
        }
        return hasExteriorTransition;
    } catch (const Standard_Failure&) {
        // A malformed edge must not open a path into a cavity.
        return false;
    }
}

bool isAttachedToExteriorShell(
    const TopoDS_Face& face,
    const TopTools_IndexedDataMapOfShapeListOfShape& edgeToFaces,
    const BRepOffset_Analyse& concavity)
{
    bool hasAdjacentFace = false;
    for (TopExp_Explorer edgeExp(face, TopAbs_EDGE); edgeExp.More(); edgeExp.Next()) {
        const TopoDS_Edge edge = TopoDS::Edge(edgeExp.Current());
        if (!edgeToFaces.Contains(edge))
            continue;
        const TopTools_ListOfShape& adjacent = edgeToFaces.FindFromKey(edge);
        bool hasOtherFace = false;
        for (TopTools_ListIteratorOfListOfShape it(adjacent); it.More(); it.Next()) {
            if (!TopoDS::Face(it.Value()).IsSame(face)) {
                hasOtherFace = true;
                hasAdjacentFace = true;
                break;
            }
        }
        if (hasOtherFace && mayTraverseExteriorShell(concavity, edge))
            return true;
    }
    // A closed analytic outer face (for example an unsplit sphere) has no
    // neighbouring face across which to classify concavity.  Keep it; a hole
    // floor/wall always has a distinct adjacent cavity face.
    return !hasAdjacentFace;
}

TopoDS_Face makeTopOpenSectionFace(const TopoDS_Shape& workpiece)
{
    Bnd_Box workpieceBounds;
    BRepBndLib::Add(workpiece, workpieceBounds);
    if (workpieceBounds.IsVoid())
        return {};
    Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
    workpieceBounds.Get(xMin, yMin, zMin, xMax, yMax, zMax);
    const double extent = std::max({xMax - xMin, yMax - yMin, zMax - zMin, 1.0});
    const double tolerance = std::max(1e-4, extent * 1e-4);

    struct TopWire { TopoDS_Wire wire; double diagonal; };
    std::vector<TopWire> topWires;
    for (TopExp_Explorer exp(workpiece, TopAbs_WIRE); exp.More(); exp.Next()) {
        const TopoDS_Wire wire = TopoDS::Wire(exp.Current());
        if (wire.IsNull() || !wire.Closed()
            || std::any_of(topWires.cbegin(), topWires.cend(),
                           [&wire](const TopWire& existing) {
                               return existing.wire.IsSame(wire);
                           })) {
            continue;
        }
        Bnd_Box bounds;
        BRepBndLib::Add(wire, bounds);
        if (bounds.IsVoid())
            continue;
        Standard_Real wxMin, wyMin, wzMin, wxMax, wyMax, wzMax;
        bounds.Get(wxMin, wyMin, wzMin, wxMax, wyMax, wzMax);
        if (wzMax < zMax - tolerance || wzMax - wzMin > tolerance)
            continue;
        const double dx = wxMax - wxMin;
        const double dy = wyMax - wyMin;
        topWires.push_back({wire, std::sqrt(dx * dx + dy * dy)});
    }
    if (topWires.size() < 2)
        return {};

    std::sort(topWires.begin(), topWires.end(),
              [](const TopWire& left, const TopWire& right) {
                  return left.diagonal > right.diagonal;
              });
    BRepBuilderAPI_MakeFace faceBuilder(topWires.front().wire);
    for (std::size_t index = 1; index < topWires.size(); ++index)
        faceBuilder.Add(topWires[index].wire);
    return faceBuilder.IsDone() ? faceBuilder.Face() : TopoDS_Face();
}

} // namespace

std::uint64_t LaserToolpathBuilder::computeFaceSignature(const TopoDS_Face& face)
{
    std::uint64_t h = 1469598103934665603ull; // FNV-1a 64-bit offset basis

    // Surface type — GeomAbs_SurfaceType is a stable enum.
    BRepAdaptor_Surface adaptor(face, /*restriction=*/Standard_True);
    mixHash(h, static_cast<std::uint64_t>(adaptor.GetType()));

    // Area (mm², rounded to 3 decimal places).
    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    const double area = props.Mass();
    mixHash(h, static_cast<std::uint64_t>(std::llround(area * 1000.0)));

    // Centroid (mm, rounded to 3 decimal places).
    const gp_Pnt centroid = props.CentreOfMass();
    mixHash(h, static_cast<std::uint64_t>(std::llround(centroid.X() * 1000.0)));
    mixHash(h, static_cast<std::uint64_t>(std::llround(centroid.Y() * 1000.0)));
    mixHash(h, static_cast<std::uint64_t>(std::llround(centroid.Z() * 1000.0)));

    // Outer-wire vertex count (discriminates faces with same area/type but
    // different topology, e.g. a rectangular vs circular face of equal area).
    TopoDS_Wire outerWire = BRepTools::OuterWire(face);
    if (!outerWire.IsNull()) {
        int vertexCount = 0;
        for (TopExp_Explorer exp(outerWire, TopAbs_VERTEX); exp.More(); exp.Next())
            ++vertexCount;
        mixHash(h, static_cast<std::uint64_t>(vertexCount));
    }

    return h;
}

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
    struct OwnedWire {
        TopoDS_Wire wire;
        TopoDS_Face owner;
    };
    std::vector<OwnedWire> outerWires;
    auto isAlreadyCollected = [&](const TopoDS_Wire& w) {
        for (const auto& ow : outerWires)
            if (ow.wire.IsSame(w)) return true;
        return false;
    };

    for (TopExp_Explorer faceExp(workpiece, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
        const TopoDS_Face& face = TopoDS::Face(faceExp.Current());
        TopoDS_Wire outer = BRepTools::OuterWire(face);
        if (!outer.IsNull() && !isAlreadyCollected(outer))
            outerWires.push_back({outer, face});
    }

    TopTools_IndexedDataMapOfShapeListOfShape edgeToFaces;
    TopExp::MapShapesAndAncestors(
        workpiece, TopAbs_EDGE, TopAbs_FACE, edgeToFaces);
    int wireIdx = 0;
    for (auto& owned : outerWires) {
        LaserContour c;
        c.wire = owned.wire;
        // 中文翻译：外轮廓 %1
        c.name = QString::fromUtf8("Outer contour %1").arg(++wireIdx);
        bindOwnedWireSurfaceContext(c, owned.owner, edgeToFaces);
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
                // 中文翻译：边缘 %1
                c.name = QString::fromUtf8("Edge %1").arg(++edgeIdx);
                result.push_back(std::move(c));
            }
        }
    }

    // Assign deterministic signature to each contour.
    for (auto& c : result)
        computeContourSignature(c);

    return result;
}

// =============================================================================
// LaserToolpathBuilder — face-classification-based contour extraction
// =============================================================================

std::vector<LaserContour> LaserToolpathBuilder::extractContours(
    const TopoDS_Shape& workpiece,
    const ContourExtractionParams& params,
    FaceClassification* classificationOut)
{
    if (workpiece.IsNull()) {
        if (classificationOut)
            *classificationOut = {};
        return {};
    }

    // ── Strategy dispatch ────────────────────────────────────────────────
    switch (params.strategy) {
    case ExtractionStrategy::LegacyOuterWire:
        if (classificationOut)
            *classificationOut = {};
        return extractContours(workpiece);
    case ExtractionStrategy::ManualFaceSelection:
        if (classificationOut)
            *classificationOut = {};
        return extractContoursFromFaces(workpiece, params.selectedMachiningFaces,
                                        params.machiningBeamDirection, params);
    case ExtractionStrategy::PlanarFaceWires: {
        QString info;
        const std::vector<TopoDS_Face> faces = selectTopVisibleFacesFromPositiveZ(
            workpiece, &info);
        if (classificationOut)
            *classificationOut = {};
        if (faces.empty())
            return {};
        return extractContoursFromFaces(workpiece, faces,
                                        params.machiningBeamDirection, params);
    }
    case ExtractionStrategy::LargestSmoothConnectedSurface:
        break;  // face-classification path below
    }

    // ── Step 1: classify faces by smooth connectivity ────────────────────
    FaceClassification classification =
        FaceClassifier::classifyFaces(workpiece, params.smoothAngleThresholdDeg);
    if (classificationOut)
        *classificationOut = classification;

    if (!classification.hasOuter())
        return extractContours(workpiece); // malformed/empty topology safety net

    // The generic default deliberately uses the largest smooth-connected
    // exterior group itself as the machining surface.  Unlike the removed
    // pipe/section mode, it does not require a separately inferred section.
    const std::vector<TopoDS_Face>& outerFaces = classification.outerGroup()->faces;
    std::vector<LaserContour> result = extractContoursFromFaces(
        workpiece, outerFaces, params.machiningBeamDirection, params);

    // Keep the classified section faces for the geometric calculations that
    // follow contour selection.  They do not participate in selecting the
    // contour, but they anchor each point's machining normal to the selected
    // outer surface, provide the adjacent-section normal, and let the lead-in
    // safety rules reject a direction that points into the section side.
    std::vector<TopoDS_Face> crossFaces;
    for (const FaceGroup* group : classification.crossSectionGroups()) {
        if (group)
            crossFaces.insert(crossFaces.end(), group->faces.begin(), group->faces.end());
    }
    for (LaserContour& contour : result) {
        bindLeadInSurfaceContext(contour, outerFaces, crossFaces);
        discretizeContourWithClassification(contour, outerFaces, crossFaces,
                                            params.deflection);
    }
    if (result.empty()) {
        // A classified but unusable boundary retains the established fallback.
        ContourExtractionParams fallback;
        fallback.smoothAngleThresholdDeg = params.smoothAngleThresholdDeg;
        fallback.deflection = params.deflection;
        fallback.strategy = ExtractionStrategy::LegacyOuterWire;
        return extractContours(workpiece, fallback);
    }
    return result;
}

std::vector<LaserContour> LaserToolpathBuilder::extractTubeContoursFromFaceGroups(
    const TopoDS_Shape& workpiece,
    const std::vector<TopoDS_Face>& outerFaces,
    const std::vector<TopoDS_Face>& crossSectionFaces,
    const ContourExtractionParams& params)
{
    if (workpiece.IsNull() || outerFaces.empty() || crossSectionFaces.empty())
        return {};

    FaceClassification classification;
    FaceGroup outerGroup;
    outerGroup.kind = FaceGroupKind::Outer;
    outerGroup.faces = outerFaces;
    classification.groups.push_back(std::move(outerGroup));
    classification.outerIdx = 0;
    FaceGroup crossSectionGroup;
    crossSectionGroup.kind = FaceGroupKind::CrossSection;
    crossSectionGroup.faces = crossSectionFaces;
    classification.groups.push_back(std::move(crossSectionGroup));
    const std::vector<TopoDS_Edge> contourEdges =
        FaceClassifier::extractContourEdges(classification);
    if (contourEdges.empty())
        return {};

    const std::vector<TopoDS_Wire> wires = FaceClassifier::chainEdgesToWires(contourEdges);
    if (wires.empty())
        return {};

    std::vector<LaserContour> result;
    result.reserve(wires.size());
    int wireIdx = 0;
    for (const TopoDS_Wire& w : wires) {
        LaserContour c;
        c.wire = w;
        // 中文翻译：加工轮廓 %1
        c.name = QString::fromUtf8("Machining contour %1").arg(++wireIdx);
        c.contourType = static_cast<int>(ContourKind::TubeCrossSection);
        // 中文翻译：外表面(%1面) ∩ 截面(%2面)
        c.sourceInfo  = QString::fromUtf8("Outer surface (%1 surface) ∩ Cross section (%2 surface)")
                            .arg(outerFaces.size()).arg(crossSectionFaces.size());

        computeContourSignature(c);

        // Discretise with face-classification-aware normals
        bindLeadInSurfaceContext(c, outerFaces, crossSectionFaces);
        discretizeContourWithClassification(c, outerFaces, crossSectionFaces,
                                            params.deflection);
        result.push_back(std::move(c));
    }
    return result;
}

// =============================================================================
// LaserToolpathBuilder - machining-face selection (planar / plate workflow)
// =============================================================================

bool LaserToolpathBuilder::isPlanarFace(const TopoDS_Face& face)
{
    if (face.IsNull())
        return false;
    BRepAdaptor_Surface surf(face, Standard_True);
    return surf.GetType() == GeomAbs_Plane;
}

std::vector<TopoDS_Face> LaserToolpathBuilder::selectTopVisibleFacesFromPositiveZ(
    const TopoDS_Shape& workpiece, QString* info)
{
    if (info)
        info->clear();

    std::vector<TopoDS_Face> result;
    if (workpiece.IsNull())
        return result;

    // An open vertical tube has no planar cap face: its valid Z-top machining
    // surface is the annular boundary formed by the outer and inner top wires.
    // Prefer that explicit highest cross-section over cavity faces.
    const TopoDS_Face topOpenSection = makeTopOpenSectionFace(workpiece);
    if (!topOpenSection.IsNull())
        return {topOpenSection};

    // Assemblies and imported open shells do not always provide usable mass
    // properties.  Their bounding-box centre is still a stable interior-side
    // reference for rejecting the inside of a bore or perforation.
    Bnd_Box workpieceBounds;
    BRepBndLib::Add(workpiece, workpieceBounds);
    if (workpieceBounds.IsVoid())
        return result;
    Standard_Real workpieceXMin, workpieceYMin, workpieceZMin;
    Standard_Real workpieceXMax, workpieceYMax, workpieceZMax;
    workpieceBounds.Get(workpieceXMin, workpieceYMin, workpieceZMin,
                        workpieceXMax, workpieceYMax, workpieceZMax);
    GProp_GProps volumeProperties;
    BRepGProp::VolumeProperties(workpiece, volumeProperties);
    const bool hasVolumeCenter = std::abs(volumeProperties.Mass()) > 1e-9;
    const gp_Pnt volumeCenter = hasVolumeCenter
        ? volumeProperties.CentreOfMass()
        : gp_Pnt((workpieceXMin + workpieceXMax) * 0.5,
                 (workpieceYMin + workpieceYMax) * 0.5,
                 (workpieceZMin + workpieceZMax) * 0.5);

    struct FaceSample {
        gp_Pnt point;
        double z{0.0};
    };
    struct FaceCandidate {
        TopoDS_Face face;
        std::vector<FaceSample> samples;
    };

    // A face can be the first hit of a -Z ray only where its outward normal has
    // a positive Z component.  Collect those cheap candidates first; their
    // cavity/exterior relationship is resolved from boundary concavity before
    // they enter the expensive ray intersector.
    constexpr std::array<double, 5> kUvFractions{0.1, 0.3, 0.5, 0.7, 0.9};
    constexpr int kMaxSamplesPerFace = 8;
    std::vector<FaceCandidate> candidates;
    for (TopExp_Explorer faceExp(workpiece, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
        const TopoDS_Face face = TopoDS::Face(faceExp.Current());
        Standard_Real uMin, uMax, vMin, vMax;
        BRepTools::UVBounds(face, uMin, uMax, vMin, vMax);
        if (!std::isfinite(uMin) || !std::isfinite(uMax)
            || !std::isfinite(vMin) || !std::isfinite(vMax)) {
            continue;
        }

        BRepAdaptor_Surface surface(face, Standard_True);
        const bool isPlanar = surface.GetType() == GeomAbs_Plane;
        // A bore wall can have a small locally upward-facing strip, even
        // though the face as a whole points toward the material centre.  Test
        // the face-wide orientation before accepting that strip as a Z-top
        // surface.  A true exterior skin points away from the reference
        // centre on average; a horizontal cylinder remains neutral and is
        // deliberately kept for its exposed upper half.
        if (!isPlanar) {
            double radialOrientationSum = 0.0;
            int radialOrientationCount = 0;
            for (double uFraction : kUvFractions) {
                const double u = uMin + (uMax - uMin) * uFraction;
                for (double vFraction : kUvFractions) {
                    const double v = vMin + (vMax - vMin) * vFraction;
                    BRepClass_FaceClassifier classifier(
                        face, gp_Pnt2d(u, v), 1e-7, Standard_True);
                    if (classifier.State() != TopAbs_IN && classifier.State() != TopAbs_ON)
                        continue;
                    gp_Pnt point;
                    gp_Vec dU;
                    gp_Vec dV;
                    surface.D1(u, v, point, dU, dV);
                    gp_Vec normal = dU.Crossed(dV);
                    if (normal.SquareMagnitude() <= 1e-18)
                        continue;
                    if (face.Orientation() == TopAbs_REVERSED)
                        normal.Reverse();
                    normal.Normalize();
                    const gp_Vec fromVolumeCenter(volumeCenter, point);
                    if (fromVolumeCenter.Magnitude() <= 1e-8)
                        continue;
                    radialOrientationSum += normal.Dot(fromVolumeCenter)
                        / fromVolumeCenter.Magnitude();
                    ++radialOrientationCount;
                }
            }
            if (radialOrientationCount > 0
                && radialOrientationSum < -1e-3 * radialOrientationCount) {
                continue;
            }
        }
        std::vector<FaceSample> samples;
        samples.reserve(kUvFractions.size() * kUvFractions.size());
        auto appendSample = [&](double u, double v) {
            gp_Pnt point;
            gp_Vec dU;
            gp_Vec dV;
            surface.D1(u, v, point, dU, dV);
            gp_Vec normal = dU.Crossed(dV);
            if (normal.SquareMagnitude() <= 1e-18)
                return;
            if (face.Orientation() == TopAbs_REVERSED)
                normal.Reverse();
            normal.Normalize();
            // STEP imports occasionally reverse a planar cap's face
            // orientation.  A Z-parallel plane is still a valid top-layer
            // candidate; the later first-hit ray decides whether it is
            // actually exposed.  Curved faces still require +Z outward
            // normal so side/hole walls remain cheap rejects.
            if ((!isPlanar && normal.Z() <= 1e-6)
                || (isPlanar && std::abs(normal.Z()) <= 1e-6)) {
                return;
            }

            // A radial/side blind-hole face points into the solid's
            // volume, while a true exterior face points away from its
            // volume centre.  This rejects those cavity faces before the
            // expensive trimmed-face and ray tests.  The later edge test
            // still handles axial blind holes whose floor faces upward.
            if (!isPlanar) {
                gp_Vec fromVolumeCenter(volumeCenter, point);
                if (fromVolumeCenter.Magnitude() > 1e-8
                    && normal.Dot(fromVolumeCenter) < -1e-7) {
                    return;
                }
            }

            // Test the trimmed face only after the cheap normal test.
            // Vertical side and hole walls therefore avoid the more
            // expensive 2-D face classifier altogether.
            BRepClass_FaceClassifier classifier(
                face, gp_Pnt2d(u, v), 1e-7, Standard_True);
            if (classifier.State() != TopAbs_IN && classifier.State() != TopAbs_ON)
                return;
            samples.push_back({point, point.Z()});
        };
        for (double uFraction : kUvFractions) {
            const double u = uMin + (uMax - uMin) * uFraction;
            for (double vFraction : kUvFractions) {
                const double v = vMin + (vMax - vMin) * vFraction;
                appendSample(u, v);
            }
        }
        // Dense perforations can cover all coarse UV probes on an otherwise
        // valid large outer face.  A face-local triangulation respects trims
        // and holes, so its triangle UV centroids provide real interior
        // probes.  Refine only that exceptional face instead of making every
        // hole wall pay a meshing cost.
        if (samples.empty()) {
            // Meshing attaches Poly_Triangulation data to the supplied BRep.
            // `face` can be a subshape of the live XCAF workpiece, so doing
            // that here would silently replace its exact display geometry with
            // this coarse, classification-only mesh.  Keep the fallback's
            // temporary triangulation entirely private to the algorithm.
            BRepBuilderAPI_Copy copy;
            copy.Perform(face, Standard_True, Standard_False);
            const TopoDS_Face sampleFace = copy.IsDone()
                ? TopoDS::Face(copy.Shape())
                : TopoDS_Face{};
            if (!sampleFace.IsNull()) {
                BRepMesh_IncrementalMesh mesh(
                    sampleFace, 0.1, Standard_False, 0.5, Standard_True);
                TopLoc_Location location;
                const Handle(Poly_Triangulation) triangulation =
                    BRep_Tool::Triangulation(sampleFace, location);
                if (!triangulation.IsNull() && triangulation->HasUVNodes()) {
                    for (Standard_Integer index = 1;
                         index <= triangulation->NbTriangles() && samples.size() < 8;
                         ++index) {
                        Standard_Integer first, second, third;
                        triangulation->Triangle(index).Get(first, second, third);
                        const gp_Pnt2d firstUv = triangulation->UVNode(first);
                        const gp_Pnt2d secondUv = triangulation->UVNode(second);
                        const gp_Pnt2d thirdUv = triangulation->UVNode(third);
                        appendSample((firstUv.X() + secondUv.X() + thirdUv.X()) / 3.0,
                                     (firstUv.Y() + secondUv.Y() + thirdUv.Y()) / 3.0);
                    }
                }
            }
        }
        if (samples.empty()) {
            constexpr int kRefinedSampleCount = 11;
            for (int uIndex = 1; uIndex <= kRefinedSampleCount; ++uIndex) {
                const double u = uMin + (uMax - uMin) * uIndex
                    / (kRefinedSampleCount + 1.0);
                for (int vIndex = 1; vIndex <= kRefinedSampleCount; ++vIndex) {
                    const double v = vMin + (vMax - vMin) * vIndex
                        / (kRefinedSampleCount + 1.0);
                    appendSample(u, v);
                }
            }
        }
        if (samples.empty())
            continue;
        std::sort(samples.begin(), samples.end(),
                  [](const FaceSample& left, const FaceSample& right) {
                      return left.z > right.z;
                  });
        if (static_cast<int>(samples.size()) > kMaxSamplesPerFace)
            samples.resize(kMaxSamplesPerFace);
        candidates.push_back({face, std::move(samples)});
    }

    if (candidates.empty()) {
        if (info)
            *info = QStringLiteral("no upward-facing surface is available for +Z visibility analysis");
        return result;
    }

    // A cavity bottom can be visible through a blind-hole opening, so Z depth
    // alone cannot define the machining exterior.  Filter candidates by their
    // boundary topology before any ray work: a true exterior face connects to
    // a neighbour across a convex/tangent edge, whereas a hole floor or wall
    // is bounded entirely by concave cavity edges.
    TopTools_IndexedDataMapOfShapeListOfShape edgeToFaces;
    TopExp::MapShapesAndAncestors(workpiece, TopAbs_EDGE, TopAbs_FACE, edgeToFaces);
    BRepOffset_Analyse concavity(workpiece, M_PI / 180.0);
    if (concavity.IsDone()) {
        candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
                                        [&edgeToFaces, &concavity](const FaceCandidate& candidate) {
                                            return !isAttachedToExteriorShell(
                                                candidate.face, edgeToFaces, concavity);
                                        }),
                         candidates.end());
    }
    if (candidates.empty()) {
        if (info)
            *info = QStringLiteral("no upward-facing exterior shell is available for +Z visibility analysis");
        return result;
    }

    const double zSpan = std::max(1.0, workpieceZMax - workpieceZMin);
    const double rayStartZ = workpieceZMax + zSpan * 0.01 + 1e-3;
    const double rayLength = rayStartZ - workpieceZMin + zSpan * 0.01 + 1e-3;

    // Intersect the complete workpiece, rather than only the candidates.
    // A tilted hole wall can locally face +Z and therefore be a candidate, but
    // its -Z ray must first hit the exterior skin above it.  Loading only
    // candidates made such a wall incorrectly hit itself.
    IntCurvesFace_ShapeIntersector intersector;
    intersector.Load(workpiece, 1e-7);
    for (const FaceCandidate& candidate : candidates) {
        for (const FaceSample& faceSample : candidate.samples) {
            const gp_Pnt& sample = faceSample.point;
            const gp_Lin ray(gp_Pnt(sample.X(), sample.Y(), rayStartZ),
                             gp_Dir(0.0, 0.0, -1.0));
            intersector.PerformNearest(ray, 0.0, rayLength);
            if (intersector.IsDone() && intersector.NbPnt() > 0
                && intersector.Face(1).IsSame(candidate.face)) {
                result.push_back(candidate.face);
                break;
            }
        }
    }

    if (result.empty() && info)
        *info = QStringLiteral("no face is visible from the +Z parallel-light direction");
    return result;
}

TopoDS_Face LaserToolpathBuilder::selectMachiningFace(
    const TopoDS_Shape& workpiece, const gp_Dir& beamDirWpc, QString* info)
{
    if (info)
        info->clear();
    if (workpiece.IsNull())
        return TopoDS_Face();

    // Among planar faces whose outward normal opposes the beam (i.e. the face
    // the laser actually hits), pick the one most directly facing the beam;
    // break near-ties by larger area. The beam travels along beamDirWpc, so a
    // face facing the laser has outwardNormal . beamDirWpc < 0, most negative
    // = most head-on.
    TopoDS_Face best;
    double bestScore = 0.0;
    double bestArea = 0.0;
    bool found = false;
    const gp_Vec beam(beamDirWpc);

    for (TopExp_Explorer fExp(workpiece, TopAbs_FACE); fExp.More(); fExp.Next()) {
        const TopoDS_Face face = TopoDS::Face(fExp.Current());
        if (face.IsNull())
            continue;
        BRepAdaptor_Surface surf(face, Standard_True);
        if (surf.GetType() != GeomAbs_Plane)
            continue;  // only planar faces qualify as the machining face

        gp_Dir normal = surf.Plane().Axis().Direction();
        if (face.Orientation() == TopAbs_REVERSED)
            normal.Reverse();

        const double score = beam.Dot(gp_Vec(normal));
        if (score >= 0.0)
            continue;  // does not face the beam

        GProp_GProps props;
        BRepGProp::SurfaceProperties(face, props);
        const double area = props.Mass();

        if (!found) {
            best = face; bestScore = score; bestArea = area; found = true;
        } else if (score < bestScore - 1e-6) {
            best = face; bestScore = score; bestArea = area;  // more head-on
        } else if (std::abs(score - bestScore) <= 1e-6 && area > bestArea) {
            best = face; bestArea = area;  // same facing, larger face
        }
    }

    if (!found && info)
        *info = QStringLiteral(
            "no planar face faces the beam (tube side or ambiguous; use tube/manual)");
    return best;
}

std::vector<LaserContour> LaserToolpathBuilder::extractContoursFromFaces(
    const TopoDS_Shape& workpiece,
    const std::vector<TopoDS_Face>& faces,
    const gp_Dir& /*beamDirWpc*/,
    const ContourExtractionParams& /*params*/)
{
    std::vector<LaserContour> result;
    if (workpiece.IsNull() || faces.empty())
        return result;

    TopTools_IndexedDataMapOfShapeListOfShape edgeToFaces;
    TopExp::MapShapesAndAncestors(
        workpiece, TopAbs_EDGE, TopAbs_FACE, edgeToFaces);

    auto wireBboxDiag = [](const TopoDS_Wire& w) -> double {
        Bnd_Box bbox;
        BRepBndLib::Add(w, bbox);
        if (bbox.IsVoid())
            return 0.0;
        Standard_Real x0, y0, z0, x1, y1, z1;
        bbox.Get(x0, y0, z0, x1, y1, z1);
        const double dx = x1 - x0, dy = y1 - y0, dz = z1 - z0;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    };

    // A machining face is commonly a smooth group (for example a tube
    // surface split into several CAD faces).  Taking each face's wires
    // independently turns the group's internal seams into false contours.
    // Build the boundary of the entire selected group instead: retain an edge
    // only when it has a non-selected adjacent face (or is a free edge).
    TopTools_IndexedMapOfShape selectedFaceMap;
    for (const TopoDS_Face& face : faces)
        if (!face.IsNull())
            selectedFaceMap.Add(face);

    struct BoundaryEdge { TopoDS_Edge edge; TopoDS_Face owner; };
    std::vector<BoundaryEdge> boundaryEdges;
    TopTools_IndexedMapOfShape collectedEdges;
    for (const TopoDS_Face& face : faces) {
        if (face.IsNull())
            continue;
        for (TopExp_Explorer edgeExp(face, TopAbs_EDGE); edgeExp.More(); edgeExp.Next()) {
            const TopoDS_Edge edge = TopoDS::Edge(edgeExp.Current());
            if (BRep_Tool::Degenerated(edge) || collectedEdges.Contains(edge))
                continue;

            int adjacentCount = 0;
            int selectedAdjacentCount = 0;
            if (edgeToFaces.Contains(edge)) {
                const TopTools_ListOfShape& adjacentFaces = edgeToFaces.FindFromKey(edge);
                for (TopTools_ListIteratorOfListOfShape it(adjacentFaces); it.More(); it.Next()) {
                    ++adjacentCount;
                    if (selectedFaceMap.Contains(it.Value()))
                        ++selectedAdjacentCount;
                }
            }
            // An edge shared exclusively by two or more selected faces is an
            // internal seam. A free edge remains a valid contour boundary.
            if (adjacentCount > 1 && adjacentCount == selectedAdjacentCount)
                continue;

            collectedEdges.Add(edge);
            boundaryEdges.push_back({edge, face});
        }
    }

    std::vector<TopoDS_Edge> edges;
    edges.reserve(boundaryEdges.size());
    for (const BoundaryEdge& item : boundaryEdges)
        edges.push_back(item.edge);
    const std::vector<TopoDS_Wire> wires = FaceClassifier::chainEdgesToWires(edges);

    const bool hasNonPlanarFace = std::any_of(faces.cbegin(), faces.cend(),
        [](const TopoDS_Face& face) { return !face.IsNull() && !LaserToolpathBuilder::isPlanarFace(face); });
    std::size_t largestWire = 0;
    for (std::size_t index = 1; index < wires.size(); ++index)
        if (wireBboxDiag(wires[index]) > wireBboxDiag(wires[largestWire]))
            largestWire = index;

    int holeIdx = 0, outerIdx = 0, tubeIdx = 0;
    for (std::size_t index = 0; index < wires.size(); ++index) {
        const TopoDS_Wire& wire = wires[index];
        if (wire.IsNull() || !wire.Closed())
            continue;

        TopoDS_Face owner;
        for (BRepTools_WireExplorer edgeExp(wire); edgeExp.More(); edgeExp.Next()) {
            const TopoDS_Edge edge = edgeExp.Current();
            const auto found = std::find_if(boundaryEdges.cbegin(), boundaryEdges.cend(),
                [&edge](const BoundaryEdge& item) { return item.edge.IsSame(edge); });
            if (found != boundaryEdges.cend()) {
                owner = found->owner;
                break;
            }
        }
        if (owner.IsNull())
            continue;

        LaserContour contour;
        contour.wire = wire;
        if (hasNonPlanarFace) {
            contour.contourType = static_cast<int>(ContourKind::TubeCrossSection);
            // 中文翻译：加工轮廓 %1
            contour.name = QString::fromUtf8("Machining contour %1").arg(++tubeIdx);
            // 中文翻译：手动加工面组边界
            contour.sourceInfo = QString::fromUtf8("Manually process quilt boundaries");
        } else {
            const bool isOuter = index == largestWire;
            contour.contourType = static_cast<int>(isOuter ? ContourKind::OuterBoundary
                                                            : ContourKind::InnerHole);
            // 中文翻译：外轮廓 %1
            contour.name = isOuter ? QString::fromUtf8("Outer contour %1").arg(++outerIdx)
                                   // 中文翻译：孔 %1
                                   : QString::fromUtf8("Hole %1").arg(++holeIdx);
            // 中文翻译：加工面组外边界
            contour.sourceInfo = isOuter ? QString::fromUtf8("Processing outer boundary of dough group")
                                         // 中文翻译：加工面组孔边界
                                         : QString::fromUtf8("Machining quilt hole boundaries");
        }
        bindOwnedWireSurfaceContext(contour, owner, edgeToFaces);
        computeContourSignature(contour);
        result.push_back(std::move(contour));
    }

    // Holes first, outer ring last (stable): keeps the part clamped while inner
    // holes are cut, and leaves the freeing outer cut for the end.
    std::stable_sort(result.begin(), result.end(),
        [](const LaserContour& a, const LaserContour& b) {
            return contourCutOrderRank(a.contourType)
                 < contourCutOrderRank(b.contourType);
        });

    return result;
}

// =============================================================================
// LaserToolpathBuilder — discretization
// =============================================================================

void LaserToolpathBuilder::discretizeContour(LaserContour& contour,
                                             const TopoDS_Shape& workpiece,
                                             double deflection)
{
    const int anchoredEdgeIndex = contour.leadIn.valid ? contour.leadIn.entryEdgeIndex : -1;
    const double anchoredParam = contour.leadIn.entryParam;
    contour.points.clear();
    if (contour.wire.IsNull())
        return;

    const gp_Pnt workpieceCenter = shapeCenter(workpiece);

    // 必须按 WireExplorer 的连接顺序遍历边，不能用 TopExp_Explorer（拓扑集合顺序不保证连贯）。
    int edgeIndex = 0;
    for (BRepTools_WireExplorer exp(contour.wire); exp.More(); exp.Next(), ++edgeIndex) {
        const TopoDS_Edge edge = exp.Current();
        if (BRep_Tool::Degenerated(edge))
            continue;

        BRepAdaptor_Curve curve(edge);
        GCPnts_UniformDeflection sampler(curve, deflection);
        if (!sampler.IsDone())
            continue;

        const bool reversed = edge.Orientation() == TopAbs_REVERSED;
        std::vector<double> parameters;
        parameters.reserve(static_cast<std::size_t>(sampler.NbPoints() + 1));
        for (int i = 1; i <= sampler.NbPoints(); ++i)
            parameters.push_back(sampler.Parameter(i));
        if (edgeIndex == anchoredEdgeIndex)
            parameters.push_back(anchoredParam);
        std::sort(parameters.begin(), parameters.end());
        parameters.erase(std::unique(parameters.begin(), parameters.end(), [](double a, double b) {
            return std::abs(a - b) <= 1e-12;
        }), parameters.end());
        if (reversed)
            std::reverse(parameters.begin(), parameters.end());

        const LeadInEdgeSurfaceContext* surfaceContext =
            edgeIndex >= 0
                && edgeIndex < static_cast<int>(contour.leadInSurfaceContext.size())
            ? &contour.leadInSurfaceContext[static_cast<std::size_t>(edgeIndex)]
            : nullptr;
        const std::vector<TopoDS_Face>* pointOuterFaces =
            surfaceContext && !surfaceContext->outerFaces.empty()
            ? &surfaceContext->outerFaces : nullptr;
        const std::vector<TopoDS_Face>* pointCrossFaces =
            surfaceContext && !surfaceContext->crossSectionFaces.empty()
            ? &surfaceContext->crossSectionFaces : nullptr;

        for (double parameter : parameters) {
            ToolpathPoint tp;
            tp.param = parameter;
            tp.sourceEdgeIndex = edgeIndex;
            curve.D0(tp.param, tp.position);

            if (!contour.points.empty()
                && contour.points.back().position.SquareDistance(tp.position) < 1e-12) {
                if (edgeIndex == anchoredEdgeIndex
                    && std::abs(tp.param - anchoredParam) <= 1e-12)
                    contour.points.back() = tp;
                continue; // 去除相邻边连接处重复点
            }

            const gp_Dir surfaceNormal = pointOuterFaces
                ? findMachiningNormal(
                      tp.position,
                      *pointOuterFaces)
                : findSurfaceNormal(workpiece, tp.position);
            tp.normal = pointOuterFaces
                ? surfaceNormal
                : enforceOutwardDirection(
                      tp.position, surfaceNormal, workpieceCenter);

            if (pointCrossFaces) {
                double crossDistance = 0.0;
                tp.crossSectionNormalValid = findClosestFaceNormal(
                    tp.position,
                    *pointCrossFaces,
                    tp.crossSectionNormal,
                    crossDistance);
            }

            gp_Pnt pDummy;
            gp_Vec tangentVec;
            curve.D1(tp.param, pDummy, tangentVec);
            if (reversed)
                tangentVec.Reverse();
            if (tangentVec.Magnitude() > 1e-10)
                tp.tangent = gp_Dir(tangentVec);
            else
                tp.tangent = gp_Dir(0, 0, 1);

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
    const int anchoredEdgeIndex = contour.leadIn.valid ? contour.leadIn.entryEdgeIndex : -1;
    const double anchoredParam = contour.leadIn.entryParam;
    contour.points.clear();
    if (contour.wire.IsNull())
        return;

    const gp_Pnt outerCenter = faceGroupCenter(outerFaces);

    // 必须按 WireExplorer 的连接顺序遍历边，并尊重每条边的 Orientation。
    // TopExp_Explorer 只是拓扑枚举，会导致矩形孔等多边轮廓边之间顺序错乱。
    int edgeIndex = 0;
    for (BRepTools_WireExplorer exp(contour.wire); exp.More(); exp.Next(), ++edgeIndex) {
        const TopoDS_Edge edge = exp.Current();
        if (BRep_Tool::Degenerated(edge))
            continue;

        BRepAdaptor_Curve curve(edge);
        GCPnts_UniformDeflection sampler(curve, deflection);
        if (!sampler.IsDone())
            continue;

        const bool reversed = edge.Orientation() == TopAbs_REVERSED;
        std::vector<double> parameters;
        parameters.reserve(static_cast<std::size_t>(sampler.NbPoints() + 1));
        for (int i = 1; i <= sampler.NbPoints(); ++i)
            parameters.push_back(sampler.Parameter(i));
        if (edgeIndex == anchoredEdgeIndex)
            parameters.push_back(anchoredParam);
        std::sort(parameters.begin(), parameters.end());
        parameters.erase(std::unique(parameters.begin(), parameters.end(), [](double a, double b) {
            return std::abs(a - b) <= 1e-12;
        }), parameters.end());
        if (reversed)
            std::reverse(parameters.begin(), parameters.end());

        const LeadInEdgeSurfaceContext* surfaceContext =
            edgeIndex >= 0
                && edgeIndex < static_cast<int>(contour.leadInSurfaceContext.size())
            ? &contour.leadInSurfaceContext[static_cast<std::size_t>(edgeIndex)]
            : nullptr;
        const std::vector<TopoDS_Face>& pointOuterFaces =
            surfaceContext && !surfaceContext->outerFaces.empty()
            ? surfaceContext->outerFaces : outerFaces;
        const std::vector<TopoDS_Face>& pointCrossFaces =
            surfaceContext && !surfaceContext->crossSectionFaces.empty()
            ? surfaceContext->crossSectionFaces : crossFaces;

        for (double parameter : parameters) {
            ToolpathPoint tp;
            tp.param = parameter;
            tp.sourceEdgeIndex = edgeIndex;
            curve.D0(tp.param, tp.position);

            if (!contour.points.empty()
                && contour.points.back().position.SquareDistance(tp.position) < 1e-12) {
                if (edgeIndex == anchoredEdgeIndex
                    && std::abs(tp.param - anchoredParam) <= 1e-12)
                    contour.points.back() = tp;
                continue; // 去除相邻边连接处重复点，避免插补重复点和角度突跳
            }

            // Compute machining normal using face classification
            tp.normal = avoidCrossSectionDirection(
                tp.position,
                findMachiningNormal(tp.position, pointOuterFaces),
                outerCenter,
                pointCrossFaces);

            double crossDistance = 0.0;
            tp.crossSectionNormalValid = findClosestFaceNormal(
                tp.position, pointCrossFaces, tp.crossSectionNormal, crossDistance);

            // Compute tangent along the curve
            gp_Pnt pDummy;
            gp_Vec tangentVec;
            curve.D1(tp.param, pDummy, tangentVec);
            if (reversed)
                tangentVec.Reverse();
            if (tangentVec.Magnitude() > 1e-10)
                tp.tangent = gp_Dir(tangentVec);
            else
                tp.tangent = gp_Dir(0, 0, 1);

            contour.points.push_back(tp);
        }
    }
}

void LaserToolpathBuilder::bindLeadInSurfaceContext(
    LaserContour& contour,
    const std::vector<TopoDS_Face>& outerFaces,
    const std::vector<TopoDS_Face>& crossFaces)
{
    contour.leadInSurfaceContext.clear();
    if (contour.wire.IsNull())
        return;

    for (BRepTools_WireExplorer exp(contour.wire); exp.More(); exp.Next()) {
        LeadInEdgeSurfaceContext context;
        const TopoDS_Edge edge = exp.Current();
        appendFaceIfContainsEdge(context.outerFaces, outerFaces, edge);
        appendFaceIfContainsEdge(context.crossSectionFaces, crossFaces, edge);
        contour.leadInSurfaceContext.push_back(std::move(context));
    }
}

// =============================================================================
// LaserToolpathBuilder — machining normal (face-classification-aware)
// =============================================================================

gp_Dir LaserToolpathBuilder::findMachiningNormal(
    const gp_Pnt& pt,
    const std::vector<TopoDS_Face>& outerFaces)
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

bool LaserToolpathBuilder::setContourStart(LaserContour& contour,
                                           int pointIndex,
                                           QString* error)
{
    if (pointIndex < 0 || pointIndex >= static_cast<int>(contour.points.size())) {
        // 中文翻译：轮廓起点索引无效
        if (error) *error = QStringLiteral("Contour start index is invalid");
        return false;
    }

    const bool hadClosingPoint = contourHasClosingPoint(contour);
    const bool closed = hadClosingPoint || (!contour.wire.IsNull() && contour.wire.Closed());
    if (hadClosingPoint) {
        if (pointIndex == static_cast<int>(contour.points.size()) - 1)
            pointIndex = 0;
        contour.points.pop_back();
    }
    if (contour.points.empty()) {
        // 中文翻译：轮廓没有可用采样点
        if (error) *error = QStringLiteral("Contour has no available sample points");
        return false;
    }

    if (closed) {
        std::rotate(contour.points.begin(),
                    contour.points.begin() + pointIndex,
                    contour.points.end());
        contour.points.push_back(contour.points.front());
    } else if (pointIndex == static_cast<int>(contour.points.size()) - 1) {
        reverseToolpathPoints(contour.points);
    } else if (pointIndex != 0) {
        // 中文翻译：开放轮廓只能选择端点作为加工起点
        if (error) *error = QStringLiteral("For open contours, only the endpoint can be selected as the starting point for processing.");
        return false;
    }

    contour.leadIn.entryPoint = contour.points.front().position;
    contour.leadIn.entryParam = contour.points.front().param;
    contour.leadIn.entryEdgeIndex = contour.points.front().sourceEdgeIndex;
    contour.leadIn.entryPointIndex = 0;
    contour.leadIn.valid = true;
    contour.leadInSolution = computeLeadInSolution(contour, contour.leadIn.length);
    return true;
}

bool LaserToolpathBuilder::setAutomaticContourStart(LaserContour& contour,
                                                    QString* error)
{
    if (contour.points.empty()) {
        if (error)
            // 中文翻译：轮廓没有可用采样点
            *error = QStringLiteral("Contour has no available sample points");
        return false;
    }

    const bool closed = contourHasClosingPoint(contour)
        || (!contour.wire.IsNull() && contour.wire.Closed());
    const int candidateCount = static_cast<int>(contour.points.size())
        - (contourHasClosingPoint(contour) ? 1 : 0);
    std::vector<int> candidates;
    candidates.reserve(static_cast<std::size_t>(std::max(0, candidateCount)));

    // Prefer samples strictly inside one source edge. At a wire vertex both
    // neighbouring trimmed faces can reject a tiny tangent-plane probe even
    // though the contour itself is valid.
    if (closed) {
        for (int i = 0; i < candidateCount; ++i) {
            const int previous = (i + candidateCount - 1) % candidateCount;
            const int next = (i + 1) % candidateCount;
            if (contour.points[previous].sourceEdgeIndex
                    == contour.points[i].sourceEdgeIndex
                && contour.points[next].sourceEdgeIndex
                    == contour.points[i].sourceEdgeIndex) {
                candidates.push_back(i);
            }
        }
    }
    for (int i = 0; i < candidateCount; ++i) {
        if (std::find(candidates.begin(), candidates.end(), i) == candidates.end())
            candidates.push_back(i);
    }

    QString lastError;
    for (const int pointIndex : candidates) {
        LaserContour candidate = contour;
        QString candidateError;
        if (!setContourStart(candidate, pointIndex, &candidateError))
            continue;
        if (!candidate.leadInSolution.valid) {
            lastError = candidate.leadInSolution.error;
            continue;
        }
        contour = std::move(candidate);
        return true;
    }

    if (error) {
        *error = lastError.isEmpty()
            // 中文翻译：轮廓没有可确定悬空侧的安全起点
            ? QStringLiteral("The contour has no safe starting point from which to determine the overhanging side")
            : lastError;
    }
    return false;
}

TopoDS_Edge LaserToolpathBuilder::computeLeadInEdge(const LaserContour& contour)
{
    if (!contour.leadInSolution.valid || contour.points.empty())
        return TopoDS_Edge();

    BRepBuilderAPI_MakeEdge edgeMaker(contour.leadInSolution.point.position,
                                     contour.points.front().position);
    if (!edgeMaker.IsDone())
        return TopoDS_Edge();

    return edgeMaker.Edge();
}

LeadInSolution LaserToolpathBuilder::computeLeadInSolution(
    const LaserContour& contour,
    double length)
{
    LeadInSolution result;
    if (!contour.leadIn.valid) {
        // 中文翻译：未选择轮廓起点
        result.error = QStringLiteral("Contour start point not selected");
        return result;
    }
    if (length <= 0.0) {
        // 中文翻译：下刀长度必须大于 0
        result.error = QStringLiteral("The cutting length must be greater than 0");
        return result;
    }
    if (contour.points.empty()) {
        // 中文翻译：轮廓没有采样点
        result.error = QStringLiteral("Contour has no sampling points");
        return result;
    }

    const ToolpathPoint& start = contour.points.front();
    if (contour.sourceShape.IsNull()) {
        // 中文翻译：无法判断悬空方向：缺少工件几何
        result.error = QStringLiteral("Unable to determine hanging direction: missing workpiece geometry");
        return result;
    }

    const gp_Vec outer(start.normal);
    const gp_Vec tangent(start.tangent);
    // 由加工面法线和轮廓切线构造面内左右方向。只用起点附近的两个
    // 测试点区分实体外表面与孔洞悬空侧，不用当前下刀长度做实体分类。
    gp_Vec direction = outer.Crossed(tangent);
    if (direction.Magnitude() <= 1e-9) {
        // 中文翻译：轮廓切线与加工面法线无法确定面内方向
        result.error = QStringLiteral("The contour tangent and the normal of the processing surface cannot determine the in-plane direction.");
        return result;
    }
    direction.Normalize();

    const std::vector<MachiningFaceProbe> machiningFaces =
        findMachiningFacesAtStart(contour, start);
    if (machiningFaces.empty()) {
        // 中文翻译：无法找到轮廓起点所在的加工外表面
        result.error = QStringLiteral("Unable to find the machined outer surface where the contour start point is located");
        return result;
    }

    // A solid classifier alone is unsafe for convex surfaces such as an
    // ellipse: a tangent-plane point can be outside the solid while its
    // projection still lies above the material face. The projected, trimmed
    // outer-face domain is therefore the final material-side gate.
    bool forwardOnSurface = leadInProjectsOntoMachiningFace(
        machiningFaces, start.position, direction, length);
    bool reverseOnSurface = leadInProjectsOntoMachiningFace(
        machiningFaces, start.position, direction.Reversed(), length);

    bool sideResolved = false;
    if (forwardOnSurface != reverseOnSurface) {
        if (forwardOnSurface)
            direction.Reverse();
        sideResolved = true;
    }
    // When BOTH sides project onto the machining face (typical for a small
    // hole: the lead-in probe crosses the hole and lands on the face on the
    // far side), do NOT bail out - fall through to the cross-section normal
    // and solid-classifier fallbacks below, which can still identify the
    // suspended (void) side from the adjacent hole-wall normal.

    // The outward normal of the exact cross-section face adjacent to this
    // contour edge points into the cut void. It ranks two directions that are
    // both outside the outer-face projection domain, but it never overrides a
    // direction proven to be above material by the check above.
    if (!sideResolved && start.crossSectionNormalValid) {
        gp_Vec crossOut(start.crossSectionNormal);
        const gp_Vec projected = crossOut - outer * crossOut.Dot(outer);
        if (projected.Magnitude() > 1e-9) {
            const gp_Dir suspended(projected);
            const double alignment = gp_Vec(direction).Dot(gp_Vec(suspended));
            if (std::abs(alignment) >= 0.5) {
                if (alignment < 0.0)
                    direction.Reverse();
                sideResolved = true;
            }
        }
    }

    if (!sideResolved) {
        for (const double probeDistance : kLeadInDirectionProbeDistances) {
            forwardOnSurface = isMaterialSideOfWorkpiece(
                contour.sourceShape, start.position, start.normal,
                direction, probeDistance);
            reverseOnSurface = isMaterialSideOfWorkpiece(
                contour.sourceShape, start.position, start.normal,
                direction.Reversed(), probeDistance);
            if (forwardOnSurface != reverseOnSurface) {
                sideResolved = true;
                break;
            }
        }

        if (!sideResolved) {
            result.error = QStringLiteral(
                // 中文翻译：下刀线投影未落在加工外表面，且实体分类未能确认材料侧，无法确定悬空侧
                "The projection of the lower knife line does not fall on the outer surface of the machining, and the material side cannot be confirmed by entity classification, and the suspended side cannot be determined.");
            return result;
        }

        if (forwardOnSurface)
            direction.Reverse();
    }

    ToolpathPoint point = start;
    point.position = start.position.Translated(direction * length);
    point.machineCoord = {};
    result.direction = gp_Dir(direction);
    result.point = std::move(point);
    result.valid = true;
    return result;
}

// =============================================================================
// LaserToolpathBuilder — machine coordinate computation (IK)
// =============================================================================

bool LaserToolpathBuilder::solveToolpathForOrder(
    const std::vector<LaserContour*>& orderedContours,
    MachineKinematics* kinematics,
    const gp_Trsf& wpcTransform,
    const lcnc::MachineModeDefinition& modeDefinition,
    const lcnc::WorkpieceSetupTransform& workpieceSetup,
    const lcnc::HeadToolGeometry& headToolGeometry,
    QString* errorMessage,
    const lcnc::SolvedMachinePose* initialPose)
{
    if (!kinematics) {
        if (errorMessage) *errorMessage = QStringLiteral("Machine kinematics is unavailable");
        return false;
    }
    QString definitionError;
    if (!modeDefinition.isValid(&definitionError)) {
        if (errorMessage) *errorMessage = definitionError;
        return false;
    }

    lcnc::ToolpathSolverRegistry registry;
    lcnc::SolvedMachinePose continuity = initialPose ? *initialPose : lcnc::SolvedMachinePose{};
    const auto applyFixedPlanarBeamNormal = [&modeDefinition](std::vector<ToolpathPoint>& points) {
        if (modeDefinition.mode != lcnc::MachiningMode::Planar3Axis)
            return;
        // Three-axis processing has one physical beam posture.  Preserve the
        // source normals on LaserContour for geometric/lead-in classification,
        // but hand the solver a machine-Z normal for every cutting point.
        // 中文翻译：三轴加工只有一个物理刀束姿态；保留轮廓源法线用于几何和引入线分类，但向求解器传入统一机床 Z 向法线。
        for (ToolpathPoint& point : points)
            point.normal = gp_Dir(0, 0, 1);
    };
    const auto applyPose = [&modeDefinition](MachineCoord& coordinate,
                                             const lcnc::SolvedMachinePose& pose) {
        coordinate = {};
        coordinate.solvedPose = pose;
        coordinate.valid = pose.valid;
        int rotarySlot = 0;
        for (int index = 0; index < modeDefinition.interpolatedAxes.count; ++index) {
            const auto& slot = modeDefinition.interpolatedAxes.axes[index];
            const double value = pose.value(index);
            switch (slot.role) {
            case lcnc::MachineAxisRole::LinearX: coordinate.x = value; break;
            case lcnc::MachineAxisRole::LinearY: coordinate.y = value; break;
            case lcnc::MachineAxisRole::LinearZ: coordinate.z = value; break;
            default:
                if (rotarySlot == 0) { coordinate.r1 = value; coordinate.r1Name = slot.name; }
                else if (rotarySlot == 1) { coordinate.r2 = value; coordinate.r2Name = slot.name; }
                ++rotarySlot;
                break;
            }
        }
    };

    for (LaserContour* contour : orderedContours) {
        if (!contour) continue;
        // Traversal normalization must see the same CAD-to-machine placement
        // as the solver.  The point solve keeps the transforms separate to
        // avoid applying WorkpieceSetupTransform twice.
        // 中文翻译：轮廓方向归一化必须使用与求解器一致的 CAD 到机床安装姿态，同时避免工件安装姿态重复应用。
        const gp_Trsf traversalTransform =
            wpcTransform.Multiplied(workpieceSetup.toTransform());
        normalizeContourTraversal(*contour, kinematics, traversalTransform);

        // The lead-in is the first motion of this contour, not an independent
        // positioning move.  Solve it in the same ordered sequence as the
        // cutting points so a rotary branch selected from the previous contour
        // remains consistent for every axis of the entry pose.
        // 中文翻译：下刀点是当前轮廓的首段运动，不得独立定位；必须和切割点在同一有序序列内求解，
        // 以便继承上一轮廓选择的旋转分支并保持所有轴位姿一致。
        const bool hasLeadIn = contour->leadInSolution.valid;
        std::vector<ToolpathPoint> worldPoints;
        worldPoints.reserve(contour->points.size() + (hasLeadIn ? 1u : 0u));
        if (hasLeadIn)
            worldPoints.push_back(contour->leadInSolution.point);
        worldPoints.insert(worldPoints.end(), contour->points.cbegin(), contour->points.cend());
        for (ToolpathPoint& point : worldPoints) {
            point.position.Transform(wpcTransform);
            point.normal.Transform(wpcTransform);
        }
        applyFixedPlanarBeamNormal(worldPoints);
        lcnc::ToolpathKinematicsRequest request;
        request.machine = kinematics;
        request.mode = modeDefinition.mode;
        request.definition = modeDefinition;
        request.workpieceSetup = workpieceSetup;
        request.headToolGeometry = headToolGeometry;
        request.points = &worldPoints;
        request.previousPose = continuity.valid ? &continuity : nullptr;
        const std::vector<lcnc::SolvedMachinePose> poses = registry.solve(request);
        if (poses.size() != worldPoints.size()) {
            if (errorMessage) *errorMessage = QStringLiteral("Solver %1 returned an invalid point count")
                .arg(modeDefinition.solverId);
            return false;
        }
        for (std::size_t index = 0; index < poses.size(); ++index) {
            if (hasLeadIn && index == 0)
                applyPose(contour->leadInSolution.point.machineCoord, poses[index]);
            else
                applyPose(contour->points[index - (hasLeadIn ? 1u : 0u)].machineCoord, poses[index]);
            if (!poses[index].valid) {
                if (errorMessage) *errorMessage = poses[index].failureReason;
                return false;
            }
            continuity = poses[index];
        }
    }
    return true;
}

bool LaserToolpathBuilder::solveTransientMotionPath(
    std::vector<ToolpathPoint>* points,
    MachineKinematics* kinematics,
    const lcnc::MachineModeDefinition& modeDefinition,
    const lcnc::WorkpieceSetupTransform& workpieceSetup,
    const lcnc::HeadToolGeometry& headToolGeometry,
    QString* errorMessage,
    const lcnc::SolvedMachinePose* initialPose)
{
    if (!points || points->size() < 2) {
        if (errorMessage) *errorMessage = QStringLiteral("Transient motion path requires at least two points");
        return false;
    }
    if (!kinematics) {
        if (errorMessage) *errorMessage = QStringLiteral("Machine kinematics is unavailable");
        return false;
    }
    QString definitionError;
    if (!modeDefinition.isValid(&definitionError)) {
        if (errorMessage) *errorMessage = definitionError;
        return false;
    }

    std::vector<ToolpathPoint> solvePoints = *points;
    if (modeDefinition.mode == lcnc::MachiningMode::Planar3Axis) {
        for (ToolpathPoint& point : solvePoints)
            point.normal = gp_Dir(0, 0, 1);
    }
    lcnc::ToolpathKinematicsRequest request;
    request.machine = kinematics;
    request.mode = modeDefinition.mode;
    request.definition = modeDefinition;
    request.workpieceSetup = workpieceSetup;
    request.headToolGeometry = headToolGeometry;
    request.points = &solvePoints;
    request.previousPose = initialPose && initialPose->valid ? initialPose : nullptr;
    lcnc::ToolpathSolverRegistry registry;
    const std::vector<lcnc::SolvedMachinePose> poses = registry.solve(request);
    if (poses.size() != points->size()) {
        if (errorMessage) *errorMessage = QStringLiteral("Transient motion solver returned an invalid point count");
        return false;
    }
    for (std::size_t index = 0; index < poses.size(); ++index) {
        const lcnc::SolvedMachinePose& pose = poses[index];
        if (!pose.valid) {
            if (errorMessage) *errorMessage = pose.failureReason;
            return false;
        }
        MachineCoord& coordinate = points->at(index).machineCoord;
        coordinate = {};
        coordinate.solvedPose = pose;
        coordinate.valid = true;
        int rotarySlot = 0;
        for (int axisIndex = 0; axisIndex < modeDefinition.interpolatedAxes.count; ++axisIndex) {
            const auto& slot = modeDefinition.interpolatedAxes.axes[axisIndex];
            const double value = pose.value(axisIndex);
            switch (slot.role) {
            case lcnc::MachineAxisRole::LinearX: coordinate.x = value; break;
            case lcnc::MachineAxisRole::LinearY: coordinate.y = value; break;
            case lcnc::MachineAxisRole::LinearZ: coordinate.z = value; break;
            default:
                if (rotarySlot == 0) { coordinate.r1 = value; coordinate.r1Name = slot.name; }
                else if (rotarySlot == 1) { coordinate.r2 = value; coordinate.r2Name = slot.name; }
                ++rotarySlot;
                break;
            }
        }
    }
    return true;
}
