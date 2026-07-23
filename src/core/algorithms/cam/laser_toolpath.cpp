#include "core/algorithms/cam/laser_toolpath.h"
#include "core/algorithms/cam/face_classifier.h"
#include "core/kinematics/ik_solver.h"
#include "core/logging/logger.h"

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
#include <BRepTools_WireExplorer.hxx>
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
#include <gp_Trsf.hxx>

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
    for (TopExp_Explorer exp(contour.sourceShape, TopAbs_FACE); exp.More(); exp.Next()) {
        const TopoDS_Face face = TopoDS::Face(exp.Current());
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
        if (gp_Vec(faceNormal).Dot(gp_Vec(start.normal)) < kNormalAlignment)
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
    // If face classification is disabled, use the legacy method directly.
    if (!params.useFaceClassification) {
        if (classificationOut)
            *classificationOut = {};
        return extractContours(workpiece);
    }

    if (workpiece.IsNull())
        return {};

    // ── Step 1: classify faces by smooth connectivity ────────────────────
    FaceClassification classification =
        FaceClassifier::classifyFaces(workpiece, params.smoothAngleThresholdDeg);
    if (classificationOut)
        *classificationOut = classification;

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
        for (auto& c : result)
            computeContourSignature(c);
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

        computeContourSignature(c);

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

            tp.normal = enforceOutwardDirection(
                tp.position,
                findSurfaceNormal(workpiece, tp.position),
                workpieceCenter);

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
                findMachiningNormal(tp.position, outerFaces, crossFaces),
                outerCenter,
                crossFaces);

            double crossDistance = 0.0;
            tp.crossSectionNormalValid = findClosestFaceNormal(
                tp.position, crossFaces, tp.crossSectionNormal, crossDistance);

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

bool LaserToolpathBuilder::setContourStart(LaserContour& contour,
                                           int pointIndex,
                                           QString* error)
{
    if (pointIndex < 0 || pointIndex >= static_cast<int>(contour.points.size())) {
        if (error) *error = QStringLiteral("轮廓起点索引无效");
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
        if (error) *error = QStringLiteral("轮廓没有可用采样点");
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
        if (error) *error = QStringLiteral("开放轮廓只能选择端点作为加工起点");
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
        result.error = QStringLiteral("未选择轮廓起点");
        return result;
    }
    if (length <= 0.0) {
        result.error = QStringLiteral("下刀长度必须大于 0");
        return result;
    }
    if (contour.points.empty()) {
        result.error = QStringLiteral("轮廓没有采样点");
        return result;
    }

    const ToolpathPoint& start = contour.points.front();
    if (contour.sourceShape.IsNull()) {
        result.error = QStringLiteral("无法判断悬空方向：缺少工件几何");
        return result;
    }

    const gp_Vec outer(start.normal);
    const gp_Vec tangent(start.tangent);
    // 由加工面法线和轮廓切线构造面内左右方向。只用起点附近的两个
    // 测试点区分实体外表面与孔洞悬空侧，不用当前下刀长度做实体分类。
    gp_Vec direction = outer.Crossed(tangent);
    if (direction.Magnitude() <= 1e-9) {
        result.error = QStringLiteral("轮廓切线与加工面法线无法确定面内方向");
        return result;
    }
    direction.Normalize();

    const std::vector<MachiningFaceProbe> machiningFaces =
        findMachiningFacesAtStart(contour, start);
    if (machiningFaces.empty()) {
        result.error = QStringLiteral("无法找到轮廓起点所在的加工外表面");
        return result;
    }

    // Small/narrow trimmed faces can reject a fixed 0.01 mm UV probe on both
    // sides even though one side is the machining face.  Probe progressively
    // while retaining the safety rule: accept a direction only when its
    // opposite side is positively classified as machining surface.
    bool sideResolved = false;
    bool forwardOnSurface = false;
    bool reverseOnSurface = false;
    for (const double probeDistance : kLeadInDirectionProbeDistances) {
        const gp_Pnt forwardProbe = start.position.Translated(direction * probeDistance);
        const gp_Pnt reverseProbe = start.position.Translated(direction * -probeDistance);
        forwardOnSurface = liesOnMachiningFace(machiningFaces, forwardProbe);
        reverseOnSurface = liesOnMachiningFace(machiningFaces, reverseProbe);
        if (forwardOnSurface != reverseOnSurface) {
            sideResolved = true;
            break;
        }
    }
    if (!sideResolved) {
        result.error = forwardOnSurface
            ? QStringLiteral("轮廓起点两侧近点均落在加工外表面，无法确定悬空侧")
            : QStringLiteral("轮廓起点两侧近点均未落在加工外表面，无法确定悬空侧");
        return result;
    }

    if (forwardOnSurface)
        direction.Reverse();

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

void LaserToolpathBuilder::computeMachineCoordinates(LaserContour& contour,
                                                     MachineKinematics* kinematics,
                                                     const gp_Trsf& wpcTransform,
                                                     MachineCoord* continuityState)
{
    if (!kinematics) return;

    normalizeContourTraversal(contour, kinematics, wpcTransform);

    const bool tracePoints = camToolpathTraceEnabled();
    if (tracePoints) {
        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "cam.toolpath.ik.begin: contour='{}' id={} entry='{}' cfg='{}' points={}",
                   contour.name.toStdString(),
                   contour.contourId,
                   contour.workpieceEntry.toStdString(),
                   kinematics->configType().toStdString(),
                   contour.points.size());
        for (const MachineAxisDef& axis : kinematics->axes()) {
            LCNC_DEBUG(lcnc::LogCode::Generic,
                       "cam.toolpath.ik.axis: name='{}' type={} parent='{}' dir=({:.6f},{:.6f},{:.6f}) origin=({:.6f},{:.6f},{:.6f}) limits=[{:.3f},{:.3f}] current={:.3f}",
                       axis.name.toStdString(),
                       motionTypeText(axis.motionType),
                       axis.parentAxis.toStdString(),
                       axis.direction.X(), axis.direction.Y(), axis.direction.Z(),
                       axis.origin.X(), axis.origin.Y(), axis.origin.Z(),
                       axis.minVal, axis.maxVal,
                       axis.currentPos);
        }
    }

    RangeStats localX, localY, localZ;
    RangeStats worldX, worldY, worldZ;
    RangeStats normalX, normalY, normalZ;
    RangeStats machineX, machineY, machineZ, machineR1, machineR2;
    int validCount = 0;
    QString r1Name;
    QString r2Name;

    MachineCoord previous = (continuityState && continuityState->valid)
        ? *continuityState
        : currentRotaryReference(kinematics);
    bool hasPrevious = previous.valid;

    if (contour.leadInSolution.valid) {
        ToolpathPoint& leadPoint = contour.leadInSolution.point;
        const gp_Pnt leadWorldPos = leadPoint.position.Transformed(wpcTransform);
        const gp_Dir leadWorldDir = leadPoint.normal.Transformed(wpcTransform);
        leadPoint.machineCoord = IKSolver::solveContinuous(
            kinematics, leadWorldPos, leadWorldDir, hasPrevious ? &previous : nullptr);
        if (leadPoint.machineCoord.valid) {
            previous = leadPoint.machineCoord;
            hasPrevious = true;
        } else {
            contour.leadInSolution.valid = false;
            contour.leadInSolution.error = QStringLiteral("下刀点五轴坐标求解失败");
        }
    }

    int pointIndex = 0;
    for (auto& pt : contour.points) {
        // Transform from workpiece-local to world frame
        gp_Pnt worldPos = pt.position.Transformed(wpcTransform);
        gp_Dir worldDir = pt.normal.Transformed(wpcTransform);

        localX.add(pt.position.X());
        localY.add(pt.position.Y());
        localZ.add(pt.position.Z());
        worldX.add(worldPos.X());
        worldY.add(worldPos.Y());
        worldZ.add(worldPos.Z());
        normalX.add(worldDir.X());
        normalY.add(worldDir.Y());
        normalZ.add(worldDir.Z());

        pt.machineCoord = IKSolver::solveContinuous(
            kinematics, worldPos, worldDir, hasPrevious ? &previous : nullptr);
        if (pt.machineCoord.valid) {
            previous = pt.machineCoord;
            hasPrevious = true;
            ++validCount;
            machineX.add(pt.machineCoord.x);
            machineY.add(pt.machineCoord.y);
            machineZ.add(pt.machineCoord.z);
            machineR1.add(pt.machineCoord.r1);
            machineR2.add(pt.machineCoord.r2);
            if (r1Name.isEmpty()) r1Name = pt.machineCoord.r1Name;
            if (r2Name.isEmpty()) r2Name = pt.machineCoord.r2Name;
        }

        if (tracePoints) {
            LCNC_DEBUG(lcnc::LogCode::Generic,
                       "cam.toolpath.ik.point: contour='{}' id={} idx={} param={:.9f} local=({:.6f},{:.6f},{:.6f}) world=({:.6f},{:.6f},{:.6f}) normal=({:.6f},{:.6f},{:.6f}) tangent=({:.6f},{:.6f},{:.6f}) machine.valid={} machine=({:.6f},{:.6f},{:.6f},{:.6f},{:.6f}) rotary='{}','{}'",
                       contour.name.toStdString(),
                       contour.contourId,
                       pointIndex,
                       pt.param,
                       pt.position.X(), pt.position.Y(), pt.position.Z(),
                       worldPos.X(), worldPos.Y(), worldPos.Z(),
                       worldDir.X(), worldDir.Y(), worldDir.Z(),
                       pt.tangent.X(), pt.tangent.Y(), pt.tangent.Z(),
                       pt.machineCoord.valid,
                       pt.machineCoord.x, pt.machineCoord.y, pt.machineCoord.z,
                       pt.machineCoord.r1, pt.machineCoord.r2,
                       pt.machineCoord.r1Name.toStdString(),
                       pt.machineCoord.r2Name.toStdString());
        }
        ++pointIndex;
    }

    if (contour.leadInSolution.valid
        && contour.leadInSolution.point.machineCoord.valid
        && !contour.points.empty()
        && contour.points.front().machineCoord.valid) {
        MachineCoord& leadCoord = contour.leadInSolution.point.machineCoord;
        const MachineCoord& startCoord = contour.points.front().machineCoord;
        leadCoord.r1 = startCoord.r1;
        leadCoord.r2 = startCoord.r2;
        leadCoord.r1Name = startCoord.r1Name;
        leadCoord.r2Name = startCoord.r2Name;
    }

    if (continuityState && previous.valid)
        *continuityState = previous;

    LCNC_INFO(lcnc::LogCode::Generic,
              "cam.toolpath.ik.summary: contour='{}' id={} entry='{}' cfg='{}' points={} valid={} localX=[{:.6f},{:.6f}] localY=[{:.6f},{:.6f}] localZ=[{:.6f},{:.6f}] worldX=[{:.6f},{:.6f}] worldY=[{:.6f},{:.6f}] worldZ=[{:.6f},{:.6f}] normalX=[{:.6f},{:.6f}] normalY=[{:.6f},{:.6f}] normalZ=[{:.6f},{:.6f}] machineX=[{:.6f},{:.6f}] machineY=[{:.6f},{:.6f}] machineZ=[{:.6f},{:.6f}] {}=[{:.6f},{:.6f}] {}=[{:.6f},{:.6f}] tracePoints={}",
              contour.name.toStdString(),
              contour.contourId,
              contour.workpieceEntry.toStdString(),
              kinematics->configType().toStdString(),
              contour.points.size(),
              validCount,
              rangeMin(localX), rangeMax(localX),
              rangeMin(localY), rangeMax(localY),
              rangeMin(localZ), rangeMax(localZ),
              rangeMin(worldX), rangeMax(worldX),
              rangeMin(worldY), rangeMax(worldY),
              rangeMin(worldZ), rangeMax(worldZ),
              rangeMin(normalX), rangeMax(normalX),
              rangeMin(normalY), rangeMax(normalY),
              rangeMin(normalZ), rangeMax(normalZ),
              rangeMin(machineX), rangeMax(machineX),
              rangeMin(machineY), rangeMax(machineY),
              rangeMin(machineZ), rangeMax(machineZ),
              r1Name.isEmpty() ? "R1" : r1Name.toStdString(),
              rangeMin(machineR1), rangeMax(machineR1),
              r2Name.isEmpty() ? "R2" : r2Name.toStdString(),
              rangeMin(machineR2), rangeMax(machineR2),
              tracePoints);
}

void LaserToolpathBuilder::computeMachineCoordinatesForOrder(
    const std::vector<LaserContour*>& orderedContours,
    MachineKinematics* kinematics,
    const gp_Trsf& wpcTransform,
    MachineCoord* initialState)
{
    if (!kinematics)
        return;

    MachineCoord continuityState;
    if (initialState && initialState->valid)
        continuityState = *initialState;

    for (LaserContour* contour : orderedContours) {
        if (!contour)
            continue;
        computeMachineCoordinates(*contour, kinematics, wpcTransform, &continuityState);
    }

    if (initialState && continuityState.valid)
        *initialState = continuityState;
}
