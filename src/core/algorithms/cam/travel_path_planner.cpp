#include "core/algorithms/cam/travel_path_planner.h"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopLoc_Location.hxx>
#include <Poly_Triangulation.hxx>
#include <Standard_Failure.hxx>
#include <gp_Pnt.hxx>
#include <gp_Sphere.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>
#include <limits>

namespace lcnc::cam_algo {
namespace {

constexpr double kEpsilon = 1e-9;
constexpr int kMinimumCurveDivisions = 4;
constexpr int kMaximumCurveDivisions = 128;

gp_Pnt pointOf(const lcnc::cam::RapidPose& pose)
{
    return gp_Pnt(pose.tcpX, pose.tcpY, pose.tcpZ);
}

gp_Vec normalized(gp_Vec value, const gp_Vec& fallback = gp_Vec(0.0, 0.0, 1.0))
{
    if (value.SquareMagnitude() <= 1e-16)
        value = fallback;
    if (value.SquareMagnitude() <= 1e-16)
        value = gp_Vec(0.0, 0.0, 1.0);
    value.Normalize();
    return value;
}

gp_Vec normalOf(const lcnc::cam::RapidPose& pose)
{
    return normalized(gp_Vec(pose.surfaceNormalX,
                             pose.surfaceNormalY,
                             pose.surfaceNormalZ));
}

void appendPreview(QVector<lcnc::cam::RapidSurfacePreviewPoint>* points,
                   const gp_Pnt& point,
                   const gp_Vec& normal)
{
    if (!points)
        return;
    const gp_Vec unit = normalized(normal);
    if (!points->isEmpty()) {
        const auto& last = points->back();
        if (gp_Pnt(last.x, last.y, last.z).Distance(point) <= 1e-7)
            return;
    }
    points->append({point.X(), point.Y(), point.Z(),
                    unit.X(), unit.Y(), unit.Z()});
}

int curveDivisions(double length, double maximumStep)
{
    const double step = std::max(0.25, maximumStep);
    return std::clamp(static_cast<int>(std::ceil(length / step)),
                      kMinimumCurveDivisions, kMaximumCurveDivisions);
}

bool buildSphericalReference(const TopoDS_Shape& workpiece,
                             const lcnc::cam::RapidPose& source,
                             const lcnc::cam::RapidPose& target,
                             double maximumStep,
                             QVector<lcnc::cam::RapidSurfacePreviewPoint>* points)
{
    if (workpiece.IsNull() || !points)
        return false;

    try {
        const gp_Pnt sourcePoint = pointOf(source);
        const gp_Pnt targetPoint = pointOf(target);
        for (TopExp_Explorer explorer(workpiece, TopAbs_FACE);
             explorer.More(); explorer.Next()) {
            BRepAdaptor_Surface surface(TopoDS::Face(explorer.Current()), Standard_True);
            if (surface.GetType() != GeomAbs_Sphere)
                continue;

            const gp_Sphere sphere = surface.Sphere();
            const gp_Pnt center = sphere.Location();
            const double radius = sphere.Radius();
            gp_Vec from(center, sourcePoint);
            gp_Vec to(center, targetPoint);
            if (radius <= kEpsilon || from.Magnitude() <= kEpsilon
                || to.Magnitude() <= kEpsilon) {
                continue;
            }
            const double tolerance = std::max(0.25, radius * 1e-3);
            if (std::abs(from.Magnitude() - radius) > tolerance
                || std::abs(to.Magnitude() - radius) > tolerance) {
                continue;
            }

            from.Normalize();
            to.Normalize();
            const double cosine = std::clamp(from.Dot(to), -1.0, 1.0);
            const double angle = std::acos(cosine);
            if (angle <= kEpsilon) {
                appendPreview(points, sourcePoint, from);
                appendPreview(points, targetPoint, to);
                return !points->isEmpty();
            }

            gp_Vec perpendicular = to - from * cosine;
            if (perpendicular.SquareMagnitude() <= 1e-16) {
                gp_Vec reference = std::abs(from.Z()) < 0.9
                    ? gp_Vec(0.0, 0.0, 1.0) : gp_Vec(1.0, 0.0, 0.0);
                perpendicular = reference - from * reference.Dot(from);
            }
            perpendicular = normalized(perpendicular);
            const int divisions = curveDivisions(radius * angle, maximumStep);
            points->reserve(divisions + 1);
            for (int index = 0; index <= divisions; ++index) {
                const double t = static_cast<double>(index) / divisions;
                gp_Vec radial = from * std::cos(angle * t)
                    + perpendicular * std::sin(angle * t);
                radial = normalized(radial, from);
                appendPreview(points, center.Translated(radial * radius), radial);
            }
            return points->size() >= 2;
        }
    } catch (const Standard_Failure&) {
        points->clear();
    }
    return false;
}

QVector<lcnc::cam::RapidSurfacePreviewPoint> buildBezierReference(
    const lcnc::cam::RapidPose& source,
    const lcnc::cam::RapidPose& target,
    const TravelPlanningRequest& request)
{
    QVector<lcnc::cam::RapidSurfacePreviewPoint> result;
    const gp_Pnt p0 = pointOf(source);
    const gp_Pnt p3 = pointOf(target);
    const double chord = p0.Distance(p3);
    const gp_Vec n0 = normalOf(source);
    const gp_Vec n3 = normalOf(target);

    // This is a geometric shaping clearance, not a tool-height correction.
    // Tool cutting/idle offsets remain Process concerns.  The modest lift keeps
    // the reference curve on the machining side of a gently varying surface
    // without reproducing every tessellation ripple.
    const double shapingClearance = std::max(
        0.25, request.minimumClearanceMm + request.proxySafetyRadiusMm);
    const double lift = shapingClearance + std::min(10.0, chord * 0.08);
    const gp_Pnt p1 = p0.Translated(n0 * lift);
    const gp_Pnt p2 = p3.Translated(n3 * lift);
    const int divisions = curveDivisions(std::max(chord, 2.0 * lift),
                                         request.surfacePathStepMm);
    result.reserve(divisions + 1);
    for (int index = 0; index <= divisions; ++index) {
        const double t = static_cast<double>(index) / divisions;
        const double u = 1.0 - t;
        const double b0 = u * u * u;
        const double b1 = 3.0 * u * u * t;
        const double b2 = 3.0 * u * t * t;
        const double b3 = t * t * t;
        const gp_Pnt point(
            b0 * p0.X() + b1 * p1.X() + b2 * p2.X() + b3 * p3.X(),
            b0 * p0.Y() + b1 * p1.Y() + b2 * p2.Y() + b3 * p3.Y(),
            b0 * p0.Z() + b1 * p1.Z() + b2 * p2.Z() + b3 * p3.Z());
        appendPreview(&result, point, normalized(n0 * u + n3 * t, n0));
    }
    if (result.size() == 1)
        appendPreview(&result, p3, n3);
    return result;
}

QVector<lcnc::cam::RapidSurfacePreviewPoint> buildReferenceCurve(
    const TravelEndpoint& source,
    const TravelEndpoint& target,
    const TravelPlanningRequest& request)
{
    QVector<lcnc::cam::RapidSurfacePreviewPoint> result;
    if (buildSphericalReference(request.workpiece, source.pose, target.pose,
                                request.surfacePathStepMm, &result)) {
        return result;
    }
    return buildBezierReference(source.pose, target.pose, request);
}

QVector<gp_Pnt> workpieceEnvelopePoints(const TopoDS_Shape& workpiece,
                                        double deflection)
{
    QVector<gp_Pnt> result;
    if (workpiece.IsNull())
        return result;
    try {
        // BRepMesh writes Poly_Triangulation back into faces.  The request
        // commonly borrows the XCAF workpiece used by the live AIS display;
        // meshing it here would replace its display mesh with this deliberately
        // coarse collision envelope and turn circular features into polygons.
        // Work on a topology/geometry-private copy and never mutate caller BRep.
        // 中文翻译：碰撞包络网格只能写入私有副本，不能污染实时工件显示网格。
        BRepBuilderAPI_Copy copy;
        copy.Perform(workpiece, Standard_True, Standard_False);
        if (!copy.IsDone())
            return result;
        const TopoDS_Shape collisionCopy = copy.Shape();
        BRepMesh_IncrementalMesh mesh(collisionCopy, std::max(0.25, deflection),
                                      Standard_False, 0.5, Standard_True);
        for (TopExp_Explorer explorer(collisionCopy, TopAbs_FACE); explorer.More(); explorer.Next()) {
            TopLoc_Location location;
            const Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(
                TopoDS::Face(explorer.Current()), location);
            if (triangulation.IsNull())
                continue;
            const gp_Trsf transform = location.Transformation();
            for (int node = 1; node <= triangulation->NbNodes(); ++node)
                result.append(triangulation->Node(node).Transformed(transform));
        }
        if (result.isEmpty()) {
            for (TopExp_Explorer explorer(collisionCopy, TopAbs_VERTEX); explorer.More(); explorer.Next())
                result.append(BRep_Tool::Pnt(TopoDS::Vertex(explorer.Current())));
        }
    } catch (const Standard_Failure&) {
        result.clear();
    }
    return result;
}

double proxyEnvelopeRadius(const TopoDS_Shape& proxy)
{
    if (proxy.IsNull())
        return 0.0;
    try {
        Bnd_Box bounds;
        BRepBndLib::Add(proxy, bounds);
        if (bounds.IsVoid())
            return 0.0;
        double xMin, yMin, zMin, xMax, yMax, zMax;
        bounds.Get(xMin, yMin, zMin, xMax, yMax, zMax);
        return std::max({std::abs(xMin), std::abs(xMax),
                         std::abs(yMin), std::abs(yMax)});
    } catch (const Standard_Failure&) {
        return 0.0;
    }
}

lcnc::cam::RapidSurfacePreviewPoint offsetSample(
    const lcnc::cam::RapidSurfacePreviewPoint& sample, double offset)
{
    const gp_Vec normal = normalized(gp_Vec(sample.normalX, sample.normalY,
                                            sample.normalZ));
    auto result = sample;
    result.x += normal.X() * offset;
    result.y += normal.Y() * offset;
    result.z += normal.Z() * offset;
    return result;
}

double collisionEnvelopeHeight(
    const QVector<gp_Pnt>& workpiecePoints,
    const QVector<lcnc::cam::RapidSurfacePreviewPoint>& route,
    double proxyRadius,
    double clearance)
{
    double highest = std::max(0.0, clearance);
    const double corridorRadius = std::max(0.1, proxyRadius + clearance);
    const double corridorRadius2 = corridorRadius * corridorRadius;
    for (const auto& sample : route) {
        const gp_Pnt routePoint(sample.x, sample.y, sample.z);
        const gp_Vec normal = normalized(gp_Vec(sample.normalX, sample.normalY,
                                                sample.normalZ));
        for (const gp_Pnt& obstaclePoint : workpiecePoints) {
            const gp_Vec delta(routePoint, obstaclePoint);
            const double axial = delta.Dot(normal);
            if (axial <= 0.0)
                continue;
            const gp_Vec radial = delta - normal * axial;
            if (radial.SquareMagnitude() <= corridorRadius2)
                highest = std::max(highest, axial + clearance);
        }
    }
    return highest;
}

std::uint8_t changedMask(const lcnc::cam::RapidPose& source,
                         const lcnc::cam::RapidPose& target)
{
    std::uint8_t mask = 0;
    for (int axis = 0; axis < lcnc::MachineAxisLayout::kMaxAxes; ++axis) {
        if (std::abs(source.axes[axis] - target.axes[axis]) > 1e-9)
            mask |= static_cast<std::uint8_t>(1u << axis);
    }
    return mask;
}

double axisTimeMs(double distance, double velocity, double acceleration)
{
    if (distance <= kEpsilon)
        return 0.0;
    velocity = std::max(1e-6, velocity);
    acceleration = std::max(1e-6, acceleration);
    const double accelerationDistance = velocity * velocity / acceleration;
    if (distance <= accelerationDistance)
        return 2000.0 * std::sqrt(distance / acceleration);
    return 1000.0 * (2.0 * velocity / acceleration
        + (distance - accelerationDistance) / velocity);
}

double segmentTimeMs(const lcnc::cam::RapidPose& source,
                     const lcnc::cam::RapidPose& target,
                     const lcnc::cam::RapidMotionProfile& profile)
{
    double result = 0.0;
    for (int axis = 0; axis < lcnc::MachineAxisLayout::kMaxAxes; ++axis) {
        result = std::max(result, axisTimeMs(
            std::abs(target.axes[axis] - source.axes[axis]),
            profile.velocity[axis], profile.acceleration[axis]));
    }
    return result;
}

lcnc::cam::RapidPose interpolatedPose(
    const lcnc::cam::RapidPose& source,
    const lcnc::cam::RapidPose& target,
    const lcnc::cam::RapidSurfacePreviewPoint& point,
    double t)
{
    lcnc::cam::RapidPose pose = source;
    for (int axis = 0; axis < lcnc::MachineAxisLayout::kMaxAxes; ++axis) {
        pose.axes[axis] = source.axes[axis]
            + (target.axes[axis] - source.axes[axis]) * t;
        pose.kinematicAxes[axis] = source.kinematicAxes[axis]
            + (target.kinematicAxes[axis] - source.kinematicAxes[axis]) * t;
    }
    pose.activeMask = source.activeMask | target.activeMask;
    pose.kinematicAxisMask = source.kinematicAxisMask | target.kinematicAxisMask;
    pose.rotaryAxis1Name = source.rotaryAxis1Name.isEmpty()
        ? target.rotaryAxis1Name : source.rotaryAxis1Name;
    pose.rotaryAxis2Name = source.rotaryAxis2Name.isEmpty()
        ? target.rotaryAxis2Name : source.rotaryAxis2Name;
    pose.tcpX = point.x;
    pose.tcpY = point.y;
    pose.tcpZ = point.z;
    pose.surfaceNormalX = point.normalX;
    pose.surfaceNormalY = point.normalY;
    pose.surfaceNormalZ = point.normalZ;
    return pose;
}

} // namespace

lcnc::cam::TravelPlanSnapshot TravelPathPlanner::plan(
    const TravelPlanningRequest& request)
{
    if (request.diagnostics)
        *request.diagnostics = {};

    lcnc::cam::TravelPlanSnapshot snapshot;
    snapshot.mode = request.fullEnvironment
        ? lcnc::cam::TravelPlanningMode::FullEnvironment
        : lcnc::cam::TravelPlanningMode::WorkpieceProxy;
    snapshot.minimumClearanceMm = std::numeric_limits<double>::infinity();

    if (request.workpiece.IsNull() || request.cutterCollisionProxy.IsNull()) {
        snapshot.failureReason = QStringLiteral(
            "Surface rapid planning requires workpiece and cutter collision geometry");
        snapshot.stale = false;
        return snapshot;
    }

    QVector<TravelEndpointPair> pairs = request.transitions;
    if (pairs.isEmpty() && request.endpoints.size() >= 2) {
        pairs.reserve(request.endpoints.size() - 1);
        for (int index = 1; index < request.endpoints.size(); ++index)
            pairs.append({request.endpoints.at(index - 1), request.endpoints.at(index)});
    }
    if (pairs.isEmpty()) {
        snapshot.failureReason = QStringLiteral(
            "Surface rapid planning requires at least two contour endpoints");
        snapshot.stale = false;
        return snapshot;
    }

    const QVector<gp_Pnt> envelopePoints = workpieceEnvelopePoints(
        request.workpiece, request.collisionSampleStepMm);
    if (envelopePoints.isEmpty()) {
        snapshot.failureReason = QStringLiteral(
            "Unable to build the workpiece collision envelope");
        snapshot.stale = false;
        return snapshot;
    }
    const double collisionRadius = proxyEnvelopeRadius(request.cutterCollisionProxy);

    for (const TravelEndpointPair& pair : pairs) {
        lcnc::cam::RapidTransition transition;
        transition.fromContourId = pair.source.contourId;
        transition.toContourId = pair.target.contourId;
        transition.pathKind = lcnc::cam::RapidPathKind::SurfaceOffset;
        transition.surfacePreviewPoints = buildReferenceCurve(
            pair.source, pair.target, request);
        if (transition.surfacePreviewPoints.size() < 2) {
            transition.failureReason = QStringLiteral(
                "Unable to build a surface reference curve between contours");
            snapshot.transitions.append(transition);
            snapshot.failureReason = transition.failureReason;
            continue;
        }

        if (request.diagnostics) {
            ++request.diagnostics->candidateCount;
            ++request.diagnostics->evaluatedCandidateCount;
        }

        lcnc::cam::RapidPose previous = pair.source.pose;
        const int last = transition.surfacePreviewPoints.size() - 1;
        QVector<lcnc::cam::RapidSurfacePreviewPoint> envelopeRoute;
        const int routeStride = std::max(1, static_cast<int>(std::ceil(
            std::max(request.collisionSampleStepMm, request.surfacePathStepMm)
            / std::max(0.25, request.surfacePathStepMm))));
        for (int index = 0; index <= last; index += routeStride)
            envelopeRoute.append(transition.surfacePreviewPoints.at(index));
        if (envelopeRoute.back().x != transition.surfacePreviewPoints.back().x
            || envelopeRoute.back().y != transition.surfacePreviewPoints.back().y
            || envelopeRoute.back().z != transition.surfacePreviewPoints.back().z) {
            envelopeRoute.append(transition.surfacePreviewPoints.back());
        }
        const double safetyHeight = collisionEnvelopeHeight(
            envelopePoints, envelopeRoute, collisionRadius,
            request.minimumClearanceMm);
        if (safetyHeight > request.maximumSafetyOffsetMm + kEpsilon) {
            transition.failureReason = QStringLiteral(
                "The workpiece collision envelope exceeds the configured maximum rapid safety offset");
            snapshot.transitions.append(transition);
            if (snapshot.failureReason.isEmpty())
                snapshot.failureReason = transition.failureReason;
            continue;
        }

        struct ExecutionSample {
            lcnc::cam::RapidSurfacePreviewPoint point;
            double progress{0.0};
        };
        QVector<ExecutionSample> executionSamples;
        if (safetyHeight > kEpsilon) {
            // One normal ascent, a small number of raised curve samples, then
            // one normal descent.  No exact solid-distance loop is involved.
            executionSamples.append({offsetSample(
                transition.surfacePreviewPoints.front(), safetyHeight), 0.0});
            for (int index = routeStride; index < last; index += routeStride) {
                executionSamples.append({offsetSample(
                    transition.surfacePreviewPoints.at(index), safetyHeight),
                    static_cast<double>(index) / last});
            }
            executionSamples.append({offsetSample(
                transition.surfacePreviewPoints.back(), safetyHeight), 1.0});
            executionSamples.append({transition.surfacePreviewPoints.back(), 1.0});
        } else {
            for (int index = 1; index <= last; ++index) {
                executionSamples.append({transition.surfacePreviewPoints.at(index),
                    static_cast<double>(index) / last});
            }
        }

        for (const ExecutionSample& sample : std::as_const(executionSamples)) {
            const double t = sample.progress;
            lcnc::cam::RapidPose target = interpolatedPose(
                pair.source.pose, pair.target.pose,
                sample.point, t);
            std::uint8_t mask = changedMask(previous, target);
            if (mask == 0 && pointOf(previous).Distance(pointOf(target)) > kEpsilon)
                mask = pair.source.pose.activeMask | pair.target.pose.activeMask;
            if (mask == 0)
                continue;
            if ((mask & request.motionProfile.supportedCoordinatedMask) != mask) {
                transition.failureReason = QStringLiteral(
                    "The controller does not support the coordinated axes required by the surface rapid curve");
                break;
            }
            const double time = segmentTimeMs(previous, target, request.motionProfile);
            const double length = pointOf(previous).Distance(pointOf(target));
            transition.segments.append({target, mask,
                                        lcnc::cam::RapidSynchronization::Coordinated,
                                        time, request.minimumClearanceMm});
            transition.estimatedTimeMs += time;
            transition.pathLengthMm += length;
            transition.maximumSurfaceOffsetMm = std::max(
                transition.maximumSurfaceOffsetMm, safetyHeight);
            previous = std::move(target);
        }

        if (!transition.failureReason.isEmpty() || transition.segments.isEmpty()) {
            if (transition.failureReason.isEmpty())
                transition.failureReason = QStringLiteral("The surface rapid curve contains no executable motion");
            if (snapshot.failureReason.isEmpty())
                snapshot.failureReason = transition.failureReason;
        } else {
            snapshot.totalEstimatedTimeMs += transition.estimatedTimeMs;
            snapshot.totalLengthMm += transition.pathLengthMm;
            transition.minimumClearanceMm = request.minimumClearanceMm;
            snapshot.minimumClearanceMm = request.minimumClearanceMm;
        }
        snapshot.transitions.append(std::move(transition));
    }

    if (!std::isfinite(snapshot.minimumClearanceMm))
        snapshot.minimumClearanceMm = 0.0;
    snapshot.stale = false;
    return snapshot;
}

} // namespace lcnc::cam_algo
