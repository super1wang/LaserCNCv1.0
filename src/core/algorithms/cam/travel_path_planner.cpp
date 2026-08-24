#include "core/algorithms/cam/travel_path_planner.h"

#include "core/logging/logger.h"

#include <BRepAdaptor_Surface.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <Standard_Failure.hxx>
#include <gp_Pnt.hxx>
#include <gp_Sphere.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>
#include <cstdint>
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
    } catch (const Standard_Failure& failure) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Unable to construct a rapid surface reference: {}",
                 failure.GetMessageString());
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

    // This is only the fallback reference-curve shaping distance.  CAM applies
    // the configured cutting/rapid normal offsets after constructing the
    // reference curve; Process never adds a height correction.
    // 中文翻译：这里只构造回退参考曲线，切割/空程法向偏置随后由 CAM 应用，Process 不再叠加高度。
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

bool hasFinitePose(const lcnc::cam::RapidPose& pose)
{
    for (int axis = 0; axis < lcnc::MachineAxisLayout::kMaxAxes; ++axis) {
        if (!std::isfinite(pose.axes[axis]) || !std::isfinite(pose.kinematicAxes[axis]))
            return false;
    }
    return std::isfinite(pose.tcpX) && std::isfinite(pose.tcpY)
        && std::isfinite(pose.tcpZ) && std::isfinite(pose.surfaceNormalX)
        && std::isfinite(pose.surfaceNormalY) && std::isfinite(pose.surfaceNormalZ);
}

bool hasValidPlanningInputs(const TravelPlanningRequest& request,
                            const QVector<TravelEndpointPair>& pairs)
{
    constexpr std::uint8_t kKnownAxes =
        static_cast<std::uint8_t>((1u << lcnc::MachineAxisLayout::kMaxAxes) - 1u);
    if (!std::isfinite(request.proxySafetyRadiusMm)
        || !std::isfinite(request.minimumClearanceMm)
        || !std::isfinite(request.maximumSafetyOffsetMm)
        || !std::isfinite(request.surfacePathStepMm)
        || request.minimumClearanceMm < 0.0
        || request.maximumSafetyOffsetMm < 0.0
        || request.surfacePathStepMm <= 0.0
        || request.motionProfile.supportedCoordinatedMask == 0
        || (request.motionProfile.supportedCoordinatedMask & ~kKnownAxes) != 0) {
        return false;
    }
    for (int axis = 0; axis < lcnc::MachineAxisLayout::kMaxAxes; ++axis) {
        if (!std::isfinite(request.motionProfile.velocity[axis])
            || !std::isfinite(request.motionProfile.acceleration[axis])
            || !std::isfinite(request.motionProfile.jerk[axis])) {
            return false;
        }
    }
    for (const TravelEndpointPair& pair : pairs) {
        if (!hasFinitePose(pair.source.pose) || !hasFinitePose(pair.target.pose)
            || !std::isfinite(pair.rapidOffsetMm)
            || !std::isfinite(pair.sourceCuttingOffsetMm)
            || !std::isfinite(pair.targetCuttingOffsetMm)
            || pair.rapidOffsetMm < 0.0
            || pair.rapidOffsetMm <= pair.sourceCuttingOffsetMm
            || pair.rapidOffsetMm <= pair.targetCuttingOffsetMm)
            return false;
    }
    return true;
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

    if (!hasValidPlanningInputs(request, pairs)) {
        snapshot.failureReason = QStringLiteral(
            "Surface rapid planning received invalid motion or geometric input");
        snapshot.stale = false;
        return snapshot;
    }

    for (const TravelEndpointPair& pair : pairs) {
        lcnc::cam::RapidTransition transition;
        transition.fromContourId = pair.source.contourId;
        transition.toContourId = pair.target.contourId;
        transition.pathKind = lcnc::cam::RapidPathKind::SurfaceOffset;
        TravelEndpoint baseSource = pair.source;
        TravelEndpoint baseTarget = pair.target;
        const gp_Vec sourceNormal = normalized(normalOf(pair.source.pose));
        const gp_Vec targetNormal = normalized(normalOf(pair.target.pose));
        baseSource.pose.tcpX -= sourceNormal.X() * pair.sourceCuttingOffsetMm;
        baseSource.pose.tcpY -= sourceNormal.Y() * pair.sourceCuttingOffsetMm;
        baseSource.pose.tcpZ -= sourceNormal.Z() * pair.sourceCuttingOffsetMm;
        baseTarget.pose.tcpX -= targetNormal.X() * pair.targetCuttingOffsetMm;
        baseTarget.pose.tcpY -= targetNormal.Y() * pair.targetCuttingOffsetMm;
        baseTarget.pose.tcpZ -= targetNormal.Z() * pair.targetCuttingOffsetMm;
        transition.surfacePreviewPoints = buildReferenceCurve(baseSource, baseTarget, request);
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

        struct ExecutionSample {
            lcnc::cam::RapidSurfacePreviewPoint point;
            double progress{0.0};
            lcnc::cam::RapidSegmentPhase phase{lcnc::cam::RapidSegmentPhase::Traverse};
        };
        QVector<ExecutionSample> executionSamples;
        // Strict three-phase geometry: one normal retract at the source,
        // surface-normal-offset traverse samples, and one normal approach to
        // the already offset cutting/lead-in target.
        executionSamples.append({offsetSample(transition.surfacePreviewPoints.front(),
                                               pair.rapidOffsetMm), 0.0,
                                 lcnc::cam::RapidSegmentPhase::Retract});
        for (int index = 1; index <= last; ++index) {
            executionSamples.append({offsetSample(transition.surfacePreviewPoints.at(index),
                                                   pair.rapidOffsetMm),
                                     static_cast<double>(index) / last,
                                     lcnc::cam::RapidSegmentPhase::Traverse});
        }
        executionSamples.append({
            {pair.target.pose.tcpX, pair.target.pose.tcpY, pair.target.pose.tcpZ,
             pair.target.pose.surfaceNormalX, pair.target.pose.surfaceNormalY,
             pair.target.pose.surfaceNormalZ},
            1.0, lcnc::cam::RapidSegmentPhase::Approach});

        QVector<lcnc::cam::RapidSurfacePreviewPoint> executablePreview;
        executablePreview.reserve(executionSamples.size() + 1);
        appendPreview(&executablePreview, pointOf(pair.source.pose), normalOf(pair.source.pose));
        for (const ExecutionSample& sample : std::as_const(executionSamples)) {
            appendPreview(&executablePreview,
                          gp_Pnt(sample.point.x, sample.point.y, sample.point.z),
                          gp_Vec(sample.point.normalX, sample.point.normalY,
                                 sample.point.normalZ));
        }
        transition.surfacePreviewPoints = std::move(executablePreview);

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
                                        time, request.minimumClearanceMm, sample.phase});
            transition.estimatedTimeMs += time;
            transition.pathLengthMm += length;
            transition.maximumSurfaceOffsetMm = std::max(
                transition.maximumSurfaceOffsetMm,
                std::abs(pair.rapidOffsetMm));
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
