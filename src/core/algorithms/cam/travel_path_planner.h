#pragma once

#include "core/project/cam/travel_plan_contracts.h"

#include <QVector>

#include <TopoDS_Shape.hxx>

namespace lcnc::cam_algo {

struct TravelEndpoint
{
    std::uint64_t contourId{0};
    lcnc::cam::RapidPose pose;
};

struct TravelEndpointPair
{
    TravelEndpoint source;
    TravelEndpoint target;
    /// Absolute offsets measured along the corresponding local surface normal.
    double rapidOffsetMm{5.0};
    double sourceCuttingOffsetMm{1.0};
    double targetCuttingOffsetMm{1.0};
};

struct TravelPlanningDiagnostics
{
    std::uint64_t candidateCount{0};
    std::uint64_t evaluatedCandidateCount{0};
    std::uint64_t exactDistanceCheckCount{0};
};

struct TravelPlanningRequest
{
    QVector<TravelEndpoint> endpoints;
    // Prefer explicit pairs when the end pose of one contour differs from the
    // lead-in pose used to enter that same contour.  The legacy endpoint chain
    // remains useful for compact tests and simple callers.
    QVector<TravelEndpointPair> transitions;
    TopoDS_Shape workpiece;
    /// Local collision proxy.  Its tip is at the origin and its longitudinal
    /// axis points along local +Z, away from the machining surface.
    TopoDS_Shape cutterCollisionProxy;
    /// Selects post-solve real-machine validation in the CAM caller.  Model
    /// visibility must not affect this value.
    bool fullEnvironment{false};
    lcnc::cam::RapidMotionProfile motionProfile;
    /// Legacy geometric shaping input. Per-transition rapid offsets are
    /// carried by TravelEndpointPair and become part of the solved CAM path.
    double proxySafetyRadiusMm{0.0};
    double minimumClearanceMm{0.0};
    /// Maximum additional normal offset the offline planner may search.
    double maximumSafetyOffsetMm{100.0};
    /// Collision checks use a coarser bounded sampling than the display/IK
    /// curve.  The greatest safe offset of adjacent checked samples is
    /// propagated across the interval.
    double collisionSampleStepMm{2.0};
    /// Maximum sample chord for the nominal reference curve.
    double surfacePathStepMm{1.0};
    TravelPlanningDiagnostics* diagnostics{nullptr};
};

/**
 * @brief Builds a bounded nominal curve on the machining side of the workpiece.
 *
 * The result carries geometry and pre-solve pose estimates only.  CAM performs
 * the authoritative continuous five-axis solve afterwards.  When a machine
 * model is loaded, CAM also validates the solved real head/environment motion;
 * that validation is deliberately separate from this lightweight generator.
 */
class TravelPathPlanner
{
public:
    static lcnc::cam::TravelPlanSnapshot plan(const TravelPlanningRequest& request);
};

} // namespace lcnc::cam_algo
