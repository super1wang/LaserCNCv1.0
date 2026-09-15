#pragma once

#include <QByteArray>

#include <cstdint>

namespace lcnc::cam {
enum class CollisionVerificationMode : std::uint8_t;
}

namespace lcnc::cam_algo {

enum class CollisionQueryPurpose : std::uint8_t
{
    OfflinePackageBuild = 0,
    PlannedRapid,
    PlannedLeadIn,
    PlannedCutting,
    InitialApproach,
    FixedMotion,
    ContinuousJog,
    BackgroundApos,
    OcctAudit
};

struct CollisionErrorBudget
{
    double minimumClearanceMm{0.5};
    double maximumOnlineFallbackRatio{0.001};
    double minimumRandomDecisionCoverage{0.95};
    double minimumPlannedDecisionCoverage{0.99};
    double maximumQueryP99Us{10.0};
    double maximumIndexMiB{100.0};
    double maximumBuildMinutes{15.0};
    double maximumExactBuildRatio{0.10};
    std::uint64_t maximumFalseSafeCount{0};

    static CollisionErrorBudget production();
    QByteArray fingerprintSha256() const;
};

struct CollisionPolicyDecision
{
    bool requireMachinePackage{false};
    bool requireJobOverlay{false};
    bool requireMotionCertificate{false};
    bool allowCoalFallback{false};
    bool allowOcctExactFallback{false};
    bool failClosedOnUnknown{true};
};

class CollisionPolicyMatrix final
{
public:
    static CollisionPolicyDecision policy(CollisionQueryPurpose purpose,
                                          bool machineEnvironment);
};

/// Global collision detection is an explicit opt-in.  It may only be enabled
/// after a machine has been loaded and its immutable safety package is ready.
/// Offline creation of a candidate package does not affect a currently
/// disabled, non-collision machining workflow.
bool canActivateCollisionDetection(bool machineLoaded,
                                   bool machinePackageExecutionEligible,
                                   bool machinePackageBuildInProgress);

/// Automatic CAM regeneration may prepare collision geometry only when the
/// selected policy requests diagnostics or mandatory certification. Explicit
/// user-triggered validation is handled separately by its force path.
bool automaticCollisionWorkEnabled(
    lcnc::cam::CollisionVerificationMode mode);

} // namespace lcnc::cam_algo
