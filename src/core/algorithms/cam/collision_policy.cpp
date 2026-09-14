#include "core/algorithms/cam/collision_policy.h"

#include <QCryptographicHash>
#include <QDataStream>
#include <QIODevice>

namespace lcnc::cam_algo {

CollisionErrorBudget CollisionErrorBudget::production()
{
    return {};
}

QByteArray CollisionErrorBudget::fingerprintSha256() const
{
    QByteArray canonical;
    QDataStream stream(&canonical, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream.setFloatingPointPrecision(QDataStream::DoublePrecision);
    stream << quint32(1)
           << minimumClearanceMm
           << maximumOnlineFallbackRatio
           << minimumRandomDecisionCoverage
           << minimumPlannedDecisionCoverage
           << maximumQueryP99Us
           << maximumIndexMiB
           << maximumBuildMinutes
           << maximumExactBuildRatio
           << quint64(maximumFalseSafeCount);
    return QCryptographicHash::hash(canonical, QCryptographicHash::Sha256);
}

CollisionPolicyDecision CollisionPolicyMatrix::policy(
    CollisionQueryPurpose purpose,
    bool machineEnvironment)
{
    CollisionPolicyDecision result;
    result.requireMachinePackage = machineEnvironment;
    result.requireJobOverlay = purpose != CollisionQueryPurpose::OfflinePackageBuild
        && purpose != CollisionQueryPurpose::OcctAudit;
    result.allowCoalFallback = true;

    switch (purpose) {
    case CollisionQueryPurpose::OfflinePackageBuild:
    case CollisionQueryPurpose::OcctAudit:
        result.allowOcctExactFallback = true;
        break;
    case CollisionQueryPurpose::PlannedRapid:
    case CollisionQueryPurpose::PlannedLeadIn:
    case CollisionQueryPurpose::PlannedCutting:
    case CollisionQueryPurpose::InitialApproach:
        result.requireMotionCertificate = true;
        // These are CAM/offline planning requests. OCCT is allowed only while
        // producing the immutable certificate, never from Process execution.
        result.allowOcctExactFallback = true;
        break;
    case CollisionQueryPurpose::FixedMotion:
    case CollisionQueryPurpose::ContinuousJog:
    case CollisionQueryPurpose::BackgroundApos:
        result.requireMotionCertificate = true;
        result.allowOcctExactFallback = false;
        break;
    }
    return result;
}

bool canActivateCollisionDetection(bool machineLoaded,
                                   bool machinePackageExecutionEligible,
                                   bool machinePackageBuildInProgress)
{
    return machineLoaded && machinePackageExecutionEligible
        && !machinePackageBuildInProgress;
}

} // namespace lcnc::cam_algo
