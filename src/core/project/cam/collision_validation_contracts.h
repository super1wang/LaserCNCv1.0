#pragma once

#include "core/kinematics/machine_topology.h"

#include <QString>
#include <QStringList>
#include <QVector>
#include <QByteArray>

#include <array>
#include <cstdint>

namespace lcnc::cam {

/// Result states are ordered by safety severity.  They intentionally contain
/// no OCC data so the exact same result can be consumed by CAM, Process and
/// the offline sandbox.
enum class CollisionValidationState : std::uint8_t
{
    Disabled = 0,
    Pending,
    Safe,
    Warning,
    Collision,
    Indeterminate
};

enum class CamMotionPhase : std::uint8_t
{
    Rapid,
    LeadIn,
    Cutting
};

/// Physical degrees of freedom used by a finalized CAM motion block.  RTCP is
/// deliberately not a motion class; it is a controller lowering policy.
enum class MotionClass : std::uint8_t
{
    SingleAxis = 0,
    Coordinated2D,
    Coordinated3D,
    Reduced4D,
    Full5D
};

enum class ControllerMotionMode : std::uint8_t
{
    PhysicalAxes = 0,
    RTCP
};

enum class ControllerQualificationState : std::uint8_t
{
    Unavailable = 0,
    Unqualified,
    Qualified
};

/// Requested mode is not an effective/qualified execution mode. Revision 0
/// explicitly means that no controller qualification authority is available.
struct ControllerQualificationSnapshot
{
    ControllerMotionMode requestedMode{ControllerMotionMode::PhysicalAxes};
    ControllerQualificationState state{ControllerQualificationState::Unavailable};
    std::uint64_t qualificationRevision{0};
    QByteArray capabilityFingerprint;
    QString sourceId;
};

inline bool controllerQualificationIsQualified(
    const ControllerQualificationSnapshot& snapshot)
{
    return snapshot.state == ControllerQualificationState::Qualified
        && snapshot.qualificationRevision != 0
        && !snapshot.capabilityFingerprint.isEmpty()
        && !snapshot.sourceId.isEmpty();
}

enum class CollisionVerificationMode : std::uint8_t
{
    Disabled = 0,
    Optional,
    Required
};

enum class MotionInterpolationKind : std::uint8_t
{
    PhysicalAxisLine = 0,
    RtcpLine
};

enum class MotionOptimizationState : std::uint8_t
{
    Raw = 0,
    Optimized,
    Reduced
};

/// A continuous certificate applies to the complete interpolation interval
/// between two adjacent canonical motion nodes.  It is intentionally an
/// OCC-free value object so Process can validate the frozen CAM result without
/// invoking geometry code online.
enum class CamMotionCertificateState : std::uint8_t
{
    Invalid = 0,
    Disabled,
    CertifiedSafe,
    Blocked,
    BoundaryUnknown
};

inline CollisionValidationState collisionValidationStateForCertificate(
    CamMotionCertificateState state)
{
    switch (state) {
    case CamMotionCertificateState::Disabled:
    case CamMotionCertificateState::CertifiedSafe:
        return CollisionValidationState::Safe;
    case CamMotionCertificateState::Blocked:
        return CollisionValidationState::Collision;
    case CamMotionCertificateState::Invalid:
    case CamMotionCertificateState::BoundaryUnknown:
    default:
        return CollisionValidationState::Indeterminate;
    }
}

struct CamMotionEdgeCertificate
{
    std::uint64_t edgeId{0};
    int firstNode{-1};
    int lastNode{-1};
    CamMotionPhase phase{CamMotionPhase::Rapid};
    CamMotionCertificateState state{CamMotionCertificateState::Invalid};
    QByteArray packageKeySha256;
    std::uint64_t environmentRevision{0};
    std::uint64_t intervalQueries{0};
    std::uint64_t broadPhaseRejected{0};
    std::uint64_t localFieldQueries{0};
    std::uint64_t localFieldRejected{0};
    std::uint64_t surfaceBvhQueries{0};
    std::uint64_t surfaceBvhRejected{0};
    std::uint64_t coalPairQueries{0};
    std::uint64_t occtExactQueries{0};
    std::uint64_t surfaceBvhQueryNs{0};
    std::uint64_t localFieldQueryNs{0};
    std::uint64_t coalQueryNs{0};
    std::uint64_t occtExactQueryNs{0};
    std::uint64_t occtExactLockWaitNs{0};
    std::uint64_t occtExactMaximumQueryNs{0};
    QString occtExactSlowestPair;
    int maximumSubdivisionDepth{0};
    bool budgetExhausted{false};
    QString fallbackSourcePair;
    QString reason;

    bool executionEligible() const
    {
        return state == CamMotionCertificateState::CertifiedSafe
            || state == CamMotionCertificateState::Disabled;
    }
};

enum class RapidSegmentPhase : std::uint8_t
{
    Retract = 0,
    Traverse,
    SafeXY,
    SafeAC,
    Approach
};

struct CollisionInterval
{
    int firstNode{-1};
    int lastNode{-1};
    CollisionValidationState state{CollisionValidationState::Pending};
    QString activeSource;
    QString passiveSource;
    QString activeEntity;
    QString passiveEntity;
    double minimumDistanceMm{-1.0};
    QString reason;
};

struct CollisionValidationSnapshot
{
    std::uint64_t key{0};
    CollisionValidationState state{CollisionValidationState::Pending};
    bool complete{false};
    bool blockWarning{true};
    QVector<CollisionValidationState> nodeStates;
    QVector<CollisionInterval> intervals;
    QString failureReason;

    bool blocksExecution(bool warningBlocksExecution = true) const
    {
        return state == CollisionValidationState::Collision
            || state == CollisionValidationState::Indeterminate
            || (warningBlocksExecution && state == CollisionValidationState::Warning)
            || state == CollisionValidationState::Pending;
    }
};

/// Final physical-coordinate motion.  It is the only representation accepted
/// by Process and simulation; no consumer may add tool heights or re-solve it.
struct CamMotionNode
{
    CamMotionPhase phase{CamMotionPhase::Cutting};
    /// Meaningful only when phase == Rapid.  It preserves CAM's strict
    /// retract/traverse/approach split for every read-only consumer.
    RapidSegmentPhase rapidPhase{RapidSegmentPhase::Traverse};
    std::uint64_t contourId{0};
    int sourceEdgeIndex{-1};
    double sourceParameter{0.0};
    // Arrival uses sourceParameter; a seam may have a different outgoing owner.
    int departureSourceEdgeIndex{-1};
    double departureSourceParameter{0.0};
    bool semanticHardBarrier{false};
    std::array<double, MachineAxisLayout::kMaxAxes> axes{};
    std::uint8_t axisMask{0};
    double tcpX{0.0};
    double tcpY{0.0};
    double tcpZ{0.0};
    /// Table-zero/MCS reference TCP used only by qualified RTCP lowering.
    double referenceTcpX{0.0};
    double referenceTcpY{0.0};
    double referenceTcpZ{0.0};
    bool referenceTcpValid{false};
    double normalX{0.0};
    double normalY{0.0};
    double normalZ{1.0};
    double estimatedTimeMs{0.0};
};

struct MotionSourceSpan
{
    std::uint64_t contourId{0};
    /// -1 denotes the explicit entry boundary preceding physical knot 0.
    int firstKnot{0};
    int lastKnot{0};
    double firstSourceParameter{0.0};
    double lastSourceParameter{1.0};
    int sourceEdgeIndex{-1}; ///< Source wire-edge owner; -1 for rapid/unknown.
};

struct MotionProcessFence
{
    /// -1 applies before the incoming entry-boundary edge; non-negative
    /// values apply at physicalKnots[knotIndex].
    int knotIndex{0};
    bool blockStart{false};
    bool blockEnd{false};
    bool laserEnabledAfterFence{false};
};

struct MotionFeedSemantics
{
    double nominalFeedPerMinute{0.0};
    double estimatedDurationMs{0.0};
    QByteArray profileHash;
};

struct MotionToleranceProof
{
    double maximumPositionDeviationMm{0.0};
    double maximumOrientationDeviationDegrees{0.0};
    double maximumRotaryDeviationDegrees{0.0};
    bool exactKnots{true};
};

/// Immutable compilation identity captured before CAM publishes a plan.  All
/// byte arrays are stable content hashes, never mutable service identities.
struct MotionCompilationContext
{
    std::uint64_t workspaceGeneration{0};
    std::uint64_t sourceToolpathRevision{0};
    QByteArray contourOrderHash;
    QByteArray machineKinematicsHash;
    QByteArray setupCalibrationHash;
    QByteArray toolProcessHash;
    QByteArray optimizationPolicyHash;
    ControllerMotionMode controllerMode{ControllerMotionMode::PhysicalAxes};
    ControllerQualificationSnapshot controllerQualification;
    QByteArray controllerCapabilityHash;
    QByteArray dynamicsSemanticHash;
    CollisionVerificationMode collisionMode{CollisionVerificationMode::Disabled};
    std::uint32_t interpolationModelVersion{1};
};

/// One continuously evaluated block in the finalized plan.  physicalKnots
/// preserve unwrapped rotary coordinates in physical-layout order.  Legacy
/// CamMotionNode arrays are derived from these blocks after finalization.
struct CamMotionBlock
{
    std::uint64_t blockId{0};
    CamMotionPhase phase{CamMotionPhase::Cutting};
    std::uint64_t contourId{0};
    MotionClass motionClass{MotionClass::Full5D};
    MotionOptimizationState optimizationState{MotionOptimizationState::Raw};
    MotionInterpolationKind interpolation{MotionInterpolationKind::PhysicalAxisLine};
    std::uint8_t activeAxisMask{0};
    /// The predecessor of physicalKnots.front() when this block begins after
    /// another semantic block. It owns the incoming canonical edge without
    /// becoming a second execution node or changing this block's phase.
    bool hasEntryBoundary{false};
    CamMotionNode entryBoundary;
    QVector<CamMotionNode> physicalKnots;
    QVector<MotionSourceSpan> sourceSpans;
    QVector<MotionProcessFence> fences;
    MotionFeedSemantics feed;
    MotionToleranceProof toleranceProof;
    QByteArray blockHash;
};

struct MotionOptimizerReport
{
    int knotsBefore{0};
    int knotsAfter{0};
    int blocksBefore{0};
    int blocksAfter{0};
    std::array<double, MachineAxisLayout::kMaxAxes> axisSpans{};
    double rotaryTravelDegrees{0.0};
    int rotaryReversals{0};
    double maximumPositionDeviationMm{0.0};
    double maximumOrientationDeviationDegrees{0.0};
    MotionClass selectedClass{MotionClass::Full5D};
    ControllerMotionMode selectedControllerMode{ControllerMotionMode::PhysicalAxes};
    QStringList rejectedCandidateReasons;
    std::uint64_t estimatedControllerCommands{0};
};

struct CamMotionPlanSnapshot
{
    std::uint64_t revision{0};
    MotionCompilationContext context;
    QVector<CamMotionBlock> blocks;
    QByteArray contextHash;
    QByteArray planHash;
    QString solverId;
    int solverVersion{0};
    QString failureReason;
    MotionOptimizerReport optimizerReport;
    /// Compatibility/display projection. It is replaced from blocks by
    /// finalizeMotionPlan() and must never be mutated back into blocks.
    QVector<CamMotionNode> nodes;
    QByteArray derivedFromPlanHash;
    QVector<CamMotionEdgeCertificate> edgeCertificates;
    CollisionValidationSnapshot collision;
};

QByteArray motionCompilationContextHash(const MotionCompilationContext& context);
QByteArray controllerQualificationSnapshotHash(
    const ControllerQualificationSnapshot& snapshot);
QByteArray motionBlockHash(const CamMotionBlock& block);
QByteArray finalMotionPlanHash(const CamMotionPlanSnapshot& plan);

/// Validates immutable block data, refreshes hashes and replaces the legacy
/// node projection atomically. Returns false without publishing a partial
/// identity when any execution value is invalid.
bool finalizeMotionPlan(CamMotionPlanSnapshot* plan, QString* errorMessage = nullptr);

bool finalMotionPlanIdentityIsCurrent(const CamMotionPlanSnapshot& plan);

} // namespace lcnc::cam
