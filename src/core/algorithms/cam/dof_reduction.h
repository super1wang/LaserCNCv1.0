#pragma once

#include "core/algorithms/cam/full5d_optimizer.h"

namespace lcnc::cam_algo {

struct PhysicalAxisAnalysis
{
    double span{0}, maximumLocalDelta{0}, travel{0}, freezeResidual{0};
    double maximumVelocityProxy{0}, maximumAccelerationProxy{0};
    int reversals{0};
    bool monotonic{true};
};

struct ReductionPolicy
{
    PoseOptimizationMode mode{PoseOptimizationMode::Off};
    bool enableDofReduction{false};
    bool enableLaserZHold{false};
    int zAxis{-1};
    int maximumCandidates{32};
    // Numerical rounding budget only, never a process machining tolerance.
    static constexpr int numericalUlps = 64;
    static constexpr int costRevision = 2;
};

/// Supplied only by a qualification authority, bound to frozen context hashes.
/// The production adapter currently supplies the default unavailable value.
struct ReductionAdmission
{
    QByteArray controllerSnapshotHash;
    QByteArray dynamicsHash;
    bool exactPhysicalAxisLine{false};
    bool feedAndDynamicsRepresentable{false};
    std::array<bool, 32> supportedActiveMasks{};
};

struct NumericalZBound
{
    bool valid{false};
    double maximumPositionDeviationMm{0};
    double maximumOrientationDeviationDegrees{0};
};

struct ReductionEvaluation
{
    MotionEvaluationContext motion;
    ReductionAdmission admission;
    // Whole-block proof, including entryBoundary; samples alone are not proof.
    std::function<NumericalZBound(const lcnc::cam::CamMotionBlock&,
        const lcnc::cam::CamMotionBlock&, int)> boundNumericalZ;
    // Reserved process-authority input. No production whitelist is qualified
    // in S3; merely supplying values must never enable relaxed Z-hold.
    struct ProcessZEnvelope {
        QString sourceId;
        std::uint64_t revision{0};
        QString intersectionModel;
        double minimumFocusMm{0}, maximumFocusMm{0};
        double minimumStandoffMm{0}, maximumStandoffMm{0};
        double minimumPhysicalZ{0}, maximumPhysicalZ{0};
        double contourToleranceMm{0}, orientationToleranceDeg{0};
        double calibrationRunoutControlBoundMm{0};
        bool externalHeightFollowing{false};
    } processEnvelope;
};

struct ReductionBlockReport
{
    std::uint64_t blockId{0};
    std::array<PhysicalAxisAnalysis, MachineAxisLayout::kMaxAxes> axes{};
    std::uint8_t selectedMask{0};
    int candidates{0};
    bool numericalZApplied{false};
    QStringList rejectedReasons;
};

struct ReductionReport
{
    QVector<ReductionBlockReport> blocks;
    bool changed{false};
    qint64 compileTimeMs{0}; // Diagnostic only; excluded from identity and cost.
};

/// Optimized reference -> admitted selection. No collision queries or IK.
/// Cancellation/stale identity fails atomically; unsupported candidates retain
/// the explicit Optimized reference. Node axisMask remains the full pose layout;
/// block activeAxisMask is the qualified continuous hold commitment.
bool reduceMotionPlan(const lcnc::cam::CamMotionPlanSnapshot& reference,
    const Full5DPolicy& limits, const ReductionPolicy& policy,
    const std::function<ReductionEvaluation(const lcnc::cam::CamMotionBlock&)>& evaluation,
    lcnc::cam::CamMotionPlanSnapshot* selected, ReductionReport* report,
    QString* error = nullptr, const std::function<bool()>& cancelled = {});

} // namespace lcnc::cam_algo
