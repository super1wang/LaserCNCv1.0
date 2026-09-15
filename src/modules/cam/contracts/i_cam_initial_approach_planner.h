#pragma once

#include "core/kernel/i_service.h"
#include "core/project/cam/collision_validation_contracts.h"
#include "core/project/cam/travel_plan_contracts.h"

#include <QMap>
#include <QString>

#include <atomic>
#include <cstdint>

namespace lcnc::cam {

enum class InitialApproachPlanningMode : std::uint8_t
{
    Manual = 0,
    Automatic
};

struct InitialApproachRequest
{
    std::uint64_t toolpathRevision{0};
    std::uint64_t targetContourId{0};
    QString machineConfigurationFingerprint;
    QMap<QString, double> axisPositions;
    InitialApproachPlanningMode planningMode{InitialApproachPlanningMode::Automatic};
    /// Signed absolute controller coordinate. Axis direction is comparison-only.
    double safetyAxisZ{0.0};
};

struct InitialApproachSnapshot
{
    std::uint64_t toolpathRevision{0};
    std::uint64_t targetContourId{0};
    QString machineConfigurationFingerprint;
    RapidTransition transition;
    CollisionVerificationMode verificationMode{
        CollisionVerificationMode::Disabled};
    CollisionValidationSnapshot collision;
    /// One immutable continuous certificate per initial-approach segment.
    /// Process must reject the plan if this vector is incomplete or any edge
    /// is not execution eligible.
    QVector<CamMotionEdgeCertificate> edgeCertificates;
    QString failureReason;

    bool isExecutable(bool blockWarning = true) const
    {
        if (!failureReason.isEmpty() || !transition.isValid())
            return false;
        if (verificationMode != CollisionVerificationMode::Required)
            return true;
        if (!collision.complete || collision.blocksExecution(blockWarning)
            || edgeCertificates.size() != transition.segments.size()) {
            return false;
        }
        for (int edge = 0; edge < edgeCertificates.size(); ++edge) {
            const auto& certificate = edgeCertificates.at(edge);
            if (!certificate.executionEligible()
                || certificate.firstNode != edge
                || certificate.lastNode != edge + 1
                || certificate.phase != CamMotionPhase::Rapid
                || (edge > 0
                    && (certificate.packageKeySha256
                            != edgeCertificates.constFirst().packageKeySha256
                        || certificate.environmentRevision
                            != edgeCertificates.constFirst().environmentRevision))) {
                return false;
            }
        }
        return true;
    }
};

class ICamInitialApproachPlanner : public lcnc::IService
{
public:
    ~ICamInitialApproachPlanner() override = default;
    virtual InitialApproachSnapshot planInitialApproach(
        const InitialApproachRequest& request,
        std::atomic_bool* cancelRequested = nullptr) const = 0;
};

} // namespace lcnc::cam
