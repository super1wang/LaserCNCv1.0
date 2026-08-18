#pragma once

#include "core/kernel/i_service.h"
#include "core/project/cam/collision_validation_contracts.h"
#include "core/project/cam/travel_plan_contracts.h"

#include <QMap>
#include <QString>

#include <atomic>
#include <cstdint>

namespace lcnc::cam {

struct InitialApproachRequest
{
    std::uint64_t toolpathRevision{0};
    std::uint64_t targetContourId{0};
    QString machineConfigurationFingerprint;
    QMap<QString, double> axisPositions;
};

struct InitialApproachSnapshot
{
    std::uint64_t toolpathRevision{0};
    std::uint64_t targetContourId{0};
    QString machineConfigurationFingerprint;
    RapidTransition transition;
    CollisionValidationSnapshot collision;
    QString failureReason;

    bool isExecutable(bool blockWarning = true) const
    {
        return failureReason.isEmpty() && transition.isValid()
            && !collision.blocksExecution(blockWarning);
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
