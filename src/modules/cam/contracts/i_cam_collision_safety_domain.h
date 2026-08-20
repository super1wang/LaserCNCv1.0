#pragma once

#include "core/kernel/i_service.h"
#include "core/project/cam/collision_validation_contracts.h"
#include "core/project/cam/travel_plan_contracts.h"

#include <QString>
#include <QVector>

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace lcnc::cam {

enum class CollisionSafetyScope : std::uint8_t
{
    FullEnvironment = 0,
    MachineOnly
};

struct CollisionSafetyPathPose
{
    RapidPose pose;
    QString workpieceEntry;
    CamMotionPhase phase{CamMotionPhase::Rapid};
};

/// OCC-free request shared by toolpath, inter-contour rapid and initial
/// approach planners. The collision module owns all geometry and exact calls.
struct CollisionSafetyPathRequest
{
    std::uint64_t environmentRevision{0};
    QVector<CollisionSafetyPathPose> poses;
    CollisionSafetyScope scope{CollisionSafetyScope::FullEnvironment};
    double clearanceMm{0.0};
    bool blockWarning{true};
};

struct CollisionSafetyDomainSnapshot
{
    std::uint64_t environmentRevision{0};
    bool ready{false};
    std::size_t geometryBodies{0};
    std::size_t certifiedSamples{0};
    std::uint64_t cacheHits{0};
    std::uint64_t cacheMisses{0};
};

class ICamCollisionSafetyDomain : public lcnc::IService
{
public:
    ~ICamCollisionSafetyDomain() override = default;

    virtual CollisionSafetyDomainSnapshot collisionSafetyDomain() const = 0;
    virtual CollisionValidationSnapshot validateCollisionPath(
        const CollisionSafetyPathRequest& request,
        std::atomic_bool* cancelRequested = nullptr) const = 0;
};

} // namespace lcnc::cam
