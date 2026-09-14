#pragma once

#include "core/project/cam/collision_validation_contracts.h"
#include "core/project/cam/travel_plan_contracts.h"

#include <QReadWriteLock>
#include <QString>
#include <QVector>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace lcnc::cam_algo {

/// One collision-source pair at one fully solved physical machine pose.
/// The cache deliberately keys authoritative physical axes and TCP/normal:
/// semantic Process axis order is ignored, while two IK branches that share a
/// TCP remain distinct because their physical axes differ.
struct CollisionSafetyDomainQuery
{
    lcnc::cam::RapidPose pose;
    QString workpieceEntry;
    lcnc::cam::CamMotionPhase phase{lcnc::cam::CamMotionPhase::Rapid};
    QString activeSource;
    QString passiveSource;
    double clearanceMm{0.0};
};

struct CollisionSafetyDomainSample
{
    lcnc::cam::CollisionValidationState state{
        lcnc::cam::CollisionValidationState::Indeterminate};
    QString activeEntity;
    QString passiveEntity;
    double minimumDistanceMm{-1.0};
    QString reason;

    bool isReusable() const
    {
        return state == lcnc::cam::CollisionValidationState::Safe
            || state == lcnc::cam::CollisionValidationState::Warning
            || state == lcnc::cam::CollisionValidationState::Collision;
    }
};

struct CollisionSafetyDomainStatistics
{
    std::uint64_t hits{0};
    std::uint64_t misses{0};
    std::uint64_t stores{0};
    std::size_t samples{0};
};

/// Thread-safe sparse safety-domain certificate cache.
///
/// Each instance belongs to one immutable collision-scene revision. Entries
/// are exact solved-pose certificates, not optimistic interpolation cells.
/// Therefore a cached Safe result can be shared by toolpath, rapid and initial
/// approach validation without weakening the exact collision contract.
class CollisionSafetyDomainCache final
{
public:
    explicit CollisionSafetyDomainCache(std::size_t maximumSamples = 250'000);
    ~CollisionSafetyDomainCache();

    CollisionSafetyDomainCache(const CollisionSafetyDomainCache&) = delete;
    CollisionSafetyDomainCache& operator=(const CollisionSafetyDomainCache&) = delete;

    bool lookup(const CollisionSafetyDomainQuery& query,
                CollisionSafetyDomainSample* sample) const;
    void store(const CollisionSafetyDomainQuery& query,
               const CollisionSafetyDomainSample& sample);
    void clear();
    CollisionSafetyDomainStatistics statistics() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace lcnc::cam_algo
