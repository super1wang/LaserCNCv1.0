#pragma once

#include <QReadWriteLock>
#include <QString>

#include <cstdint>
#include <memory>

namespace lcnc::cam {

struct TravelCollisionGeometryCache;

enum class JobSafetyOverlayState : std::uint8_t
{
    Unavailable = 0,
    Building,
    Ready,
    StaleUnsafe,
    Invalid
};

struct JobSafetyOverlayStatus
{
    JobSafetyOverlayState state{JobSafetyOverlayState::Unavailable};
    std::uint64_t environmentRevision{0};
    int geometryBodyCount{0};
    int workpieceBodyCount{0};
    bool buildInProgress{false};
    QString reason;

    bool executionEligible() const
    {
        return state == JobSafetyOverlayState::Ready && !buildInProgress;
    }
};

struct JobSafetyOverlayRuntimeSnapshot
{
    JobSafetyOverlayStatus status;
    std::shared_ptr<TravelCollisionGeometryCache> geometry;
};

/// Owns the immutable per-job machine/workpiece collision overlay. Any
/// mutation or incomplete rebuild removes execution eligibility immediately.
class JobSafetyOverlayManager final
{
public:
    JobSafetyOverlayStatus status() const;
    JobSafetyOverlayRuntimeSnapshot runtimeSnapshot() const;
    std::shared_ptr<TravelCollisionGeometryCache> geometry() const;

    void beginBuild(std::uint64_t environmentRevision);
    bool publish(std::uint64_t environmentRevision,
                 std::shared_ptr<TravelCollisionGeometryCache> geometry);
    void finishBuildFailure(std::uint64_t environmentRevision,
                            const QString& reason);
    void invalidate(const QString& reason);
    void clear();

private:
    mutable QReadWriteLock m_lock;
    JobSafetyOverlayStatus m_status;
    std::shared_ptr<TravelCollisionGeometryCache> m_geometry;
};

QString jobSafetyOverlayStateName(JobSafetyOverlayState state);

} // namespace lcnc::cam
