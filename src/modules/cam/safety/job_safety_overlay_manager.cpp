#include "modules/cam/safety/job_safety_overlay_manager.h"

#include "modules/cam/collision/collision_geometry_cache.h"

#include <QReadLocker>
#include <QWriteLocker>

namespace lcnc::cam {

JobSafetyOverlayStatus JobSafetyOverlayManager::status() const
{
    QReadLocker locker(&m_lock);
    return m_status;
}

JobSafetyOverlayRuntimeSnapshot JobSafetyOverlayManager::runtimeSnapshot() const
{
    QReadLocker locker(&m_lock);
    JobSafetyOverlayRuntimeSnapshot result;
    result.status = m_status;
    if (m_status.executionEligible())
        result.geometry = m_geometry;
    return result;
}

std::shared_ptr<TravelCollisionGeometryCache> JobSafetyOverlayManager::geometry() const
{
    QReadLocker locker(&m_lock);
    return m_geometry;
}

void JobSafetyOverlayManager::beginBuild(std::uint64_t environmentRevision)
{
    QWriteLocker locker(&m_lock);
    m_geometry.reset();
    m_status = {};
    m_status.state = JobSafetyOverlayState::Building;
    m_status.environmentRevision = environmentRevision;
    m_status.buildInProgress = true;
}

bool JobSafetyOverlayManager::publish(
    std::uint64_t environmentRevision,
    std::shared_ptr<TravelCollisionGeometryCache> geometry)
{
    QWriteLocker locker(&m_lock);
    if (!geometry || geometry->bodies.isEmpty()
        || environmentRevision != m_status.environmentRevision) {
        return false;
    }
    m_geometry = std::move(geometry);
    m_status.state = JobSafetyOverlayState::Ready;
    m_status.buildInProgress = false;
    m_status.reason.clear();
    m_status.geometryBodyCount = m_geometry->bodies.size();
    m_status.workpieceBodyCount = 0;
    for (const auto& body : m_geometry->bodies) {
        if (body.workpiece)
            ++m_status.workpieceBodyCount;
    }
    return true;
}

void JobSafetyOverlayManager::finishBuildFailure(
    std::uint64_t environmentRevision,
    const QString& reason)
{
    QWriteLocker locker(&m_lock);
    if (environmentRevision != m_status.environmentRevision)
        return;
    m_geometry.reset();
    m_status.state = JobSafetyOverlayState::Invalid;
    m_status.buildInProgress = false;
    m_status.reason = reason;
}

void JobSafetyOverlayManager::invalidate(const QString& reason)
{
    QWriteLocker locker(&m_lock);
    m_geometry.reset();
    if (m_status.state == JobSafetyOverlayState::Unavailable)
        return;
    m_status.state = JobSafetyOverlayState::StaleUnsafe;
    m_status.buildInProgress = false;
    m_status.reason = reason;
}

void JobSafetyOverlayManager::clear()
{
    QWriteLocker locker(&m_lock);
    m_geometry.reset();
    m_status = {};
}

QString jobSafetyOverlayStateName(JobSafetyOverlayState state)
{
    switch (state) {
    case JobSafetyOverlayState::Unavailable: return QStringLiteral("Unavailable");
    case JobSafetyOverlayState::Building: return QStringLiteral("Building");
    case JobSafetyOverlayState::Ready: return QStringLiteral("Ready");
    case JobSafetyOverlayState::StaleUnsafe: return QStringLiteral("StaleUnsafe");
    case JobSafetyOverlayState::Invalid: return QStringLiteral("Invalid");
    }
    return QStringLiteral("Invalid");
}

} // namespace lcnc::cam
