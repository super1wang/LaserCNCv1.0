#pragma once

#include "core/kernel/i_service.h"
#include "core/algorithms/cam/collision_policy.h"
#include "core/project/cam/collision_validation_contracts.h"
#include "core/project/cam/travel_plan_contracts.h"

#include <QString>
#include <QByteArray>
#include <QMap>
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
    cam_algo::CollisionQueryPurpose purpose{
        cam_algo::CollisionQueryPurpose::PlannedRapid};
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

enum class CamMotionPermitKind : std::uint8_t
{
    FixedMotion = 0,
    ContinuousJog
};

struct CamMotionPermitRequest
{
    CamMotionPermitKind kind{CamMotionPermitKind::FixedMotion};
    QMap<QString, double> firstApos;
    QMap<QString, double> lastApos;
    QString commandedAxis;
    int direction{0};
    double maximumDistance{0.0};
    qint64 validityMs{2000};
};

/// Dispatch-time authority for one immutable fixed motion, or one short jog
/// horizon. Continuous jog must renew before expiresUtcMs; stopping is always
/// permitted and needs no certificate.
struct CamMotionPermit
{
    CamMotionPermitKind kind{CamMotionPermitKind::FixedMotion};
    QByteArray tokenSha256;
    CamMotionCertificateState state{CamMotionCertificateState::Invalid};
    QByteArray packageKeySha256;
    std::uint64_t environmentRevision{0};
    qint64 issuedUtcMs{0};
    qint64 expiresUtcMs{0};
    QString commandedAxis;
    int direction{0};
    double maximumDistance{0.0};
    QMap<QString, double> firstApos;
    QMap<QString, double> lastApos;
    QString reason;

    bool executionEligible(qint64 nowUtcMs) const
    {
        return (state == CamMotionCertificateState::CertifiedSafe
                || state == CamMotionCertificateState::Disabled)
            && tokenSha256.size() == 32
            && nowUtcMs >= issuedUtcMs && nowUtcMs <= expiresUtcMs;
    }
};

class ICamCollisionSafetyDomain : public lcnc::IService
{
public:
    ~ICamCollisionSafetyDomain() override = default;

    virtual CollisionSafetyDomainSnapshot collisionSafetyDomain() const = 0;
    virtual CollisionValidationSnapshot validateCollisionPath(
        const CollisionSafetyPathRequest& request,
        std::atomic_bool* cancelRequested = nullptr) const = 0;
    virtual CamMotionPermit requestMotionPermit(
        const CamMotionPermitRequest& request) const = 0;
};

} // namespace lcnc::cam
