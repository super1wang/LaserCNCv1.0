#pragma once

#include "core/kernel/i_service.h"
#include "core/project/cam/collision_validation_contracts.h"

#include <QSet>
#include <QString>
#include <QVector>

#include <cstdint>

namespace lcnc::cam {

enum class CollisionSemanticRole : std::uint8_t
{
    MovingPart,
    StaticFrame,
    Workpiece
};

struct CollisionSourceDescriptor
{
    QString id;             // cutter, workpiece, axis:<UPPERCASE_NAME>
    QString displayName;
    CollisionSemanticRole semanticRole{CollisionSemanticRole::MovingPart};
    bool activeCandidate{false};
    bool passiveCandidate{false};
    bool available{false};
    int bodyCount{0};
};

struct CollisionConfigurationSnapshot
{
    CollisionVerificationMode verificationMode{CollisionVerificationMode::Disabled};
    /// Compatibility projection for existing preview/UI consumers.
    bool enabled{false};
    bool valid{false};
    bool activationAvailable{false};
    QString activationFailureReason;
    QString machineProfilePath;
    std::uint64_t revision{0};
    QVector<CollisionSourceDescriptor> sources;
    QSet<QString> activeSources;
    QSet<QString> passiveSources;
    int unassignedMachineBodyCount{0};
    /// Source roles are derived from the immutable machine assembly and the
    /// workpiece mount chain. They are informative, not operator-editable.
    bool sourceSelectionMutable{false};
};

/// CAM-owned and OCC-free collision source configuration contract.
class ICamCollisionConfigurationProvider : public lcnc::IService
{
public:
    ~ICamCollisionConfigurationProvider() override = default;

    virtual CollisionConfigurationSnapshot collisionConfiguration() const = 0;
    virtual bool setCollisionDetectionEnabled(bool enabled,
                                              QString* errorMessage = nullptr) = 0;
    virtual void setCollisionSources(const QSet<QString>& active,
                                     const QSet<QString>& passive) = 0;
};

} // namespace lcnc::cam
