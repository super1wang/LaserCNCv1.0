#pragma once

#include "core/kernel/i_service.h"

#include <QSet>
#include <QString>
#include <QVector>

#include <cstdint>

namespace lcnc::cam {

enum class CollisionSemanticRole : std::uint8_t
{
    CuttingHead,
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
    bool enabled{false};
    bool valid{false};
    QString machineProfilePath;
    std::uint64_t revision{0};
    QVector<CollisionSourceDescriptor> sources;
    QSet<QString> activeSources;
    QSet<QString> passiveSources;
    int unassignedMachineBodyCount{0};
};

/// CAM-owned and OCC-free collision source configuration contract.
class ICamCollisionConfigurationProvider : public lcnc::IService
{
public:
    ~ICamCollisionConfigurationProvider() override = default;

    virtual CollisionConfigurationSnapshot collisionConfiguration() const = 0;
    virtual void setCollisionDetectionEnabled(bool enabled) = 0;
    virtual void setCollisionSources(const QSet<QString>& active,
                                     const QSet<QString>& passive) = 0;
};

} // namespace lcnc::cam
