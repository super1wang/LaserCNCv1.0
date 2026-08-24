#pragma once

#include "core/machine/machine_safety_package.h"

#include <QByteArray>
#include <QReadWriteLock>
#include <QString>

#include <memory>
#include <cstdint>

class QTemporaryDir;

namespace lcnc::cam_algo {
class MachineSafetyIndex;
}

namespace lcnc::cam {

enum class MachineSafetyPackageState : std::uint8_t
{
    Unavailable = 0,
    Loading,
    Building,
    ReadyCurrent,
    ReadySuperseded,
    StaleUnsafe,
    Invalid
};

struct MachineSafetyPackageStatus
{
    MachineSafetyPackageState state{MachineSafetyPackageState::Unavailable};
    QString packagePath;
    QByteArray packageKeySha256;
    QByteArray modelSha256;
    QByteArray runtimeConfigurationSha256;
    QString reason;
    bool buildInProgress{false};

    bool executionEligible() const
    {
        return !buildInProgress
            && (state == MachineSafetyPackageState::ReadyCurrent
                || state == MachineSafetyPackageState::ReadySuperseded);
    }
};

struct MachineSafetyPackageRuntimeSnapshot
{
    MachineSafetyPackageStatus status;
    std::shared_ptr<cam_algo::MachineSafetyIndex> index;
};

/// Owns the execution eligibility and immutable runtime lifetime of one .lmsp.
/// Resource parsing/writing remains in core::MachineSafetyPackage; this class
/// is the CAM state machine that prevents a half-built or stale package from
/// becoming an execution certificate.
class MachineSafetyPackageManager final
{
public:
    MachineSafetyPackageStatus status() const;
    MachineSafetyPackageRuntimeSnapshot runtimeSnapshot() const;

    void beginLoad(const QString& packagePath);
    void beginBuild(const QByteArray& sourceModelSha256,
                    const QByteArray& runtimeConfigurationSha256);
    void publish(const MachineSafetyPackageLoadResult& package,
                 std::shared_ptr<cam_algo::MachineSafetyIndex> index,
                 std::shared_ptr<QTemporaryDir> extraction);
    void finishBuildFailure(const QString& reason);
    void invalidate(const QString& reason);
    void markInvalid(const QString& reason);
    void clear();

private:
    mutable QReadWriteLock m_lock;
    MachineSafetyPackageStatus m_status;
    std::shared_ptr<cam_algo::MachineSafetyIndex> m_index;
    std::shared_ptr<QTemporaryDir> m_extraction;
};

QString machineSafetyPackageStateName(MachineSafetyPackageState state);

} // namespace lcnc::cam
