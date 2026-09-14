#include "modules/cam/safety/machine_safety_package_manager.h"

#include "core/algorithms/cam/machine_safety_index.h"

#include <QReadLocker>
#include <QSet>
#include <QTemporaryDir>
#include <QWriteLocker>

#include <utility>

namespace lcnc::cam {
namespace {

bool coversRequiredMachinePairs(const cam_algo::MachineSafetyIndex& index)
{
    QSet<QString> axes;
    for (const auto& body : index.bodies())
        axes.insert(body.axisName.trimmed().toUpper());
    QSet<QString> pairs;
    for (const auto& pair : index.pairs()) {
        if (pair.firstBody >= index.bodies().size()
            || pair.secondBody >= index.bodies().size()) {
            return false;
        }
        QString first = index.bodies().at(pair.firstBody)
                            .axisName.trimmed().toUpper();
        QString second = index.bodies().at(pair.secondBody)
                             .axisName.trimmed().toUpper();
        if (first > second)
            std::swap(first, second);
        pairs.insert(first + QLatin1Char('|') + second);
    }
    const QVector<QPair<QString, QString>> required{
        {QStringLiteral("Z"), QStringLiteral("A")},
        {QStringLiteral("Z"), QStringLiteral("C")},
        {QStringLiteral("X"), QStringLiteral("A")},
        {QStringLiteral("X"), QStringLiteral("C")}};
    for (const auto& requiredPair : required) {
        if (!axes.contains(requiredPair.first)
            || !axes.contains(requiredPair.second)) {
            continue;
        }
        QString first = requiredPair.first;
        QString second = requiredPair.second;
        if (first > second)
            std::swap(first, second);
        if (!pairs.contains(first + QLatin1Char('|') + second))
            return false;
    }
    return true;
}

} // namespace

MachineSafetyPackageStatus MachineSafetyPackageManager::status() const
{
    QReadLocker locker(&m_lock);
    return m_status;
}

MachineSafetyPackageRuntimeSnapshot
MachineSafetyPackageManager::runtimeSnapshot() const
{
    QReadLocker locker(&m_lock);
    MachineSafetyPackageRuntimeSnapshot result;
    result.status = m_status;
    if (m_status.executionEligible())
        result.index = m_index;
    return result;
}

void MachineSafetyPackageManager::beginLoad(const QString& packagePath)
{
    QWriteLocker locker(&m_lock);
    m_index.reset();
    m_extraction.reset();
    m_status = {};
    m_status.state = MachineSafetyPackageState::Loading;
    m_status.packagePath = packagePath;
}

void MachineSafetyPackageManager::beginBuild(
    const QByteArray& sourceModelSha256,
    const QByteArray& runtimeConfigurationSha256)
{
    QWriteLocker locker(&m_lock);
    const bool compatibleCurrent = m_status.executionEligible() && m_index
        && m_status.modelSha256 == sourceModelSha256
        && m_status.runtimeConfigurationSha256 == runtimeConfigurationSha256;
    m_status.buildInProgress = true;
    m_status.reason.clear();
    if (compatibleCurrent) {
        m_status.state = MachineSafetyPackageState::ReadySuperseded;
        return;
    }
    m_index.reset();
    m_extraction.reset();
    m_status.packageKeySha256.clear();
    m_status.modelSha256 = sourceModelSha256;
    m_status.runtimeConfigurationSha256 = runtimeConfigurationSha256;
    m_status.state = MachineSafetyPackageState::Building;
}

void MachineSafetyPackageManager::publish(
    const MachineSafetyPackageLoadResult& package,
    std::shared_ptr<cam_algo::MachineSafetyIndex> index,
    std::shared_ptr<QTemporaryDir> extraction)
{
    QWriteLocker locker(&m_lock);
    if (!index || !index->isValid() || !coversRequiredMachinePairs(*index)
        || package.manifest.packageKeySha256.size() != 32
        || package.manifest.modelSha256.size() != 32
        || package.manifest.runtimeConfigurationSha256.size() != 32) {
        m_index.reset();
        m_extraction.reset();
        m_status.state = MachineSafetyPackageState::Invalid;
        m_status.buildInProgress = false;
        m_status.reason = QStringLiteral(
            "Incomplete machine safety package runtime or collision-pair coverage");
        return;
    }
    m_index = std::move(index);
    m_extraction = std::move(extraction);
    m_status = {};
    m_status.state = MachineSafetyPackageState::ReadyCurrent;
    m_status.packagePath = package.packagePath;
    m_status.packageKeySha256 = package.manifest.packageKeySha256;
    m_status.modelSha256 = package.manifest.modelSha256;
    m_status.runtimeConfigurationSha256 =
        package.manifest.runtimeConfigurationSha256;
}

void MachineSafetyPackageManager::finishBuildFailure(const QString& reason)
{
    QWriteLocker locker(&m_lock);
    m_status.buildInProgress = false;
    m_status.reason = reason;
    if (m_status.state == MachineSafetyPackageState::ReadySuperseded) {
        m_status.state = MachineSafetyPackageState::ReadyCurrent;
        return;
    }
    m_index.reset();
    m_extraction.reset();
    m_status.state = MachineSafetyPackageState::Invalid;
}

void MachineSafetyPackageManager::invalidate(const QString& reason)
{
    QWriteLocker locker(&m_lock);
    if (m_status.state == MachineSafetyPackageState::Unavailable)
        return;
    m_index.reset();
    m_extraction.reset();
    m_status.state = MachineSafetyPackageState::StaleUnsafe;
    m_status.buildInProgress = false;
    m_status.reason = reason;
}

void MachineSafetyPackageManager::markInvalid(const QString& reason)
{
    QWriteLocker locker(&m_lock);
    m_index.reset();
    m_extraction.reset();
    m_status.state = MachineSafetyPackageState::Invalid;
    m_status.buildInProgress = false;
    m_status.reason = reason;
}

void MachineSafetyPackageManager::clear()
{
    QWriteLocker locker(&m_lock);
    m_index.reset();
    m_extraction.reset();
    m_status = {};
}

QString machineSafetyPackageStateName(MachineSafetyPackageState state)
{
    switch (state) {
    case MachineSafetyPackageState::Unavailable: return QStringLiteral("Unavailable");
    case MachineSafetyPackageState::Loading: return QStringLiteral("Loading");
    case MachineSafetyPackageState::Building: return QStringLiteral("Building");
    case MachineSafetyPackageState::ReadyCurrent: return QStringLiteral("ReadyCurrent");
    case MachineSafetyPackageState::ReadySuperseded: return QStringLiteral("ReadySuperseded");
    case MachineSafetyPackageState::StaleUnsafe: return QStringLiteral("StaleUnsafe");
    case MachineSafetyPackageState::Invalid: return QStringLiteral("Invalid");
    }
    return QStringLiteral("Invalid");
}

} // namespace lcnc::cam
