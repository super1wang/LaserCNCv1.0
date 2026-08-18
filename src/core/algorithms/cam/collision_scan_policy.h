#pragma once

#include "core/project/cam/collision_validation_contracts.h"

#include <QSet>
#include <QString>
#include <QStringList>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <utility>

namespace lcnc::cam_algo {

// OCCT geometry algorithms are allowed to parallelise inside one collision
// scan, but two scan sessions must never operate on the same frozen machine
// geometry at the same time.  The timed acquisition keeps cancellation and
// module shutdown responsive while a previous scan is winding down.
// 中文翻译：单次碰撞扫描内部可并行，但不同扫描会话不得同时操作冻结的机台几何。
inline std::timed_mutex& collisionScanExecutionMutex()
{
    static std::timed_mutex mutex;
    return mutex;
}

template <typename Cancelled>
bool acquireCollisionScanExecution(std::unique_lock<std::timed_mutex>& lock,
                                   Cancelled&& cancelled)
{
    while (!lock.try_lock_for(std::chrono::milliseconds(50))) {
        if (std::forward<Cancelled>(cancelled)())
            return false;
    }
    return !std::forward<Cancelled>(cancelled)();
}

// Active sources are deliberately processed in separate phases.  The cutter
// is cheap and safety-critical, Z is commonly the largest Compound, and the
// remaining head axes follow in a stable order.  A phase must finish before
// the next one starts; only trajectory nodes within that phase are parallel.
// 中文翻译：主动源按切割头、Z 轴、其余轴稳定排序并分阶段执行；阶段之间不并发。
inline QStringList orderedActiveCollisionSources(const QSet<QString>& sources)
{
    QStringList remaining(sources.cbegin(), sources.cend());
    remaining.removeAll(QStringLiteral("cutter"));
    remaining.removeAll(QStringLiteral("axis:Z"));
    std::sort(remaining.begin(), remaining.end(), [](const QString& lhs, const QString& rhs) {
        return QString::compare(lhs, rhs, Qt::CaseSensitive) < 0;
    });

    QStringList ordered;
    if (sources.contains(QStringLiteral("cutter")))
        ordered.append(QStringLiteral("cutter"));
    if (sources.contains(QStringLiteral("axis:Z")))
        ordered.append(QStringLiteral("axis:Z"));
    ordered.append(remaining);
    return ordered;
}

inline int collisionStateSeverity(qint8 state)
{
    switch (state) {
    case 1: return 4; // confirmed collision
    case 3: return 3; // indeterminate
    case 2: return 2; // clearance warning
    case 0: return 1; // confirmed safe
    default: return 0; // pending
    }
}

inline lcnc::cam::CollisionValidationState classifyCollisionDistance(
    double distanceMm, double clearanceMm, double zeroToleranceMm)
{
    if (!std::isfinite(distanceMm) || !std::isfinite(clearanceMm)
        || !std::isfinite(zeroToleranceMm)) {
        return lcnc::cam::CollisionValidationState::Indeterminate;
    }
    if (distanceMm <= std::max(0.0, zeroToleranceMm))
        return lcnc::cam::CollisionValidationState::Collision;
    if (distanceMm <= std::max(0.0, clearanceMm))
        return lcnc::cam::CollisionValidationState::Warning;
    return lcnc::cam::CollisionValidationState::Safe;
}

} // namespace lcnc::cam_algo
