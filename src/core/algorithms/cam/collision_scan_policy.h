#pragma once

#include "core/kinematics/machine_kinematics.h"
#include "core/project/cam/collision_validation_contracts.h"

#include <QSet>
#include <QString>
#include <QStringList>

#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <utility>

namespace lcnc::cam_algo {

// CAM generation and offline simulation must be reproducible from the
// committed toolpath alone.  Live MachineAxisDef::currentPos values belong to
// the machine/view state and must not become an implicit initial cutter pose.
// Axes outside the solved layout use the mode's explicit locked target, or
// zero when the mode does not prescribe one.
// 中文翻译：CAM 生成与离线仿真不得把实时轴反馈当成隐式起始刀头位姿；
// 未参与求解的轴只采用加工模式锁定值，未配置锁定值时采用零位。
inline QList<MachineAxisDef> offlinePlanningAxisBaseline(
    const QList<MachineAxisDef>& source,
    const lcnc::MachineModeDefinition& definition)
{
    QList<MachineAxisDef> result = source;
    for (MachineAxisDef& axis : result) {
        const auto locked = definition.lockedAxisTargets.constFind(axis.name);
        axis.currentPos = locked == definition.lockedAxisTargets.cend()
            ? 0.0 : locked.value();
    }
    return result;
}

inline void applyOfflineMotionPose(
    MachineKinematics* kinematics,
    const QList<MachineAxisDef>& baseline,
    const lcnc::MachineAxisLayout& layout,
    const std::array<double, lcnc::MachineAxisLayout::kMaxAxes>& values,
    std::uint8_t activeMask)
{
    if (!kinematics)
        return;
    for (const MachineAxisDef& axis : baseline)
        kinematics->setAxisPosition(axis.name, axis.currentPos);
    for (int index = 0; index < layout.count; ++index) {
        if ((activeMask & (1u << index)) == 0)
            continue;
        const QString& name = layout.axes[index].name;
        if (!name.isEmpty())
            kinematics->setAxisPosition(name, values[index]);
    }
}

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

/// Converts the geometric part of a CAM motion node from workpiece-local
/// coordinates to the machine-world frame used by machine collision bodies.
/// Solved machine axes are already in the machine frame and are not changed.
/// 中文翻译：把运动节点的 TCP 与法线从工件局部坐标转换到机床世界坐标；
/// 已求解的机床轴坐标保持不变。
inline void transformMotionNodeGeometry(
    lcnc::cam::CamMotionNode* node,
    const gp_Trsf& workpieceTransform)
{
    if (!node)
        return;
    gp_Pnt tcp(node->tcpX, node->tcpY, node->tcpZ);
    tcp.Transform(workpieceTransform);
    gp_Vec normal(node->normalX, node->normalY, node->normalZ);
    if (normal.SquareMagnitude() <= 1.0e-18)
        normal = gp_Vec(0.0, 0.0, 1.0);
    normal.Transform(workpieceTransform);
    if (normal.SquareMagnitude() <= 1.0e-18)
        normal = gp_Vec(0.0, 0.0, 1.0);
    normal.Normalize();
    node->tcpX = tcp.X();
    node->tcpY = tcp.Y();
    node->tcpZ = tcp.Z();
    node->normalX = normal.X();
    node->normalY = normal.Y();
    node->normalZ = normal.Z();
}

} // namespace lcnc::cam_algo
