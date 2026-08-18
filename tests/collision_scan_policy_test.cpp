#include "core/algorithms/cam/collision_scan_policy.h"

#include <QCoreApplication>
#include <QTextStream>

#include <limits>
#include <mutex>

namespace {

int fail(const QString& message)
{
    QTextStream(stderr) << message << '\n';
    return 1;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    const QSet<QString> sources = {
        QStringLiteral("axis:B"),
        QStringLiteral("axis:Z"),
        QStringLiteral("cutter"),
        QStringLiteral("axis:A"),
    };
    const QStringList expected = {
        QStringLiteral("cutter"),
        QStringLiteral("axis:Z"),
        QStringLiteral("axis:A"),
        QStringLiteral("axis:B"),
    };
    if (lcnc::cam_algo::orderedActiveCollisionSources(sources) != expected)
        return fail(QStringLiteral("Active collision sources are not staged deterministically"));

    if (lcnc::cam_algo::collisionStateSeverity(1)
            <= lcnc::cam_algo::collisionStateSeverity(3)
        || lcnc::cam_algo::collisionStateSeverity(3)
            <= lcnc::cam_algo::collisionStateSeverity(2)
        || lcnc::cam_algo::collisionStateSeverity(2)
            <= lcnc::cam_algo::collisionStateSeverity(0)) {
        return fail(QStringLiteral("Collision result severity order is invalid"));
    }

    using State = lcnc::cam::CollisionValidationState;
    if (lcnc::cam_algo::classifyCollisionDistance(0.0, 0.5, 1e-7)
            != State::Collision
        || lcnc::cam_algo::classifyCollisionDistance(0.25, 0.5, 1e-7)
            != State::Warning
        || lcnc::cam_algo::classifyCollisionDistance(0.75, 0.5, 1e-7)
            != State::Safe
        || lcnc::cam_algo::classifyCollisionDistance(
               std::numeric_limits<double>::quiet_NaN(), 0.5, 1e-7)
            != State::Indeterminate) {
        return fail(QStringLiteral("Exact distance classification is invalid"));
    }

    std::unique_lock<std::timed_mutex> owner(
        lcnc::cam_algo::collisionScanExecutionMutex());
    std::unique_lock<std::timed_mutex> cancelled(
        lcnc::cam_algo::collisionScanExecutionMutex(), std::defer_lock);
    if (lcnc::cam_algo::acquireCollisionScanExecution(cancelled, [] { return true; }))
        return fail(QStringLiteral("Cancelled scan acquired the global execution guard"));
    owner.unlock();

    std::unique_lock<std::timed_mutex> next(
        lcnc::cam_algo::collisionScanExecutionMutex(), std::defer_lock);
    if (!lcnc::cam_algo::acquireCollisionScanExecution(next, [] { return false; }))
        return fail(QStringLiteral("Released collision execution guard could not be acquired"));

    return 0;
}
