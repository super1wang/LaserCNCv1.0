#include "core/algorithms/cam/collision_safety_domain.h"

#include <QCoreApplication>

#include <iostream>

namespace {

lcnc::cam_algo::CollisionSafetyDomainQuery queryAt(double x)
{
    lcnc::cam_algo::CollisionSafetyDomainQuery query;
    query.pose.axes[0] = x;
    query.pose.kinematicAxes[0] = x;
    query.pose.activeMask = 0x01;
    query.pose.kinematicAxisMask = 0x01;
    query.pose.tcpX = x;
    query.workpieceEntry = QStringLiteral("0:1:1");
    query.activeSource = QStringLiteral("cutter");
    query.passiveSource = QStringLiteral("axis:BASE");
    query.clearanceMm = 0.5;
    return query;
}

bool require(bool condition, const char* message)
{
    if (!condition)
        std::cerr << message << '\n';
    return condition;
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    lcnc::cam_algo::CollisionSafetyDomainCache cache(2);
    lcnc::cam_algo::CollisionSafetyDomainSample safe;
    safe.state = lcnc::cam::CollisionValidationState::Safe;
    safe.minimumDistanceMm = 8.0;
    const auto first = queryAt(1.0);
    cache.store(first, safe);

    lcnc::cam_algo::CollisionSafetyDomainSample loaded;
    if (!require(cache.lookup(first, &loaded), "exact pose must hit")
        || !require(loaded.state == lcnc::cam::CollisionValidationState::Safe,
                    "cached state changed")) {
        return 1;
    }
    auto differentSemanticOrder = first;
    differentSemanticOrder.pose.axes[0] = 999.0;
    differentSemanticOrder.pose.activeMask = 0x1f;
    if (!require(cache.lookup(differentSemanticOrder, &loaded),
                 "semantic Process axis order must not fragment physical-pose cache")) {
        return 1;
    }
    auto differentPhysicalBranch = first;
    differentPhysicalBranch.pose.kinematicAxes[3] = 180.0;
    differentPhysicalBranch.pose.kinematicAxisMask |= 0x08;
    if (!require(!cache.lookup(differentPhysicalBranch, &loaded),
                 "different physical IK branches must not share a certificate")) {
        return 1;
    }
    auto differentPhase = first;
    differentPhase.phase = lcnc::cam::CamMotionPhase::Cutting;
    if (!require(!cache.lookup(differentPhase, &loaded),
                 "motion phase must participate in the key")) {
        return 1;
    }
    auto differentClearance = first;
    differentClearance.clearanceMm = 1.0;
    if (!require(!cache.lookup(differentClearance, &loaded),
                 "clearance must participate in the key")) {
        return 1;
    }
    cache.store(queryAt(2.0), safe);
    cache.store(queryAt(3.0), safe);
    const auto statistics = cache.statistics();
    if (!require(statistics.samples <= 2, "cache capacity was not enforced")
        || !require(statistics.hits >= 1, "cache hit was not counted")
        || !require(statistics.misses >= 2, "cache miss was not counted")) {
        return 1;
    }
    return 0;
}
