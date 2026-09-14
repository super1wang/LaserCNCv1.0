#pragma once

#include "core/algorithms/cam/surface_collision_prefilter.h"

#include <gp_Trsf.hxx>

#include <memory>
#include <string>

namespace lcnc::cam {

class CoalCollisionBodyModel;
class CoalCollisionPair;

struct CoalCollisionQueryResult
{
    bool available{false};
    bool definitelySeparated{false};
    double distanceLowerBoundMm{0.0};
};

std::shared_ptr<CoalCollisionBodyModel> buildCoalCollisionBodyModel(
    const cam_algo::SurfaceCollisionModel& surface,
    std::string* error = nullptr);

std::shared_ptr<CoalCollisionPair> buildCoalCollisionPair(
    const std::shared_ptr<CoalCollisionBodyModel>& first,
    const std::shared_ptr<CoalCollisionBodyModel>& second);

/// Creates an independent mutable query context while sharing the immutable
/// body BVHs. Use one clone per worker to avoid serializing cached GJK state.
std::shared_ptr<CoalCollisionPair> cloneCoalCollisionPair(
    const std::shared_ptr<CoalCollisionPair>& source);

CoalCollisionQueryResult queryCoalCollisionPair(
    const std::shared_ptr<CoalCollisionPair>& pair,
    const gp_Trsf& firstTransform,
    const gp_Trsf& secondTransform,
    double requiredSeparationMm);

bool coalCollisionBackendAvailable();

} // namespace lcnc::cam
