#include "modules/cam/collision/coal_collision_backend.h"

#include <cmath>
#include <mutex>
#include <utility>

#if defined(LCNC_WITH_COAL_COLLISION)
#include <coal/BV/OBBRSS.h>
#include <coal/BVH/BVH_model.h>
#include <coal/collision.h>
#include <coal/math/transform.h>
#endif

namespace lcnc::cam {

class CoalCollisionBodyModel final
{
public:
    cam_algo::SurfaceCollisionModel surface;
#if defined(LCNC_WITH_COAL_COLLISION)
    std::shared_ptr<coal::BVHModel<coal::OBBRSS>> model;
#endif
};

class CoalCollisionPair final
{
public:
#if defined(LCNC_WITH_COAL_COLLISION)
    std::shared_ptr<CoalCollisionBodyModel> first;
    std::shared_ptr<CoalCollisionBodyModel> second;
    std::unique_ptr<coal::ComputeCollision> collision;
    coal::CollisionRequest request{coal::DISTANCE_LOWER_BOUND, 1};
    std::mutex mutex;
#endif
};

#if defined(LCNC_WITH_COAL_COLLISION)
namespace {

coal::Transform3s coalTransform(const gp_Trsf& source)
{
    coal::Matrix3s rotation;
    coal::Vec3s translation;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column)
            rotation(row, column) = source.Value(row + 1, column + 1);
        translation(row) = source.Value(row + 1, 4);
    }
    return {rotation, translation};
}

} // namespace
#endif

std::shared_ptr<CoalCollisionBodyModel> buildCoalCollisionBodyModel(
    const cam_algo::SurfaceCollisionModel& surface,
    std::string* error)
{
    if (error)
        error->clear();
#if defined(LCNC_WITH_COAL_COLLISION)
    const auto soup = surface.triangleSoup();
    if (soup.vertices.empty() || soup.triangles.empty()) {
        if (error)
            *error = "Surface collision mesh is unavailable for Coal";
        return {};
    }
    std::vector<coal::Vec3s> vertices;
    std::vector<coal::Triangle> triangles;
    vertices.reserve(soup.vertices.size());
    triangles.reserve(soup.triangles.size());
    for (const auto& point : soup.vertices)
        vertices.emplace_back(point[0], point[1], point[2]);
    for (const auto& triangle : soup.triangles)
        triangles.emplace_back(triangle[0], triangle[1], triangle[2]);

    auto result = std::make_shared<CoalCollisionBodyModel>();
    result->surface = surface;
    result->model = std::make_shared<coal::BVHModel<coal::OBBRSS>>();
    if (result->model->beginModel(static_cast<int>(triangles.size()),
                                  static_cast<int>(vertices.size())) != 0
        || result->model->addSubModel(vertices, triangles) != 0
        || result->model->endModel() != 0) {
        if (error)
            *error = "Coal BVH construction failed";
        return {};
    }
    result->model->computeLocalAABB();
    return result;
#else
    (void)surface;
    if (error)
        *error = "Coal collision backend is not compiled";
    return {};
#endif
}

std::shared_ptr<CoalCollisionPair> buildCoalCollisionPair(
    const std::shared_ptr<CoalCollisionBodyModel>& first,
    const std::shared_ptr<CoalCollisionBodyModel>& second)
{
#if defined(LCNC_WITH_COAL_COLLISION)
    if (!first || !second || !first->model || !second->model)
        return {};
    auto pair = std::make_shared<CoalCollisionPair>();
    pair->first = first;
    pair->second = second;
    pair->collision = std::make_unique<coal::ComputeCollision>(
        first->model.get(), second->model.get());
    pair->request.enable_contact = false;
    pair->request.gjk_initial_guess = coal::GJKInitialGuess::CachedGuess;
    return pair;
#else
    (void)first;
    (void)second;
    return {};
#endif
}

std::shared_ptr<CoalCollisionPair> cloneCoalCollisionPair(
    const std::shared_ptr<CoalCollisionPair>& source)
{
#if defined(LCNC_WITH_COAL_COLLISION)
    if (!source)
        return {};
    return buildCoalCollisionPair(source->first, source->second);
#else
    (void)source;
    return {};
#endif
}

CoalCollisionQueryResult queryCoalCollisionPair(
    const std::shared_ptr<CoalCollisionPair>& pair,
    const gp_Trsf& firstTransform,
    const gp_Trsf& secondTransform,
    double requiredSeparationMm)
{
    CoalCollisionQueryResult output;
#if defined(LCNC_WITH_COAL_COLLISION)
    if (!pair || !pair->collision || !std::isfinite(requiredSeparationMm)
        || requiredSeparationMm < 0.0) {
        return output;
    }
    std::lock_guard<std::mutex> lock(pair->mutex);
    output.available = true;
    pair->request.security_margin = requiredSeparationMm;
    pair->request.break_distance = requiredSeparationMm;
    pair->request.distance_upper_bound = requiredSeparationMm;
    coal::CollisionResult result;
    const std::size_t contacts = (*pair->collision)(
        coalTransform(firstTransform), coalTransform(secondTransform),
        pair->request, result);
    pair->request.updateGuess(result);
    output.distanceLowerBoundMm = result.distance_lower_bound;
    output.definitelySeparated = contacts == 0
        && result.distance_lower_bound > requiredSeparationMm;
    if (output.definitelySeparated
        && cam_algo::surfaceCollisionContainmentDetected(
            pair->first->surface, firstTransform,
            pair->second->surface, secondTransform)) {
        output.definitelySeparated = false;
        output.distanceLowerBoundMm = 0.0;
    }
#else
    (void)pair;
    (void)firstTransform;
    (void)secondTransform;
    (void)requiredSeparationMm;
#endif
    return output;
}

bool coalCollisionBackendAvailable()
{
#if defined(LCNC_WITH_COAL_COLLISION)
    return true;
#else
    return false;
#endif
}

} // namespace lcnc::cam
