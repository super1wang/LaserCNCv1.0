#include "modules/cam/collision/workpiece_clearance_field.h"

#include <gp_Vec.hxx>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <limits>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace lcnc::cam {
namespace {

struct CellKey
{
    std::int64_t x{0};
    std::int64_t y{0};
    std::int64_t z{0};

    bool operator==(const CellKey& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct CellKeyHash
{
    std::size_t operator()(const CellKey& key) const noexcept
    {
        std::uint64_t hash = 1469598103934665603ull;
        const auto mix = [&hash](std::uint64_t value) {
            hash ^= value;
            hash *= 1099511628211ull;
        };
        mix(static_cast<std::uint64_t>(key.x));
        mix(static_cast<std::uint64_t>(key.y));
        mix(static_cast<std::uint64_t>(key.z));
        return static_cast<std::size_t>(hash);
    }
};

struct CellProof
{
    bool valid{false};
    bool outsideCertified{false};
    double distanceLowerBoundMm{0.0};
};

CellKey cellKey(const gp_Pnt& point, double sizeMm)
{
    return {static_cast<std::int64_t>(std::floor(point.X() / sizeMm)),
            static_cast<std::int64_t>(std::floor(point.Y() / sizeMm)),
            static_cast<std::int64_t>(std::floor(point.Z() / sizeMm))};
}

gp_Pnt cellCenter(const CellKey& key, double sizeMm)
{
    return gp_Pnt((static_cast<double>(key.x) + 0.5) * sizeMm,
                  (static_cast<double>(key.y) + 0.5) * sizeMm,
                  (static_cast<double>(key.z) + 0.5) * sizeMm);
}

double coordinate(const std::array<double, 3>& point, int axis)
{
    return point[static_cast<std::size_t>(axis)];
}

using Polygon = std::vector<std::array<double, 3>>;

Polygon clipPolygonAtAxis(const Polygon& input, int axis,
                          double boundary, bool keepGreater)
{
    Polygon output;
    if (input.empty())
        return output;
    output.reserve(input.size() + 1);
    const auto inside = [=](const std::array<double, 3>& point) {
        return keepGreater ? coordinate(point, axis) >= boundary
                           : coordinate(point, axis) <= boundary;
    };
    for (std::size_t index = 0; index < input.size(); ++index) {
        const auto& first = input[index];
        const auto& last = input[(index + 1) % input.size()];
        const bool firstInside = inside(first);
        const bool lastInside = inside(last);
        if (firstInside)
            output.push_back(first);
        if (firstInside == lastInside)
            continue;
        const double denominator = coordinate(last, axis)
            - coordinate(first, axis);
        if (std::abs(denominator) <= std::numeric_limits<double>::epsilon())
            continue;
        const double parameter = std::clamp(
            (boundary - coordinate(first, axis)) / denominator, 0.0, 1.0);
        std::array<double, 3> intersection{};
        for (int component = 0; component < 3; ++component) {
            intersection[static_cast<std::size_t>(component)] =
                first[static_cast<std::size_t>(component)]
                + parameter * (last[static_cast<std::size_t>(component)]
                               - first[static_cast<std::size_t>(component)]);
        }
        output.push_back(intersection);
    }
    return output;
}

} // namespace

struct LocalClearanceField::Impl
{
    using Cache = std::unordered_map<CellKey, CellProof, CellKeyHash>;

    cam_algo::SurfaceCollisionModel model;
    double coarseCellSizeMm{2.0};
    double fineCellSizeMm{0.25};
    mutable std::shared_mutex coarseMutex;
    mutable std::shared_mutex fineMutex;
    mutable Cache coarseCells;
    mutable Cache fineCells;
    mutable std::atomic_uint64_t coarseQueries{0};
    mutable std::atomic_uint64_t coarseHits{0};
    mutable std::atomic_uint64_t fineQueries{0};
    mutable std::atomic_uint64_t fineHits{0};
    mutable std::atomic_uint64_t cellsBuilt{0};

    CellProof buildCell(const CellKey& key, double cellSizeMm) const
    {
        CellProof proof;
        if (!model.isValid() || !model.isClosedSolid())
            return proof;
        const auto query = model.pointDistance(cellCenter(key, cellSizeMm));
        if (!query.valid)
            return proof;
        const double cellHalfDiagonal = std::sqrt(3.0) * cellSizeMm * 0.5;
        proof.valid = true;
        proof.distanceLowerBoundMm = std::max(
            0.0, query.meshDistanceMm - cellHalfDiagonal
                     - model.linearDeflectionMm());
        proof.outsideCertified = !query.insideClosedSolid
            && proof.distanceLowerBoundMm > 0.0;
        cellsBuilt.fetch_add(1, std::memory_order_relaxed);
        return proof;
    }

    CellProof queryCell(const gp_Pnt& point, double cellSizeMm,
                        Cache* cache, std::shared_mutex* mutex,
                        std::atomic_uint64_t* queryCounter,
                        std::atomic_uint64_t* hitCounter) const
    {
        queryCounter->fetch_add(1, std::memory_order_relaxed);
        const CellKey key = cellKey(point, cellSizeMm);
        {
            std::shared_lock lock(*mutex);
            const auto found = cache->find(key);
            if (found != cache->end()) {
                hitCounter->fetch_add(1, std::memory_order_relaxed);
                return found->second;
            }
        }
        const CellProof built = buildCell(key, cellSizeMm);
        std::unique_lock lock(*mutex);
        const auto [found, inserted] = cache->emplace(key, built);
        if (!inserted)
            hitCounter->fetch_add(1, std::memory_order_relaxed);
        return found->second;
    }

    bool certifies(const CellProof& proof, double radiusMm,
                   double separationMm) const
    {
        return proof.valid && proof.outsideCertified
            && proof.distanceLowerBoundMm > radiusMm + separationMm;
    }
};

QVector<ConservativeCollisionSphere> buildConservativeSphereCover(
    const cam_algo::SurfaceCollisionModel& model,
    int maximumSlices)
{
    QVector<ConservativeCollisionSphere> result;
    if (!model.isValid() || maximumSlices < 1)
        return result;
    const auto soup = model.triangleSoup();
    if (soup.vertices.empty() || soup.triangles.empty())
        return result;
    std::array<double, 3> minimum{
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity()};
    std::array<double, 3> maximum{
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()};
    for (const auto& vertex : soup.vertices) {
        for (int axis = 0; axis < 3; ++axis) {
            minimum[axis] = std::min(minimum[axis], vertex[axis]);
            maximum[axis] = std::max(maximum[axis], vertex[axis]);
        }
    }
    const std::array<double, 3> extent{
        maximum[0] - minimum[0], maximum[1] - minimum[1],
        maximum[2] - minimum[2]};
    const int slicingAxis = extent[0] >= extent[1] && extent[0] >= extent[2]
        ? 0 : (extent[1] >= extent[2] ? 1 : 2);
    const int sliceCount = std::clamp(
        static_cast<int>(std::ceil(extent[slicingAxis] / 0.75)),
        1, maximumSlices);
    const double sliceWidth = extent[slicingAxis] > 0.0
        ? extent[slicingAxis] / static_cast<double>(sliceCount) : 1.0;
    struct SliceBounds {
        std::array<double, 3> minimum{
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity()};
        std::array<double, 3> maximum{
            -std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity()};
        bool valid{false};
    };
    QVector<SliceBounds> slices(sliceCount);
    for (const auto& triangle : soup.triangles) {
        double triangleMinimum = std::numeric_limits<double>::infinity();
        double triangleMaximum = -std::numeric_limits<double>::infinity();
        for (const std::uint32_t index : triangle) {
            triangleMinimum = std::min(
                triangleMinimum, coordinate(soup.vertices[index], slicingAxis));
            triangleMaximum = std::max(
                triangleMaximum, coordinate(soup.vertices[index], slicingAxis));
        }
        const int firstSlice = std::clamp(
            static_cast<int>(std::floor(
                (triangleMinimum - minimum[slicingAxis]) / sliceWidth)),
            0, sliceCount - 1);
        const int lastSlice = std::clamp(
            static_cast<int>(std::floor(
                (triangleMaximum - minimum[slicingAxis]) / sliceWidth)),
            0, sliceCount - 1);
        for (int slice = firstSlice; slice <= lastSlice; ++slice) {
            const double slabMinimum = minimum[slicingAxis]
                + static_cast<double>(slice) * sliceWidth;
            const double slabMaximum = slice == sliceCount - 1
                ? maximum[slicingAxis] : slabMinimum + sliceWidth;
            Polygon clipped{
                soup.vertices[triangle[0]], soup.vertices[triangle[1]],
                soup.vertices[triangle[2]]};
            clipped = clipPolygonAtAxis(
                clipped, slicingAxis, slabMinimum, true);
            clipped = clipPolygonAtAxis(
                clipped, slicingAxis, slabMaximum, false);
            if (clipped.empty())
                continue;
            auto& bounds = slices[slice];
            bounds.valid = true;
            for (const auto& vertex : clipped) {
                for (int axis = 0; axis < 3; ++axis) {
                    bounds.minimum[axis] = std::min(
                        bounds.minimum[axis], vertex[axis]);
                    bounds.maximum[axis] = std::max(
                        bounds.maximum[axis], vertex[axis]);
                }
            }
        }
    }
    for (const auto& bounds : slices) {
        if (!bounds.valid)
            continue;
        const gp_Pnt center(
            (bounds.minimum[0] + bounds.maximum[0]) * 0.5,
            (bounds.minimum[1] + bounds.maximum[1]) * 0.5,
            (bounds.minimum[2] + bounds.maximum[2]) * 0.5);
        const double dx = bounds.maximum[0] - bounds.minimum[0];
        const double dy = bounds.maximum[1] - bounds.minimum[1];
        const double dz = bounds.maximum[2] - bounds.minimum[2];
        result.append({center,
                       0.5 * std::sqrt(dx * dx + dy * dy + dz * dz)
                           + std::numeric_limits<double>::epsilon() * 32.0});
    }
    return result;
}

QVector<ConservativeCollisionSphereNode> buildConservativeSphereHierarchy(
    const QVector<ConservativeCollisionSphere>& leaves)
{
    QVector<ConservativeCollisionSphereNode> result;
    if (leaves.isEmpty())
        return result;
    result.reserve(leaves.size() * 2);
    QVector<int> level;
    level.reserve(leaves.size());
    for (const auto& sphere : leaves) {
        level.append(result.size());
        result.append({sphere, -1, -1});
    }
    const auto enclosingSphere = [](const ConservativeCollisionSphere& first,
                                    const ConservativeCollisionSphere& second) {
        const double distance = first.center.Distance(second.center);
        if (distance + second.radiusMm <= first.radiusMm)
            return first;
        if (distance + first.radiusMm <= second.radiusMm)
            return second;
        if (distance <= std::numeric_limits<double>::epsilon()) {
            return ConservativeCollisionSphere{
                first.center, std::max(first.radiusMm, second.radiusMm)};
        }
        const double radius = 0.5
            * (distance + first.radiusMm + second.radiusMm);
        gp_Vec direction(first.center, second.center);
        direction.Multiply((radius - first.radiusMm) / distance);
        gp_Pnt center = first.center.Translated(direction);
        return ConservativeCollisionSphere{
            center, radius + std::numeric_limits<double>::epsilon() * 64.0};
    };
    while (level.size() > 1) {
        QVector<int> next;
        next.reserve((level.size() + 1) / 2);
        for (int index = 0; index < level.size(); index += 2) {
            if (index + 1 >= level.size()) {
                next.append(level.at(index));
                continue;
            }
            const int first = level.at(index);
            const int second = level.at(index + 1);
            const auto sphere = enclosingSphere(
                result.at(first).sphere, result.at(second).sphere);
            next.append(result.size());
            result.append({sphere, first, second});
        }
        level = std::move(next);
    }
    return result;
}

LocalClearanceField::LocalClearanceField(
    cam_algo::SurfaceCollisionModel model,
    double coarseCellSizeMm,
    double fineCellSizeMm)
    : m_impl(std::make_unique<Impl>())
{
    m_impl->model = std::move(model);
    m_impl->coarseCellSizeMm = coarseCellSizeMm;
    m_impl->fineCellSizeMm = fineCellSizeMm;
}

LocalClearanceField::~LocalClearanceField() = default;

bool LocalClearanceField::isValid() const
{
    return m_impl && m_impl->model.isValid() && m_impl->model.isClosedSolid()
        && std::isfinite(m_impl->coarseCellSizeMm)
        && std::isfinite(m_impl->fineCellSizeMm)
        && m_impl->coarseCellSizeMm > m_impl->fineCellSizeMm
        && m_impl->fineCellSizeMm > 0.0;
}

bool LocalClearanceField::certifiesSphereSeparated(
    const gp_Pnt& localCenter,
    double sphereRadiusMm,
    double requiredSeparationMm) const
{
    if (!isValid() || !std::isfinite(sphereRadiusMm)
        || !std::isfinite(requiredSeparationMm)
        || sphereRadiusMm < 0.0 || requiredSeparationMm < 0.0) {
        return false;
    }
    const CellProof coarse = m_impl->queryCell(
        localCenter, m_impl->coarseCellSizeMm,
        &m_impl->coarseCells, &m_impl->coarseMutex,
        &m_impl->coarseQueries, &m_impl->coarseHits);
    if (m_impl->certifies(coarse, sphereRadiusMm, requiredSeparationMm))
        return true;
    const CellProof fine = m_impl->queryCell(
        localCenter, m_impl->fineCellSizeMm,
        &m_impl->fineCells, &m_impl->fineMutex,
        &m_impl->fineQueries, &m_impl->fineHits);
    return m_impl->certifies(fine, sphereRadiusMm, requiredSeparationMm);
}

void LocalClearanceField::prefetchPathPoints(
    const QVector<gp_Pnt>& localPoints) const
{
    if (!isValid())
        return;
    for (const gp_Pnt& point : localPoints) {
        (void)m_impl->queryCell(
            point, m_impl->fineCellSizeMm,
            &m_impl->fineCells, &m_impl->fineMutex,
            &m_impl->fineQueries, &m_impl->fineHits);
    }
}

LocalClearanceFieldStatistics LocalClearanceField::statistics() const
{
    LocalClearanceFieldStatistics result;
    if (!m_impl)
        return result;
    result.coarseQueries = m_impl->coarseQueries.load(std::memory_order_relaxed);
    result.coarseHits = m_impl->coarseHits.load(std::memory_order_relaxed);
    result.fineQueries = m_impl->fineQueries.load(std::memory_order_relaxed);
    result.fineHits = m_impl->fineHits.load(std::memory_order_relaxed);
    result.cellsBuilt = m_impl->cellsBuilt.load(std::memory_order_relaxed);
    return result;
}

} // namespace lcnc::cam
