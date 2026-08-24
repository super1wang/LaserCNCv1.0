#pragma once

#include "core/algorithms/cam/surface_collision_prefilter.h"

#include <QVector>

#include <gp_Pnt.hxx>

#include <cstdint>
#include <memory>

namespace lcnc::cam {

/// One conservative outer-cover sphere in a collision body's local frame.
/// The union of all spheres covers the complete sliced mesh envelope.
struct ConservativeCollisionSphere
{
    gp_Pnt center;
    double radiusMm{0.0};
};

/// Binary outer-cover hierarchy. A certified parent rejects its complete
/// subtree with one field lookup; an unresolved parent is refined to children.
struct ConservativeCollisionSphereNode
{
    ConservativeCollisionSphere sphere;
    int firstChild{-1};
    int secondChild{-1};

    bool isLeaf() const { return firstChild < 0 && secondChild < 0; }
};

QVector<ConservativeCollisionSphere> buildConservativeSphereCover(
    const cam_algo::SurfaceCollisionModel& model,
    int maximumSlices = 24);

QVector<ConservativeCollisionSphereNode> buildConservativeSphereHierarchy(
    const QVector<ConservativeCollisionSphere>& leaves);

struct LocalClearanceFieldStatistics
{
    std::uint64_t coarseQueries{0};
    std::uint64_t coarseHits{0};
    std::uint64_t fineQueries{0};
    std::uint64_t fineHits{0};
    std::uint64_t cellsBuilt{0};
};

/// Sparse, conservative distance-lower-bound cache in one passive body's
/// immutable local coordinate system. It is path-driven: only cells touched by
/// motion proxies are materialized, and unresolved coarse cells are refined to
/// the fine grid before the existing mesh/exact backends are considered.
class LocalClearanceField final
{
public:
    explicit LocalClearanceField(
        cam_algo::SurfaceCollisionModel model,
        double coarseCellSizeMm = 2.0,
        double fineCellSizeMm = 0.25);
    ~LocalClearanceField();

    LocalClearanceField(const LocalClearanceField&) = delete;
    LocalClearanceField& operator=(const LocalClearanceField&) = delete;

    bool isValid() const;

    /// Proves that a complete outer-cover sphere is separated from the local
    /// solid. requiredSeparationMm already includes motion and policy margins.
    bool certifiesSphereSeparated(const gp_Pnt& localCenter,
                                  double sphereRadiusMm,
                                  double requiredSeparationMm) const;

    /// Warms the path-hot fine cells without changing their proof semantics.
    void prefetchPathPoints(const QVector<gp_Pnt>& localPoints) const;

    LocalClearanceFieldStatistics statistics() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace lcnc::cam
