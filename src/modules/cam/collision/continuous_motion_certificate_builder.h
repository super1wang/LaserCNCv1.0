#pragma once

#include "core/project/cam/collision_validation_contracts.h"

#include <QByteArray>
#include <QVector>

#include <functional>
#include <memory>
#include <cstdint>

namespace lcnc::cam_algo {
class MachineSafetyIndex;
}

namespace lcnc::cam {

struct ToolpathExportSnapshot;
struct TravelCollisionGeometryCache;

struct ContinuousMotionCertificateBuildContext
{
    std::shared_ptr<cam_algo::MachineSafetyIndex> machineIndex;
    std::shared_ptr<TravelCollisionGeometryCache> geometry;
    QByteArray packageKeySha256;
    QByteArray runtimeConfigurationSha256;
    double clearanceMm{0.5};
    int maximumSubdivisionDepth{12};
    /// Rapid paths are already densely sampled by the planner. An unresolved
    /// sampled edge fails closed at this bounded depth instead of expanding
    /// to the generic offline limit.
    int maximumRapidSubdivisionDepth{4};
    std::uint64_t maximumIntervalsPerEdge{1024};
    std::uint64_t maximumCoalPairQueriesPerEdge{2048};
    /// Production default forbids serialized OCCT exact fallback. Explicit
    /// offline diagnostics may opt in with a finite budget.
    std::uint64_t maximumOcctExactQueriesPerEdge{0};
    /// Split a close interval before any mesh/exact query when its conservative
    /// body-motion radius is larger than this value.
    double maximumNarrowPhaseMotionBoundMm{5.0};
    int maximumParallelWorkers{8};
    int maximumParallelCoalQueries{1};
    /// A pair slower than this once is disabled for the remainder of the
    /// build; the unresolved interval remains fail-closed.
    std::uint64_t maximumCoalQueryWallTimeMs{50};
};

/// Builds exactly nodes.size()-1 certificates. Any unavailable proof is
/// represented as BoundaryUnknown; the function never promotes uncertainty
/// to safety. Production uses LMSI, local field, Surface-BVH and bounded Coal;
/// OCCT exact is opt-in for explicit offline diagnostics only.
QVector<CamMotionEdgeCertificate> buildContinuousMotionCertificates(
    const ToolpathExportSnapshot& snapshot,
    const ContinuousMotionCertificateBuildContext& context,
    const std::function<bool()>& cancelled = {},
    const std::function<void(int completedEdges, int totalEdges)>& progress = {});

} // namespace lcnc::cam
