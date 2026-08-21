#pragma once

#include <cstdint>

namespace lcnc::cam::events {

struct ContourSequenceChanged {
    std::uint64_t revision{0};
};
struct TravelPathVisibilityToggled {
    bool visible{false};
};
struct ContourOrderLabelVisibilityToggled {
    bool visible{false};
};

enum class ExecutionPlanChangeKind {
    Configuration,
    Sequence,
    Layers,
};

/// OCC-free cross-module notification. Consumers re-read immutable provider
/// snapshots instead of depending on CamModule or LayerManager concrete types.
struct ExecutionPlanChanged {
    ExecutionPlanChangeKind kind{ExecutionPlanChangeKind::Configuration};
};

struct OfflineSimulationSourceChanged {};

} // namespace lcnc::cam::events
