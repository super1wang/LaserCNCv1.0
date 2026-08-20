#pragma once

#include <cstdint>

namespace lcnc::cam::events {

struct ContourSequenceChanged { std::uint64_t revision{0}; };
struct TravelPathVisibilityToggled { bool visible{false}; };
struct ContourOrderLabelVisibilityToggled { bool visible{false}; };

} // namespace lcnc::cam::events
