#pragma once

#include "core/project/cam/collision_validation_contracts.h"

namespace lcnc::process {

enum class GtnFeedMetric : std::uint8_t {
    Unavailable, LinearMillimetres, WeightedMillimetres, RotaryDegrees
};

// Slots are Group command slots, not CAM layout indices or axis-name guesses.
struct GtnAxisBinding {
    int physicalIndex{-1};
    QString name;
    MachineAxisRole role{MachineAxisRole::Unspecified};
    int controllerAxis{0}; // GTN 1-based physical axis number
    double minimum{0};
    double maximum{0};
};

struct GtnMotionCell {
    bool supported{false};
    bool continuousInterpolationQualified{false};
    bool inactiveAxisHoldQualified{false};
    bool feedMappingQualified{false};
};

// Supplied only by a controller qualification authority. Defaults deliberately
// provide no authority. Neither SDK availability nor settings can qualify it.
struct GtnLoweringProfile {
    cam::ControllerQualificationState state{cam::ControllerQualificationState::Unavailable};
    std::uint64_t revision{0};
    QString sourceId;
    QString groupSemanticsId; // qualification record includes SDK/firmware/model/IO semantics
    cam::ControllerMotionMode mode{cam::ControllerMotionMode::PhysicalAxes};
    bool absoluteRotaryTurnsQualified{false}; // SDK short/long-path defaults are not authority
    std::array<GtnAxisBinding, 5> axes;
    std::array<GtnMotionCell, 5> cells;
    GtnFeedMetric metric{GtnFeedMetric::Unavailable};
    // Qualified Group command velocity reference-axis ratios, in slot order.
    // Weighted rotary ratios have units mm/degree. No implicit mm/degree mix.
    std::array<double, 5> referenceRatios{};
    double surfaceRadiusMm{0}; // RotaryDegrees: explicit surface-feed conversion
    double rapidFeedMmPerSecond{0};
};

QByteArray gtnLoweringProfileHash(const GtnLoweringProfile& profile);
} // namespace lcnc::process
