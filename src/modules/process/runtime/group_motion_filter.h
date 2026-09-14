#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace lcnc::process {

enum class GroupMotionDecision { Invalid, Skip, Append };

// Compare to the last appended point, never the last skipped point. This
// bounds accumulated path loss to one pulse per axis rather than one pulse
// per input sample. Rotary coordinates are absolute: no modulo or sign flip.
inline GroupMotionDecision classifyGroupMotion(
    const std::array<double, 5>& target,
    const std::array<double, 5>& reference,
    const std::array<double, 5>& resolution,
    bool rtcp,
    std::array<double, 5>* tolerance = nullptr)
{
    bool same = true;
    for (std::size_t i = 0; i < target.size(); ++i) {
        if (!std::isfinite(target[i]) || !std::isfinite(reference[i])
            || (!rtcp && (!std::isfinite(resolution[i]) || resolution[i] <= 0.0)))
            return GroupMotionDecision::Invalid;
        const double epsilon = 8.0 * std::numeric_limits<double>::epsilon()
            * std::max({1.0, std::abs(target[i]), std::abs(reference[i])});
        const double limit = (rtcp ? 1.0e-9 : 1.0 / resolution[i]) + epsilon;
        const double difference = std::abs(target[i] - reference[i]);
        if (!std::isfinite(limit) || !std::isfinite(difference))
            return GroupMotionDecision::Invalid;
        if (tolerance) (*tolerance)[i] = limit;
        if (difference > limit) same = false;
    }
    return same ? GroupMotionDecision::Skip : GroupMotionDecision::Append;
}

} // namespace lcnc::process
