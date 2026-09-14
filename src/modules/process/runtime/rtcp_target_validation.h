#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace lcnc::process {

inline bool finiteRtcpPose(const std::array<double, 5>& pose)
{
    return std::all_of(pose.cbegin(), pose.cend(),
                       [](double value) { return std::isfinite(value); });
}

inline bool rtcpAxesAgree(const std::array<double, 5>& predicted,
                          const std::array<double, 5>& actual,
                          double tolerance, double* maximumError)
{
    if (!maximumError) return false;
    *maximumError = std::numeric_limits<double>::infinity();
    if (!finiteRtcpPose(predicted) || !finiteRtcpPose(actual)
        || !std::isfinite(tolerance) || tolerance <= 0.0)
        return false;
    double error = 0.0;
    for (int index = 0; index < 5; ++index)
        error = std::max(error, std::abs(actual[index] - predicted[index]));
    *maximumError = error;
    return std::isfinite(error) && error <= tolerance;
}

} // namespace lcnc::process
