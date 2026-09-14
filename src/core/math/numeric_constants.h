#pragma once

namespace lcnc::math {

inline constexpr double kPi = 3.141592653589793238462643383279502884;
inline constexpr double kDegreesPerRadian = 180.0 / kPi;
inline constexpr double kRadiansPerDegree = kPi / 180.0;

constexpr double degreesToRadians(double degrees) noexcept {
    return degrees * kRadiansPerDegree;
}

constexpr double radiansToDegrees(double radians) noexcept {
    return radians * kDegreesPerRadian;
}

} // namespace lcnc::math
