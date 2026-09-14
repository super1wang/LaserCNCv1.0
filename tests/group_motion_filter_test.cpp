#include "modules/process/runtime/group_motion_filter.h"

#include <iostream>
#include <limits>

using lcnc::process::classifyGroupMotion;
using lcnc::process::GroupMotionDecision;

int main()
{
    const std::array<double, 5> resolutions{10000, 10000, 10000, 1000, 1000};
    // 2026-09-10 physical-log regression: GTN rejected this approach as 11802.
    const std::array<double, 5> reference{292.3723, 40, 4.1329, 8.213, 0};
    auto target = std::array<double, 5>{292.3722711978372, 40, 4.133, 8.21321070173757, 0};
    if (classifyGroupMotion(target, reference, resolutions, false) != GroupMotionDecision::Skip
        || classifyGroupMotion(target, reference, resolutions, true) != GroupMotionDecision::Append) {
        std::cerr << "ACS near-zero regression or RTCP tolerance isolation failed\n";
        return 1;
    }
    for (std::size_t i = 0; i < 5; ++i) {
        for (double direction : {-1.0, 1.0}) {
            target = reference;
            target[i] += direction / resolutions[i];
            if (classifyGroupMotion(target, reference, resolutions, false) != GroupMotionDecision::Skip)
                return 2;
            target[i] = reference[i] + direction * 1.01 / resolutions[i];
            if (classifyGroupMotion(target, reference, resolutions, false) != GroupMotionDecision::Append)
                return 3;
        }
    }
    // Skipped samples do not become the anchor: tiny increments must eventually
    // produce motion instead of erasing a long densely sampled contour.
    auto anchor = reference;
    int appended = 0;
    for (int i = 1; i <= 12; ++i) {
        target = reference;
        target[0] += i * 0.4 / resolutions[0];
        if (classifyGroupMotion(target, anchor, resolutions, false) == GroupMotionDecision::Append) {
            anchor = target;
            ++appended;
        }
    }
    if (appended != 4 || anchor != target) return 4;
    target = reference;
    target[4] += 360.0;
    if (classifyGroupMotion(target, reference, resolutions, false) != GroupMotionDecision::Append)
        return 5;
    target[0] = std::numeric_limits<double>::quiet_NaN();
    if (classifyGroupMotion(target, reference, resolutions, false) != GroupMotionDecision::Invalid)
        return 6;
    target = reference;
    target[2] = std::numeric_limits<double>::infinity();
    if (classifyGroupMotion(target, reference, resolutions, true) != GroupMotionDecision::Invalid)
        return 7;
    auto invalidResolution = resolutions;
    invalidResolution[4] = 0;
    if (classifyGroupMotion(reference, reference, invalidResolution, false) != GroupMotionDecision::Invalid)
        return 8;
    return 0;
}
