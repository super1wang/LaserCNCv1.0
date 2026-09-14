#pragma once

namespace lcnc::process {

constexpr long kGtnMachiningDriverFaultMask = (1L << 1) | (1L << 4) | (1L << 7) | (1L << 8);
constexpr long kGtnMachiningPositiveLimitBit = 1L << 5;
constexpr long kGtnMachiningNegativeLimitBit = 1L << 6;

constexpr long gtnMachiningFault(long status, bool hardwarePositive,
    bool hardwareNegative, bool softwarePositive, bool softwareNegative)
{
    long fault = status & (kGtnMachiningDriverFaultMask
        | kGtnMachiningPositiveLimitBit | kGtnMachiningNegativeLimitBit);
    if (hardwarePositive || softwarePositive) fault |= kGtnMachiningPositiveLimitBit;
    if (hardwareNegative || softwareNegative) fault |= kGtnMachiningNegativeLimitBit;
    return fault;
}

} // namespace lcnc::process
