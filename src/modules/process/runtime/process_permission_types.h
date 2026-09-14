#pragma once

namespace lcnc::process {

enum class PermissionLevel
{
    Developers = 9,
    Factory = 7,
    Simulate = 6,
    Administrator = 4,
    Technician = 2,
    Operator = 1,
    None = 0
};

} // namespace lcnc::process
