#pragma once

#include <QString>

namespace lcnc::process {

enum class Axis
{
    X = 0, Y = 1, Z = 2, A = 3, B = 4, C = 5
};

/// Configured way to establish an axis machine reference from the Ribbon
/// homing command.
enum class AxisHomingMethod
{
    Disabled,
    ControllerHome,
    SetCurrentPosition,
};

/// One ordered homing operation. Commands are executed in vector order.
struct AxisHomingCommand
{
    Axis axis{Axis::X};
    AxisHomingMethod method{AxisHomingMethod::ControllerHome};
    double position{0.0};
    QString axisName;
};

} // namespace lcnc::process
