#pragma once

#include <QString>

namespace lcnc::process {

/**
 * @brief Stable workflow node type used by the new process flow document.
 */
enum class ProcessNodeType
{
    Base,
    Start,
    Stop,
    If,
    Loop,
    Group,
    RunGroup,
    IO,
    Camera,
    Monitor,
    Calculation,
    Compare,
    Wait,
    Commands,
    Feeding,
    RunGroupCheck,
    Axis,
    AxesMove,
    Measurement,
    MarkAcquire,
    Alignment,
    AutoFocus,
    EnergySwitch,
    Cutting,
    OverCutting,
};

/**
 * @brief Runtime/editor state persisted with process workflow nodes.
 */
enum class ProcessNodeState
{
    Enabled,
    Disabled,
    Editing,
    Unavailable,
    Running,
    Stopped,
    Paused,
    Unused,
    Pending,
};

QString processNodeTypeToString(ProcessNodeType type);
ProcessNodeType processNodeTypeFromString(const QString& text);
QString processNodeStateToString(ProcessNodeState state);
ProcessNodeState processNodeStateFromString(const QString& text);
bool processNodeTypeCanHaveChildren(ProcessNodeType type);

} // namespace lcnc::process
