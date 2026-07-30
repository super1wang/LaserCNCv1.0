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
    OutputSignal,
    Camera,
    InputSignalWait,
    Calculation,
    Compare,
    Wait,
    Commands,
    Feeding,
    RunGroupCheck,
    SingleAxisMove,
    MultiAxisMove,
    Measurement,
    MarkAcquire,
    Alignment,
    AutoFocus,
    EnergySwitch,
    NormalCutting,
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
