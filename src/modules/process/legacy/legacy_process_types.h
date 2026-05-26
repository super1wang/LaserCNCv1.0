#pragma once

#include <QString>

#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;

namespace lcnc::process::legacy {

enum class PermissionLevel
{
    Developers = 9,
    Factory = 7,
    Simulate = 6,
    Administrator = 4,
    Technician = 2,
    Operator = 1,
    None = 0,
};

enum class ItemType
{
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
    Base,
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

enum class ItemState
{
    StateSave,
    Disable,
    Enable,
    Editing,
    Unavailable,
    Run,
    Stop,
    Pause,
    Unuse,
    Unrun,
};

struct Item
{
    int iParentIndex{-1};
    int iChildrenIndex{-1};
    ItemType Type{ItemType::Base};
    map<QString, QString> maps;
};

} // namespace lcnc::process::legacy

using lcnc::process::legacy::Item;
using lcnc::process::legacy::ItemState;
using lcnc::process::legacy::ItemType;
using lcnc::process::legacy::PermissionLevel;
