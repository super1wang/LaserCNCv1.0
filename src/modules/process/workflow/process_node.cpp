#include "modules/process/workflow/process_node.h"

#include <QUuid>

namespace lcnc::process {

QString createProcessNodeId()
{
    return QUuid::createUuid().toString(QUuid::WithoutBraces);
}

QString defaultProcessNodeName(ProcessNodeType type)
{
    return processNodeTypeToString(type);
}

ProcessNode::ProcessNode()
    : id(createProcessNodeId())
    , name(defaultProcessNodeName(type))
{
}

ProcessNode::ProcessNode(ProcessNodeType nodeType, const QString& nodeName)
    : id(createProcessNodeId())
    , type(nodeType)
    , name(nodeName.isEmpty() ? defaultProcessNodeName(nodeType) : nodeName)
{
}

QString processNodeTypeToString(ProcessNodeType type)
{
    switch (type) {
    case ProcessNodeType::Start: return QStringLiteral("Start");
    case ProcessNodeType::Stop: return QStringLiteral("Stop");
    case ProcessNodeType::If: return QStringLiteral("If");
    case ProcessNodeType::Loop: return QStringLiteral("Loop");
    case ProcessNodeType::Group: return QStringLiteral("Group");
    case ProcessNodeType::RunGroup: return QStringLiteral("RunGroup");
    case ProcessNodeType::IO: return QStringLiteral("IO");
    case ProcessNodeType::Camera: return QStringLiteral("Camera");
    case ProcessNodeType::Monitor: return QStringLiteral("Monitor");
    case ProcessNodeType::Calculation: return QStringLiteral("Calculation");
    case ProcessNodeType::Compare: return QStringLiteral("Compare");
    case ProcessNodeType::Wait: return QStringLiteral("Wait");
    case ProcessNodeType::Commands: return QStringLiteral("Commands");
    case ProcessNodeType::Feeding: return QStringLiteral("Feeding");
    case ProcessNodeType::RunGroupCheck: return QStringLiteral("RunGroupCheck");
    case ProcessNodeType::Axis: return QStringLiteral("Axis");
    case ProcessNodeType::AxesMove: return QStringLiteral("AxesMove");
    case ProcessNodeType::Measurement: return QStringLiteral("Measurement");
    case ProcessNodeType::MarkAcquire: return QStringLiteral("MarkAcquire");
    case ProcessNodeType::Alignment: return QStringLiteral("Alignment");
    case ProcessNodeType::AutoFocus: return QStringLiteral("AutoFocus");
    case ProcessNodeType::EnergySwitch: return QStringLiteral("EnergySwitch");
    case ProcessNodeType::Cutting: return QStringLiteral("Cutting");
    case ProcessNodeType::OverCutting: return QStringLiteral("OverCutting");
    case ProcessNodeType::Base:
    default: return QStringLiteral("Base");
    }
}

ProcessNodeType processNodeTypeFromString(const QString& text)
{
    const QString normalized = text.trimmed();
    if (normalized == QStringLiteral("Start")) return ProcessNodeType::Start;
    if (normalized == QStringLiteral("Stop")) return ProcessNodeType::Stop;
    if (normalized == QStringLiteral("If")) return ProcessNodeType::If;
    if (normalized == QStringLiteral("Loop")) return ProcessNodeType::Loop;
    if (normalized == QStringLiteral("Group")) return ProcessNodeType::Group;
    if (normalized == QStringLiteral("RunGroup")) return ProcessNodeType::RunGroup;
    if (normalized == QStringLiteral("IO")) return ProcessNodeType::IO;
    if (normalized == QStringLiteral("Camera")) return ProcessNodeType::Camera;
    if (normalized == QStringLiteral("Monitor")) return ProcessNodeType::Monitor;
    if (normalized == QStringLiteral("Calculation")) return ProcessNodeType::Calculation;
    if (normalized == QStringLiteral("Compare")) return ProcessNodeType::Compare;
    if (normalized == QStringLiteral("Wait")) return ProcessNodeType::Wait;
    if (normalized == QStringLiteral("Commands")) return ProcessNodeType::Commands;
    if (normalized == QStringLiteral("Feeding")) return ProcessNodeType::Feeding;
    if (normalized == QStringLiteral("RunGroupCheck")) return ProcessNodeType::RunGroupCheck;
    if (normalized == QStringLiteral("Axis")) return ProcessNodeType::Axis;
    if (normalized == QStringLiteral("AxesMove")) return ProcessNodeType::AxesMove;
    if (normalized == QStringLiteral("Measurement")) return ProcessNodeType::Measurement;
    if (normalized == QStringLiteral("MarkAcquire")) return ProcessNodeType::MarkAcquire;
    if (normalized == QStringLiteral("Alignment")) return ProcessNodeType::Alignment;
    if (normalized == QStringLiteral("AutoFocus")) return ProcessNodeType::AutoFocus;
    if (normalized == QStringLiteral("EnergySwitch")) return ProcessNodeType::EnergySwitch;
    if (normalized == QStringLiteral("Cutting")) return ProcessNodeType::Cutting;
    if (normalized == QStringLiteral("OverCutting")) return ProcessNodeType::OverCutting;
    return ProcessNodeType::Base;
}

QString processNodeStateToString(ProcessNodeState state)
{
    switch (state) {
    case ProcessNodeState::Disabled: return QStringLiteral("Disable");
    case ProcessNodeState::Editing: return QStringLiteral("Editing");
    case ProcessNodeState::Unavailable: return QStringLiteral("Unavailable");
    case ProcessNodeState::Running: return QStringLiteral("Run");
    case ProcessNodeState::Stopped: return QStringLiteral("Stop");
    case ProcessNodeState::Paused: return QStringLiteral("Pause");
    case ProcessNodeState::Unused: return QStringLiteral("Unuse");
    case ProcessNodeState::Pending: return QStringLiteral("Unrun");
    case ProcessNodeState::Enabled:
    default: return QStringLiteral("Enable");
    }
}

ProcessNodeState processNodeStateFromString(const QString& text)
{
    const QString normalized = text.trimmed();
    if (normalized == QStringLiteral("Disable")) return ProcessNodeState::Disabled;
    if (normalized == QStringLiteral("Editing")) return ProcessNodeState::Editing;
    if (normalized == QStringLiteral("Unavailable")) return ProcessNodeState::Unavailable;
    if (normalized == QStringLiteral("Run")) return ProcessNodeState::Running;
    if (normalized == QStringLiteral("Stop")) return ProcessNodeState::Stopped;
    if (normalized == QStringLiteral("Pause")) return ProcessNodeState::Paused;
    if (normalized == QStringLiteral("Unuse")) return ProcessNodeState::Unused;
    if (normalized == QStringLiteral("Unrun")) return ProcessNodeState::Pending;
    return ProcessNodeState::Enabled;
}

bool processNodeTypeCanHaveChildren(ProcessNodeType type)
{
    return type == ProcessNodeType::Group
        || type == ProcessNodeType::If
        || type == ProcessNodeType::Loop
        || type == ProcessNodeType::RunGroup;
}

} // namespace lcnc::process
