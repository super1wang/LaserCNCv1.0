#include "modules/process/workflow/process_node_registry.h"

namespace lcnc::process {

namespace {

QVariantMap map(std::initializer_list<std::pair<QString, QVariant>> values)
{
    QVariantMap result;
    for (const auto& item : values)
        result.insert(item.first, item.second);
    return result;
}

QString valueText(const QVariantMap& parameters, const QString& key, const QVariant& fallback = {})
{
    return parameters.value(key, fallback).toString();
}

} // namespace

const ProcessNodeRegistry& ProcessNodeRegistry::instance()
{
    static const ProcessNodeRegistry registry;
    return registry;
}

const ProcessNodeDescriptor* ProcessNodeRegistry::descriptor(ProcessNodeType type) const
{
    for (const auto& descriptor : m_descriptors) {
        if (descriptor.type == type)
            return &descriptor;
    }
    return nullptr;
}

QVector<ProcessNodeType> ProcessNodeRegistry::addableTypes() const
{
    QVector<ProcessNodeType> result;
    for (const auto& descriptor : m_descriptors) {
        if (descriptor.type != ProcessNodeType::Base)
            result.append(descriptor.type);
    }
    return result;
}

ProcessNode ProcessNodeRegistry::createDefaultNode(ProcessNodeType type) const
{
    ProcessNode node(type);
    if (const auto* item = descriptor(type)) {
        node.name = item->displayName;
        node.parameters = item->defaultParameters;
    }
    return node;
}

QString ProcessNodeRegistry::summary(const ProcessNode& node) const
{
    if (!node.enabled)
        return QStringLiteral("Disabled");
    if (node.parameters.contains(QStringLiteral("info")))
        return node.parameters.value(QStringLiteral("info")).toString();

    const QVariantMap& p = node.parameters;
    switch (node.type) {
    case ProcessNodeType::Start:
        return QStringLiteral("Entry");
    case ProcessNodeType::Stop:
        return QStringLiteral("Exit");
    case ProcessNodeType::Wait:
        return QStringLiteral("%1 ms").arg(p.value(QStringLiteral("durationMs"), 1000).toInt());
    case ProcessNodeType::Axis:
        return QStringLiteral("%1 -> %2").arg(
            valueText(p, QStringLiteral("axis"), QStringLiteral("X")),
            valueText(p, QStringLiteral("position"), 0.0));
    case ProcessNodeType::AxesMove:
        return QStringLiteral("XYZ %1,%2,%3 F%4").arg(
            valueText(p, QStringLiteral("x"), 0.0),
            valueText(p, QStringLiteral("y"), 0.0),
            valueText(p, QStringLiteral("z"), 0.0),
            valueText(p, QStringLiteral("feedRate"), 100.0));
    case ProcessNodeType::Feeding:
        return QStringLiteral("F%1").arg(valueText(p, QStringLiteral("feedRate"), 100.0));
    case ProcessNodeType::Cutting:
        return QStringLiteral("Contour %1 F%2 E%3").arg(
            valueText(p, QStringLiteral("contourId"), QStringLiteral("<unset>")),
            valueText(p, QStringLiteral("feedRate"), 100.0),
            valueText(p, QStringLiteral("laserEnergy"), 10.0));
    case ProcessNodeType::OverCutting:
        return QStringLiteral("Length %1").arg(valueText(p, QStringLiteral("length"), 0.0));
    case ProcessNodeType::IO:
        return QStringLiteral("%1 %2=%3").arg(
            valueText(p, QStringLiteral("channel"), QStringLiteral("DO0")),
            valueText(p, QStringLiteral("action"), QStringLiteral("set")),
            valueText(p, QStringLiteral("value"), 1));
    case ProcessNodeType::EnergySwitch:
        return QStringLiteral("Energy %1").arg(valueText(p, QStringLiteral("laserEnergy"), 10.0));
    case ProcessNodeType::Loop:
        return QStringLiteral("%1 loops").arg(valueText(p, QStringLiteral("count"), 1));
    case ProcessNodeType::If:
    case ProcessNodeType::Compare:
        return valueText(p, QStringLiteral("expression"), QStringLiteral("true"));
    case ProcessNodeType::Commands:
        return valueText(p, QStringLiteral("command"), QStringLiteral("noop"));
    case ProcessNodeType::Camera:
        return valueText(p, QStringLiteral("cameraId"), QStringLiteral("default"));
    case ProcessNodeType::Measurement:
        return valueText(p, QStringLiteral("target"), QStringLiteral("feature"));
    case ProcessNodeType::MarkAcquire:
        return valueText(p, QStringLiteral("markId"), QStringLiteral("mark"));
    case ProcessNodeType::Alignment:
        return valueText(p, QStringLiteral("method"), QStringLiteral("two-point"));
    case ProcessNodeType::AutoFocus:
        return QStringLiteral("Range %1").arg(valueText(p, QStringLiteral("range"), 5.0));
    case ProcessNodeType::Monitor:
        return valueText(p, QStringLiteral("signal"), QStringLiteral("ready"));
    case ProcessNodeType::RunGroup:
    case ProcessNodeType::RunGroupCheck:
        return valueText(p, QStringLiteral("groupName"), QStringLiteral("default"));
    case ProcessNodeType::Group:
        return QStringLiteral("Children %1").arg(node.children.size());
    default:
        return QStringLiteral("Ready");
    }
}

bool ProcessNodeRegistry::canPlaceNode(ProcessNodeType type, const ProcessNodeType* parentType) const
{
    if (const auto* item = descriptor(type)) {
        if (item->topLevelOnly && parentType)
            return false;
    }

    if (!parentType)
        return true;
    return canHaveChildren(*parentType);
}

bool ProcessNodeRegistry::canHaveChildren(ProcessNodeType type) const
{
    if (const auto* item = descriptor(type))
        return item->canHaveChildren;
    return false;
}

ProcessNodeRegistry::ProcessNodeRegistry()
{
    registerBuiltIns();
}

void ProcessNodeRegistry::registerBuiltIns()
{
    add(ProcessNodeType::Start, QStringLiteral("Structure"), false, true, {}, QStringLiteral("start"));
    add(ProcessNodeType::Stop, QStringLiteral("Structure"), false, true, {}, QStringLiteral("stop"));
    add(ProcessNodeType::Wait, QStringLiteral("Structure"), false, false,
        map({ { QStringLiteral("durationMs"), 1000 } }), QStringLiteral("wait"));
    add(ProcessNodeType::Group, QStringLiteral("Structure"), true, false, {}, QStringLiteral("group"));
    add(ProcessNodeType::RunGroup, QStringLiteral("Structure"), true, false,
        map({ { QStringLiteral("groupName"), QStringLiteral("default") } }), QStringLiteral("runGroup"));
    add(ProcessNodeType::RunGroupCheck, QStringLiteral("Structure"), false, false,
        map({ { QStringLiteral("groupName"), QStringLiteral("default") } }), QStringLiteral("runGroupCheck"));
    add(ProcessNodeType::If, QStringLiteral("Structure"), true, false,
        map({ { QStringLiteral("expression"), QStringLiteral("true") } }), QStringLiteral("if"));
    add(ProcessNodeType::Loop, QStringLiteral("Structure"), true, false,
        map({ { QStringLiteral("count"), 1 } }), QStringLiteral("loop"));

    add(ProcessNodeType::Axis, QStringLiteral("Motion"), false, false,
        map({ { QStringLiteral("axis"), QStringLiteral("X") }, { QStringLiteral("position"), 0.0 } }), QStringLiteral("axis"));
    add(ProcessNodeType::AxesMove, QStringLiteral("Motion"), false, false,
        map({ { QStringLiteral("x"), 0.0 }, { QStringLiteral("y"), 0.0 }, { QStringLiteral("z"), 0.0 }, { QStringLiteral("feedRate"), 100.0 } }), QStringLiteral("axesMove"));
    add(ProcessNodeType::Feeding, QStringLiteral("Motion"), false, false,
        map({ { QStringLiteral("feedRate"), 100.0 } }), QStringLiteral("feeding"));
    add(ProcessNodeType::AutoFocus, QStringLiteral("Motion"), false, false,
        map({ { QStringLiteral("range"), 5.0 }, { QStringLiteral("speed"), 1.0 } }), QStringLiteral("autoFocus"));

    add(ProcessNodeType::Cutting, QStringLiteral("Process"), false, false,
        map({ { QStringLiteral("contourId"), QString() }, { QStringLiteral("feedRate"), 100.0 }, { QStringLiteral("laserEnergy"), 10.0 }, { QStringLiteral("dryRun"), true } }), QStringLiteral("cutting"));
    add(ProcessNodeType::OverCutting, QStringLiteral("Process"), false, false,
        map({ { QStringLiteral("length"), 0.0 }, { QStringLiteral("feedRate"), 100.0 } }), QStringLiteral("overCutting"));
    add(ProcessNodeType::EnergySwitch, QStringLiteral("Process"), false, false,
        map({ { QStringLiteral("laserEnergy"), 10.0 } }), QStringLiteral("energySwitch"));

    add(ProcessNodeType::IO, QStringLiteral("Device"), false, false,
        map({ { QStringLiteral("channel"), QStringLiteral("DO0") }, { QStringLiteral("action"), QStringLiteral("set") }, { QStringLiteral("value"), 1 } }), QStringLiteral("io"));
    add(ProcessNodeType::Commands, QStringLiteral("Device"), false, false,
        map({ { QStringLiteral("command"), QStringLiteral("noop") } }), QStringLiteral("commands"));
    add(ProcessNodeType::Monitor, QStringLiteral("Device"), false, false,
        map({ { QStringLiteral("signal"), QStringLiteral("ready") }, { QStringLiteral("expected"), true } }), QStringLiteral("monitor"));
    add(ProcessNodeType::Camera, QStringLiteral("Vision"), false, false,
        map({ { QStringLiteral("cameraId"), QStringLiteral("default") }, { QStringLiteral("exposureMs"), 10 } }), QStringLiteral("camera"));
    add(ProcessNodeType::Measurement, QStringLiteral("Vision"), false, false,
        map({ { QStringLiteral("target"), QStringLiteral("feature") }, { QStringLiteral("tolerance"), 0.01 } }), QStringLiteral("measurement"));
    add(ProcessNodeType::MarkAcquire, QStringLiteral("Vision"), false, false,
        map({ { QStringLiteral("markId"), QStringLiteral("mark") } }), QStringLiteral("markAcquire"));
    add(ProcessNodeType::Alignment, QStringLiteral("Vision"), false, false,
        map({ { QStringLiteral("method"), QStringLiteral("two-point") } }), QStringLiteral("alignment"));
    add(ProcessNodeType::Calculation, QStringLiteral("Logic"), false, false,
        map({ { QStringLiteral("expression"), QStringLiteral("0") }, { QStringLiteral("output"), QStringLiteral("result") } }), QStringLiteral("calculation"));
    add(ProcessNodeType::Compare, QStringLiteral("Logic"), false, false,
        map({ { QStringLiteral("expression"), QStringLiteral("left == right") } }), QStringLiteral("compare"));
}

void ProcessNodeRegistry::add(ProcessNodeType type,
                              const QString& category,
                              bool canHaveChildren,
                              bool topLevelOnly,
                              QVariantMap defaults,
                              const QString& executorKey)
{
    ProcessNodeDescriptor descriptor;
    descriptor.type = type;
    descriptor.displayName = processNodeTypeToString(type);
    descriptor.category = category;
    descriptor.canHaveChildren = canHaveChildren;
    descriptor.topLevelOnly = topLevelOnly;
    descriptor.defaultParameters = std::move(defaults);
    descriptor.executorKey = executorKey.isEmpty() ? processNodeTypeToString(type) : executorKey;
    m_descriptors.append(std::move(descriptor));
}

} // namespace lcnc::process
