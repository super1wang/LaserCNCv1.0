#include "modules/process/workflow/process_node_registry.h"

#include "modules/process/steps/process_step_registry.h"

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
    static ProcessNodeDescriptor pluginDescriptor;
    const auto& stepRegistry = ProcessStepRegistry::instance();
    for (const auto& d : stepRegistry.descriptorsAll()) {
        if (d.type == type) {
            pluginDescriptor = d;
            return &pluginDescriptor;
        }
    }
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
        if (descriptor.type != ProcessNodeType::Base && descriptor.addable)
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
    if (auto step = ProcessStepRegistry::instance().step(node.type))
        return step->summary(node);
    if (node.parameters.contains(QStringLiteral("info")))
        return node.parameters.value(QStringLiteral("info")).toString();

    const QVariantMap& p = node.parameters;
    switch (node.type) {
    case ProcessNodeType::Start:
        // 中文翻译：入口 / 全局变量 %1 项
        return QStringLiteral("Entry/global variable %1 item").arg(p.value(QStringLiteral("variables"), QVariantList{}).toList().size());
    case ProcessNodeType::Stop:
        // 中文翻译：流程结束
        return valueText(p, QStringLiteral("message"), QStringLiteral("End of process"));
    case ProcessNodeType::Axis:
        return QStringLiteral("%1 %2 %3 F%4").arg(
            valueText(p, QStringLiteral("axis"), QStringLiteral("X")),
            valueText(p, QStringLiteral("mode"), QStringLiteral("absolute")),
            valueText(p, QStringLiteral("target"), 0.0),
            valueText(p, QStringLiteral("velocity"), 5.0));
    case ProcessNodeType::AxesMove:
        // 中文翻译：%1，多轴 %2 项
        return QStringLiteral("%1, multi-axis %2 items").arg(
            valueText(p, QStringLiteral("multiMode"), QStringLiteral("sequential")),
            QString::number(p.value(QStringLiteral("axes"), QVariantList{}).toList().size()));
    case ProcessNodeType::IO:
        return QStringLiteral("%1 %2=%3").arg(
            valueText(p, QStringLiteral("signalType"), QStringLiteral("digital")),
            valueText(p, QStringLiteral("ioName"), QStringLiteral("aLaser")),
            valueText(p, QStringLiteral("value"), 1));
    case ProcessNodeType::Monitor:
        // 中文翻译：等待 %1=%2 timeout=%3ms
        return QStringLiteral("Wait %1=%2 timeout=%3ms").arg(
            valueText(p, QStringLiteral("ioName"), QStringLiteral("aStart")),
            valueText(p, QStringLiteral("targetValue"), true),
            valueText(p, QStringLiteral("timeoutMs"), 5000));
    case ProcessNodeType::Cutting:
        // 中文翻译：普通切割
        return QStringLiteral("Ordinary cutting");
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

bool ProcessNodeRegistry::isRequired(ProcessNodeType type) const
{
    if (const auto* item = descriptor(type))
        return item->required;
    return false;
}

bool ProcessNodeRegistry::isDeletable(ProcessNodeType type) const
{
    if (const auto* item = descriptor(type))
        return item->deletable;
    return true;
}

bool ProcessNodeRegistry::isDisableable(ProcessNodeType type) const
{
    if (const auto* item = descriptor(type))
        return item->disableable;
    return true;
}

bool ProcessNodeRegistry::isMovable(ProcessNodeType type) const
{
    if (const auto* item = descriptor(type))
        return item->movable;
    return true;
}

ProcessNodeRegistry::ProcessNodeRegistry()
{
    registerBuiltIns();
}

void ProcessNodeRegistry::registerBuiltIns()
{
    add(ProcessNodeType::Start, QStringLiteral("Structure"), false, true,
        map({ { QStringLiteral("variables"), QVariantList{} } }), QStringLiteral("start"), true, false);
    add(ProcessNodeType::Stop, QStringLiteral("Structure"), false, true,
        // 中文翻译：流程结束
        map({ { QStringLiteral("message"), QStringLiteral("End of process") }, { QStringLiteral("safeStopOutputs"), true }, { QStringLiteral("stopMotion"), false } }), QStringLiteral("stop"), true, false);

    add(ProcessNodeType::SingleAxisMove, QStringLiteral("Motion"), false, false,
        map({ { QStringLiteral("axis"), QStringLiteral("X") },
              { QStringLiteral("velocity"), 5.0 },
              { QStringLiteral("mode"), QStringLiteral("absolute") },
              { QStringLiteral("target"), 0.0 },
              { QStringLiteral("timeoutMs"), 30000 } }), QStringLiteral("singleAxisMove"));
    add(ProcessNodeType::MultiAxisMove, QStringLiteral("Motion"), false, false,
        map({ { QStringLiteral("multiMode"), QStringLiteral("sequential") },
              { QStringLiteral("axes"), QVariantList{} },
              { QStringLiteral("timeoutMs"), 30000 } }), QStringLiteral("multiAxisMove"));

    add(ProcessNodeType::OutputSignal, QStringLiteral("IO"), false, false,
        map({ { QStringLiteral("signalType"), QStringLiteral("digital") },
              { QStringLiteral("ioName"), QStringLiteral("aLaser") },
              { QStringLiteral("value"), true } }), QStringLiteral("outputSignal"));
    add(ProcessNodeType::InputSignalWait, QStringLiteral("IO"), false, false,
        map({ { QStringLiteral("signalType"), QStringLiteral("digital") },
              { QStringLiteral("ioName"), QStringLiteral("aStart") },
              { QStringLiteral("targetValue"), true },
              { QStringLiteral("timeoutMs"), 5000 },
              { QStringLiteral("pollIntervalMs"), 100 } }), QStringLiteral("inputSignalWait"));

    add(ProcessNodeType::NormalCutting, QStringLiteral("Process"), false, false,
        map({ { QStringLiteral("selectionMode"), QStringLiteral("allEnabled") },
              { QStringLiteral("startNumber"), 1 },
              { QStringLiteral("endNumber"), 0 },
              { QStringLiteral("compensationIndex"), QString() } }), QStringLiteral("normalCutting"));
}

void ProcessNodeRegistry::add(ProcessNodeType type,
                              const QString& category,
                              bool canHaveChildren,
                              bool topLevelOnly,
                              QVariantMap defaults,
                              const QString& executorKey,
                              bool required,
                              bool addable)
{
    ProcessNodeDescriptor descriptor;
    descriptor.type = type;
    descriptor.displayName = processNodeTypeToString(type);
    descriptor.category = category;
    descriptor.canHaveChildren = canHaveChildren;
    descriptor.topLevelOnly = topLevelOnly;
    descriptor.required = required;
    descriptor.addable = addable;
    descriptor.deletable = !required;
    descriptor.disableable = !required;
    descriptor.movable = !required;
    descriptor.defaultParameters = std::move(defaults);
    descriptor.executorKey = executorKey.isEmpty() ? processNodeTypeToString(type) : executorKey;
    m_descriptors.append(std::move(descriptor));
}

} // namespace lcnc::process
